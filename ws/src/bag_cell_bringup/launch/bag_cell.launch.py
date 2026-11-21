"""Whole baggage cell: Gazebo room + UR20 + MoveIt + belt + camera + router + carts + metrics.

    ros2 launch bag_cell_bringup bag_cell.launch.py
    ros2 run bag_cell_motion cell_node --ros-args -p num_bags:=20 -p use_sim_time:=true
"""
import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("bag_cell_bringup")
    world = PathJoinSubstitution([bringup, "worlds", "bag_hall.sdf"])
    robot = PathJoinSubstitution([FindPackageShare("bag_cell_description"), "urdf",
                                  "bag_cell_robot.urdf.xacro"])
    sim_time = {"use_sim_time": True}

    args = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Gazebo window (false = faster runs, results only)"),
        DeclareLaunchArgument("results_dir",
                              default_value=os.path.expanduser("~/bag_cell_results")),
        DeclareLaunchArgument("seed", default_value="1", description="random seed (bag positions on the belt)"),
    ]

    # Gazebo finds the sign textures (model://bag_hall_textures/...) here
    textures = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                                         PathJoinSubstitution([bringup, "models"]))

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])),
        launch_arguments={"ur_type": "ur20", "world_file": world,
                          "description_file": robot, "launch_rviz": "false",
                          "gazebo_gui": LaunchConfiguration("gui")}.items())

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, "launch", "moveit.launch.py"])),
        launch_arguments={"rviz": LaunchConfiguration("rviz")}.items())

    # Gazebo <-> ROS: camera images, photo-eye beam, belt speed command, spawn/remove
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="cell_bridge",
        arguments=["/photo_eye@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                   "/model/belt/link/base_link/track_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                   "/overhead_cam/image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/overhead_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/overhead_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                   # spawn / remove models (belt, cart manager)
                   "/world/bag_hall/create@ros_gz_interfaces/srv/SpawnEntity",
                   "/world/bag_hall/remove@ros_gz_interfaces/srv/DeleteEntity"],
        parameters=[sim_time], output="screen")

    nodes = [
        Node(package="bag_cell_scene", executable="scene_publisher", parameters=[sim_time]),
        # true bag poses by name (gz-transport -> /cell/bag_poses)
        Node(package="bag_cell_motion", executable="gz_pose_relay", parameters=[sim_time]),
        Node(package="bag_cell_scene", executable="belt_controller", output="screen",
             parameters=[sim_time, {"seed": LaunchConfiguration("seed")}]),
        Node(package="bag_cell_scene", executable="flight_router", output="screen",
             parameters=[sim_time]),
        Node(package="bag_cell_perception", executable="bag_detector", output="screen",
             parameters=[sim_time]),
        Node(package="bag_cell_planning", executable="cart_manager", output="screen",
             parameters=[sim_time]),
        Node(package="bag_cell_planning", executable="metrics_logger", output="screen",
             parameters=[sim_time, {"output_dir": LaunchConfiguration("results_dir")}]),
    ]
    return LaunchDescription(args + [textures, ur_sim, moveit, bridge] + nodes)
