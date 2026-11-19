"""Whole baggage cell: Gazebo room + UR10e + MoveIt + belt + camera + router + carts + metrics.

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
        DeclareLaunchArgument("results_dir",
                              default_value=os.path.expanduser("~/bag_cell_results")),
        DeclareLaunchArgument("seed", default_value="1", description="random seed (no-reads, bag offsets)"),
    ]

    # Gazebo finds the sign textures (model://bag_hall_textures/...) here
    textures = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                                         PathJoinSubstitution([bringup, "models"]))

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])),
        launch_arguments={"ur_type": "ur10e", "world_file": world,
                          "description_file": robot, "launch_rviz": "false"}.items())

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, "launch", "moveit.launch.py"])),
        launch_arguments={"rviz": LaunchConfiguration("rviz")}.items())

    # Gazebo <-> ROS: camera images, photo-eye beam, belt speed command
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="cell_bridge",
        arguments=["/photo_eye@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                   "/model/belt/link/base_link/track_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                   "/overhead_cam/image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/overhead_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/overhead_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        parameters=[sim_time], output="screen")

    nodes = [
        Node(package="bag_cell_scene", executable="scene_publisher", parameters=[sim_time]),
        Node(package="bag_cell_scene", executable="belt_controller", output="screen",
             parameters=[{"seed": LaunchConfiguration("seed")}]),
        Node(package="bag_cell_scene", executable="flight_router", output="screen"),
        Node(package="bag_cell_perception", executable="bag_detector", output="screen",
             parameters=[sim_time]),
        Node(package="bag_cell_planning", executable="cart_manager", output="screen"),
        Node(package="bag_cell_planning", executable="metrics_logger", output="screen",
             parameters=[{"output_dir": LaunchConfiguration("results_dir")}]),
    ]
    return LaunchDescription(args + [textures, ur_sim, moveit, bridge] + nodes)
