"""Start the robot's main loop (run it after bag_cell.launch.py is up):
    ros2 launch bag_cell_bringup cell.launch.py num_bags:=10
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("bag_cell_bringup"), "config", "cell_node.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("num_bags", default_value="10"),
        Node(package="bag_cell_motion", executable="cell_node", output="screen",
             parameters=[params, {"num_bags": LaunchConfiguration("num_bags")}]),
    ])
