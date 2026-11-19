"""MoveIt for the UR10e: copy of ur_moveit_config/launch/ur_moveit.launch.py (move_group +
RViz), only change: our config/kinematics.yaml (longer IK timeout)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    kinematics = Path(get_package_share_directory("bag_cell_bringup")) / "config" / "kinematics.yaml"
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur10e"})
        .robot_description_kinematics(file_path=str(kinematics))
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": True}

    wait = Node(package="ur_robot_driver", executable="wait_for_robot_description",
                output="screen")
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), sim_time,
                    {"publish_robot_description_semantic": True}])
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config",
                                               "moveit.rviz"])],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines, moveit_config.joint_limits, sim_time])
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        wait,
        # start MoveIt only once the robot description is published
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=[move_group, rviz])),
    ])
