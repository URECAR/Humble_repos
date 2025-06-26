import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur10e_hande")
        .robot_description(file_path="config/ur10e_hande_robot.urdf.xacro")
        .to_moveit_configs()
    )

    package = "ur10e_hande_mtc"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable=LaunchConfiguration("exe"),
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            os.path.join(package_shared_path, "config", "ur10e_hande_config.yaml"),
        ],
    )

    arg = DeclareLaunchArgument(name="exe")
    return LaunchDescription([arg, node])
