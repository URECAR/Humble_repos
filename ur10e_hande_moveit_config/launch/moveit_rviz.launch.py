from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur10e_hande_robot", package_name="ur10e_hande_moveit_config").to_moveit_configs()
    
    # RViz 설정 파일 경로
    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur10e_hande_moveit_config"),
        "config",
        "moveit.rviz"
    ])
    
    # RViz 노드 생성 (올바른 파라미터와 함께)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    
    return LaunchDescription([rviz_node])