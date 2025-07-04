
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to the ur10e_hande_moveit_config package
    moveit_config_package_path = FindPackageShare('ur10e_hande_moveit_config')

    # Robot State Publisher (RSP) launch file
    rsp_launch_file = PathJoinSubstitution([
        moveit_config_package_path,
        'launch',
        'rsp.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Include the Robot State Publisher launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rsp_launch_file]),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),

        # Launch the GUI Node
        Node(
            package='ur10e_hande_gui',
            executable='ur10e_hande_gui_node',
            name='ur10e_hande_gui_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # We assume that servo is already launched separately, for example via:
        # ros2 launch ur10e_hande_moveit_config servo.launch.py
        # If not, you would add the servo launch description here.
    ])

