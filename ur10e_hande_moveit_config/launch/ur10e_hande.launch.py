#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # Launch arguments
    mode = LaunchConfiguration('mode')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    rviz = LaunchConfiguration('rviz')
    
    # UR 드라이버 launch 파일 포함
    ur_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'),
            'launch',
            'ur_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': 'ur10e',
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',  # 별도로 실행
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true'
        }.items()
    )
    
    # Hande 그리퍼 드라이버 노드
    gripper_driver_node = Node(
        package='ur10e_hande_bringup',
        executable='robotiq_hande_driver.py',
        name='robotiq_hande_driver',
        output='screen',
        parameters=[{
            'mode': mode,
            'host': robot_ip,
            'port': 63352
        }]
    )
    
    # 그리퍼 컨트롤러 spawner (가상 모드일 때만)
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hande_gripper_controller'],
        condition=IfCondition(PythonExpression(['"', mode, '" == "virtual"']))
    )
    
    # Robot description for MoveIt
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare('xacro'), 'xacro']),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ur10e_hande_description'),
            'urdf',
            'ur10e_hande_description.urdf.xacro'
        ]),
        ' use_fake_hardware:=', use_fake_hardware
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # MoveIt launch 파일 포함
    moveit_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ur10e_hande_moveit_config'),
            'launch',
            'demo.launch.py'
        ]),
        launch_arguments={
            'mode': mode,
            'use_fake_hardware': use_fake_hardware
        }.items()
    )
    
    # RViz launch (옵션)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur10e_hande_moveit_config'),
        'config',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(rviz)
    )
    
    return [
        ur_launch,
        gripper_driver_node,
        gripper_controller_spawner,
        robot_state_publisher_node,
        moveit_launch,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments 선언
        DeclareLaunchArgument(
            'mode',
            default_value='virtual',
            description='그리퍼 모드: virtual (가상) 또는 real (실제)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='로봇 IP 주소'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='가짜 하드웨어 인터페이스 사용 여부'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='RViz 실행 여부'
        ),
        
        # OpaqueFunction을 사용하여 런타임에 launch 설정
        OpaqueFunction(function=launch_setup)
    ])