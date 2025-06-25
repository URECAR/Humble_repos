#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', 
        default_value='192.168.56.101',
        description='UR 로봇 IP 주소'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='가상 하드웨어 사용 여부'
    )
    
    execute_arg = DeclareLaunchArgument(
        'execute',
        default_value='true',
        description='계획된 경로 실행 여부'
    )
    
    # Launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    execute = LaunchConfiguration('execute')
    
    # UR10e + Hande MoveIt 설정
    moveit_config = MoveItConfigsBuilder("ur10e_hande_robot", package_name="ur10e_hande_moveit_config").to_moveit_configs()

    # 1. UR 드라이버 실행
    ur_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'),
            'launch', 'ur10e.launch.py'
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_tool_communication': 'true',
            'launch_rviz': 'false',
            'use_fake_hardware': use_fake_hardware
        }.items()
    )

    # 2. Hande 그리퍼 드라이버
    hande_driver = Node(
        package='ur10e_hande_bringup',
        executable='robotiq_hande_driver.py',
        name='robotiq_hande_driver',
        output='screen',
        parameters=[{
            'mode': 'real',
            'host': robot_ip,
            'port': 63352
        }]
    )

    # 3. Move Group (ExecuteTaskSolutionCapability 포함)
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
    
    run_move_group_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    move_group_capabilities,
                ],
            )
        ]
    )

    # 4. RViz 설정
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur10e_hande_mtc_demo"),
        "config", "mtc.rviz"
    ])
    
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            )
        ]
    )

    # 5. MTC Pick/Place Demo 노드
    pick_place_demo_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ur10e_hande_mtc_demo',
                executable='pick_place_demo',
                name='pick_place_demo',
                output='screen',
                # 필요한 MoveIt 파라미터들 명시
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    {
                        'execute': execute,
                        'max_solutions': 10
                    }
                ]

            )
        ]
    )

    return LaunchDescription([
        # Arguments
        robot_ip_arg,
        use_fake_hardware_arg,
        execute_arg,
        
        # Nodes and launch files
        ur_driver_launch,
        hande_driver,
        run_move_group_node,
        rviz_node,
        # pick_place_demo_node,
    ])