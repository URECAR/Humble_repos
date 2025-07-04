#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument('mode', default_value='virtual', 
                                   description='모드: virtual/real')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.56.101',
                                       description='로봇 IP')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='true',
                                                description='가짜 하드웨어 사용 여부')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true',
                                          description='RViz 실행 여부')
    
    # Launch configurations
    mode = LaunchConfiguration('mode')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # Robot description (UR10e + Hande 통합) - ParameterValue 사용
    robot_description_content = ParameterValue(
        Command([
            'xacro ', 
            PathJoinSubstitution([
                FindPackageShare('ur10e_hande_description'),
                'urdf',
                'ur10e_hande_description.urdf.xacro'
            ]),
            ' use_fake_hardware:=', use_fake_hardware
        ]),
        value_type=str
    )
    
    # 1. Robot State Publisher (TF 변환)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # 2. UR 드라이버 (실제 모드) 또는 가상 컨트롤러
    ur_driver_real = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'),
            'launch', 'ur_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': 'ur10e',
            'robot_ip': robot_ip,
            'use_fake_hardware': 'false',
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller'
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "real"']))
    )
    
    # 3. Controller Manager (가상 모드)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            PathJoinSubstitution([
                FindPackageShare('ur10e_hande_moveit_config'),
                'config', 'ros2_controllers.yaml'
            ])
        ],
        condition=IfCondition(use_fake_hardware)
    )
    
    # 4. Joint State Broadcaster (핵심!)
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    # 5. UR 컨트롤러 Spawner (가상 모드)
    ur_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['scaled_joint_trajectory_controller'],
                output='screen'
            )
        ],
        condition=IfCondition(use_fake_hardware)
    )
    
    # 6. Hande 그리퍼 컨트롤러 Spawner
    hande_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['hande_gripper_controller'],
                output='screen'
            )
        ]
    )
    
    # 7. Hande 그리퍼 드라이버
    hande_driver = Node(
        package='ur10e_hande_bringup',
        executable='hande_driver.py',
        name='hande_driver',
        output='screen',
        parameters=[{
            'mode': mode,
            'host': robot_ip,
            'port': 63352
        }]
    )
    
    # 8. MoveIt Move Group
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('ur10e_hande_moveit_config'),
                    'launch', 'move_group.launch.py'
                ])
            )
        ]
    )
    
    # 9. RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ur10e_hande_moveit_config'),
        'config', 'moveit.rviz'
    ])
    
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config],
                parameters=[{'robot_description': robot_description_content}]
            )
        ],
        condition=IfCondition(launch_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        mode_arg,
        robot_ip_arg, 
        use_fake_hardware_arg,
        launch_rviz_arg,
        
        # Core nodes
        robot_state_publisher,
        ur_driver_real,
        controller_manager,
        
        # Controllers (순서대로 실행)
        joint_state_broadcaster_spawner,
        ur_controller_spawner,
        hande_controller_spawner,
        
        # Drivers
        hande_driver,
        
        # High-level
        moveit_launch,
        rviz
    ])