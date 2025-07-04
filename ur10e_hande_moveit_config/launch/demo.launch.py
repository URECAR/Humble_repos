from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Launch arguments
    mode = LaunchConfiguration('mode')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
    moveit_config = MoveItConfigsBuilder("ur10e_hande_robot", package_name="ur10e_hande_moveit_config").to_moveit_configs()
    
    # 2F-140 호환 그리퍼 드라이버 노드
    gripper_driver_node = Node(
        package='ur10e_hande_bringup',  # 패키지명은 실제 패키지에 맞게 수정
        executable='robotiq_hande_driver.py',  # 2F-140 드라이버를 Hande용으로 수정한 실행파일
        name='hande_driver',
        output='screen',
        parameters=[{
            'mode': mode,
            'host': '192.168.56.101',  # 실제 로봇 IP로 수정
            'port': 63352
        }]
    )
    
    # MoveIt demo launch 생성
    demo_launch = generate_demo_launch(moveit_config)
    
    # 모든 launch action을 리스트로 결합
    launch_actions = demo_launch.entities + [gripper_driver_node]
    
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments 선언
        DeclareLaunchArgument(
            'mode',
            default_value='virtual',
            description='그리퍼 모드 (virtual/real)'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Use fake hardware interface'
        ),
        
        # OpaqueFunction을 사용하여 런타임에 launch 설정
        OpaqueFunction(function=launch_setup)
    ])