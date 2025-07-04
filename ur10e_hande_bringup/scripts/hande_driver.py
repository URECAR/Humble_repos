#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import sys
import os

# ROS2 메시지 및 액션
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool, UInt8
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

# 커스텀 메시지
from ur10e_hande_bringup.msg import Robotiq2FGripperCommand, Robotiq2FGripperStatus
from ur10e_hande_bringup.srv import GripperControl


class RobotiqHandeDriver(Node):
    """Robotiq Hande 그리퍼 ROS2 드라이버 (mimic 동작 구현)"""
    
    def __init__(self):
        super().__init__('hande_driver')
        
        # 파라미터 선언 및 가져오기
        self.declare_parameter('mode', 'virtual')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 63352)
        
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        self.get_logger().info(f"Robotiq Hande 그리퍼 드라이버 시작: {self.mode} 모드")
        
        # 상태 변수 - 실제로는 하나의 위치값으로 제어
        self.position = 0.0        # 실제 그리퍼 위치 (왼쪽 기준)
        self.target_position = 0.0 # 목표 위치
        self.velocity = 0.0        # 속도
        
        self.moving = False
        self.activated = True
        self.object_detected = False
        
        # 시뮬레이션용 변수
        self.movement_active = False
        self.movement_start_time = None
        self.movement_duration = 1.0
        
        # 액션 상태 관리
        self.current_action_goal = None
        self.action_lock = threading.Lock()
        
        # ROS2 인터페이스 초기화
        self._init_ros_interfaces()
        
        # 상태 업데이트 타이머
        self.update_timer = self.create_timer(0.1, self.update_state)
        
        self.get_logger().info("Robotiq Hande 그리퍼 드라이버 초기화 완료 (mimic 동작)")
    
    def _init_ros_interfaces(self):
        """ROS2 인터페이스 초기화"""
        # 퍼블리셔들
        self.status_pub = self.create_publisher(Robotiq2FGripperStatus, 'hande/status', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.controller_state_pub = self.create_publisher(JointTrajectoryControllerState, 'hande_gripper_controller/state', 10)
        
        # 편의용 퍼블리셔들
        self.width_pub = self.create_publisher(Float64, 'hande/width', 10)
        self.object_pub = self.create_publisher(Bool, 'hande/object_detected', 10)
        self.moving_pub = self.create_publisher(Bool, 'hande/moving', 10)
        self.position_pub = self.create_publisher(UInt8, 'hande/position', 10)
        
        # 구독자들
        self.cmd_sub = self.create_subscription(
            Robotiq2FGripperCommand, 'hande/command', self.command_callback, 10)
        
        # 액션 서버들 - 두 조인트 모두 포함
        self.trajectory_action_server = ActionServer(
            self, FollowJointTrajectory, 'hande_gripper_controller/follow_joint_trajectory',
            self.execute_trajectory)
        
        self.gripper_action_server = ActionServer(
            self, GripperCommand, 'hande/gripper_action',
            self.execute_gripper_command)
        
        # 기존 서비스 (호환성을 위해 유지)
        self.control_service = self.create_service(
            GripperControl, 'hande/control', self.handle_control_service)
        
        self.get_logger().info("MoveIt 호환 액션 서버: /hande_gripper_controller/follow_joint_trajectory (mimic 동작)")
        self.get_logger().info("그리퍼 액션 서버: /hande/gripper_action")
        self.get_logger().info("그리퍼 제어 서비스: /hande/control")
    
    def execute_gripper_command(self, goal_handle):
        """그리퍼 명령 액션 실행"""
        self.get_logger().info(f'그리퍼 액션 요청 수신: position={goal_handle.request.command.position:.4f}m')
        
        with self.action_lock:
            self.current_action_goal = goal_handle
        
        # 목표 위치 설정 (0.0 = 열림, 0.025 = 닫힘)
        target_pos = goal_handle.request.command.position
        target_pos = max(0.0, min(target_pos, 0.025))  # 범위 제한
        
        self.target_position = target_pos
        self.start_movement_simulation()
        
        # 이동 완료까지 대기
        feedback_msg = GripperCommand.Feedback()
        result = GripperCommand.Result()
        
        timeout = 5.0
        start_time = time.time()
        
        while self.moving and (time.time() - start_time) < timeout:
            # 피드백 발행
            feedback_msg.position = self.position
            feedback_msg.effort = 0.0
            feedback_msg.stalled = self.object_detected
            feedback_msg.reached_goal = False
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.1)
        
        # 결과 설정
        if self.moving:
            self.get_logger().warn('그리퍼 액션 타임아웃')
            goal_handle.abort()
            result.position = self.position
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
        else:
            self.get_logger().info(f'그리퍼 액션 완료: {self.position:.4f}m')
            goal_handle.succeed()
            result.position = self.position
            result.effort = 0.0
            result.stalled = self.object_detected
            result.reached_goal = abs(self.position - target_pos) < 0.001
        
        with self.action_lock:
            self.current_action_goal = None
        
        return result
    
    def execute_trajectory(self, goal_handle):
        """MoveIt trajectory 실행 (mimic 동작 구현)"""
        self.get_logger().info('MoveIt 궤적 실행 요청 수신')
        
        trajectory = goal_handle.request.trajectory
        
        if not trajectory.points:
            self.get_logger().warn('궤적 포인트가 없습니다')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # 조인트 인덱스 찾기
        left_idx = None
        right_idx = None
        
        try:
            if 'hande_left_finger_joint' in trajectory.joint_names:
                left_idx = trajectory.joint_names.index('hande_left_finger_joint')
            if 'hande_right_finger_joint' in trajectory.joint_names:
                right_idx = trajectory.joint_names.index('hande_right_finger_joint')
        except ValueError:
            pass
        
        if left_idx is None and right_idx is None:
            self.get_logger().warn(f'그리퍼 조인트가 궤적에 없습니다. 조인트: {trajectory.joint_names}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # 마지막 포인트의 위치를 목표로 설정
        target_point = trajectory.points[-1]
        
        # 왼쪽 조인트 기준으로 제어 (실제 그리퍼는 하나의 액추에이터)
        if left_idx is not None and len(target_point.positions) > left_idx:
            self.target_position = max(0.0, min(target_point.positions[left_idx], 0.025))
        elif right_idx is not None and len(target_point.positions) > right_idx:
            # 오른쪽만 있는 경우도 처리 (같은 값 사용)
            self.target_position = max(0.0, min(target_point.positions[right_idx], 0.025))
        
        # 시간 설정
        trajectory_time = target_point.time_from_start.sec + target_point.time_from_start.nanosec * 1e-9
        if trajectory_time <= 0:
            trajectory_time = 0.5
        
        self.movement_duration = min(trajectory_time, 2.0)
        
        self.get_logger().info(f'MoveIt 목표: {self.target_position:.4f}m (mimic으로 양쪽 동일), 시간: {trajectory_time:.2f}s')
        
        self.start_movement_simulation()
        
        # 이동 완료 대기 (피드백 포함)
        feedback_msg = FollowJointTrajectory.Feedback()
        joint_names = []
        if left_idx is not None:
            joint_names.append('hande_left_finger_joint')
        if right_idx is not None:
            joint_names.append('hande_right_finger_joint')
        
        feedback_msg.joint_names = joint_names
        
        start_time = time.time()
        feedback_rate = 20
        
        while self.moving:
            elapsed = time.time() - start_time
            
            if elapsed > (self.movement_duration + 0.5):
                self.get_logger().warn(f'MoveIt 궤적 실행 타임아웃: {elapsed:.2f}s')
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            
            # 피드백 발행 (mimic: 양쪽 동일한 값)
            feedback_msg.header.stamp = self.get_clock().now().to_msg()
            
            actual_positions = []
            actual_velocities = []
            desired_positions = []
            desired_velocities = []
            error_positions = []
            error_velocities = []
            
            if left_idx is not None:
                actual_positions.append(self.position)
                actual_velocities.append(self.velocity)
                desired_positions.append(self.target_position)
                desired_velocities.append(0.0)
                error_positions.append(self.target_position - self.position)
                error_velocities.append(0.0 - self.velocity)
            
            if right_idx is not None:
                # 오른쪽도 같은 값 (mimic 동작)
                actual_positions.append(self.position)
                actual_velocities.append(self.velocity)
                desired_positions.append(self.target_position)
                desired_velocities.append(0.0)
                error_positions.append(self.target_position - self.position)
                error_velocities.append(0.0 - self.velocity)
            
            feedback_msg.actual.positions = actual_positions
            feedback_msg.actual.velocities = actual_velocities
            feedback_msg.desired.positions = desired_positions
            feedback_msg.desired.velocities = desired_velocities
            feedback_msg.error.positions = error_positions
            feedback_msg.error.velocities = error_velocities
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0 / feedback_rate)
        
        self.get_logger().info(f'MoveIt 궤적 실행 완료: {time.time() - start_time:.2f}s')
        goal_handle.succeed()
        
        return FollowJointTrajectory.Result()
    
    def command_callback(self, msg):
        """그리퍼 명령 콜백 (기존 호환성)"""
        if msg.r_act and not self.activated:
            self.activated = True
            self.get_logger().info("그리퍼 활성화")
        elif not msg.r_act and self.activated:
            self.activated = False
            self.get_logger().info("그리퍼 비활성화")
        
        if self.activated and msg.r_gto:
            target_position_m = (msg.r_pr / 255.0) * 0.025
            self.target_position = target_position_m
            self.start_movement_simulation()
            self.get_logger().info(f"명령 수신: {msg.r_pr} → {target_position_m:.4f}m")
    
    def handle_control_service(self, request, response):
        """그리퍼 제어 서비스 핸들러 (기존 호환성)"""
        self.get_logger().info(f"서비스 요청: command_type={request.command_type}, value={request.value}")
        
        try:
            if request.command_type == 0:  # POSITION (0-255)
                target_position_m = (request.value / 255.0) * 0.025
                self.target_position = target_position_m
                self.start_movement_simulation()
                response.success = True
                response.message = "위치 이동 명령 성공"
                
            elif request.command_type == 3:  # OPEN
                self.target_position = 0.0
                self.start_movement_simulation()
                response.success = True
                response.message = "그리퍼 열기 성공"
                
            elif request.command_type == 4:  # CLOSE
                self.target_position = 0.025
                self.start_movement_simulation()
                response.success = True
                response.message = "그리퍼 닫기 성공"
                
            else:
                response.success = False
                response.message = "알 수 없는 명령 유형"
                
        except Exception as e:
            response.success = False
            response.message = f"명령 처리 오류: {str(e)}"
            self.get_logger().error(f"서비스 처리 오류: {e}")
        
        return response
    
    def start_movement_simulation(self):
        """가상 모드 이동 시뮬레이션 시작 (mimic 동작)"""
        # 현재 위치 저장
        self._start_pos = self.position
        
        self.moving = True
        self.movement_active = True
        self.movement_start_time = time.time()
        
        self.get_logger().info(f"이동 시작: {self.position:.4f}m → {self.target_position:.4f}m (mimic: 양쪽 동일)")
    
    def update_state(self):
        """상태 업데이트 및 발행 (mimic 동작)"""
        current_time = time.time()
        
        # 가상 모드 시뮬레이션
        if self.movement_active:
            elapsed = current_time - self.movement_start_time
            
            if elapsed < self.movement_duration:
                # 선형 보간으로 이동
                progress = elapsed / self.movement_duration
                start_pos = getattr(self, '_start_pos', self.position)
                if not hasattr(self, '_start_pos'):
                    self._start_pos = self.position
                
                self.position = self._start_pos + (self.target_position - self._start_pos) * progress
                self.velocity = (self.target_position - self._start_pos) / self.movement_duration
                self.moving = True
                
                # 물체 감지 시뮬레이션 (닫을 때)
                if self.target_position > 0.02 and self.position > 0.015:
                    if not self.object_detected and current_time % 10 < 2:
                        self.object_detected = True
                        self.get_logger().info("가상 물체 감지")
            else:
                # 이동 완료
                self.position = self.target_position
                self.velocity = 0.0
                self.moving = False
                self.movement_active = False
                
                if hasattr(self, '_start_pos'):
                    delattr(self, '_start_pos')
                
                if self.position < 0.002:
                    self.object_detected = False
                
                self.get_logger().info(f"이동 완료: {self.position:.4f}m (mimic: 양쪽 동일)")
        
        # 상태 발행
        self.publish_gripper_state()
    
    def publish_gripper_state(self):
        """그리퍼 상태 발행 (mimic 동작: 양쪽 동일한 값)"""
        current_time = self.get_clock().now()
        
        # Joint State 발행 - 양쪽 핑거에 동일한 값 (mimic 동작)
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['hande_left_finger_joint', 'hande_right_finger_joint']
        joint_state.position = [self.position, self.position]  # 양쪽 동일
        joint_state.velocity = [self.velocity, self.velocity]  # 양쪽 동일
        joint_state.effort = []
        
        self.joint_state_pub.publish(joint_state)
        
        # Controller State 발행 (MoveIt 호환) - 양쪽 동일한 값
        controller_state = JointTrajectoryControllerState()
        controller_state.header.stamp = current_time.to_msg()
        controller_state.joint_names = ['hande_left_finger_joint', 'hande_right_finger_joint']
        controller_state.actual.positions = [self.position, self.position]  # 양쪽 동일
        controller_state.actual.velocities = [self.velocity, self.velocity]  # 양쪽 동일
        controller_state.desired.positions = [self.target_position, self.target_position]  # 양쪽 동일
        controller_state.desired.velocities = [0.0, 0.0]
        controller_state.error.positions = [self.target_position - self.position, self.target_position - self.position]
        controller_state.error.velocities = [0.0 - self.velocity, 0.0 - self.velocity]
        
        self.controller_state_pub.publish(controller_state)
        
        # 호환성 상태 메시지
        status_msg = Robotiq2FGripperStatus()
        status_msg.g_act = self.activated
        status_msg.g_gto = self.moving
        status_msg.g_sta = self.activated
        status_msg.g_obj = 1 if self.object_detected else 0
        status_msg.g_flt = 0
        status_msg.g_pr = int((self.target_position / 0.025) * 255)
        status_msg.g_po = int((self.position / 0.025) * 255)
        status_msg.g_cu = int(0.5 * 255) if self.moving else int(0.1 * 255)
        
        self.status_pub.publish(status_msg)
        
        # 편의용 토픽들
        self.width_pub.publish(Float64(data=0.05 - self.position * 2))  # 총 그리퍼 폭
        self.object_pub.publish(Bool(data=self.object_detected))
        self.moving_pub.publish(Bool(data=self.moving))
        self.position_pub.publish(UInt8(data=int((self.position / 0.025) * 255)))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotiqHandeDriver()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"드라이버 실행 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()