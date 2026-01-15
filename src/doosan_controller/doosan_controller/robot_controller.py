#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import time
from PyQt5.QtWidgets import QApplication
from doosan_controller.gui_window import RobotControlGUI
from doosan_controller.motion_executor import MotionExecutor


class RobotControllerNode(Node):
    """
    Main ROS2 Node for Doosan E0509 Robot Arm Control
    실제 로봇/시뮬레이터와 ROS2 토픽으로 통신
    """
    
    def __init__(self):
        super().__init__('doosan_controller_node')
        
        # Robot state variables
        self.robot_connected = False
        self.is_moving = False
        self.current_joint_states = [0.0] * 6
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.log_messages = []
        self.last_joint_state_time = None
        
        # Publishers - Doosan 컨트롤러로 명령 전송
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/dsr_moveit_controller/joint_trajectory',
            10
        )
        
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # Subscribers - 로봇 상태 수신
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Motion executor
        self.motion_executor = MotionExecutor(self)
        
        # Timer for connection check
        self.create_timer(1.0, self.check_robot_connection)
        
        self.add_log("Node initialized - Waiting for robot connection...")
        self.add_log("Please start Doosan simulator or connect to real robot")
        self.get_logger().info("Doosan Controller Node started")
    
    def joint_state_callback(self, msg):
        """
        /joint_states 토픽에서 관절 상태 수신
        실제 로봇의 현재 관절 각도를 받아옴
        """
        if len(msg.position) >= 6:
            self.current_joint_states = list(msg.position[:6])
            self.last_joint_state_time = self.get_clock().now()
            
            # 첫 연결 시 로그
            if not self.robot_connected:
                self.robot_connected = True
                self.add_log("✓ Robot connected successfully!")
                self.add_log("Receiving joint states from robot")
            
            # Forward kinematics 계산하여 위치 업데이트
            # (실제로는 복잡한 계산이지만 여기서는 근사)
            self._update_pose_from_joints()
    
    def _update_pose_from_joints(self):
        """
        관절 각도로부터 말단 위치 계산 (간단한 근사)
        실제로는 Doosan의 FK 서비스를 사용해야 함
        """
        # 간단한 근사 계산
        import math
        j1, j2, j3 = self.current_joint_states[0:3]
        
        # 대략적인 위치 계산 (실제 DH parameter 사용 권장)
        L1, L2, L3 = 400.0, 300.0, 200.0  # mm (예시 링크 길이)
        
        self.current_pose['x'] = (L2 * math.cos(j2) + L3 * math.cos(j2 + j3)) * math.cos(j1)
        self.current_pose['y'] = (L2 * math.cos(j2) + L3 * math.cos(j2 + j3)) * math.sin(j1)
        self.current_pose['z'] = L1 + L2 * math.sin(j2) + L3 * math.sin(j2 + j3)
    
    def check_robot_connection(self):
        """
        주기적으로 로봇 연결 상태 확인
        일정 시간 동안 joint_states를 받지 못하면 연결 해제로 판단
        """
        if self.last_joint_state_time is not None:
            time_since_last = (self.get_clock().now() - self.last_joint_state_time).nanoseconds / 1e9
            
            if time_since_last > 3.0:  # 3초 이상 메시지 없음
                if self.robot_connected:
                    self.robot_connected = False
                    self.add_log("⚠ Robot connection lost")
        else:
            if self.robot_connected:
                self.robot_connected = False
    
    def execute_motion(self, targets, coordinate_mode, velocity, acceleration):
        """
        목표 위치로 이동 명령 실행
        
        Args:
            targets: List of target poses [(x,y,z), ...]
            coordinate_mode: 'absolute' or 'relative'
            velocity: Maximum velocity (m/s)
            acceleration: Maximum acceleration (m/s^2)
        """
        if not self.robot_connected:
            self.add_log("❌ ERROR: Robot not connected")
            self.add_log("Please start the Doosan simulator or connect to robot")
            return False
        
        if self.is_moving:
            self.add_log("⚠ WARNING: Robot already moving")
            return False
        
        self.add_log("")
        self.add_log(f"Starting motion: {len(targets)} target(s)")
        self.add_log(f"Mode: {coordinate_mode}, Vel: {velocity} m/s, Acc: {acceleration} m/s²")
        
        # Start motion in separate thread
        self.motion_executor.execute(targets, coordinate_mode, velocity, acceleration)
        
        return True
    
    def stop_motion(self):
        """Emergency stop"""
        self.add_log("")
        self.add_log("🛑 EMERGENCY STOP triggered")
        self.motion_executor.stop()
        self.is_moving = False
        
        # Send stop command to robot
        # Doosan 로봇의 경우 별도의 stop 토픽이 있을 수 있음
        self.get_logger().warn("Emergency stop requested")
    
    def send_cartesian_move(self, x, y, z):
        """
        카르테시안 좌표로 이동 명령 전송
        JointTrajectory를 통해 Doosan 컨트롤러로 전송
        
        Args:
            x, y, z: Target position in mm
        """
        import math
        
        # mm를 m로 변환
        x_m = x / 1000.0
        y_m = y / 1000.0
        z_m = z / 1000.0
        
        # E0509 로봇의 대략적인 링크 길이
        L1 = 0.409  # m (베이스 높이)
        L2 = 0.367  # m (상완)
        L3 = 0.124  # m (전완)
        
        # 2D 평면 계산
        r = math.sqrt(x_m**2 + y_m**2)
        z_adj = z_m - L1
        
        # Joint 1: 베이스 회전 (z축 기준)
        j1 = math.atan2(y_m, x_m)
        
        # 도달 거리
        reach = math.sqrt(r**2 + z_adj**2)
        
        # 도달 가능 범위 체크
        max_reach = L2 + L3
        if reach > max_reach:
            self.add_log(f"⚠ Target too far: {reach:.3f}m (max: {max_reach:.3f}m)")
            # 최대 범위로 제한
            scale = max_reach / reach * 0.95
            r *= scale
            z_adj *= scale
            reach = max_reach * 0.95
        
        # Joint 3: 엘보 각도 (코사인 법칙)
        cos_j3 = (reach**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_j3 = max(-1.0, min(1.0, cos_j3))  # 범위 제한
        j3 = math.acos(cos_j3)
        
        # Joint 2: 어깨 각도
        alpha = math.atan2(z_adj, r)
        beta = math.atan2(L3 * math.sin(j3), L2 + L3 * math.cos(j3))
        j2 = alpha - beta
        
        # Joint 4, 5, 6: 손목 (간단히 0으로)
        joint_positions = [j1, j2, j3, 0.0, 0.0, 0.0]
        
        # JointTrajectory 메시지 생성 (수동 명령과 동일한 형식)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        # 궤적 포인트 생성
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 명시적으로 6개
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        
        msg.points = [point]
        
        # 발행
        self.trajectory_pub.publish(msg)
        
        # 로그
        joint_deg = [math.degrees(j) for j in joint_positions]
        self.get_logger().info(
            f"Published trajectory to ({x}, {y}, {z}): "
            f"joints=[{', '.join([f'{j:.1f}°' for j in joint_deg])}]"
        )
    
    def add_log(self, message):
        """Add message to log buffer"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.log_messages.append(log_entry)
        
        # Keep only last 100 messages
        if len(self.log_messages) > 100:
            self.log_messages.pop(0)
        
        self.get_logger().info(message)
    
    def get_robot_state(self):
        """Return current robot state for GUI"""
        return {
            'connected': self.robot_connected,
            'moving': self.is_moving,
            'joint_angles': self.current_joint_states,
            'position': self.current_pose,
            'logs': self.log_messages.copy()
        }


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Create ROS2 node
    controller_node = RobotControllerNode()
    
    # Create Qt Application
    app = QApplication(sys.argv)
    
    # Create GUI window
    gui = RobotControlGUI(controller_node)
    gui.show()
    
    # ROS2 executor in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Run Qt event loop
    exit_code = app.exec_()
    
    # Cleanup
    controller_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=1.0)
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()