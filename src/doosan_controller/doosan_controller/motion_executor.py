#!/usr/bin/env python3

import threading
import time
import math


class MotionExecutor:
    """
    Executes robot motion in a separate thread
    실제 로봇으로 명령을 전송하고 완료를 대기
    """
    
    def __init__(self, controller_node):
        self.controller = controller_node
        self.execution_thread = None
        self.stop_flag = False
    
    def execute(self, targets, coordinate_mode, velocity, acceleration):
        """
        Start motion execution in separate thread
        
        Args:
            targets: List of (x, y, z) tuples in mm
            coordinate_mode: 'absolute' or 'relative'
            velocity: Maximum velocity in m/s
            acceleration: Maximum acceleration in m/s^2
        """
        if self.execution_thread and self.execution_thread.is_alive():
            self.controller.add_log("ERROR: Motion already in progress")
            return
        
        self.stop_flag = False
        self.execution_thread = threading.Thread(
            target=self._execute_motion,
            args=(targets, coordinate_mode, velocity, acceleration),
            daemon=True
        )
        self.execution_thread.start()
    
    def stop(self):
        """Stop current motion execution"""
        self.stop_flag = True
        if self.execution_thread:
            self.execution_thread.join(timeout=2.0)
    
    def _execute_motion(self, targets, coordinate_mode, velocity, acceleration):
        """
        Internal method to execute motion sequence
        Runs in separate thread
        """
        self.controller.is_moving = True
        self.controller.add_log(f"Motion execution started: {len(targets)} points")
        
        try:
            current_pos = self.controller.current_pose.copy()
            
            for idx, target in enumerate(targets):
                if self.stop_flag:
                    self.controller.add_log("Motion stopped by user")
                    break
                
                # Convert mm to m and calculate target
                x, y, z = target[0], target[1], target[2]
                
                # Calculate target based on coordinate mode
                if coordinate_mode == 'relative':
                    target_x = current_pos['x'] + x
                    target_y = current_pos['y'] + y
                    target_z = current_pos['z'] + z
                else:  # absolute
                    target_x, target_y, target_z = x, y, z
                
                self.controller.add_log(
                    f"Moving to point {idx + 1}/{len(targets)}: "
                    f"({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) mm"
                )
                
                # Send command to actual robot
                self.controller.send_cartesian_move(target_x, target_y, target_z)
                
                # Wait for motion to complete
                # 실제로는 로봇의 상태를 모니터링해야 하지만,
                # 여기서는 시간 기반으로 대기
                estimated_time = self._estimate_motion_time(
                    current_pos,
                    {'x': target_x, 'y': target_y, 'z': target_z},
                    velocity
                )
                
                self._wait_for_motion(estimated_time)
                
                if self.stop_flag:
                    break
                
                # Update current position
                current_pos = {'x': target_x, 'y': target_y, 'z': target_z}
                
                self.controller.add_log(f"✓ Reached point {idx + 1}")
            
            if not self.stop_flag:
                self.controller.add_log("")
                self.controller.add_log("✅ Motion sequence completed successfully")
            
        except Exception as e:
            self.controller.add_log(f"❌ ERROR during motion: {str(e)}")
            self.controller.get_logger().error(f"Motion execution error: {str(e)}")
        
        finally:
            self.controller.is_moving = False
    
    def _estimate_motion_time(self, start_pos, end_pos, velocity):
        """
        Estimate time required for motion
        
        Args:
            start_pos: Starting position dict with x, y, z
            end_pos: Ending position dict with x, y, z
            velocity: Maximum velocity in m/s
        
        Returns:
            Estimated time in seconds
        """
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']
        dz = end_pos['z'] - start_pos['z']
        
        # Distance in mm
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Convert to meters and calculate time
        distance_m = distance / 1000.0
        estimated_time = (distance_m / velocity) * 1.2 + 1.0  # 20% margin + 1s
        
        return max(estimated_time, 2.0)  # Minimum 2 seconds
    
    def _wait_for_motion(self, duration):
        """
        Wait for motion to complete with stop flag checking
        
        실제 구현에서는 로봇의 moving 상태를 체크해야 함
        여기서는 시간 기반으로 대기
        
        Args:
            duration: Time to wait in seconds
        """
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            if self.stop_flag:
                break
            time.sleep(0.1)