#!/usr/bin/env python3
"""
ROS2 Ausfuehrung Node - Movement Control Executor (Unitree GO2)

Verantwortlichkeit:
- Bewegungskommandos via /cmd_vel senden (Twist messages)
- Roboter-Status von unitree_ros2 überwachen
- Höhenbefehle ausführen
- Motor-Status zurückmelden

Hinweis: Diese Node nutzt das offizielle unitree_ros2 SDK.
Bewegungskommandos werden über Standard ROS2 Topics gesendet.
"""

import rclpy
from rclpy.node import Node
import time

# ROS2 Standard Messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

# TODO: Custom messages (uncomment after build)
# from entrance_detection_msgs.msg import MotorStatus, MovementCommand


class AusfuehrungNode(Node):
    """Movement Control Executor Node for Unitree GO2"""
    
    def __init__(self):
        super().__init__('ausfuehrung_node')
        
        # Publisher für Bewegungskommandos (unitree_ros2 Standard)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TODO: Publisher for motor status
        # self.status_pub = self.create_publisher(MotorStatus, '/status/motor', 10)
        
        # TODO: Service for movement commands
        # self.command_srv = self.create_service(...)
        
        # TODO: Subscriber für Roboter-Status (von unitree_ros2)
        # self.robot_state_sub = self.create_subscription(
        #     RobotState, '/robot_state', self.robot_state_callback, 10
        # )
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.3)   # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        self.declare_parameter('simulation_mode', False)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # State
        self.is_ready = True
        self.in_motion = False
        self.current_velocity = Twist()
        
        # Status publishing timer (10 Hz)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        mode_str = "SIMULATION" if self.simulation_mode else "REAL ROBOT"
        self.get_logger().info(f'Ausfuehrung Node initialized ({mode_str})')
        self.get_logger().info(f'Publishing to: /cmd_vel')
    
    def execute_movement_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=1.0):
        """
        Execute movement command
        
        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s)
            angular_z: Rotation velocity (rad/s)
            duration: Duration to move (seconds)
        """
        if not self.is_ready:
            self.get_logger().warn('Robot not ready, rejecting command')
            return False
        
        # Limit velocities
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        linear_y = max(-self.max_linear_speed, min(self.max_linear_speed, linear_y))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        
        self.get_logger().info(
            f'Moving: linear=({linear_x:.2f}, {linear_y:.2f}), '
            f'angular={angular_z:.2f}, duration={duration:.1f}s'
        )
        
        self.in_motion = True
        self.is_ready = False
        
        try:
            # Create Twist message
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.linear.y = linear_y
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = angular_z
            
            # Publish velocity command for specified duration
            rate = self.create_rate(50)  # 50 Hz
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                self.cmd_vel_pub.publish(cmd)
                self.current_velocity = cmd
                rate.sleep()
            
            # Stop movement
            self.stop_movement()
            
            self.get_logger().info('Movement complete')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Movement control failed: {e}')
            self.stop_movement()
            return False
        
        finally:
            self.in_motion = False
            self.is_ready = True
    
    def move_forward(self, distance, speed=0.2):
        """
        Move forward by specified distance
        
        Args:
            distance: Distance to move (meters)
            speed: Forward speed (m/s)
        """
        duration = abs(distance / speed)
        direction = 1.0 if distance > 0 else -1.0
        return self.execute_movement_command(linear_x=speed * direction, duration=duration)
    
    def turn(self, angle_deg, speed=0.3):
        """
        Turn by specified angle
        
        Args:
            angle_deg: Angle to turn (degrees, positive = left)
            speed: Angular speed (rad/s)
        """
        import math
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad / speed)
        direction = 1.0 if angle_deg > 0 else -1.0
        return self.execute_movement_command(angular_z=speed * direction, duration=duration)
    
    def crouch(self):
        """Lower body height for cave entrance passage"""
        self.get_logger().info('Crouching...')
        # Note: Body height control via unitree_ros2 may use different topic
        # For GO2, this might be via sport_client or specific service
        # Placeholder for now
        if self.simulation_mode:
            self.get_logger().info('[SIMULATION] Body lowered')
        else:
            self.get_logger().warn('Body height control not yet implemented')
            # TODO: Implement via unitree_ros2 service or topic
        return True
    
    def stand_normal(self):
        """Return to normal standing height"""
        self.get_logger().info('Standing up...')
        if self.simulation_mode:
            self.get_logger().info('[SIMULATION] Body raised to normal')
        else:
            self.get_logger().warn('Body height control not yet implemented')
            # TODO: Implement via unitree_ros2 service or topic
        return True
    
    def stop_movement(self):
        """Stop all movement immediately"""
        cmd = Twist()  # All zeros
        for _ in range(10):  # Send multiple stop commands
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.01)
        self.current_velocity = cmd
        self.get_logger().info('Movement stopped')
    
    def emergency_stop(self):
        """Emergency stop - freeze all motion"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        self.is_ready = False
        self.stop_movement()
    
    def publish_status(self):
        """Publish motor status"""
        # TODO: Publish MotorStatus message
        # status = MotorStatus()
        # status.header.stamp = self.get_clock().now().to_msg()
        # status.in_motion = self.in_motion
        # status.ready = self.is_ready
        # status.current_velocity = self.current_velocity
        # self.status_pub.publish(status)
        
        # For now, just log occasionally
        if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:
            self.get_logger().info(
                f'Status: ready={self.is_ready}, moving={self.in_motion}',
                throttle_duration_sec=10.0
            )
    
    def robot_state_callback(self, msg):
        """Callback for robot state from unitree_ros2"""
        # TODO: Process robot state message
        # Update internal state based on actual robot status
        pass
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down movement controller')
        self.stop_movement()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AusfuehrungNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
