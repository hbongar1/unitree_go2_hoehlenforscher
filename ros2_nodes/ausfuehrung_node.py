#!/usr/bin/env python3
"""
ROS2 Ausfuehrung Node - Motor Control Executor (Unitree GO2)

Verantwortlichkeit:
- Unitree SportClient initialisieren
- Motor-Befehle via Service empfangen
- Körperhöhe ändern via SetBodyHeight()
- Motor-Status zurückmelden
"""

import rclpy
from rclpy.node import Node
import time

# Unitree SDK
try:
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    UNITREE_AVAILABLE = True
except ImportError:
    UNITREE_AVAILABLE = False
    print("WARNING: unitree_sdk2py not available - running in simulation mode")

# TODO: Custom messages (uncomment after build)
# from entrance_detection_msgs.msg import MotorStatus


class AusfuehrungNode(Node):
    """Motor Control Executor Node for Unitree GO2"""
    
    def __init__(self):
        super().__init__('ausfuehrung_node')
        
        # TODO: Publisher for motor status
        # self.status_pub = self.create_publisher(MotorStatus, '/status/motor', 10)
        
        # TODO: Service for motor commands
        # self.command_srv = self.create_service(...)
        
        # Parameters
        self.declare_parameter('network_interface', 'enp129s0')
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('max_height_change_rate', 0.05)  # m/s
        self.declare_parameter('simulation_mode', not UNITREE_AVAILABLE)
        
        self.network_interface = self.get_parameter('network_interface').value
        self.timeout = self.get_parameter('timeout').value
        self.max_rate = self.get_parameter('max_height_change_rate').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # Unitree SDK
        self.sport_client = None
        self.current_height = 0.30  # Default normal height
        self.is_ready = True
        self.in_motion = False
        
        # Initialize Unitree SDK
        if not self.simulation_mode:
            self.initialize_unitree_sdk()
        else:
            self.get_logger().warn('Running in SIMULATION MODE (no Unitree SDK)')
        
        # Status publishing timer (10 Hz)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('Ausfuehrung Node initialized')
    
    def initialize_unitree_sdk(self):
        """Initialize Unitree GO2 SDK connection"""
        try:
            # Initialize channel factory
            ChannelFactoryInitialize(0, self.network_interface)
            
            # Create sport client
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(self.timeout)
            self.sport_client.Init()
            
            self.get_logger().info(
                f'Unitree SDK initialized on interface: {self.network_interface}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Unitree SDK: {e}')
            self.get_logger().warn('Falling back to simulation mode')
            self.simulation_mode = True
    
    def execute_height_command(self, target_height, posture_mode):
        """Execute motor command to change body height"""
        if not self.is_ready:
            self.get_logger().warn('Robot not ready, rejecting command')
            return False
        
        self.get_logger().info(
            f'Executing: height={target_height:.2f}m, mode={posture_mode}'
        )
        
        # Mark as in motion
        self.in_motion = True
        self.is_ready = False
        
        try:
            if self.simulation_mode:
                # Simulate height change
                self.simulate_height_change(target_height)
            else:
                # Real robot control
                self.control_robot_height(target_height, posture_mode)
            
            # Update current height
            self.current_height = target_height
            
            self.get_logger().info(f'Height change complete: {target_height:.2f}m')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Motor control failed: {e}')
            return False
        
        finally:
            self.in_motion = False
            self.is_ready = True
    
    def control_robot_height(self, target_height, posture_mode):
        """Control actual Unitree GO2 robot"""
        if not self.sport_client:
            raise RuntimeError("Sport client not initialized")
        
        # Calculate height change time based on rate limit
        height_delta = abs(target_height - self.current_height)
        duration = height_delta / self.max_rate
        
        self.get_logger().info(f'Height change will take ~{duration:.1f}s')
        
        # Send height command to robot
        # Note: Unitree SDK method may vary depending on version
        # This is a placeholder - adjust based on actual SDK API
        try:
            # Example command (adjust based on SDK documentation):
            # self.sport_client.BodyHeight(target_height)
            # OR
            # self.sport_client.SetBodyHeight(target_height)
            
            # For now, simulate wait time
            time.sleep(duration)
            
            self.get_logger().info('Robot height adjusted')
            
        except Exception as e:
            self.get_logger().error(f'SDK command failed: {e}')
            raise
    
    def simulate_height_change(self, target_height):
        """Simulate height change for testing without robot"""
        height_delta = abs(target_height - self.current_height)
        duration = height_delta / self.max_rate
        
        self.get_logger().info(f'[SIMULATION] Changing height over {duration:.1f}s')
        time.sleep(min(duration, 2.0))  # Cap simulation time
        self.get_logger().info(f'[SIMULATION] Height now: {target_height:.2f}m')
    
    def publish_status(self):
        """Publish motor status"""
        # TODO: Publish MotorStatus message
        # status = MotorStatus()
        # status.header.stamp = self.get_clock().now().to_msg()
        # status.current_body_height = self.current_height
        # status.in_motion = self.in_motion
        # status.ready = self.is_ready
        # status.error_message = ""
        # self.status_pub.publish(status)
        
        # For now, just log occasionally
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().info(
                f'Status: h={self.current_height:.2f}m, '
                f'ready={self.is_ready}, moving={self.in_motion}',
                throttle_duration_sec=5.0
            )
    
    def emergency_stop(self):
        """Emergency stop - freeze all motion"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        self.is_ready = False
        
        if not self.simulation_mode and self.sport_client:
            try:
                # Send emergency stop to robot
                # self.sport_client.StandDown()  # or similar method
                pass
            except Exception as e:
                self.get_logger().error(f'Emergency stop failed: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.sport_client and not self.simulation_mode:
            self.get_logger().info('Shutting down Unitree connection')
            # Clean up SDK resources if needed
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
