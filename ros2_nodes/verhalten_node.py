#!/usr/bin/env python3
"""
ROS2 Verhalten Node - Decision Logic and State Machine

Verantwortlichkeit:
- EntranceState empfangen
- State Machine: NORMAL → ANALYSE → NIEDRIG/LIEGEND/BLOCKIERT
- Entscheidung treffen: Weiterfahren/Ducken/Hinlegen/Stopp
- Motor-Befehle via Service senden
"""

import rclpy
from rclpy.node import Node
from enum import Enum

# TODO: Custom messages (uncomment after build)
# from entrance_detection_msgs.msg import EntranceState, MotorStatus


class RobotState(Enum):
    """State machine states"""
    NORMAL = "NORMAL"
    ANALYSE = "ANALYSE"
    NIEDRIG = "NIEDRIG"
    LIEGEND = "LIEGEND"
    BLOCKIERT = "BLOCKIERT"


class VerhaltenNode(Node):
    """Behavior/Decision Logic Node with State Machine"""
    
    def __init__(self):
        super().__init__('verhalten_node')
        
        # TODO: Subscribers (uncomment after custom messages)
        # self.state_sub = self.create_subscription(
        #     EntranceState, '/perception/entrance_state', self.state_callback, 10
        # )
        # self.motor_status_sub = self.create_subscription(
        #     MotorStatus, '/status/motor', self.motor_status_callback, 10
        # )
        
        # TODO: Service client for motor control
        # self.motor_client = self.create_client(...)
        
        # Parameters
        self.declare_parameter('normal_height', 0.30)
        self.declare_parameter('low_height', 0.15)
        self.declare_parameter('prone_height', 0.08)
        self.declare_parameter('safety_clearance', 0.10)
        self.declare_parameter('min_passable_height', 0.10)
        
        self.normal_height = self.get_parameter('normal_height').value
        self.low_height = self.get_parameter('low_height').value
        self.prone_height = self.get_parameter('prone_height').value
        self.safety_clearance = self.get_parameter('safety_clearance').value
        self.min_passable_height = self.get_parameter('min_passable_height').value
        
        # State machine
        self.current_state = RobotState.NORMAL
        self.motor_ready = True
        self.last_command_time = self.get_clock().now()
        
        self.get_logger().info('Verhalten Node initialized')
        self.get_logger().info(f'Height config: Normal={self.normal_height}m, '
                             f'Low={self.low_height}m, Prone={self.prone_height}m')
    
    def state_callback(self, msg):
        """Process incoming entrance state and make decision"""
        self.get_logger().info(f'Received state: {msg.state}')
        
        # State machine transitions
        if msg.state == "NORMAL":
            self.transition_to_normal()
        
        elif msg.state == "NIEDRIG":
            if msg.action_required:
                self.transition_to_niedrig(msg.required_height)
        
        elif msg.state == "LIEGEND":
            if msg.action_required:
                self.transition_to_liegend(msg.required_height)
        
        elif msg.state == "BLOCKIERT":
            self.transition_to_blockiert()
    
    def motor_status_callback(self, msg):
        """Receive motor status feedback"""
        self.motor_ready = msg.ready
        
        if msg.error_message:
            self.get_logger().error(f'Motor error: {msg.error_message}')
            self.emergency_stop()
    
    def transition_to_normal(self):
        """Transition to NORMAL state"""
        if self.current_state != RobotState.NORMAL:
            self.get_logger().info('Transitioning to NORMAL state')
            self.current_state = RobotState.NORMAL
            self.send_motor_command(self.normal_height, "STAND")
    
    def transition_to_niedrig(self, required_height):
        """Transition to NIEDRIG (low crouch) state"""
        self.get_logger().info(f'Transitioning to NIEDRIG state (h={required_height:.2f}m)')
        self.current_state = RobotState.NIEDRIG
        
        # Clamp height to safe range
        target_height = max(self.prone_height, min(required_height, self.low_height))
        self.send_motor_command(target_height, "CROUCH")
    
    def transition_to_liegend(self, required_height):
        """Transition to LIEGEND (prone) state"""
        self.get_logger().info(f'Transitioning to LIEGEND state (h={required_height:.2f}m)')
        self.current_state = RobotState.LIEGEND
        
        # Use minimum height
        target_height = max(self.prone_height, required_height)
        self.send_motor_command(target_height, "PRONE")
    
    def transition_to_blockiert(self):
        """Transition to BLOCKIERT (blocked) state"""
        self.get_logger().warn('Entrance BLOCKED - stopping robot')
        self.current_state = RobotState.BLOCKIERT
        self.emergency_stop()
    
    def send_motor_command(self, target_height, posture_mode):
        """Send command to motor controller"""
        if not self.motor_ready:
            self.get_logger().warn('Motor not ready, command queued')
            return
        
        self.get_logger().info(
            f'Sending motor command: height={target_height:.2f}m, mode={posture_mode}'
        )
        
        # TODO: Implement service call to ausfuehrung_node
        # request = MotorCommand.Request()
        # request.target_height = target_height
        # request.posture_mode = posture_mode
        # future = self.motor_client.call_async(request)
        
        self.last_command_time = self.get_clock().now()
    
    def emergency_stop(self):
        """Emergency stop command"""
        self.get_logger().error('EMERGENCY STOP')
        # TODO: Call emergency stop service
        # self.motor_client.call_async(emergency_stop_request)


def main(args=None):
    rclpy.init(args=args)
    node = VerhaltenNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
