#!/usr/bin/env python3
"""
Test-Node für Unitree GO2 SportMode Bewegungen
Testet verschiedene Bewegungsbefehle über die ROS2 Sport API
"""

import rclpy
from rclpy.node import Node
import time

# Unitree ROS2 API imports
try:
    from unitree_api.msg import Request
    UNITREE_API_AVAILABLE = True
except ImportError:
    UNITREE_API_AVAILABLE = False
    Request = None
    print("⚠️  unitree_api not available - please install unitree_ros2")


class SportMovementTestNode(Node):
    """Test-Node für SportMode Bewegungen"""
    
    def __init__(self):
        super().__init__('sport_movement_test')
        
        if not UNITREE_API_AVAILABLE:
            self.get_logger().error("❌ Unitree API not available!")
            return
        
        # Publisher für Sport Request
        self.sport_req_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )
        
        # Warte bis Publisher bereit ist
        time.sleep(0.5)
        
        self.get_logger().info("✅ Sport Movement Test Node initialized")
        self.get_logger().info("Publisher ready on /api/sport/request")
    
    def send_sit_command(self):
        """Sendet Sit-Befehl"""
        self.get_logger().info("📤 Sending SIT command...")
        req = Request()
        req.header.identity.id = 0
        req.parameter = '{"name": "sit"}'
        self.sport_req_pub.publish(req)
        self.get_logger().info("✅ Sit command sent")
    
    def send_stand_command(self):
        """Sendet Stand-Befehl"""
        self.get_logger().info("📤 Sending STAND command...")
        req = Request()
        req.header.identity.id = 0
        req.parameter = '{"name": "stand_up"}'
        self.sport_req_pub.publish(req)
        self.get_logger().info("✅ Stand command sent")
    
    def send_body_height_command(self, height: float):
        """Sendet BodyHeight-Befehl
        
        Args:
            height: Relative Höhe in Metern (negativ = tiefer, positiv = höher)
        """
        self.get_logger().info(f"📤 Sending BODY HEIGHT command: {height:.3f}m...")
        req = Request()
        req.header.identity.id = 0
        req.parameter = f'{{"name": "body_height", "height": {height:.3f}}}'
        self.sport_req_pub.publish(req)
        self.get_logger().info(f"✅ Body height command sent: {height:.3f}m")
    
    def send_move_command(self, vx: float, vy: float, vyaw: float):
        """Sendet Bewegungsbefehl
        
        Args:
            vx: Geschwindigkeit vorwärts/rückwärts in m/s
            vy: Geschwindigkeit seitwärts in m/s
            vyaw: Rotationsgeschwindigkeit in rad/s
        """
        self.get_logger().info(f"📤 Sending MOVE command: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}...")
        req = Request()
        req.header.identity.id = 0
        req.parameter = f'{{"name": "move", "vx": {vx:.3f}, "vy": {vy:.3f}, "vyaw": {vyaw:.3f}}}'
        self.sport_req_pub.publish(req)
        self.get_logger().info("✅ Move command sent")
    
    def run_test_sequence(self):
        """Führt eine Test-Sequenz durch"""
        if not UNITREE_API_AVAILABLE:
            self.get_logger().error("Cannot run test - API not available")
            return
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Starting Sport Movement Test Sequence")
        self.get_logger().info("=" * 60)
        
        try:
            # Test 1: Body Height anpassen
            self.get_logger().info("\n--- Test 1: Body Height Adjustment ---")
            self.get_logger().info("Lowering body by 10cm...")
            self.send_body_height_command(-0.10)
            time.sleep(3)
            
            self.get_logger().info("Raising body back to normal...")
            self.send_body_height_command(0.0)
            time.sleep(3)
            
            # Test 2: Hinsetzen
            self.get_logger().info("\n--- Test 2: Sit Down ---")
            self.send_sit_command()
            time.sleep(3)
            
            # Test 3: Aufstehen
            self.get_logger().info("\n--- Test 3: Stand Up ---")
            self.send_stand_command()
            time.sleep(3)
            
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("✅ Test Sequence Completed Successfully!")
            self.get_logger().info("=" * 60)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error during test sequence: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    if not UNITREE_API_AVAILABLE:
        print("❌ Cannot start test - unitree_api not available")
        print("Please install: sudo apt install ros-humble-unitree-api")
        return
    
    node = SportMovementTestNode()
    
    try:
        # Führe Test-Sequenz aus
        node.run_test_sequence()
        
        # Halte Node am Leben für manuelle Tests
        print("\n" + "=" * 60)
        print("Test sequence finished. Node is still running.")
        print("Press Ctrl+C to exit.")
        print("=" * 60)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
