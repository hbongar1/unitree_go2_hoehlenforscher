import rclpy
from rclpy.node import Node

#Base class for all ROS 2 nodes in this project
class BaseNode(Node):
    
    def __init__(self, name: str, description: str = ""):
        super().__init__(name)
        self.description = description
        self.get_logger().info(f"Initialized node: {name}")
        if description:
            self.get_logger().info(f"Description: {description}")
    
    def process(self, data):
        raise NotImplementedError("Subclasses must implement the process method")
