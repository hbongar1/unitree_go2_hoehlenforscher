import rclpy
from go2_my_nodes_py.base_node import BaseNode
from sensor_msgs.msg import PointCloud2
from go2_msgs.msg import Entrance


class CloudToEntranceNode(BaseNode):
    """Node to handle communication from Cloud to Entrance"""
    
    def __init__(self):
        super().__init__(
            name="cloud_to_entrance_node",
            description="Node to handle communication from Cloud to Entrance"
        )

        # Subscriber hinzufügen
        self.subscription = self.create_subscription(
            PointCloud2,                     # Message-Typ für Punktwolke
            '/utlidar/cloud_deskewed',       # Topic-Name
            self.cloud_data_callback,        # Callback-Funktion
            10                               # QoS-Profil (Queue-Größe)
        )

        # Publisher hinzufügen
        self.publisher = self.create_publisher(
            Entrance, 'detected_entrances', 10)
    
    def cloud_data_callback(self, msg: PointCloud2):
        """Callback wird aufgerufen, wenn Punktwolke-Daten ankommen"""
        self.get_logger().info(f"Received cloud with {msg.width * msg.height} points")
        
        # Erstelle default Entrance message als Platzhalter
        entrance_msg = Entrance()
        entrance_msg.header = msg.header
        entrance_msg.position.x = 0.0
        entrance_msg.position.y = 0.0
        entrance_msg.position.z = 0.0
        entrance_msg.width = 1.0
        entrance_msg.height = 2.0
        
        # Publishe die Entrance message
        self.publisher.publish(entrance_msg)
        self.get_logger().info(f"Published entrance: width={entrance_msg.width}m, height={entrance_msg.height}m")
    
    def process(self, data):
        """Process data from Cloud to Entrance"""
        # Hier kommt die Logik zur Verarbeitung der Daten vom Cloud zum Eingang
        processed_data = data  # Platzhalter für tatsächliche Verarbeitung
        return processed_data


def main(args=None):
    rclpy.init(args=args)
    node = CloudToEntranceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

