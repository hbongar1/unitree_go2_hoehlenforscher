import rclpy
from rclpy.action import ActionClient
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.msg import Entrance
from go2_msgs.action import NavigateToEntrance


class EntranceDecisionNode(BaseNode):
    """Node to make decisions based on detected entrances"""
    
    def __init__(self):
        super().__init__(
            name="entrance_decision_node",
            description="Node to decide which action to take based on entrance data"
        )

        # Subscriber für Entrance Messages
        self.subscription = self.create_subscription(
            Entrance,
            'detected_entrances',
            self.entrance_callback,
            10
        )
        
        # Action Client für Navigation
        self._action_client = ActionClient(
            self,
            NavigateToEntrance,
            'navigate_to_entrance'
        )
        
        # Parameter für Durchgangsentscheidungen
        self.min_width = 0.8  # Minimale Breite in Metern
        self.min_height = 1.5  # Minimale Höhe in Metern
    
    def entrance_callback(self, msg: Entrance):
        """Callback wird aufgerufen, wenn Entrance-Daten ankommen"""
        self.get_logger().info(
            f"Received entrance: width={msg.width}m, height={msg.height}m, "
            f"position=({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})"
        )
        
        # Entscheidungslogik
        decision = self.make_decision(msg)
        self.execute_action(decision, msg)
    
    def make_decision(self, entrance: Entrance) -> str:
        """Trifft Entscheidung basierend auf Entrance-Daten"""
        
        # Prüfe ob Durchgang passierbar ist
        if entrance.width >= self.min_width and entrance.height >= self.min_height:
            return "PASS_THROUGH"
        elif entrance.width < self.min_width:
            return "TOO_NARROW"
        elif entrance.height < self.min_height:
            return "TOO_LOW"
        else:
            return "BLOCKED"
    
    def execute_action(self, decision: str, entrance: Entrance):
        """Führt Aktion basierend auf Entscheidung aus"""
        
        if decision == "PASS_THROUGH":
            self.get_logger().info(
                f"✓ DECISION: Entrance is passable! "
                f"Proceeding through entrance at position "
                f"({entrance.position.x:.2f}, {entrance.position.y:.2f})"
            )
            # Sende Action Goal
            self.send_navigation_goal(entrance)
            
        elif decision == "TOO_NARROW":
            self.get_logger().warn(
                f"✗ DECISION: Entrance too narrow ({entrance.width}m < {self.min_width}m). "
                f"Looking for alternative route."
            )
            # Hier könnte Alternative gesucht werden
            
        elif decision == "TOO_LOW":
            self.get_logger().warn(
                f"✗ DECISION: Entrance too low ({entrance.height}m < {self.min_height}m). "
                f"Looking for alternative route."
            )
            # Hier könnte Alternative gesucht werden
            
        else:
            self.get_logger().warn(
                f"✗ DECISION: Entrance blocked. Waiting or searching for alternative."
            )
    
    def send_navigation_goal(self, entrance: Entrance):
        """Sendet Navigation Goal an Action Server"""
        
        # Warte auf Action Server
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        
        # Erstelle Goal
        goal_msg = NavigateToEntrance.Goal()
        goal_msg.target_position = entrance.position
        goal_msg.entrance_width = entrance.width
        goal_msg.entrance_height = entrance.height
        
        self.get_logger().info(f"Sending navigation goal to ({entrance.position.x:.2f}, {entrance.position.y:.2f})")
        
        # Sende Goal asynchron mit Callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback wenn Server Goal akzeptiert/ablehnt"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by action server')
            return
        
        self.get_logger().info('Goal accepted by action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback für Feedback während der Ausführung"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Navigation feedback: {feedback.status} - "
            f"Progress: {feedback.progress_percentage:.1f}%, "
            f"Distance: {feedback.distance_remaining:.2f}m"
        )
    
    def get_result_callback(self, future):
        """Callback wenn Action abgeschlossen ist"""
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Navigation successful! {result.message}')
        else:
            self.get_logger().error(f'Navigation failed: {result.message}')
    
    def process(self, data):
        """Process method from BaseNode (optional)"""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = EntranceDecisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
