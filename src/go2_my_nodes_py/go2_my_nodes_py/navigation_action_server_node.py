import rclpy
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.action import NavigateToEntrance


class NavigationActionServerNode(BaseNode):
    """Action Server for navigation to entrance"""
    
    def __init__(self):
        super().__init__(
            name="navigation_action_server",
            description="Action server for navigating to detected entrances"
        )
        
        # Action Server erstellen
        self._action_server = ActionServer(
            self,
            NavigateToEntrance,
            'navigate_to_entrance',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("Navigation Action Server ready!")
    
    def goal_callback(self, goal_request):
        """Callback wenn neues Goal empfangen wird"""
        self.get_logger().info(
            f"Received navigation goal to position "
            f"({goal_request.target_position.x:.2f}, {goal_request.target_position.y:.2f}, {goal_request.target_position.z:.2f})"
        )
        # Akzeptiere immer das Goal (könnte hier Validierung machen)
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Callback wenn Goal abgebrochen werden soll"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Führt die Navigation aus (simuliert)"""
        self.get_logger().info("Executing navigation goal...")
        
        # Goal-Daten extrahieren
        target = goal_handle.request.target_position
        width = goal_handle.request.entrance_width
        height = goal_handle.request.entrance_height
        
        # Simuliere Navigation in Schritten
        total_distance = self._calculate_distance(target)
        steps = 10
        
        for i in range(steps):
            # Prüfe ob Goal abgebrochen wurde
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToEntrance.Result()
                result.success = False
                result.message = "Navigation canceled by user"
                self.get_logger().warn("Navigation canceled!")
                return result
            
            # Berechne Fortschritt
            progress = (i + 1) / steps * 100.0
            remaining = total_distance * (1.0 - (i + 1) / steps)
            
            # Sende Feedback
            feedback = NavigateToEntrance.Feedback()
            feedback.status = f"Navigating to entrance (step {i+1}/{steps})"
            feedback.progress_percentage = progress
            feedback.distance_remaining = remaining
            
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f"Progress: {progress:.1f}% - Distance remaining: {remaining:.2f}m"
            )
            
            # Simuliere Navigation (warte 0.5 Sekunden)
            time.sleep(0.5)
        
        # Navigation erfolgreich abgeschlossen
        goal_handle.succeed()
        
        result = NavigateToEntrance.Result()
        result.success = True
        result.message = f"Successfully navigated through entrance ({width}m x {height}m)"
        
        self.get_logger().info(f"Navigation completed successfully!")
        return result
    
    def _calculate_distance(self, target):
        """Berechnet Distanz zum Ziel (vereinfacht)"""
        # Annahme: Roboter ist am Ursprung (0, 0, 0)
        distance = (target.x**2 + target.y**2 + target.z**2)**0.5
        return max(distance, 1.0)  # Mindestens 1 Meter
    
    def process(self, data):
        """Process method from BaseNode (nicht verwendet)"""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
