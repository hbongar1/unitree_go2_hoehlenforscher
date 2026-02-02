import rclpy
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.action import NavigateToEntrance
from geometry_msgs.msg import Twist


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
        
        # Publisher für Bewegungsbefehle (für Höhenanpassung)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Roboter-Parameter
        self.robot_standing_height = 0.45    # Normale Höhe im Stand (45cm)
        self.min_crouching_height = 0.30     # Minimale Höhe beim Beugen (30cm)
        self.current_body_height = self.robot_standing_height  # Aktuelle Höhe
        self.height_adjustment_speed = 0.2  # m/s - Anpassungsgeschwindigkeit
        
        self.get_logger().info("Navigation Action Server ready!")
    
    def goal_callback(self, goal_request):
        """Callback wenn neues Goal empfangen wird"""
        required_height = goal_request.required_height_adjustment
        self.get_logger().info(
            f"📍 Received navigation goal to position "
            f"({goal_request.target_position.x:.2f}, {goal_request.target_position.y:.2f}, {goal_request.target_position.z:.2f}) | "
            f"Entrance: {goal_request.entrance_width:.3f}m x {goal_request.entrance_height:.3f}m | "
            f"Required height adjustment: {required_height:.3f}m"
        )
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Callback wenn Goal abgebrochen werden soll"""
        self.get_logger().info("Received cancel request")
        # Stelle Roboter auf normale Höhe zurück
        self._set_body_height(self.robot_standing_height)
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Führt die Navigation aus mit optionaler Höhenanpassung"""
        self.get_logger().info("🚀 Executing navigation goal...")
        
        # Goal-Daten extrahieren
        target = goal_handle.request.target_position
        width = goal_handle.request.entrance_width
        height = goal_handle.request.entrance_height
        required_height = goal_handle.request.required_height_adjustment
        
        try:
            # Schritt 1: Passe Körperhöhe an, falls nötig
            if required_height > 0.0:
                self.get_logger().info(
                    f"⬇️  Adjusting body height: {self.current_body_height:.3f}m → {required_height:.3f}m"
                )
                await self._adjust_body_height_async(required_height, goal_handle)
            else:
                self.get_logger().info("✅ No height adjustment needed - standing normally")
            
            # Schritt 2: Simuliere Navigation zum Eingang
            total_distance = self._calculate_distance(target)
            steps = 10
            
            for i in range(steps):
                # Prüfe ob Goal abgebrochen wurde
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = NavigateToEntrance.Result()
                    result.success = False
                    result.message = "Navigation canceled by user"
                    self.get_logger().warn("❌ Navigation canceled!")
                    # Stelle auf normale Höhe zurück
                    if required_height > 0.0:
                        self._set_body_height(self.robot_standing_height)
                    return result
                
                # Berechne Fortschritt
                progress = (i + 1) / steps * 100.0
                remaining = total_distance * (1.0 - (i + 1) / steps)
                
                # Sende Feedback
                feedback = NavigateToEntrance.Feedback()
                feedback.status = f"Navigating through entrance (step {i+1}/{steps})"
                feedback.progress_percentage = (20 if required_height > 0.0 else 0) + progress * 0.8  # 20% für Höhenanpassung
                feedback.distance_remaining = remaining
                
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f"📊 Progress: {feedback.progress_percentage:.1f}% | Distance remaining: {remaining:.2f}m"
                )
                
                # Simuliere Navigation
                time.sleep(0.5)
            
            # Schritt 3: Navigation erfolgreich - Stelle auf normale Höhe zurück
            if required_height > 0.0:
                self.get_logger().info(
                    f"⬆️  Navigation complete - restoring to standing height: {required_height:.3f}m → {self.robot_standing_height:.3f}m"
                )
                await self._adjust_body_height_async(self.robot_standing_height, goal_handle)
            
            # Navigation erfolgreich abgeschlossen
            goal_handle.succeed()
            
            result = NavigateToEntrance.Result()
            result.success = True
            result.message = f"Successfully navigated through entrance ({width:.2f}m x {height:.2f}m)"
            
            self.get_logger().info(f"✅ Navigation completed successfully!")
            return result
            
        except Exception as e:
            self.get_logger().error(f"❌ Error during navigation: {e}")
            # Stelle auf normale Höhe zurück bei Fehler
            if required_height > 0.0:
                self._set_body_height(self.robot_standing_height)
            
            goal_handle.abort()
            result = NavigateToEntrance.Result()
            result.success = False
            result.message = f"Navigation failed: {str(e)}"
            return result
    
    async def _adjust_body_height_async(self, target_height: float, goal_handle):
        """
        Passt die Körperhöhe asynchron an.
        Simuliert schrittweise Anpassung mit Feedback.
        """
        steps = 5
        current_height = self.current_body_height
        height_diff = target_height - current_height
        
        for step in range(steps):
            # Berechne interpolierte Höhe
            progress = (step + 1) / steps
            new_height = current_height + height_diff * progress
            
            # Setze Körperhöhe
            self._set_body_height(new_height)
            
            self.get_logger().info(
                f"⚙️  Height adjustment: {current_height:.3f}m → {new_height:.3f}m "
                f"({progress*100:.0f}%)"
            )
            
            # Gebe Feedback
            feedback = NavigateToEntrance.Feedback()
            feedback.status = f"Adjusting body height to {target_height:.3f}m ({progress*100:.0f}%)"
            feedback.progress_percentage = progress * 20  # Erste 20% für Höhenanpassung
            feedback.distance_remaining = 0.0
            goal_handle.publish_feedback(feedback)
            
            time.sleep(0.3)
    
    def _set_body_height(self, height: float):
        """
        Setzt die Körperhöhe des Roboters.
        
        In der echten Implementierung würde dies über die Unitree SDK erfolgen:
        - z.B. über Sport-Modus mit vordefinierten Höhen
        - oder über direkte Motor-Befehle
        """
        # Validiere Höhe
        height = max(self.min_crouching_height, min(height, self.robot_standing_height))
        self.current_body_height = height
        
        # In der echten Implementierung:
        # self._unitree_sdk.set_body_height(height)
        # Für jetzt: Loggen für Simulation
        self.get_logger().info(f"🎯 [SDK] Setting body height to {height:.3f}m")
        
        # Optional: Veröffentliche Bewegungsbefehl für Visualisierung
        twist_msg = Twist()
        twist_msg.linear.z = (height - self.robot_standing_height) * 0.1  # Scaling
        self.cmd_vel_pub.publish(twist_msg)
    
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
