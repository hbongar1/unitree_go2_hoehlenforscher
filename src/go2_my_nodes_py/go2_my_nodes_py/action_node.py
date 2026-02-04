import rclpy
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.action import Height_Adjustment
from geometry_msgs.msg import Twist

# Unitree SDK imports
try:
    from unitree_sdk2py.core.sportclient import SportClient
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False
    SportClient = None


class action_node(BaseNode):
    """Action Server for navigation to entrance"""
    
    def __init__(self):
        super().__init__(
            name="navigation_action_server",
            description="Action server for navigating to detected entrances"
        )
        
        # Initialisiere Unitree SportClient falls verfügbar
        self.sport_client = None
        if UNITREE_SDK_AVAILABLE:
            try:
                self.sport_client = SportClient()
                self.sport_client.Init()
                self.get_logger().info("✅ Unitree SportClient initialized")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Could not initialize SportClient: {e}")
        else:
            self.get_logger().warn("⚠️  Unitree SDK not available - using simulation mode")
        
        # Action Server erstellen
        self._action_server = ActionServer(
            self,
            Height_Adjustment,
            'height_adjustment',
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
        self.is_sitting = False  # Flag: Roboter sitzt gerade
        
        self.get_logger().info("Height Adjustment Action Server ready!")
    
    def goal_callback(self, goal_request):
        """Callback wenn neues Goal empfangen wird"""
        passable = goal_request.passable
        required_height = goal_request.required_height_adjustment
        self.get_logger().info(
            f"📍 Received height adjustment goal: "
            f"passable={passable}, required_height={required_height:.3f}m"
        )
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Callback wenn Goal abgebrochen werden soll"""
        self.get_logger().info("Received cancel request")
        # Stelle Roboter auf normale Höhe zurück
        self._set_body_height(self.robot_standing_height)
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Führt die Höhenanpassung und Bewegung aus"""
        self.get_logger().info("🚀 Executing height adjustment goal...")
        
        # Goal-Daten extrahieren
        passable = goal_handle.request.passable
        required_height = goal_handle.request.required_height_adjustment
        
        try:
            # Schritt 0: Prüfe ob Roboter sitzt und Umgebung sich verbessert hat
            if self.is_sitting and passable:
                self.get_logger().info(
                    f"✅ Environment improved! Roboter sitzt noch, aber Eingang ist jetzt passierbar → Stand up"
                )
                # Roboter aufstehen lassen
                await self._stand_up_async(goal_handle)
                self.is_sitting = False
            
            # Schritt 1: Prüfe ob Eingang passierbar ist
            if not passable:
                self.get_logger().warn(
                    f"❌ Entrance not passable - sitting down at {self.robot_sitting_height:.3f}m"
                )
                
                # Nutze SDK zum Hinsetzen falls verfügbar
                if self.sport_client:
                    try:
                        result = self.sport_client.Sit()
                        if result == 0:
                            self.get_logger().info("✅ Successfully sat down (SDK)")
                            self.is_sitting = True
                        else:
                            self.get_logger().warn(f"⚠️  SDK Sit() returned error code {result}")
                    except Exception as e:
                        self.get_logger().error(f"❌ Error executing Sit(): {e}")
                
                # Erfolgreiche Aktion
                goal_handle.succeed()
                result = Height_Adjustment.Result()
                result.success = True
                result.message = "Sat down due to non-passable entrance"
                self.get_logger().info("✅ Successfully sat down!")
                return result
            
            # Schritt 2: Eingang passierbar - passe Höhe an falls nötig
            if required_height > 0.0 and required_height < self.robot_standing_height:
                self.get_logger().info(
                    f"⬇️  Adjusting body height: {self.current_body_height:.3f}m → {required_height:.3f}m"
                )
                await self._adjust_body_height_async(required_height, goal_handle)
            else:
                self.get_logger().info(
                    f"✅ No height adjustment needed - standing at {self.robot_standing_height:.3f}m"
                )
            
            # Erfolgreiche Aktion
            goal_handle.succeed()
            result = Height_Adjustment.Result()
            result.success = True
            result.message = f"Height adjusted to {self.current_body_height:.3f}m successfully"
            self.get_logger().info("✅ Height adjustment completed successfully!")
            return result
            
        except Exception as e:
            self.get_logger().error(f"❌ Error during execution: {e}")
            # Stelle auf normale Höhe zurück bei Fehler
            self._set_body_height(self.robot_standing_height)
            
            goal_handle.abort()
            result = Height_Adjustment.Result()
            result.success = False
            result.message = f"Execution failed: {str(e)}"
            return result
    
    
    async def _stand_up_async(self, goal_handle):
        """
        Roboter steht von sitzender Position auf.
        Nutzt SDK RiseSit() Methode.
        """
        self.get_logger().info("⬆️  Standing up from sitting position...")
        
        if self.sport_client:
            try:
                # Nutze echten SDK - RiseSit zum Aufstehen vom Sitzen
                result = self.sport_client.RiseSit()
                if result == 0:
                    self.get_logger().info("✅ Successfully stood up (SDK)")
                    # Setze aktuelle Höhe auf Standing-Höhe
                    self.current_body_height = self.robot_standing_height
                else:
                    self.get_logger().warn(f"⚠️  SDK RiseSit() returned error code {result}")
            except Exception as e:
                self.get_logger().error(f"❌ Error executing RiseSit(): {e}")
        else:
            # Simulation
            self.get_logger().info("⬆️  [SIMULATION] Standing up from sitting position")
            self.current_body_height = self.robot_standing_height
        
        # Gebe Feedback
        feedback = Height_Adjustment.Feedback()
        feedback.status = "Standing up from sitting position"
        goal_handle.publish_feedback(feedback)
        
        time.sleep(1.0)  # Gib SDK Zeit zum Ausführen
    
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
            feedback = Height_Adjustment.Feedback()
            feedback.status = f"Adjusting body height to {target_height:.3f}m ({progress*100:.0f}%)"
            goal_handle.publish_feedback(feedback)
            
            time.sleep(0.3)
    
    def _set_body_height(self, height: float):
        """
        Setzt die Körperhöhe des Roboters.
        Verwendet Unitree SDK falls verfügbar, sonst Simulation.
        
        height: Absolute Höhe in Metern
        """
        # Validiere Höhe
        height = max(self.min_crouching_height, min(height, self.robot_standing_height))
        self.current_body_height = height
        
        # Berechne relative Höhe für SDK (relativ zur Standardhöhe 0.33m)
        relative_height = height - self.robot_standing_height
        
        if self.sport_client:
            try:
                # Nutze echten SDK
                result = self.sport_client.BodyHeight(relative_height)
                if result == 0:
                    self.get_logger().info(
                        f"🎯 Body height set to {height:.3f}m (relative: {relative_height:.3f}m)"
                    )
                else:
                    self.get_logger().warn(
                        f"⚠️  SDK returned error code {result} for BodyHeight"
                    )
            except Exception as e:
                self.get_logger().error(f"❌ Error setting body height: {e}")
        else:
            # Simulation für Testing
            self.get_logger().info(
                f"🎯 [SIMULATION] Setting body height to {height:.3f}m"
            )
        
        # Optional: Veröffentliche Bewegungsbefehl für Visualisierung
        twist_msg = Twist()
        twist_msg.linear.z = (height - self.robot_standing_height) * 0.1
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
    node = action_node()
    
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
