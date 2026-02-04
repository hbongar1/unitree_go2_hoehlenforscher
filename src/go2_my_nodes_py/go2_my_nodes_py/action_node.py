import rclpy
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.action import HeightAdjustment
from geometry_msgs.msg import Twist

# Unitree ROS2 API imports
try:
    from unitree_api.msg import Request
    UNITREE_API_AVAILABLE = True
except ImportError:
    UNITREE_API_AVAILABLE = False
    Request = None


class action_node(BaseNode):
    """Action Server for navigation to entrance"""
    
    def __init__(self):
        super().__init__(
            name="navigation_action_server",
            description="Action server for navigating to detected entrances"
        )
        
        # Publisher für Sportmode Request Messages
        if UNITREE_API_AVAILABLE:
            self.sport_req_pub = self.create_publisher(
                Request, 
                '/api/sport/request', 
                10
            )
            self.get_logger().info("✅ Unitree Sport API publisher initialized")
        else:
            self.sport_req_pub = None
            self.get_logger().warn("⚠️  Unitree API not available - using simulation mode")
        
        # Action Server erstellen
        self._action_server = ActionServer(
            self,
            HeightAdjustment,
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
                    "❌ Entrance not passable - sitting down"
                )
                
                # Nutze ROS2 Sport API zum Hinsetzen
                self._send_sit_command()
                self.is_sitting = True
                
                # Erfolgreiche Aktion
                goal_handle.succeed()
                result = HeightAdjustment.Result()
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
            result = HeightAdjustment.Result()
            result.success = True
            result.message = f"Height adjusted to {self.current_body_height:.3f}m successfully"
            self.get_logger().info("✅ Height adjustment completed successfully!")
            return result
            
        except Exception as e:
            self.get_logger().error(f"❌ Error during execution: {e}")
            # Stelle auf normale Höhe zurück bei Fehler
            self._set_body_height(self.robot_standing_height)
            
            goal_handle.abort()
            result = HeightAdjustment.Result()
            result.success = False
            result.message = f"Execution failed: {str(e)}"
            return result
    
    
    async def _stand_up_async(self, goal_handle):
        """
        Roboter steht von sitzender Position auf.
        Nutzt SDK RiseSit() Methode.
        """
        self.get_logger().info("⬆️  Standing up from sitting position...")
        
        # Nutze ROS2 Sport API zum Aufstehen
        self._send_stand_up_command()
        self.current_body_height = self.robot_standing_height
        
        # Gebe Feedback
        feedback = HeightAdjustment.Feedback()
        feedback.status = "Standing up from sitting position"
        goal_handle.publish_feedback(feedback)
        
        time.sleep(1.0)  # Gib Zeit zum Ausführen
    
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
            feedback = HeightAdjustment.Feedback()
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
        
        # Berechne relative Höhe (relativ zur Standardhöhe)
        relative_height = height - self.robot_standing_height
        
        # Nutze ROS2 Sport API
        self._send_body_height_command(relative_height)
        
        self.get_logger().info(
            f"🎯 Body height set to {height:.3f}m (relative: {relative_height:.3f}m)"
        )
    
    def _send_sit_command(self):
        """Sendet Sit-Befehl über Sport API"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating sit")
            return
        
        try:
            req = Request()
            # Für Sit-Befehl müssen wir die entsprechende API-Struktur verwenden
            # Dies ist ein Platzhalter - die genaue Implementierung hängt von der API ab
            req.header.identity.id = 0
            req.parameter = '{"name": "sit"}'
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ Sit command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending sit command: {e}")
    
    def _send_stand_up_command(self):
        """Sendet StandUp-Befehl über Sport API"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating stand up")
            return
        
        try:
            req = Request()
            req.header.identity.id = 0
            req.parameter = '{"name": "stand_up"}'
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ Stand up command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending stand up command: {e}")
    
    def _send_body_height_command(self, height: float):
        """Sendet BodyHeight-Befehl über Sport API"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating body height")
            return
        
        try:
            req = Request()
            req.header.identity.id = 0
            req.parameter = f'{{"name": "body_height", "height": {height:.3f}}}'
            self.sport_req_pub.publish(req)
            self.get_logger().info(f"✅ Body height command sent: {height:.3f}m")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending body height command: {e}")
    
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
