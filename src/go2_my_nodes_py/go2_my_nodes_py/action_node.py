import rclpy
import time
import json
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
    
    # Sport API IDs (siehe unitree_ros2/example/src/include/common/ros2_sport_client.h)
    ROBOT_SPORT_API_ID_BALANCESTAND = 1002
    ROBOT_SPORT_API_ID_STOPMOVE = 1003
    ROBOT_SPORT_API_ID_STANDUP = 1004
    ROBOT_SPORT_API_ID_STANDDOWN = 1005
    ROBOT_SPORT_API_ID_EULER = 1007
    ROBOT_SPORT_API_ID_MOVE = 1008
    ROBOT_SPORT_API_ID_SIT = 1009
    ROBOT_SPORT_API_ID_RISESIT = 1010
    ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015
    
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
            # Schritt 1: Eingang nicht passierbar → hinsetzen, kurz warten, wieder aufstehen
            if not passable:
                self.get_logger().warn("❌ Entrance not passable - sit down, then stand up")
                self._send_sit_command()
                self.is_sitting = True
                time.sleep(2.0)
                await self._return_to_standing_async(
                    goal_handle,
                    reason="Standing up after sit",
                    wait_before=10.0
                )
                self.is_sitting = False

                goal_handle.succeed()
                result = HeightAdjustment.Result()
                result.success = True
                result.message = "Sat down due to non-passable entrance and returned to standing"
                self.get_logger().info("✅ Sit/stand sequence completed!")
                return result

            # Schritt 2: Eingang passierbar, aber Höhe anpassen → Euler (API 1007), dann wieder stehen
            if required_height > 0.0 and required_height < self.robot_standing_height:
                self.get_logger().info(
                    f"🧭 Height adjustment required ({required_height:.3f}m) → performing Euler, then stand"
                )
                await self._perform_euler_async(goal_handle)
                await self._return_to_standing_async(
                    goal_handle,
                    reason="Standing after Euler",
                    wait_before=10.0
                )
            else:
                # Schritt 3: Eingang passierbar im Stand → RiseSit, dann wieder aufstehen
                self.get_logger().info("✅ Entrance passable - rise sit, then stand up")
                await self._rise_sit_async(goal_handle)
                await self._return_to_standing_async(
                    goal_handle,
                    reason="Standing after rise sit",
                    wait_before=10.0
                )
            
            # Erfolgreiche Aktion
            goal_handle.succeed()
            result = HeightAdjustment.Result()
            result.success = True
            result.message = "Action completed and returned to standing"
            self.get_logger().info("✅ Action sequence completed successfully!")
            return result
            
        except Exception as e:
            self.get_logger().error(f"❌ Error during execution: {e}")
            # Stelle auf normale Höhe zurück bei Fehler
            self._set_body_height(self.robot_standing_height)
            self._send_balance_stand_command()
            
            goal_handle.abort()
            result = HeightAdjustment.Result()
            result.success = False
            result.message = f"Execution failed: {str(e)}"
            return result

    async def _return_to_standing_async(
        self,
        goal_handle,
        reason: str = "Returning to standing",
        wait_before: float = 0.0
    ):
        """Stellt sicher, dass der Roboter wieder im Stand ist."""
        if wait_before > 0.0:
            self.get_logger().info(f"⏳ Waiting {wait_before:.0f}s before standing")
            time.sleep(wait_before)
        self.get_logger().info(f"⬆️  {reason}")
        self._send_stand_up_command()
        self._send_balance_stand_command()
        self.current_body_height = self.robot_standing_height

        feedback = HeightAdjustment.Feedback()
        feedback.status = reason
        goal_handle.publish_feedback(feedback)

        time.sleep(1.5)
        self.get_logger().info("⏳ Waiting 5s after standing before finishing action")
        time.sleep(5.0)

    async def _perform_euler_async(self, goal_handle, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """Führt Euler-Command (API 1007) aus und wartet kurz."""
        self.get_logger().info(f"🧭 Performing Euler: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        self._send_euler_command(roll, pitch, yaw)

        feedback = HeightAdjustment.Feedback()
        feedback.status = "Performing Euler maneuver"
        goal_handle.publish_feedback(feedback)

        time.sleep(2.0)

    async def _rise_sit_async(self, goal_handle):
        """Führt RiseSit aus und wartet kurz."""
        self.get_logger().info("⬆️  RiseSit...")
        self._send_stand_up_command()

        feedback = HeightAdjustment.Feedback()
        feedback.status = "RiseSit"
        goal_handle.publish_feedback(feedback)

        time.sleep(2.0)
    
    
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
        Ruft StandDown/StandUp einmal auf (keine Schleife da diese Befehle keine Parameter haben).
        """
        current_height = self.current_body_height
        
        # Gebe Feedback vor Anpassung
        feedback = HeightAdjustment.Feedback()
        feedback.status = f"Adjusting body height from {current_height:.3f}m to {target_height:.3f}m"
        goal_handle.publish_feedback(feedback)
        
        # Setze Körperhöhe EINMAL (sendet einen StandDown/StandUp Befehl)
        self._set_body_height(target_height)
        
        self.get_logger().info(
            f"⚙️  Height adjustment: {current_height:.3f}m → {target_height:.3f}m"
        )
        
        # Warte länger damit der Befehl ausgeführt werden kann
        time.sleep(2.5)
    
    def _set_body_height(self, height: float):
        """
        Setzt die Körperhöhe des Roboters.
        Verwendet StandDown/StandUp statt BodyHeight (nicht verfügbar für Go2).
        
        height: Absolute Höhe in Metern
        """
        # Validiere Höhe
        height = max(self.min_crouching_height, min(height, self.robot_standing_height))
        self.current_body_height = height
        
        # Berechne relative Höhe (relativ zur Standardhöhe)
        relative_height = height - self.robot_standing_height
        
        # Nutze StandDown für niedrigere Höhe, StandUp für höhere Höhe
        if relative_height < -0.05:  # Mehr als 5cm niedriger
            self._send_stand_down_command()
            self.get_logger().info(f"🎯 Lowering body height with StandDown (target: {height:.3f}m)")
        elif relative_height > 0.05:  # Mehr als 5cm höher
            self._send_stand_up_high_command()
            self.get_logger().info(f"🎯 Raising body height with StandUp (target: {height:.3f}m)")
        else:
            self._send_balance_stand_command()
            self.get_logger().info(f"🎯 Normal height with BalanceStand (target: {height:.3f}m)")
    
    def _send_sit_command(self):
        """Sendet Sit-Befehl über Sport API"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating sit")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_SIT
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ Sit command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending sit command: {e}")
    
    def _send_stand_up_command(self):
        """Sendet RiseSit-Befehl über Sport API (steht von Sit-Position auf)"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating stand up")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_RISESIT
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ RiseSit command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending stand up command: {e}")
    
    def _send_stand_down_command(self):
        """Sendet StandDown-Befehl über Sport API (senkt Körperhöhe ab)"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating stand down")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_STANDDOWN
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ StandDown command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending stand down command: {e}")
    
    def _send_stand_up_high_command(self):
        """Sendet StandUp-Befehl über Sport API (hebt Körperhöhe an)"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating stand up high")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_STANDUP
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ StandUp command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending stand up high command: {e}")
    
    def _send_balance_stand_command(self):
        """Sendet BalanceStand-Befehl über Sport API"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating balance stand")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_BALANCESTAND
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ BalanceStand command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending balance stand command: {e}")

    def _send_euler_command(self, roll: float, pitch: float, yaw: float):
        """Sendet Euler-Befehl über Sport API (API 1007)"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating Euler")
            return

        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_EULER
            req.parameter = json.dumps({"x": float(roll), "y": float(pitch), "z": float(yaw)})
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ Euler command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending Euler command: {e}")

    
    def _send_move_command(self, vx: float, vy: float, vyaw: float):
        """Sendet Move-Befehl über Sport API
        
        Args:
            vx: Geschwindigkeit vorwärts/rückwärts in m/s
            vy: Geschwindigkeit seitwärts in m/s
            vyaw: Rotationsgeschwindigkeit in rad/s
        """
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating move")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_MOVE
            req.parameter = json.dumps({"x": float(vx), "y": float(vy), "z": float(vyaw)})
            self.sport_req_pub.publish(req)
            self.get_logger().info(f"✅ Move command sent: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending move command: {e}")
    
    def _send_stop_move_command(self):
        """Sendet StopMove-Befehl über Sport API"""
        if not self.sport_req_pub:
            self.get_logger().warn("Sport API not available - simulating stop")
            return
        
        try:
            req = Request()
            req.header.identity.api_id = self.ROBOT_SPORT_API_ID_STOPMOVE
            self.sport_req_pub.publish(req)
            self.get_logger().info("✅ StopMove command sent via Sport API")
        except Exception as e:
            self.get_logger().error(f"❌ Error sending stop move command: {e}")
    
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
