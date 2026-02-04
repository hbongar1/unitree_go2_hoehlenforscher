import rclpy
from rclpy.action import ActionClient
from collections import deque
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.msg import Entrance
from go2_msgs.action import HeightAdjustment
import numpy as np


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
            'entrance_detection_ransac',
            self.entrance_callback,
            10
        )
        
        # Action Client für Höhenanpassung
        self._action_client = ActionClient(
            self,
            HeightAdjustment,
            'height_adjustment'
        )
        
        # Parameter für Durchgangsentscheidungen
        self.min_width = 0.3  # Minimale Breite in Metern
        self.min_height = 0.35  # Minimale Höhe in Metern
        
        # Roboter-Parameter
        self.robot_standing_height = 0.45  # Höhe im Stand (45cm für GO2)
        self.robot_width = 0.30  # Breite des Roboters (30cm)
        self.min_crouching_height = 0.30  # Minimale Höhe beim Beugen (30cm)
        
        # Messungen sammeln
        self.entrance_buffer_size = 5  # Anzahl der Messungen zum Mitteln
        self.entrance_measurements = deque(maxlen=self.entrance_buffer_size)
        self.decision_sent = False  # Flag um doppelte Entscheidungen zu vermeiden
    
    def entrance_callback(self, msg: Entrance):
        """Callback wird aufgerufen, wenn Entrance-Daten ankommen"""
        self.get_logger().info(
            f"Received entrance: width={msg.width:.3f}m, height={msg.height:.3f}m"
        )
        
        # Sammle Messungen
        self.entrance_measurements.append({
            'width': msg.width,
            'height': msg.height
        })
        
        # Nur Entscheidung treffen, wenn genug Messungen gesammelt wurden
        if len(self.entrance_measurements) >= self.entrance_buffer_size:
            # Mittele die Messungen
            avg_entrance = self.calculate_average_entrance()
            
            # Treffe Entscheidung
            decision, required_height = self.make_decision(avg_entrance)
            
            # Führe nur einmal eine Aktion aus
            if not self.decision_sent:
                self.execute_action(decision, avg_entrance, required_height)
                self.decision_sent = True
                # Leere Buffer nach Entscheidung
                self.entrance_measurements.clear()
        else:
            self.get_logger().info(
                f"Collecting measurements: {len(self.entrance_measurements)}/{self.entrance_buffer_size}"
            )
    
    
    def calculate_average_entrance(self) -> dict:
        """Berechnet Durchschnittswerte aus den gesammelten Messungen"""
        widths = [m['width'] for m in self.entrance_measurements]
        heights = [m['height'] for m in self.entrance_measurements]
        
        avg_width = np.mean(widths)
        avg_height = np.mean(heights)
        
        self.get_logger().info(
            f"📊 Average entrance measurements: "
            f"width={avg_width:.3f}m±{np.std(widths):.3f}, "
            f"height={avg_height:.3f}m±{np.std(heights):.3f}"
        )
        
        return {
            'width': avg_width,
            'height': avg_height
        }
    
    def make_decision(self, entrance: dict) -> tuple:
        """
        Trifft Entscheidung basierend auf gemittelten Entrance-Daten.
        
        Returns:
            (decision: str, required_height: float)
            - decision: "PASS_THROUGH_STANDING", "PASS_THROUGH_CROUCHING", "TOO_NARROW", "TOO_LOW"
            - required_height: Höhe auf die sich der Roboter einstellen muss (0 = kein Anpassen nötig)
        """
        width = entrance['width']
        height = entrance['height']
        
        # Prüfe Breite
        if width < self.robot_width:
            self.get_logger().warn(
                f"❌ Entrance too narrow: {width:.3f}m < {self.robot_width:.3f}m"
            )
            return "TOO_NARROW", 0.0
        
        # Prüfe ob im Stand durchpassbar
        if height >= self.robot_standing_height:
            self.get_logger().info(
                f"✅ Can pass through STANDING: "
                f"entrance height {height:.3f}m >= standing height {self.robot_standing_height:.3f}m"
            )
            return "PASS_THROUGH_STANDING", 0.0
        
        # Prüfe ob im Beugen durchpassbar
        elif height >= self.min_crouching_height:
            required_height = height - 0.05  # Kleine Sicherheitsmarge
            self.get_logger().info(
                f"⚠️  Must crouch to pass through: "
                f"entrance height {height:.3f}m < standing height {self.robot_standing_height:.3f}m "
                f"→ adjust to {required_height:.3f}m"
            )
            return "PASS_THROUGH_CROUCHING", required_height
        
        # Zu niedrig
        else:
            self.get_logger().warn(
                f"❌ Entrance too low: {height:.3f}m < minimum crouching height {self.min_crouching_height:.3f}m"
            )
            return "TOO_LOW", 0.0
    
    def execute_action(self, decision: str, entrance: dict, required_height: float = 0.0):
        """Führt Action aus, um die Entscheidung an die Action Node zu schicken"""
        
        if decision == "PASS_THROUGH_STANDING":
            self.get_logger().info(
                f"✅ Can pass through STANDING - sending action goal"
            )
            self.send_action_goal(entrance, required_height, decision)
            
        elif decision == "PASS_THROUGH_CROUCHING":
            self.get_logger().info(
                f"⚠️  Must crouch to pass through (required height: {required_height:.3f}m)"
            )
            self.send_action_goal(entrance, required_height, decision)
            
        elif decision == "TOO_NARROW":
            self.get_logger().warn(
                f"🚫 Entrance too narrow ({entrance['width']:.3f}m < {self.robot_width:.3f}m). "
                f"Looking for alternative route."
            )
            
        elif decision == "TOO_LOW":
            self.get_logger().warn(
                f"🚫 Entrance too low ({entrance['height']:.3f}m < {self.min_crouching_height:.3f}m). "
                f"Looking for alternative route."
            )
    
    def send_action_goal(self, entrance: dict, required_height: float, decision: str):
        """Sendet Action Goal an den Action Server für Höhenanpassung"""
        
        # Warte auf Action Server
        self.get_logger().info("⏳ Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return
        
        # Determine if passable
        passable = decision in ["PASS_THROUGH_STANDING", "PASS_THROUGH_CROUCHING"]
        
        # Erstelle Goal für HeightAdjustment
        goal_msg = HeightAdjustment.Goal()
        goal_msg.passable = passable
        goal_msg.required_height_adjustment = required_height if passable else 0.0
        
        self.get_logger().info(
            f"📤 Sending action goal: passable={passable}, "
            f"required_height_adjustment={goal_msg.required_height_adjustment:.3f}m"
        )
        
        # Sende Goal asynchron mit Callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Speichere Infos für Callbacks
        self._required_height = required_height
        self._decision = decision
    
    def goal_response_callback(self, future):
        """Callback wenn Server Goal akzeptiert/ablehnt"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected by action server')
            return
        
        self.get_logger().info('✅ Goal accepted by action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback für Feedback während der Ausführung"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"🔄 Action feedback: {feedback.status}"
        )
    
    def get_result_callback(self, future):
        """Callback wenn Action abgeschlossen ist"""
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'✅ Action successful! {result.message}'
            )
        else:
            self.get_logger().error(
                f'❌ Action failed: {result.message}'
            )
        
        # Bereit für nächste Entscheidung
        self.decision_sent = False
    
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
