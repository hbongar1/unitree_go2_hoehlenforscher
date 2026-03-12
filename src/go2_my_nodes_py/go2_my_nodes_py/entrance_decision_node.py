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

        # Subscribe to entrance detections
        self.subscription = self.create_subscription(
            Entrance,
            'entrance_detection_ransac',
            self.entrance_callback,
            10
        )

        # Action client for height adjustment
        self._action_client = ActionClient(
            self,
            HeightAdjustment,
            'height_adjustment'
        )

        # Passage decision thresholds
        self.min_width = 0.3    # minimum width in meters
        self.min_height = 0.35  # minimum height in meters

        # Robot dimensions
        self.robot_standing_height = 0.45  # standing height (45 cm for GO2)
        self.robot_width = 0.30            # robot width (30 cm)
        self.min_crouching_height = 0.30   # minimum crouching height (30 cm)

        # Measurement buffer
        self.entrance_buffer_size = 5  # number of measurements to average
        self.entrance_measurements = deque(maxlen=self.entrance_buffer_size)
        self.action_in_progress = False  # block new decisions while an action is running
    
    def entrance_callback(self, msg: Entrance):
        """Called when new entrance data arrives."""
        if self.action_in_progress:
            return

        self.entrance_measurements.append({'width': msg.width, 'height': msg.height})

        if len(self.entrance_measurements) >= self.entrance_buffer_size:
            avg_entrance = self.calculate_average_entrance()
            decision, required_height = self.make_decision(avg_entrance)
            self.execute_action(decision, avg_entrance, required_height)
            self.entrance_measurements.clear()
        else:
            self.get_logger().info(
                f"Collecting measurements: {len(self.entrance_measurements)}/{self.entrance_buffer_size}"
            )
    
    
    def calculate_average_entrance(self) -> dict:
        """Compute and return average width and height from buffered measurements."""
        widths = [m['width'] for m in self.entrance_measurements]
        heights = [m['height'] for m in self.entrance_measurements]

        avg_width = np.mean(widths)
        avg_height = np.mean(heights)

        self.get_logger().info(
            f"Average entrance: width={avg_width:.3f}m ±{np.std(widths):.3f}, "
            f"height={avg_height:.3f}m ±{np.std(heights):.3f}"
        )

        return {'width': avg_width, 'height': avg_height}
    
    def make_decision(self, entrance: dict) -> tuple:
        """
        Decide how the robot should handle the detected entrance.

        Returns:
            (decision: str, required_height: float)
            - decision: "PASS_THROUGH_STANDING", "PASS_THROUGH_CROUCHING", "TOO_NARROW", "TOO_LOW"
            - required_height: target body height in meters (0.0 if no adjustment needed)
        """
        width = entrance['width']
        height = entrance['height']

        if width < self.robot_width:
            self.get_logger().warn(
                f"Entrance too narrow: {width:.3f}m < robot width {self.robot_width:.3f}m"
            )
            return "TOO_NARROW", 0.0

        if height >= self.robot_standing_height:
            self.get_logger().info(
                f"Can pass through standing: height {height:.3f}m >= {self.robot_standing_height:.3f}m"
            )
            return "PASS_THROUGH_STANDING", 0.0

        if height >= self.min_crouching_height:
            required_height = height - 0.05  # 5 cm safety margin
            self.get_logger().info(
                f"Must crouch: height {height:.3f}m → adjust to {required_height:.3f}m"
            )
            return "PASS_THROUGH_CROUCHING", required_height

        self.get_logger().warn(
            f"Entrance too low: {height:.3f}m < minimum crouching height {self.min_crouching_height:.3f}m"
        )
        return "TOO_LOW", 0.0
    
    def execute_action(self, decision: str, entrance: dict, required_height: float = 0.0):
        """Send an action goal based on the given decision."""
        if self.action_in_progress:
            return

        self.action_in_progress = True
        self.send_action_goal(entrance, required_height, decision)
    
    def send_action_goal(self, entrance: dict, required_height: float, decision: str):
        """Send a HeightAdjustment goal to the action server."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available, aborting.")
            self.action_in_progress = False
            return

        passable = decision in ["PASS_THROUGH_STANDING", "PASS_THROUGH_CROUCHING"]

        goal_msg = HeightAdjustment.Goal()
        goal_msg.passable = passable
        goal_msg.required_height_adjustment = required_height if passable else 0.0

        self.get_logger().info(
            f"Sending goal: decision={decision}, passable={passable}, "
            f"required_height_adjustment={goal_msg.required_height_adjustment:.3f}m"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Store state for result callback
        self._required_height = required_height
        self._decision = decision
    
    def goal_response_callback(self, future):
        """Handles the action server's response to the sent goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by action server')
            self.action_in_progress = False
            return

        self.get_logger().info('Goal accepted by action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handles action feedback during execution."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Action feedback: {feedback.status}")
    
    def get_result_callback(self, future):
        """Handles the final result of the action."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Action succeeded: {result.message}')
        else:
            self.get_logger().error(f'Action failed: {result.message}')
        self.action_in_progress = False
    
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
