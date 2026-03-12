# Action Node for Height Adjustment based on Entrance Detection Results
# Height adjustment not implemented yet - this node serves as a placeholder for future implementation of height adjustment logic based on entrance detection results.

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from go2_my_nodes_py.base_node import BaseNode
from go2_msgs.action import HeightAdjustment


class action_node(BaseNode):
    """Action Server for height adjustment based on entrance detection results"""
    
    def __init__(self):
        super().__init__(
            name="height_adjustment_action_server",
            description="Action server for adjusting robot height when passing entrances"
        )
        
        # Action Server setup
        self._action_server = ActionServer(
            self,
            HeightAdjustment,
            'height_adjustment',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Robot parameters
        self.robot_standing_height = 0.45    # Normal standing height (45cm)
        self.min_crouching_height = 0.30     # Minimum height when crouching (30cm)
        self.current_body_height = self.robot_standing_height
        
        self.get_logger().info("Height Adjustment Action Server initialized and ready")
    
    #Callback when new height adjustment goal is received
    def goal_callback(self, goal_request):
        passable = goal_request.passable
        required_height = goal_request.required_height_adjustment
        self.get_logger().info(
            f"Height adjustment goal: passable={passable}, required_height={required_height:.3f}m"
        )
        return GoalResponse.ACCEPT
    
    #Callback when goal cancellation is requested
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Height adjustment goal cancellation requested")
        return CancelResponse.ACCEPT
    

    #Executes height adjustment based on entrance passability decision.
    #This callback receives the entrance detection decision and adjusts robot height accordingly.
    async def execute_callback(self, goal_handle):

        passable = goal_handle.request.passable
        required_height = goal_handle.request.required_height_adjustment
        
        self.get_logger().info(
            f"Processing height adjustment: passable={passable}, required_height={required_height:.3f}m"
        )
        
        try:
            # Decision 1: Entrance is not passable
            if not passable:
                self.get_logger().warn("Entrance decision: NOT_PASSABLE - no adjustment needed")
                feedback = HeightAdjustment.Feedback()
                feedback.status = "NOT_PASSABLE: entrance too narrow or low"
                goal_handle.publish_feedback(feedback)
                
                goal_handle.succeed()
                result = HeightAdjustment.Result()
                result.success = True
                result.message = "Entrance assessment complete: passage not viable"
                return result

            # Decision 2: Entrance is passable with height adjustment required
            if required_height > 0.0 and required_height < self.robot_standing_height:
                self.get_logger().info(
                    f"Entrance decision: HEIGHT_ADJUSTMENT ({required_height:.3f}m)"
                )
                feedback = HeightAdjustment.Feedback()
                feedback.status = f"Adjusting height from {self.current_body_height:.3f}m to {required_height:.3f}m"
                goal_handle.publish_feedback(feedback)
                
                # PLACEHOLDER: Implement height adjustment logic here
                # TODO: Add code to execute the actual height adjustment command to robot:
                # 1. Send Sport API command to lower/raise robot body
                # 2. Monitor current body height during adjustment
                #
                # Pattern: 
                # self._adjust_robot_height_to(required_height)  # To be implemented
                # self.current_body_height = required_height
                # =========================================================================
                
                self.get_logger().info(f"[PLACEHOLDER] Height adjustment to {required_height:.3f}m - implementation pending")

            # Decision 3: Entrance is passable in standing position    
            else:
                self.get_logger().info("Entrance decision: PASSABLE_STANDING - no adjustment needed")
                feedback = HeightAdjustment.Feedback()
                feedback.status = "PASSABLE_STANDING: entrance accessible at normal robot height"
                goal_handle.publish_feedback(feedback)
            
            # Complete action successfully
            goal_handle.succeed()
            result = HeightAdjustment.Result()
            result.success = True
            result.message = "Height adjustment process completed"
            self.get_logger().info("Height adjustment action completed successfully")
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error in height adjustment execution: {e}")
            goal_handle.abort()
            result = HeightAdjustment.Result()
            result.success = False
            result.message = f"Height adjustment failed: {str(e)}"
            return result
    
    # Process method from BaseNode (not used in action server)
    def process(self, data):
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
