import rclpy

from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        self.garbage_queue: list[PoseStamped] = []
        self.current_goals: list[PoseStamped] = []
        self.previous_poses_remaining = -1
        self.is_idle = True

        self.create_subscription(PoseArray, 'garbage_locations', self.save_locations, 10)

        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.goal_pose = PoseStamped()

        self.delete_garbage_publisher = self.create_publisher(PoseStamped, 'delete_garbage', 10)
    
    def save_locations(self, locations: PoseArray):
        for location in locations.poses:
            pose = PoseStamped()
            pose.header = locations.header
            pose.pose = location
            self.garbage_queue.append(pose)
        
        if self.is_idle:
            self.send_goal()

    def send_goal(self):
        if len(self.garbage_queue) == 0:
            self.is_idle = True
            self.get_logger().info("Goal queue is Empty.")
            return
        
        self.is_idle = False
        
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        self.current_goals.extend(self.garbage_queue)
        self.garbage_queue.clear()

        message = NavigateThroughPoses.Goal()
        message.poses = self.current_goals

        self.get_logger().info('Sending goal pose...')
        send_goal_future = self.action_client.send_goal_async(message, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_idle = True
            self.get_logger().info('Goal rejected')
            self.garbage_queue.extend(self.current_goals)
        else:
            self.get_logger().info('Goal accepted')
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: NavigateThroughPoses.Result = future.result().result
        self.get_logger().info(f'Goal result: {result}')
        if result.error_code == 0:
            self.is_idle = True
            # self.delete_garbage_publisher.publish(self.current_goals)
        
        self.send_goal()

    def feedback_callback(self, feedback_msg):
        feedback: NavigateThroughPoses.Feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')
        if self.previous_poses_remaining != -1 and feedback.number_of_poses_remaining < self.previous_poses_remaining:
            garbage_to_delete = self.current_goals[len(self.current_goals) - feedback.number_of_poses_remaining - 1]
            self.get_logger().info(f"Garbage to be deleted: {garbage_to_delete}")
            self.delete_garbage_publisher.publish(garbage_to_delete)
        
        self.previous_poses_remaining = feedback.number_of_poses_remaining


def main(args=None):
    rclpy.init(args=args)

    goal_sender = Nav2GoalSender()
    try:
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        goal_sender.get_logger().info('Nav2 Goal Sender Node stopped cleanly')
    except Exception as e:
        goal_sender.get_logger().fatal(f'Exception in Nav2 Goal Sender Node: {e}')
    finally:
        goal_sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
