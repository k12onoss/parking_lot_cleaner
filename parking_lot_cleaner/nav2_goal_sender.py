import rclpy
import json

from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle


class RobotStatus:
    def __init__(self) -> None:
        self.goals: list[PoseStamped] = list()
        self.poses_remaining: int = -1
        self.is_idle = True
    
    def __str__(self) -> str:
        return f"Robot Status(is_idle: {self.is_idle}, poses_remaining: {self.poses_remaining}, goals: {self.goals})"
    
    def __repr__(self) -> str:
        return f"Robot Status(is_idle: {self.is_idle}, poses_remaining: {self.poses_remaining}, goals: {self.goals})"


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        self.garbage_dict: dict[int, list[PoseStamped]] = {}
        self.robot_status_dict: dict[int, RobotStatus] = {}
        self.action_clients: dict[int, ActionClient] = {}

        self.create_subscription(String, 'job_sequence', self.save_sequence, 10)

        self.delete_garbage_publisher = self.create_publisher(PoseStamped, 'delete_garbage', 10)

    def save_sequence(self, job_sequence: String):
        sequence: list[tuple[int, list[tuple[float, float]]]] = json.loads(job_sequence.data)

        for robot_id, seq in sequence:
            poses: list[PoseStamped] = self.garbage_dict.setdefault(robot_id, [])

            for x, y in seq:
                pose = PoseStamped()

                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'

                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                poses.append(pose)

            robot_status = self.robot_status_dict.setdefault(robot_id, RobotStatus())
            if robot_status.is_idle:
                self.send_goal(robot_id)

    def send_goal(self, robot_id: int):
        poses = self.garbage_dict.get(robot_id)
        
        if len(poses) == 0:
            return
        
        robot_status = self.robot_status_dict.get(robot_id)

        robot_status.is_idle = False

        action_client = self.action_clients.setdefault(robot_id, self.create_action_client(robot_id))

        if not action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server for robot{robot_id} not available')
            robot_status.is_idle = True
            return

        robot_status.goals.extend(poses)
        poses.clear()
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = robot_status.goals
        
        future: Future = action_client.send_goal_async(goal_msg, self.feedback_callback)
        future.add_done_callback(lambda response: self.goal_response_callback(response, robot_id))

    def goal_response_callback(self, future: Future, robot_id: int):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info(f'Goal accepted by robot{robot_id}')
            get_result_future: Future = goal_handle.get_result_async()
            get_result_future.add_done_callback(lambda result: self.get_result_callback(result, robot_id))
        else:
            robot_status = self.robot_status_dict.get(robot_id)
            robot_status.is_idle = True
            self.get_logger().info(f'Goal rejected by robot{robot_id}')
            self.garbage_dict.get(robot_id).extend(robot_status.goals)
            robot_status.goals.clear()

    def get_result_callback(self, future: Future, robot_id: int):
        result: NavigateThroughPoses.Result = future.result()
        
        robot_status = self.robot_status_dict.get(robot_id)
        robot_status.is_idle = True
        robot_status.goals.clear()
        
        self.send_goal(robot_id)
    
    def feedback_callback(self, feedback_msg):
        feedback: NavigateThroughPoses.Feedback = feedback_msg.feedback
        self.delete_garbage_publisher.publish(feedback.current_pose)
    
    def create_action_client(self, robot_id: int) -> ActionClient:
        action: str = 'robot{}/navigate_through_poses'.format(robot_id)
        return ActionClient(self, NavigateThroughPoses, action)


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
