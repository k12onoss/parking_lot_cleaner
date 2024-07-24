import os
import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import NavigateToPose

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(NavigateToPose, 'goal_pose', 10)
        # self.subscription = self.create_subscription(
        #     Image,
        #     'X3/image',  # Change this to your image topic
        #     self.listener_callback,
        #     10
        # )
        # self.br = CvBridge()
        # self.image_count = 0
        # self.output_dir = 'images'
        # if not os.path.exists(self.output_dir):
        #     os.makedirs(self.output_dir)

    # def listener_callback(self, msg):
    #     # self.get_logger().info('Receiving image...')
    #     current_frame = self.br.imgmsg_to_cv2(msg)
    #     image_path = os.path.join(self.output_dir, f'image_{self.image_count:04d}.png')
    #     cv2.imwrite(image_path, current_frame)
    #     # self.get_logger().info(f'Saved image {self.image_count:04d}')
    #     self.image_count += 1


    def publish_goal(self):
        goal_msg = NavigateToPose()

        # Fill in the header
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'  # Use the frame of your map

        # Fill in the goal position
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.position.z = 0.0

        # Fill in the goal orientation (Quaternion)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.publisher_.publish(goal_msg)
        self.get_logger().info('Published goal position: ({}, {}, {})'.format(
            goal_msg.pose.pose.position.x,
            goal_msg.pose.pose.position.y,
            goal_msg.pose.pose.position.z))

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    goal_publisher.publish_goal()

    try:
        rclpy.spin(goal_publisher)
    except KeyboardInterrupt:
        goal_publisher.get_logger().info('Goal Publisher Node stopped cleanly')
    except Exception as e:
        goal_publisher.get_logger().fatal(f'Exception in Goal Publisher Node: {e}')
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
