import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameters('', [('pose.x', 0.0), ('pose.y', 0.0), ('pose.z', 0.0), ('pose.roll', 0.0), ('pose.pitch', 0.0), ('pose.yaw', 0.0)])

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.timer = self.create_timer(1, self.publish_initial_pose)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = self.get_parameter('pose.x').get_parameter_value().double_value
        msg.pose.pose.position.y = self.get_parameter('pose.y').get_parameter_value().double_value
        msg.pose.pose.position.z = self.get_parameter('pose.z').get_parameter_value().double_value
        
        msg.pose.pose.orientation.x = self.get_parameter('pose.roll').get_parameter_value().double_value
        msg.pose.pose.orientation.y = self.get_parameter('pose.pitch').get_parameter_value().double_value
        msg.pose.pose.orientation.z = self.get_parameter('pose.yaw').get_parameter_value().double_value
        msg.pose.pose.orientation.w = 1.0
        
        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.06853891945200942
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info('Initial pose published')

        self.timer.cancel()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    initial_pose_publisher = InitialPosePublisher()
    try:
        rclpy.spin(initial_pose_publisher)
    except KeyboardInterrupt:
        initial_pose_publisher.get_logger().info('Initial Pose Publisher Node stopped cleanly')
    except Exception as e:
        initial_pose_publisher.get_logger().fatal(f'Exception in Initial Pose Publisher Node: {e}')
    finally:
        initial_pose_publisher.destroy_node()


if __name__ == '__main__':
    main()
