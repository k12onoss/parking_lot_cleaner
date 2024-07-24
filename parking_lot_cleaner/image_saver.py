import os
import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            'X3/image',  # Change this to your image topic
            self.listener_callback,
            10
        )
        self.br = CvBridge()
        self.image_count = 0
        self.output_dir = 'images'
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def listener_callback(self, msg):
        # self.get_logger().info('Receiving image...')
        current_frame = self.br.imgmsg_to_cv2(msg)
        image_path = os.path.join(self.output_dir, f'image_{self.image_count:04d}.png')
        cv2.imwrite(image_path, current_frame)
        # self.get_logger().info(f'Saved image {self.image_count:04d}')
        self.image_count += 1

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
