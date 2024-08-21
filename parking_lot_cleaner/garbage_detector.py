import os
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class GarbageDetector(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.timer = self.create_timer(15, self.sub_to_topic)
        self.bridge = CvBridge()

        self.is_idle = True

        pkg_share = get_package_share_directory('parking_lot_cleaner')
        model_path = os.path.join(pkg_share, 'trained_model', 'yolov8_garbage_detection.pt')
        self.model = YOLO(model_path)

        self.publisher = self.create_publisher(Image, 'env_map', 10)
    
    def sub_to_topic(self):
        self.sub = self.create_subscription(Image, 'X3/image', self.listener_callback, 10)
        self.timer.cancel()

    def listener_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg)
        if self.is_idle:
            self.create_map(current_frame)
        
        self.destroy_subscription(self.sub)
    
    def create_map(self, current_frame: np.ndarray):
        self.is_idle = False

        results = self.model.predict(current_frame, save=True)

        env_map = np.zeros((240, 320), dtype=np.uint8)
        for result in results:
            boxes = result.boxes.xyxy  
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.tolist()[:4]
                x_min = int(x1)
                x_max = int(x2)
                y_min = int(y1)
                y_max = int(y2)
                # Update the environment map
                env_map[y_min:y_max, x_min:x_max] = 1

        np.save('envmap.npy', env_map)

        image_msg = self.bridge.cv2_to_imgmsg(env_map)
        self.publisher.publish(image_msg)

        self.is_idle = True


def main(args=None):
    rclpy.init(args=args)

    node = GarbageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Garbage Detector Node stopped cleanly')
    except Exception as e:
        node.get_logger().fatal(f'Exception in Garbage Detector Node: {e.with_traceback()}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
