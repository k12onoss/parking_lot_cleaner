import os
import rclpy
import random

from rclpy.node import Node
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from ros_gz_interfaces.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

class SpawnModelClient(Node):
    def __init__(self):
        super().__init__('garbage_spawner_client')
        self.cli = self.create_client(SpawnEntity, 'world/default/create')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.count = 0

        self.timer = self.create_timer(10, self.send_request)

    def send_request(self) -> None:
        pkg_line_follower = get_package_share_directory('parking_lot_cleaner')
        model_path = os.path.join(pkg_line_follower, 'models', 'blue_garbage_bag', 'model.sdf')

        if not os.path.isfile(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            return
        
        x = 0
        while -0.2 < x < 0.2:
            x = random.random() * 7 - 3.5
        
        y = 0
        while -0.2 < y < 0.2:
            y = random.random() * 6 - 3
        
        request = SpawnEntity.Request()
        request.entity_factory.sdf_filename = model_path
        request.entity_factory.name = f"bag_{self.count}"
        request.entity_factory.allow_renaming = False
        request.entity_factory.relative_to = "default"
        request.entity_factory.pose.position.x = x
        request.entity_factory.pose.position.y = y
        request.entity_factory.pose.position.z = 0.0

        self.count += 1

        self.get_logger().info(f"{self.count}")

        future: Future =  self.cli.call_async(request)
        # self.get_logger().info(f'Sending model spawn request {self.count}...')

        # Use a callback to handle the response asynchronously
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Model spawn request sent successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to send model spawn request: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SpawnModelClient()

    # Use MultiThreadedExecutor to handle both timer callbacks and service responses
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
