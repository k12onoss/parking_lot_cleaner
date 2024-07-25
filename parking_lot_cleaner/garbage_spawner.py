import os
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from ros_gz_interfaces.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

class GarbageSpawner(Node):
    def __init__(self):
        super().__init__('garbage_spawner')
        self.cli = self.create_client(SpawnEntity, 'world/default/create')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.count = 0
        self.mu = 2  # Mean number of garbage items generated
        self.lambda_interval = 10  # Rate parameter for exponential distribution

        # Initial garbage generation event
        self.timer = None
        self.schedule_next_generation()

    def schedule_next_generation(self):
        # Cancel the previous timer if it exists
        if self.timer is not None:
            self.timer.cancel()
        
        interval = np.random.exponential(scale=self.lambda_interval)
        self.timer = self.create_timer(interval, self.generate_garbage)

    def generate_garbage(self):
        num_garbage = np.random.poisson(self.mu)
        for _ in range(num_garbage):
            self.spawn_garbage()

        # Schedule next garbage generation
        self.schedule_next_generation()

    def spawn_garbage(self):
        pkg_share = get_package_share_directory('parking_lot_cleaner')
        model_path = os.path.join(pkg_share, 'models', 'blue_garbage_bag', 'model.sdf')

        if not os.path.isfile(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            return
        
        x, y = self.random_coordinates()

        self.count += 1

        request = SpawnEntity.Request()
        request.entity_factory.sdf_filename = model_path
        request.entity_factory.name = f"bag_{self.count}"
        request.entity_factory.allow_renaming = False
        request.entity_factory.relative_to = "default"
        request.entity_factory.pose.position.x = x
        request.entity_factory.pose.position.y = y
        request.entity_factory.pose.position.z = 0.0

        self.get_logger().info(f"Spawning garbage bag {self.count} at ({x:.2f}, {y:.2f})")

        future: Future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def random_coordinates(self):
        width, height = 7.0, 6.0
        x = np.random.uniform(-width / 2, width / 2)
        y = np.random.uniform(-height / 2, height / 2)
        return x, y

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Model spawn request sent successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to send model spawn request: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GarbageSpawner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
