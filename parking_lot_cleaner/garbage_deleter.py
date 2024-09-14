import rclpy

from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.srv import DeleteEntity


class GarbageDeleter(Node):
    def __init__(self) -> None:
        super().__init__('garbage_deleter')

        self.garbage_details: dict[tuple[float,float], str] = {}
        self.create_subscription(String, 'garbage_details', self.save_garbage_details, 10)

        self.create_subscription(PoseStamped, 'delete_garbage', self.delete_garbage, 10)

        self.delete_entity_client = self.create_client(DeleteEntity, 'world/default/remove')
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
    
    def save_garbage_details(self, garbage_detail):
        name, x_pose, y_pose = garbage_detail.data.split(',')
        x_pose = float(x_pose)
        y_pose = float(y_pose)

        self.garbage_details[(x_pose, y_pose)] = name
    
    def delete_garbage(self, pose: PoseStamped):
        x_pose = pose.pose.position.x
        y_pose = pose.pose.position.y

        temp: list = []
        for coord, name in self.garbage_details.items():
            if abs(x_pose - coord[0]) < 0.25 and abs(y_pose - coord[1]) < 0.25:
                self.get_logger().info('Name of the Garbage to be deleted: {}'.format(name))

                request = DeleteEntity.Request()
                request.entity.name = name
                request.entity.type = 2

                future = self.delete_entity_client.call_async(request)
                future.add_done_callback(self.response_callback)
                temp.append(coord)
        
        for coord in temp:
            self.garbage_details.pop(coord)

    def response_callback(self, future: Future):
        try:
            response: DeleteEntity.Response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully deleted entity')
            else:
                self.get_logger().warn(f'Failed to delete entity')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = GarbageDeleter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Garbage Deleter Node stopped cleanly')
    except Exception as e:
        node.get_logger().fatal(f'Exception in Garbage Deleter Node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()