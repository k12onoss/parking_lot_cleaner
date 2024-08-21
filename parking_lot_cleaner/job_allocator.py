import random
import math
import numpy as np
import time
import heapq
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Define the Node class for A* algorithm
class CustomNode:
    def __init__(self, position, g, h):
        self.position = position
        self.g = g  # Cost from start to current node
        self.h = h  # Heuristic cost from current node to goal
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f


class AStar:
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, matrix, node):
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for direction in directions:
            neighbor_position = (node.position[0] + direction[0], node.position[1] + direction[1])
            if 0 <= neighbor_position[0] < len(matrix) and 0 <= neighbor_position[1] < len(matrix[0]):
                if matrix[neighbor_position[0]][neighbor_position[1]] != 2:
                    neighbors.append(neighbor_position)
        return neighbors

    def a_star(self, matrix, start, goal):
        open_list = []
        heapq.heappush(open_list, CustomNode(start, 0, self.heuristic(start, goal)))
        g_score = {start: 0}

        while open_list:
            current_node = heapq.heappop(open_list)
            current_position = current_node.position

            if current_position == goal:
                return current_node.g

            neighbors = self.get_neighbors(matrix, current_node)
            for neighbor_position in neighbors:
                tentative_g_score = g_score[current_position] + 1

                if neighbor_position not in g_score or tentative_g_score < g_score[neighbor_position]:
                    g_score[neighbor_position] = tentative_g_score
                    heapq.heappush(open_list, CustomNode(neighbor_position, tentative_g_score, self.heuristic(neighbor_position, goal)))

        return float('inf')


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def distance_to(self, other, data):
        distance = AStar().a_star(data, (self.x, self.y), (other.x, other.y))
        if distance == float('inf'):
            print(f"No path found between ({self.x}, {self.y}) and ({other.x}, {other.y})")
        
        return distance
    
    def to_tuple(self) -> tuple[float]:
        return (float(self.x),float(self.y))

    def __repr__(self):
        return f"({self.x}, {self.y})"


class Robot:
    def __init__(self, robo_id, speed, dump_capacity, T0, t, data):
        self.robo_id = robo_id
        self.speed = speed
        self.dump_capacity = dump_capacity
        self.current_dump = 0
        self.T0 = T0
        self.t = t
        self.data = data
        self.time_remaining = 1000
        self.last_loc = Point(0, 0)
        self.last_time = 0
        self.tasks = []

    def assign_task(self, task_loc, task_litter, current_time):
        travel_time = self.last_loc.distance_to(task_loc, self.data) / self.speed
        total_time = travel_time + task_litter * self.t
        self.tasks.append(task_loc)
        self.last_loc = task_loc
        self.last_time = current_time + total_time
        self.current_dump += task_litter
        self.time_remaining -= total_time

    def assign1(self, task_loc, task_litter, current_time):
        self.assign_task(task_loc, task_litter, current_time)


class SACostOptimization:
    def __init__(self, rooms_to_be_allocated, centroid, robots, distance_matrix, precedence_matrix, littering, t, T0=100, r=0.9, Ts=1, Lk=10, maxIterate=1000):
        self.rooms_to_be_allocated = rooms_to_be_allocated
        self.centroid = centroid
        self.robots = robots
        self.distance_matrix = distance_matrix
        self.precedence_matrix = precedence_matrix
        self.littering = littering
        self.t = t
        self.T0 = T0  # Initial temperature
        self.r = r  # Cooling rate
        self.Ts = Ts  # Stopping temperature
        self.Lk = Lk  # Number of iterations at each temperature
        self.maxIterate = maxIterate  # Maximum number of iterations
        self.alpha = 5
        self.beta = 5

    def initial_job_allocation(self):
        # print(self.rooms_to_be_allocated)
        for room in self.rooms_to_be_allocated:
            task_loc = Point(*self.centroid[room])
            task_litter = self.littering[room]
            min_time = float('inf')
            nearest_robo = None
            # print(task_loc)
            for robot in self.robots:
                task_time = robot.last_time
                if task_time < min_time:
                    min_time = task_time
                    nearest_robo = robot

            if nearest_robo:
                nearest_robo.assign_task(task_loc, task_litter, current_time=nearest_robo.last_time)
                # print(f"Assigned task {task_loc} to robot {nearest_robo.robo_id}")
            # else:
            #     print(f"No robot could take task {task_loc} due to constraints")
        return [(robot.robo_id, robot.tasks) for robot in self.robots]

    def create_neighbor(self, task_order):
        new_order = list(task_order)
        i, j = random.sample(range(len(task_order)), 2)
        new_order[i], new_order[j] = new_order[j], new_order[i]
        return new_order

    def calculate_cost(self):
        total_cost = 0
        for robot in self.robots:
            robot_cost = 0
            total_litter = 0
            total_time = 0
            previous_task = Point(0, 0)
            for task in robot.tasks:
                if isinstance(task, Point):
                    travel_time = self.distance_matrix[(previous_task.x, previous_task.y)][(task.x, task.y)] / robot.speed
                    cleaning_time = self.littering.get((task.x, task.y), 0) * self.t
                    robot_cost += travel_time + cleaning_time
                    total_litter += self.littering.get((task.x, task.y), 0)
                    
                    total_time += travel_time + cleaning_time
                    previous_task = task
            
            if total_litter > robot.dump_capacity:
                robot_cost += (total_litter - robot.dump_capacity)*self.alpha #dump penalty
            if total_time > robot.T0:
                robot_cost += (total_time - robot.T0)*self.beta #
            total_cost  = max(total_cost, robot_cost)
        return total_cost
    
    def main_run(self):
        initial_order = list(range(len(self.rooms_to_be_allocated)))
        current_solution = self.initial_job_allocation()
        current_cost = self.calculate_cost()
        T = self.T0

        best_cost = current_cost
        best_solution = current_solution

        start_time = time.time()

        while T > self.Ts:
            for _ in range(self.Lk):
                new_order = self.create_neighbor(initial_order)  # Create a new neighbor order
                self.rooms_to_be_allocated = [self.rooms_to_be_allocated[i] for i in new_order]
                # Reset the tasks of each robot to empty
                for robot in self.robots:
                    robot.tasks = []
                    robot.current_dump = 0
                    robot.last_loc = Point(0, 0)
                    robot.time_remaining = self.T0
                    robot.last_time = 0
                
                # Allocate tasks based on the new order
                current_solution = self.initial_job_allocation()
                new_cost = self.calculate_cost()

                delta = new_cost - current_cost
                if delta < 0 or random.random() < math.exp(-delta / T):
                    # current_solution = self.job_allocation(new_order)
                    current_cost = new_cost
                    initial_order = new_order  # Update initial_order to new_order

                if current_cost < best_cost:
                    best_cost = current_cost
                    best_solution = current_solution

            T *= self.r
        end_time = time.time()
        return best_solution, best_cost, end_time-start_time


class JobAllocator(Node):
    def __init__(self) -> None:
        super().__init__('job_allocator')

        self.is_idle = True
        self.bridge = CvBridge()
        self.create_subscription(Image, 'env_map', self.map_cb, 10)

        self.Dumping_area = Point(0, 0)
        self.Charging_Point = Point(0, 10)
        self.speed = 10
        self.num_robots = 1
        self.T0 = 1500
        self.dump_capacity = 10000
        self.t = 1 # Time required to clean a single litter

        self.publisher = self.create_publisher(PoseArray, 'garbage_locations', 10)
    
    def map_cb(self, img_msg):
        data = self.bridge.imgmsg_to_cv2(img_msg)
        if self.is_idle:
            self.allocate_job(data)
    
    def allocate_job(self, data):
        np.save('envmap.npy', data)
        self.get_logger().info('Env Map received: {}'.format(data.shape))
        self.is_idle = False

        rooms_to_be_allocated, centroid, littering = self.find_rooms_with_ones(data, 8)
        jobs_to_be_allocated = [Point(*centroid[room]) for room in rooms_to_be_allocated]
        jobs_to_be_allocated.insert(0, self.Dumping_area)  # Adding the starting point as the 0th job
        jobs_to_be_allocated.append(self.Charging_Point)  # Adding Charging_Point as a point in the list

        distance_matrix = self.calculate_distance_matrix(jobs_to_be_allocated, data)

        # Initialize the precedence matrix
        P = np.zeros((self.num_robots, len(jobs_to_be_allocated), len(jobs_to_be_allocated)))

        robots = [Robot(i, self.speed, self.dump_capacity, self.T0, self.t, data) for i in range(self.num_robots)]

        sa_optimizer = SACostOptimization(rooms_to_be_allocated, centroid, robots, distance_matrix, P, littering, self.t)
        minSolution, minCost, elapsed_time = sa_optimizer.main_run()

        locations = []
        for point in minSolution[0][1]:
            locations.append(point.to_tuple())

        self.get_logger().info(f'{locations}')

        # for i in range(0, len(locations), 8):
        self.publish_locations(locations)

        print("Minimum Solution:", minSolution)
        print("Minimum Cost:", minCost)
        print("Elapsed Time:", elapsed_time)
    
    def find_rooms_with_ones(self, matrix, n):
        M, N = matrix.shape
        
        rooms_to_be_allocated = []
        centroid = {}
        littering = {}

        for start_row in range(0, M - n + 1, n):
            for start_col in range(0, N - n + 1, n):
                cell = matrix[start_row:start_row + n, start_col:start_col + n]
                ones_count = np.sum(cell == 1)
                if ones_count > 0:
                    centroid_coords = (start_row + n // 2, start_col + n // 2)
                    room_id = (start_row // n, start_col // n)
                    rooms_to_be_allocated.append(room_id)
                    centroid[room_id] = centroid_coords
                    littering[room_id] = ones_count

        return rooms_to_be_allocated, centroid, littering

    def calculate_distance_matrix(self, points, data):
        distance_matrix = {}
        for p1 in points:
            distance_matrix[(p1.x, p1.y)] = {}
            for p2 in points:
                if p1 == p2:
                    distance_matrix[(p1.x, p1.y)][(p2.x, p2.y)] = 0
                else:
                    distance_matrix[(p1.x, p1.y)][(p2.x, p2.y)] = AStar().a_star(data, (p1.x, p1.y), (p2.x, p2.y))
        return distance_matrix
    
    def publish_locations(self, locations: list[tuple]):
        poses = []
        for location in locations:
            x, y = location

            pose = Pose()
            pose.position.x = (120 - x) * 8.66 / 240
            pose.position.y = (160 - y) * 11.55 / 320
            pose.position.z = 0.0

            poses.append(pose)

        message = PoseArray()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = poses

        self.publisher.publish(message)

        self.is_idle = True

        self.get_logger().info('locations published: {}'.format(message))


def main(args=None):
    rclpy.init(args=args)

    node = JobAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Job Allocator Node stopped cleanly')
    except Exception as e:
        node.get_logger().fatal(f'Exception in Job Allocator Node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
