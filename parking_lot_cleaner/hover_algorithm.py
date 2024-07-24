import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist

import numpy as np

class KalmanFilter:
    def __init__(self, time_steps, n, m, A, C, measure_noise) -> None:
        self.n = n

        # Time step
        self.dt = time_steps

        # Initial state
        self.x = np.zeros(n)

        # Initial covariance matrix
        self.P = np.eye(n)  # Small initial uncertainty

        # State transition matrix
        self.A = A

        # Observation matrix
        self.C = C

        # Process noise covariance
        # self.Q = np.zeros((n,n))
        self.Q = np.eye(n) * 0.01

        # Measurement noise covariance (from your data)
        # self.R = np.zeros((m,m))
        self.R = np.eye(m) * measure_noise
    
    def compute(self, y):
        # Predict the state
        self.x = np.dot(self.A, self.x)

        # Predict the covariance
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

        # Measurement residual
        y = y - np.dot(self.C, self.x)

        # Residual covariance
        S = np.dot(self.C, np.dot(self.P, self.C.T)) + self.R

        # Kalman gain
        K = np.dot(self.P, np.dot(self.C.T, np.linalg.inv(S)))

        # Update the state
        self.x = self.x + np.dot(K, y)

        # Update the covariance
        I = np.eye(self.n)
        self.P = np.dot(I - np.dot(K, self.C), self.P)

class PIDController:
    def __init__(self, kp, ki, kd, setpoint, anti_windup_limit, filter_coeff, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.dt = dt
        
        # Initialize terms for PID control
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Anti-windup limit
        self.anti_windup_limit = anti_windup_limit
        
        # Low-pass filter coefficient
        self.filter_coeff = filter_coeff
        self.filtered_error = 0.0

    def compute(self, measurement):
        error = self.setpoint - measurement
        
        # Apply low-pass filter to the error
        self.filtered_error = self.filter_coeff * error + (1 - self.filter_coeff) * self.filtered_error

        # Proportional term
        proportional = self.kp * self.filtered_error

        # Integral term with anti-windup
        self.integral += self.filtered_error * self.dt
        if self.integral > self.anti_windup_limit:
            self.integral = self.anti_windup_limit
        elif self.integral < -self.anti_windup_limit:
            self.integral = -self.anti_windup_limit
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (self.filtered_error - self.previous_error) / self.dt

        # Compute PID output
        output = proportional + integral + derivative

        # Save the current state for the next update
        self.previous_error = self.filtered_error

        return output


class Hover(Node):
    def __init__(self):
        super().__init__('hover_drone')

        altitude_setpoint = 10
        self.time_steps = 0.01

        self.linear_coordinate = (0, 0, 0)
        self.linear_velo = (0,0,0)
        self.angular_coordinate = (0, 0, 0)

        self.angular_velocity_filter = self.get_angular_velocity_filter()
        self.linear_acceleration_filter = self.get_linear_acceleration_filter()
        self.altitude_filter = self.get_altitude_filter()

        self.x_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)
        self.y_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)
        self.z_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=altitude_setpoint, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)
        self.roll_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)
        self.pitch_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)
        self.yaw_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)
        self.alt_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=altitude_setpoint, anti_windup_limit=2, filter_coeff=0.1, dt=self.time_steps)

        self.imu_subscription = self.create_subscription(Imu, 'X3/imu', self.imu_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, 'X3/lidar', self.lidar_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, 'X3/cmd_vel', 10)

        self.x_file = open("plot/x.txt", 'w')
        self.y_file = open("plot/y.txt", 'w')
        self.z_file = open("plot/z.txt", 'w')
        self.vx_file = open("plot/vx.txt", 'w')
        self.vy_file = open("plot/vy.txt", 'w')
        self.vz_file = open("plot/vz.txt", 'w')
        self.ax_file = open("plot/ax.txt", 'w')
        self.ay_file = open("plot/ay.txt", 'w')
        self.az_file = open("plot/az.txt", 'w')
        self.sax_file = open("plot/sax.txt", 'w')
        self.say_file = open("plot/say.txt", 'w')
        self.saz_file = open("plot/saz.txt", 'w')
        self.ax_file = open("plot/ax.txt", 'w')
        self.ay_file = open("plot/ay.txt", 'w')
        self.az_file = open("plot/az.txt", 'w')
        self.pidx_file = open("plot/pidx.txt", 'w')
        self.pidy_file = open("plot/pidy.txt", 'w')
        self.pidz_file = open("plot/pidz.txt", 'w')
    
    def get_angular_velocity_filter(self) -> KalmanFilter:
        A = np.eye(6)
        for i in range(3):
            A[i, i+3] = self.time_steps

        H = np.zeros((3, 6))
        H[0:3, 3:6] = np.eye(3)  # angular velocities
        
        return KalmanFilter(self.time_steps, 6, 3, A, H, 0.000081)

    def get_linear_acceleration_filter(self) -> KalmanFilter:
        A = np.eye(9)
        for i in range(3):
            A[i, i+3] = self.time_steps
            A[i, i+6] = 0.5 * self.time_steps * self.time_steps
            A[i+3, i+6] = self.time_steps

        C = np.zeros((3, 9))
        C[0:3, 6:9] = np.eye(3)  # linear accelerations
        
        return KalmanFilter(self.time_steps, 9, 3, A, C, 0.000441)
    
    def get_altitude_filter(self) -> KalmanFilter:
        A = np.eye(1)

        C = np.eye(1)
        
        return KalmanFilter(self.time_steps, 1, 1, A, C, 0.000441)
        
    def imu_callback(self, sensor_data):
        roll, pitch, yaw = self.angular_coordinate
        roll += sensor_data.angular_velocity.x * self.time_steps
        pitch += sensor_data.angular_velocity.y * self.time_steps
        yaw += sensor_data.angular_velocity.z * self.time_steps
        self.angular_coordinate = (roll, pitch, yaw)

        # x_old, y_old, z_old = self.linear_coordinate
        # vx_old, vy_old, vz_old = self.linear_velo

        # vx_new = vx_old + (-sensor_data.linear_acceleration.x) * self.time_steps
        # vy_new = vy_old + (-sensor_data.linear_acceleration.y) * self.time_steps
        # vz_new = vz_old + (-sensor_data.linear_acceleration.z) * self.time_steps
        # x_new = x_old + vx_old * self.time_steps + 0.5 * (-sensor_data.linear_acceleration.x) * (self.time_steps ** 2)
        # y_new = y_old + vy_old * self.time_steps + 0.5 * (-sensor_data.linear_acceleration.y) * (self.time_steps ** 2)
        # z_new = z_old + vz_old * self.time_steps + 0.5 * (-sensor_data.linear_acceleration.z) * (self.time_steps ** 2)

        # self.linear_coordinate = (x_new, y_new, z_new)
        # self.linear_velo = (vx_new, vy_new, vz_new)

        z1 = np.array([sensor_data.angular_velocity.x, sensor_data.angular_velocity.y, sensor_data.angular_velocity.z])
        z2 = np.array([-sensor_data.linear_acceleration.x, -sensor_data.linear_acceleration.y, -sensor_data.linear_acceleration.z])

        self.angular_velocity_filter.compute(z1)
        self.linear_acceleration_filter.compute(z2)

        # output_yaw = self.yaw_controller.compute(yaw)
        # output_x = self.x_controller.compute(x_new)
        # output_y = self.y_controller.compute(y_new)
        # output_z = self.z_controller.compute(z_new)

        output_x = -0.1 * self.linear_acceleration_filter.x[0]
        output_y = -0.1 * self.linear_acceleration_filter.x[1]
        output_z = -0.1 * self.linear_acceleration_filter.x[2]

        vel_command = Twist()
        # vel_command.angular.z = -output_yaw
        vel_command.linear.x = output_x
        vel_command.linear.y = output_y
        # vel_command.linear.z = output_z
        # self.vel_publisher.publish(vel_command)

        self.x_file.write("{}\n".format(self.linear_acceleration_filter.x[0]))
        self.y_file.write("{}\n".format(self.linear_acceleration_filter.x[1]))
        self.z_file.write("{}\n".format(self.linear_acceleration_filter.x[2]))
        self.vx_file.write("{}\n".format(self.linear_acceleration_filter.x[3]))
        self.vy_file.write("{}\n".format(self.linear_acceleration_filter.x[4]))
        self.vz_file.write("{}\n".format(self.linear_acceleration_filter.x[5]))
        self.ax_file.write("{}\n".format(self.linear_acceleration_filter.x[6]))
        self.ay_file.write("{}\n".format(self.linear_acceleration_filter.x[7]))
        self.az_file.write("{}\n".format(self.linear_acceleration_filter.x[8]))
        self.sax_file.write("{}\n".format(-sensor_data.linear_acceleration.x))
        self.say_file.write("{}\n".format(-sensor_data.linear_acceleration.y))
        self.saz_file.write("{}\n".format(-sensor_data.linear_acceleration.z))
        self.pidx_file.write("{}\n".format(output_x))
        self.pidy_file.write("{}\n".format(output_y))
        self.pidz_file.write("{}\n".format(output_z))

    def lidar_callback(self, sensor_data):
        curr_altitude = 0
        for i in range(len(sensor_data.ranges)):
            if sensor_data.range_min <= sensor_data.ranges[i] <= sensor_data.range_max:
                curr_altitude = max(curr_altitude, sensor_data.ranges[i])
        
        z = np.array([curr_altitude])
        self.altitude_filter.compute(z)

        output_alt = self.alt_controller.compute(self.altitude_filter.x[0])
        vel_command = Twist()
        vel_command.linear.z = output_alt
        self.vel_publisher.publish(vel_command)


def main(args=None):
    rclpy.init(args=args)
    hover_node = Hover()
    rclpy.spin(hover_node)
    
    hover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
