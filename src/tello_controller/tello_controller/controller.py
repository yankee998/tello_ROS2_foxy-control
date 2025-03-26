import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_error = error
        return output

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/tello/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/tello/odometry', self.odom_cb, 10)
        
        # Initialize current pose
        self.current_pose = Odometry().pose.pose
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0
        
        # Waypoints (x, y, z)
        self.waypoints = [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
            [0.0, 0.0, 1.0]
        ]
        self.current_waypoint = 0
        
        # PID controllers with max output limits
        self.x_pid = PIDController(1.0, 0.05, 0.2, 2.0)
        self.y_pid = PIDController(1.0, 0.05, 0.2, 2.0)
        self.z_pid = PIDController(2.0, 0.1, 0.5, 5.0)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Tello controller initialized!")
        
    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        
    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("Mission complete! Hovering at final position.")
            return
            
        target = self.waypoints[self.current_waypoint]
        error_x = target[0] - self.current_pose.position.x
        error_y = target[1] - self.current_pose.position.y
        error_z = target[2] - self.current_pose.position.z
        
        cmd_vel = Twist()
        cmd_vel.linear.x = self.x_pid.compute(error_x, 0.1)
        cmd_vel.linear.y = self.y_pid.compute(error_y, 0.1)
        cmd_vel.linear.z = self.z_pid.compute(error_z, 0.1)
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        if distance < 0.15:  # Fixed: Added missing colon
            self.current_waypoint += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint-1} at {target}")

def main(args=None):
    rclpy.init(args=args)
    controller = TelloController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()