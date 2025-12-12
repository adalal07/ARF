import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist
import time

class ExternalViewPDFollower(Node):
    def __init__(self, drone_frame='aruco_0', target_frame='aruco_4'):
        super().__init__('external_view_follower')

        # --- CONFIGURATION ---
        self.DRONE_FRAME = drone_frame
        self.PERSON_FRAME = target_frame
        self.SAFE_DISTANCE = 1.0 
        
        # --- TUNING (The Art of Control) ---
        
        # Distance (Forward/Back)
        self.Kp_dist = 0.6  # Power to reach target
        self.Kd_dist = 0.1  # Braking power

        # Yaw (Turning)
        self.Kp_yaw  = 1.5
        self.Kd_yaw  = 0.2

        # Altitude (Up/Down)
        self.Kp_alt  = 1.0
        self.Kd_alt  = 0.1
        # ---------------------

        self.publisher_ = self.create_publisher(Twist, '/drone_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State storage for D-term calculations
        self.prev_error_dist = 0.0
        self.prev_error_yaw = 0.0
        self.prev_error_alt = 0.0
        
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info("PD Follower Started.")

    def control_loop(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.DRONE_FRAME, self.PERSON_FRAME, rclpy.time.Time())
        except Exception:
            self.publisher_.publish(Twist()) # Stop if lost
            return

        # 1. Calculate DT (Delta Time)
        # We need exact time difference for accurate Derivative math
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt == 0: return # Prevent division by zero on first run

        # 2. Get Raw Positions
        rel_x = t.transform.translation.x
        rel_y = t.transform.translation.y
        rel_z = t.transform.translation.z

        cmd = Twist()

        # --- CONTROLLER 1: YAW (Turn to Face) ---
        # Error: Angle to target (0 means we are facing them)
        yaw_error = math.atan2(rel_y, rel_x)
        
        # Derivative: (Current Error - Prev Error) / Time
        yaw_derivative = (yaw_error - self.prev_error_yaw) / dt
        
        # PD Equation: Output = (Kp * Error) + (Kd * Derivative)
        cmd.angular.z = (self.Kp_yaw * yaw_error) + (self.Kd_yaw * yaw_derivative)
        
        self.prev_error_yaw = yaw_error


        # --- CONTROLLER 2: DISTANCE (Forward/Back) ---
        current_dist = math.sqrt(rel_x**2 + rel_y**2)
        dist_error = current_dist - self.SAFE_DISTANCE
        
        dist_derivative = (dist_error - self.prev_error_dist) / dt
        
        # Logic: Only drive forward if facing roughly the right way
        if abs(yaw_error) < 0.8:
            cmd.linear.x = (self.Kp_dist * dist_error) + (self.Kd_dist * dist_derivative)
        else:
            cmd.linear.x = 0.0
            
        self.prev_error_dist = dist_error


        # --- CONTROLLER 3: ALTITUDE ---
        alt_error = rel_z # We want relative Z to be 0
        alt_derivative = (alt_error - self.prev_error_alt) / dt
        
        cmd.linear.z = (self.Kp_alt * alt_error) + (self.Kd_alt * alt_derivative)
        
        self.prev_error_alt = alt_error


        # --- SAFETY CLAMPS ---
        # Tello SDK expects velocity roughly between -1.0 and 1.0 (remapped to 100)
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)
        cmd.linear.z = max(min(cmd.linear.z, 0.5), -0.5)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
        cmd.linear.y = 0.0

        self.publisher_.publish(cmd)