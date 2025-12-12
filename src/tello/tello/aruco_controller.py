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
        self.SAFE_DISTANCE = 0.01
        
        # --- TUNING (The Art of Control) ---
        
        # Distance (Forward/Back)
        self.Kp_dist = 0.0#0.9  # Power to reach target
        self.Kd_dist = 0.0# 0.1  # Braking power

        # Lateral (Left/Right)
        self.Kp_y  = 0.1
        self.Kd_y  = 0.2

        # Altitude (Up/Down)
        self.Kp_alt  = 1.0
        self.Kd_alt  = 0.1
        # ---------------------

        self.publisher_ = self.create_publisher(Twist, '/drone_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State storage for D-term calculations
        self.prev_error_dist = 0.0
        self.prev_error_y = 0.0
        self.prev_error_alt = 0.0
        
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info("PD Follower Started.")

    def control_loop(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.PERSON_FRAME, self.DRONE_FRAME, rclpy.time.Time())
        except Exception:
            print("Can't find transform between frames")
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
        #print(f"Curr drone position: x: {rel_x}, y: {rel_y}, z: {rel_z}")

        cmd = Twist()

        # --- CONTROLLER 1: LATERAL (Left/Right) ---
        # Error: Lateral offset from target (0 means we are aligned)
        y_error = rel_y
        #print(f"relx: {rel_x}, rely: {rel_y}, Y error: {y_error}")
        
        # Derivative: (Current Error - Prev Error) / Time
        y_derivative = (y_error - self.prev_error_y) / dt
        
        # PD Equation: Output = (Kp * Error) + (Kd * Derivative)
        cmd.linear.y = (self.Kp_y * y_error) + (self.Kd_y * y_derivative)
        
        self.prev_error_y = y_error


        # --- CONTROLLER 2: DISTANCE (Forward/Back) ---
        current_dist = math.sqrt(rel_x**2 + rel_y**2)
        dist_error = current_dist - self.SAFE_DISTANCE
        
        dist_derivative = (dist_error - self.prev_error_dist) / dt
        
        # Logic: Only drive forward if roughly aligned
        if abs(y_error) < 0.8:
            cmd.linear.x = (self.Kp_dist * dist_error) #+ (self.Kd_dist * dist_derivative)
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
        #cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)
        #cmd.linear.z = max(min(cmd.linear.z, 0.5), -0.5)
        #cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        #print(f"Publishing command: linear.x: {cmd.linear.x}, linear.z: {cmd.linear.z}, angular.z: {cmd.angular.z}")
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ExternalViewPDFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()