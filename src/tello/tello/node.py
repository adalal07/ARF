import sys
import cv2
import yaml
import numpy
import threading
import time
import math

import ament_index_python
import pdb
import rclpy
from rclpy.node import Node

from djitellopy import Tello
from geometry_msgs.msg import PoseStamped, Twist
# from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
# from tello.constants import TELLO_IP, CONNECTION_TIMEOUT

class TelloNode(Node):
    def __init__(self):
        super().__init__("TelloNode")
        print("node initialized")

        self.declare_parameter('connect_timeout', 10.0)
        self.declare_parameter('tello_ip', 10.0)

        Tello.TELLO_IP = '192.168.10.1'
        Tello.RESPONSE_TIMEOUT = 10 #int(self.connect_timeout)
        self.camera_info_file = ""

        self.camera_info = None
        
        # Check if camera info file was received as argument
        # if len(self.camera_info_file) == 0:
        #     share_directory = ament_index_python.get_package_share_directory('tello')
        #     self.camera_info_file = share_directory + '/ost.yaml'

        # # Read camera info from YAML file
        # with open(self.camera_info_file, 'r') as file:
        #     self.camera_info = yaml.load(file, Loader=yaml.FullLoader)
            # self.node.get_logger().info('Tello: Camera information YAML' + self.camera_info.__str__())

        self.get_logger().info('Tello: Connecting to drone')

        self.tello = Tello()
        # Try to connect and wait for state, but don't fail if state packets aren't received yet
        # The drone may need a moment to start sending state packets
        try:
            self.tello.connect(True)
            self.get_logger().info('Tello: Connected to drone (state packets received)')
        except Exception as e:
            self.get_logger().warning(f'Tello: Connected but state packets not received yet: {e}')
            self.get_logger().info('Tello: Will continue and retry state access...')
            # Still send the command to enter SDK mode
            self.tello.send_control_command("command")

        # Give more time for state packets to arrive (drone may need time to start broadcasting)
        self.get_logger().info('Tello: Waiting for state packets...')
        max_wait = 10  # Wait up to 10 seconds
        wait_interval = 0.5
        state_received = False
        
        for i in range(int(max_wait / wait_interval)):
            time.sleep(wait_interval)
            current_state = self.tello.get_current_state()
            if current_state:
                self.get_logger().info(f'Tello: State packets received! Keys: {list(current_state.keys())}')
                state_received = True
                break
        
        if not state_received:
            self.get_logger().warning('Tello: State packets still not received after waiting')
            self.get_logger().warning('This may indicate:')
            self.get_logger().warning('  1. Container not using host networking (rebuild devcontainer)')
            self.get_logger().warning('  2. Drone not connected to WiFi network')
            self.get_logger().warning('  3. Computer not connected to Tello WiFi network')
            self.get_logger().warning('  4. Firewall blocking UDP port 8890')
            # Continue anyway - some operations may still work
        
        # Create publishers for velocities and accelerations
        self.create_publishers()
        self.create_subscribers()

        # Try to get battery with error handling
        try:
            battery = self.tello.get_battery()
            self.get_logger().info(f"Battery: {battery}%")
            print(f"Battery: {battery}%")
        except Exception as e:
            self.get_logger().error(f"Failed to get battery: {e}")
            current_state = self.tello.get_current_state()
            self.get_logger().error(f"Current state dictionary: {current_state}")
            if not current_state:
                self.get_logger().error("State dictionary is empty - UDP packets on port 8890 are not reaching the container")
                self.get_logger().error("Please verify:")
                self.get_logger().error("  1. Devcontainer was rebuilt with --network=host")
                self.get_logger().error("  2. You are connected to the Tello's WiFi network")
                self.get_logger().error("  3. The drone is powered on and in SDK mode")
            # Don't raise - allow code to continue for testing
            self.get_logger().warning("Continuing despite state access failure...")
        
        # input("Press Enter to start pose capture...")
        # self.publish_velocity_acceleration()
        # self.start_pose_capture()

        input("Preparing for takeoff...")
        self.tello.takeoff()

        input("Move sequence?")
        self.tello.rotate_clockwise(180)

        input("flip?")
        self.tello.flip()

        input("Preparing for landing...")
        self.terminate("Shutting down drone...")
        #self.tello.land()
    
    def create_subscribers(self):
        self.sub_drone_vel = self.create_subscription(Twist, 'drone_velocity', self.drone_velocity_callback, 10)
        return

    def create_publishers(self):
        # self.pub_image_raw = self.create_publisher(Image, 'image_raw', 1)
        self.pub_drone_pose = self.create_publisher(PoseStamped, 'drone_pose', 1)
        self.pub_velocity = self.create_publisher(Twist, 'drone_velocity', 1)
        self.pub_acceleration = self.create_publisher(Twist, 'drone_acceleration', 1)
        return

    def start_video_capture(self, rate=1.0/30.0):
        # Enable tello stream
        self.tello.streamon()
        time.sleep(rate)

        # OpenCV bridge
        self.bridge = CvBridge()

        def video_capture_thread():
            frame_read = None
            pdb.set_trace()
            while True:
                try:
                    frame_read = self.tello.get_frame_read()
                    break
                except djitellopy.tello.TelloException as e:
                    self.get_logger().warn(f'Failed to get frame read: {e}, retrying...')
                    time.sleep(0.5)

            while True:
                # Get frame from drone
                frame = frame_read.frame

                # Publish opencv frame using CV bridge
                msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'bgr8')
                msg.header.frame_id = self.drone
                self.pub_image_raw.publish(msg)

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread
    
    def publish_velocity_acceleration(self, rate=0.1):
        """Publish drone velocities and accelerations as Twist messages"""
        def publish_thread():
            while True:
                try:
                    # Create velocity Twist message
                    velocity_msg = Twist()
                    velocity_msg.linear.x = self.tello.get_speed_x()
                    velocity_msg.linear.y = self.tello.get_speed_y()
                    velocity_msg.linear.z = self.tello.get_speed_z()
                    # Angular velocities (if available, otherwise set to 0)
                    velocity_msg.angular.x = 0.0
                    velocity_msg.angular.y = 0.0
                    velocity_msg.angular.z = 0.0
                    self.pub_velocity.publish(velocity_msg)
                    
                    # Create acceleration Twist message
                    acceleration_msg = Twist()
                    acceleration_msg.linear.x = self.tello.get_acceleration_x()
                    acceleration_msg.linear.y = self.tello.get_acceleration_y()
                    acceleration_msg.linear.z = self.tello.get_acceleration_z()
                    # Angular accelerations (if available, otherwise set to 0)
                    acceleration_msg.angular.x = 0.0
                    acceleration_msg.angular.y = 0.0
                    acceleration_msg.angular.z = 0.0
                    self.pub_acceleration.publish(acceleration_msg)
                    
                    time.sleep(rate)
                except Exception as e:
                    self.get_logger().error(f'Error publishing velocity/acceleration: {e}')
                    time.sleep(rate)
        
        thread = threading.Thread(target=publish_thread)
        thread.start()
        return thread
    
    def start_pose_capture(self):
        """Subscribe to velocities and accelerations, calculate and publish position"""
        # Initialize position tracking variables
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_time = None
        self.last_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Create subscribers for velocity and acceleration
        self.sub_velocity = self.create_subscription(
            Twist,
            'drone_velocity',
            self.velocity_callback,
            10
        )
        
        self.sub_acceleration = self.create_subscription(
            Twist,
            'drone_acceleration',
            self.acceleration_callback,
            10
        )
        
        def pose_capture_thread():
            """Thread to periodically publish the calculated pose"""
            while True:
                try:
                    # Get orientation from drone
                    roll = self.tello.get_roll()
                    pitch = self.tello.get_pitch()
                    yaw = self.tello.get_yaw()
                    
                    # Create pose message
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'map'
                    
                    # Set position (calculated from velocity integration)
                    pose_msg.pose.position.x = float(self.position['x'])
                    pose_msg.pose.position.y = float(self.position['y'])
                    pose_msg.pose.position.z = float(self.position['z'])
                    
                    # Convert roll, pitch, yaw to quaternion
                    # Using Euler to quaternion conversion
                    cy = math.cos(yaw * 0.5)
                    sy = math.sin(yaw * 0.5)
                    cp = math.cos(pitch * 0.5)
                    sp = math.sin(pitch * 0.5)
                    cr = math.cos(roll * 0.5)
                    sr = math.sin(roll * 0.5)
                    
                    pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
                    pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
                    pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
                    pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
                    
                    self.pub_drone_pose.publish(pose_msg)
                    time.sleep(0.1)
                except Exception as e:
                    self.get_logger().error(f'Error in pose capture: {e}')
                    time.sleep(0.1)

        thread = threading.Thread(target=pose_capture_thread)
        thread.start()
        return thread
    
    def velocity_callback(self, msg):
        """Callback to integrate velocity and update position"""
        current_time = time.time()
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            
            # Integrate velocity to get position change
            # Using trapezoidal integration for better accuracy
            avg_vx = (self.last_velocity['x'] + msg.linear.x) / 2.0
            avg_vy = (self.last_velocity['y'] + msg.linear.y) / 2.0
            avg_vz = (self.last_velocity['z'] + msg.linear.z) / 2.0
            
            self.position['x'] += avg_vx * dt
            self.position['y'] += avg_vy * dt
            self.position['z'] += avg_vz * dt
        
        self.last_time = current_time
        self.last_velocity['x'] = msg.linear.x
        self.last_velocity['y'] = msg.linear.y
        self.last_velocity['z'] = msg.linear.z
    
    def acceleration_callback(self, msg):
        """Callback for acceleration (can be used for more advanced integration if needed)"""
        # Currently using velocity integration, but acceleration can be used
        # for double integration or Kalman filtering in the future
        pass
    
    def terminate(self, error):
        self.get_logger().error(str(error))
        #self.tello.land()
        self.tello.end()

def main(args=None):
    rclpy.init(args=args)

    # node = rclpy.create_node('tello')
    drone = TelloNode()

    rclpy.spin(drone)

    # drone.cb_shutdown()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()