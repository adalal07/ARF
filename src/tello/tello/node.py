import sys
import cv2
import yaml
import numpy
import threading
import time

import ament_index_python
import pdb
import rclpy
from rclpy.node import Node

from djitellopy import Tello
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
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
        self.tello.connect(False)

        self.get_logger().info('Tello: Connected to drone')
        #self.get_logger().info(f"Battery: {self.tello.get_battery()}")

        self.create_publishers()
        self.create_subscribers()
        self.get_logger().info('Tello: Initialized subscribers/publishers')
        
        # input("Preparing for takeoff...")
        # self.tello.takeoff()

        input("Starting video capture...")
        self.start_video_capture()

        input("Preparing for landing...")
        self.terminate("Shutting down drone...")
        #self.tello.land()
    
    def create_subscribers(self):
        return

    def create_publishers(self):
        self.pub_image_raw = self.create_publisher(Image, 'image_raw', 1)
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

if __name__ == 'main':
    main()