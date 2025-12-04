import sys

import rclpy
from rclpy.node import Node

from djitellopy import Tello
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

        self.get_logger().info('Tello: Connecting to drone')

        self.tello = Tello()
        self.tello.connect()

        self.get_logger().info('Tello: Connected to drone')

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