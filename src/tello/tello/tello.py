import sys

import rclpy
from rclpy.node import Node

from djitellopy import Tello
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
from tello.constants import TELLO_IP, CONNECTION_TIMEOUT

class TelloNode(Node):
    def __init__(self):
        super("TelloNode")

        self.declare_parameter('connect_timeout', TELLO_IP)
        self.declare_parameter('tello_ip', CONNECTION_TIMEOUT)

        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        self.node.get_logger().info('Tello: Connecting to drone')

        self.tello = Tello()
        self.tello.connect()

        self.node.get_logger().info('Tello: Connected to drone')

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tello')
    drone = TelloNode(node)

    rclpy.spin(node)

    drone.cb_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()