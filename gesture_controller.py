import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
import time

class GestureTestController(Node):
    def __init__(self):
        super().__init__('gesture_test_controller')

        # --- COMMUNICATIONS ---
        # 1. Listen for Gestures
        self.sub_gesture = self.create_subscription(
            String, 
            '/hand_gesture', 
            self.gesture_callback, 
            10
        )

        # 2. Talk to Drone (Movement)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. Talk to Drone (Actions)
        self.pub_flip = self.create_publisher(String, '/flip', 1)
        self.pub_land = self.create_publisher(Empty, '/land', 1)
        self.pub_takeoff = self.create_publisher(Empty, '/takeoff', 1)

        # --- STATE MANAGEMENT ---
        self.current_gesture = "Unknown"
        self.state = "Hover"  # Initial State
        self.last_flip_time = 0.0

        # Run control loop at 10Hz (send commands 10 times a second)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Gesture Test Controller Started.")
        self.get_logger().info("COMMANDS: Pointing=Forward, Fist=Hover, Open Hand=Land, 2 Fingers=Flip")

    def gesture_callback(self, msg):
        """
        Translates raw hand gestures into State Changes.
        """
        new_gesture = msg.data
        
        # Anti-spam: Only log when gesture changes
        if new_gesture != self.current_gesture:
            self.get_logger().info(f"Gesture Detected: {new_gesture}")
            self.current_gesture = new_gesture

        # --- MAPPING LOGIC ---

        # 1. FIST -> START MOVING (Simulate Follow)
        if new_gesture == "Fist":
            if self.state != "MOVING":
                self.get_logger().info("State: MOVING FORWARD")
                self.state = "MOVING"

        # 2. OPEN HAND -> STOP (Hover)
        elif new_gesture == "Open Hand":
            if self.state != "HOVER":
                self.get_logger().info("State: HOVERING")
                self.state = "HOVER"

        # 3. 1 FINGER (Pointing) -> FLIP
        elif new_gesture == "Pointing":
            # Check cooldown (5 seconds) so we don't spam flips
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self.last_flip_time) > 5.0:
                self.get_logger().info("ACTION: DO A BARREL ROLL!")
                self.perform_flip()
                self.last_flip_time = now
            else:
                self.get_logger().warn("Flip is on cooldown...")

        # 4. OPTIONAL: 2 Fingers -> Land (Just in case you need to abort)
        elif new_gesture == "2 Fingers":
             self.get_logger().info("ACTION: LANDING")
             self.pub_land.publish(Empty())

    def perform_flip(self):
        """Publishes the flip command to the driver node"""
        msg = String()
        msg.data = "l"  # 'l' = Left, 'r' = Right, 'f' = Forward
        self.pub_flip.publish(msg)
        # Reset state to Hover so we don't drift after flipping
        self.state = "HOVER"

    def control_loop(self):
        """
        Constantly sends velocity commands based on current State.
        """
        cmd = Twist()

        if self.state == "HOVER":
            # Send 0 velocity to hold position
            cmd.linear.x = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0

        elif self.state == "MOVING":
            # SIMULATED FOLLOW: Move forward slowly
            # Since we removed Vision, we just move forward blindly.
            # BE CAREFUL INDOORS!
            cmd.linear.x = 0.2  # 0.2 m/s Forward
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0

        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GestureTestController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()