import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')

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

        # --- STATE MANAGEMENT ---
        self.current_gesture = "Unknown"
        self.move_state = "HOVER"  # Default state
        
        # --- CONFIGURATION ---
        self.SPEED = 0.2          # Linear speed in m/s
        self.ROTATE_SPEED = 0.5   # Rotational speed in rad/s

        # Run control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Gesture Controller Started.")
        self.get_logger().info("MAPPING: Fist=Stop, Pointing=Fwd, 2=Back, 3=Up, 4=Down, 5=Rotate")

    def gesture_callback(self, msg):
        """
        Updates the internal state based on the gesture received.
        """
        new_gesture = msg.data
        
        # Only log when the gesture actually changes to reduce spam
        if new_gesture != self.current_gesture:
            self.get_logger().info(f"Gesture detected: {new_gesture}")
            self.current_gesture = new_gesture
            
            # Map Gesture -> Motion State
            if new_gesture == "Fist":
                self.move_state = "HOVER"
            elif new_gesture == "Pointing":  # 1 Finger
                self.move_state = "FORWARD"
            elif new_gesture == "2 Fingers":
                self.move_state = "BACKWARD"
            elif new_gesture == "3 Fingers":
                self.move_state = "UP"
            elif new_gesture == "4 Fingers":
                self.move_state = "DOWN"
            elif new_gesture == "Open Hand" or new_gesture == "5 Fingers":
                self.move_state = "ROTATE"
            else:
                # If gesture is unknown or lost, safer to hover
                self.move_state = "HOVER"

    def control_loop(self):
        """
        Publishes velocity commands based on the current 'move_state'.
        """
        cmd = Twist()

        # 1. FIST = HOVER / STOP
        if self.move_state == "HOVER":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0

        # 2. POINTING = FORWARDS
        elif self.move_state == "FORWARD":
            cmd.linear.x = self.SPEED

        # 3. 2 FINGERS = BACKWARDS
        elif self.move_state == "BACKWARD":
            cmd.linear.x = -self.SPEED

        # 4. 3 FINGERS = UP
        elif self.move_state == "UP":
            cmd.linear.z = self.SPEED

        # 5. 4 FINGERS = DOWN
        elif self.move_state == "DOWN":
            cmd.linear.z = -self.SPEED

        # 6. 5 FINGERS (Open Hand) = ROTATE
        elif self.move_state == "ROTATE":
            cmd.angular.z = self.ROTATE_SPEED  # Rotates Counter-Clockwise

        # Publish the command to the drone
        self.pub_vel.publish(drone_vel)

def main(args=None):
    rclpy.init(args=args)
    node = GestureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: Send a stop command before shutting down
        stop_cmd = Twist()
        node.pub_vel.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()