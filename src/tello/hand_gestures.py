import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import mediapipe as mp


class HandGestures(Node):
    def __init__(self):
        super().__init__('hand_gesture_node')

        self.declare_parameter('image_topic', 'camera/image_raw')
        image_topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.pub_gesture = self.create_publisher(String, 'hand_gesture', 10)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.get_logger().info(f"Subscribed to image topic: {image_topic}")
        self.get_logger().info("Publishing gestures on: /hand_gesture")

    # Count fingers ----------------------------------------
    def count_fingers(self, hand_landmarks):
        tips = [4, 8, 12, 16, 20]
        finger_count = 0

        for tip_id in tips[1:]:  # skip thumb for now
            tip_y = hand_landmarks.landmark[tip_id].y
            lower_joint_y = hand_landmarks.landmark[tip_id - 2].y
            if tip_y < lower_joint_y:
                finger_count += 1

        # Thumb
        thumb_tip_x = hand_landmarks.landmark[4].x
        thumb_joint_x = hand_landmarks.landmark[3].x
        if thumb_tip_x > thumb_joint_x:
            finger_count += 1

        return finger_count

    # Classify gesture --------------------------------------
    def classify_gesture(self, finger_count):
        if finger_count == 0: return "Fist"
        if finger_count == 1: return "Pointing"
        if finger_count == 5: return "Open Hand"
        return f"{finger_count} fingers"

    # ROS image callback ------------------------------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        gesture_msg = String()

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand skeleton
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )

                fingers = self.count_fingers(hand_landmarks)
                gesture = self.classify_gesture(fingers)

                gesture_msg.data = gesture
                self.pub_gesture.publish(gesture_msg)

                # overlay on video
                cv2.putText(frame, gesture, (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 0), 2)

        # Show debug window (optional)
        cv2.imshow("Gesture Node", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
