import argparse
import threading
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    """Simple ROS 2 node that publishes frames from a local camera."""

    def __init__(self, overrides=None):
        super().__init__('camera_publisher')
        overrides = overrides or {}
        self.bridge = CvBridge()

        self.declare_parameter('camera_index', overrides.get('camera_index', 0))
        self.declare_parameter('frame_width', overrides.get('frame_width', 640))
        self.declare_parameter('frame_height', overrides.get('frame_height', 480))
        self.declare_parameter('frame_rate', overrides.get('frame_rate', 30.0))
        self.declare_parameter('frame_id', overrides.get('frame_id', 'camera_link'))
        self.declare_parameter('topic_name', overrides.get('topic_name', 'camera/image_raw'))
        self.declare_parameter(
            'save_directory',
            overrides.get('save_directory', 'data/camera_publisher/published_frames')
        )

        camera_index = int(self.get_parameter('camera_index').value)
        frame_width = int(self.get_parameter('frame_width').value)
        frame_height = int(self.get_parameter('frame_height').value)
        frame_rate = float(self.get_parameter('frame_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.save_dir = Path(str(self.get_parameter('save_directory').value))
        self.save_dir.mkdir(parents=True, exist_ok=True)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # only keep the latest frame
            reliability=ReliabilityPolicy.RELIABLE  # match reliable subscribers
        )
        self.publisher_ = self.create_publisher(Image, self.topic_name, qos_profile)
        self.cap = cv2.VideoCapture("tcp://host.docker.internal:8081", cv2.CAP_FFMPEG)
        # Try to keep capture buffering minimal so we always read the freshest frame.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.capture_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        if frame_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        if frame_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        if frame_rate > 0:
            self.cap.set(cv2.CAP_PROP_FPS, frame_rate)

        if not self.cap.isOpened():
            raise RuntimeError(f'Unable to open camera index {camera_index}')

        timer_period = max(1.0 / frame_rate if frame_rate > 0 else 1.0 / 30.0, 0.001)
        self.timer = self.create_timer(timer_period, self.publish_frame)
        self.get_logger().info(
            f'Publishing camera index {camera_index} to {self.topic_name} at ~{1.0 / timer_period:.1f} Hz'
        )

    def publish_frame(self):
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is None:
            self.get_logger().warning('No frame available to publish')
            return

        now = self.get_clock().now()
        stamp_msg = now.to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)
        self.save_frame(frame, now.nanoseconds)

    def save_frame(self, frame, timestamp_ns: int):
        filename = f'{timestamp_ns}.png'
        out_path = self.save_dir / filename
        try:
            cv2.imwrite(str(out_path), frame)
        except Exception as exc:
            self.get_logger().warning(f'Failed to save frame to {out_path}: {exc}')

    def destroy_node(self):
        self.capture_running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

    def _capture_loop(self):
        """Continuously grab frames so the latest is always available."""
        while self.capture_running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            with self.frame_lock:
                self.latest_frame = frame


def main():
    parser = argparse.ArgumentParser(description='Publish frames from a local camera to a ROS 2 topic.')
    parser.add_argument('--camera-index', type=int, help='Camera index to open')
    parser.add_argument('--width', type=int, help='Frame width override')
    parser.add_argument('--height', type=int, help='Frame height override')
    parser.add_argument('--fps', type=float, help='Target frame rate')
    parser.add_argument('--topic', type=str, help='Topic name to publish to')
    parser.add_argument('--frame-id', type=str, help='TF frame id for outgoing images')
    args, ros_args = parser.parse_known_args()

    overrides = {}
    if args.camera_index is not None:
        overrides['camera_index'] = args.camera_index
    if args.width is not None:
        overrides['frame_width'] = args.width
    if args.height is not None:
        overrides['frame_height'] = args.height
    if args.fps is not None:
        overrides['frame_rate'] = args.fps
    if args.topic is not None:
        overrides['topic_name'] = args.topic
    if args.frame_id is not None:
        overrides['frame_id'] = args.frame_id

    rclpy.init(args=ros_args)

    node = CameraPublisher(overrides=overrides)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

