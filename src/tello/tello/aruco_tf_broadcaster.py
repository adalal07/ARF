import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
import tf_transformations


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""
    # tf_transformations expects a 4x4 matrix
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    q = tf_transformations.quaternion_from_matrix(T)
    return q


class ArucoTfBroadcaster(Node):
    """Detect ArUco markers from an image topic and publish their poses as TF frames."""

    def __init__(self):
        super().__init__('aruco_tf_broadcaster')

        # Parameters
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('marker_length', 0.05)  # meters
        self.declare_parameter(
            'camera_matrix',
            [800.0, 0.0, 320.0,
             0.0, 800.0, 320.0,
             0.0, 0.0, 1.0]
        )
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        image_topic = self.get_parameter('image_topic').value
        self.marker_length = float(self.get_parameter('marker_length').value)
        cam_vals = [float(v) for v in self.get_parameter('camera_matrix').value]
        if len(cam_vals) != 9:
            raise ValueError('camera_matrix must have 9 values (row-major 3x3)')
        self.camera_matrix = np.array(cam_vals, dtype=float).reshape(3, 3)
        dist_vals = [float(v) for v in self.get_parameter('dist_coeffs').value]
        if len(dist_vals) not in (4, 5, 8):
            raise ValueError('dist_coeffs must have 4, 5, or 8 values')
        self.dist_coeffs = np.array(dist_vals, dtype=float).reshape(-1, 1)
        self.frame_id = str(self.get_parameter('frame_id').value)

        dict_name = str(self.get_parameter('dictionary').value)
        cv_dict_name = f'DICT_{dict_name}' if not dict_name.startswith('DICT_') else dict_name
        if not hasattr(cv2.aruco, cv_dict_name):
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cv_dict_name))
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Subscribed to image topic: {image_topic}")
        self.get_logger().info(f"Publishing TF for markers relative to frame: {self.frame_id}")
        self.get_logger().info(f"Marker length: {self.marker_length} m, dictionary: {cv_dict_name}")

        half = self.marker_length / 2.0
        self.marker_obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Failed to convert image: {exc}')
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        for marker_corners, marker_id in zip(corners, ids):
            img_points = marker_corners[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                self.marker_obj_points,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue

            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

            t = tvec.reshape(3)

            t_msg = TransformStamped()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = self.frame_id
            t_msg.child_frame_id = f'aruco_{int(marker_id)}'

            t_msg.transform.translation.x = float(t[0])
            t_msg.transform.translation.y = float(t[1])
            t_msg.transform.translation.z = float(t[2])
            t_msg.transform.rotation.x = float(qx)
            t_msg.transform.rotation.y = float(qy)
            t_msg.transform.rotation.z = float(qz)
            t_msg.transform.rotation.w = float(qw)

            self.tf_broadcaster.sendTransform(t_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
