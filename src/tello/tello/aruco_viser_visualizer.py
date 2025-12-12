#!/usr/bin/env python3
"""
Real-time ArUco frame visualization using Viser.

This script subscribes to TF transforms for all ArUco markers and visualizes
them in a 3D scene using Viser. Frames are automatically added and updated
as markers are detected.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import threading
import time
import numpy as np
import viser
from typing import Dict
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import cv2


class ArucoViserVisualizer(Node):
    """Visualize ArUco marker frames in real-time using Viser."""

    def __init__(self):
        super().__init__('aruco_viser_visualizer')

        # Parameters
        self.declare_parameter('base_frame', 'camera_link')
        self.declare_parameter('update_rate', 30.0)  # Hz
        self.declare_parameter('frame_length', 0.1)  # meters
        self.declare_parameter('frame_radius', 0.01)  # meters
        self.declare_parameter('server_port', 8080)
        self.declare_parameter('image_topic', 'camera/image_raw')

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.frame_length = float(self.get_parameter('frame_length').value)
        self.frame_radius = float(self.get_parameter('frame_radius').value)
        self.server_port = int(self.get_parameter('server_port').value)
        image_topic = str(self.get_parameter('image_topic').value)

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Track known ArUco frames
        self.known_frames: Dict[str, bool] = {}
        
        # Camera feed setup
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # Subscribe to camera feed
        # Use RELIABLE to ensure we get images, and increase depth for buffering
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # Buffer a few frames
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile
        )
        self.get_logger().info(f"Subscribed to camera feed: {image_topic}")
        self.image_count = 0  # Track received images
        
        # Viser server
        self.server = viser.ViserServer(port=self.server_port)
        self.get_logger().info(f"Viser server started on port {self.server_port}")
        self.get_logger().info(f"Open http://localhost:{self.server_port} in your browser")

        # Start update loop in separate thread
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()

        # Add GUI controls
        self._setup_gui()
        
        # Add camera feed display
        self._setup_camera_feed()
        
        # Add origin/base frame visualization
        self._update_origin_frame()
        
        # Color palette for different ArUco markers
        self.frame_colors = self._generate_color_palette()

    def _setup_gui(self):
        """Set up GUI controls for the visualization."""
        with self.server.gui.add_folder("ArUco Visualization"):
            self.frame_length_slider = self.server.gui.add_slider(
                "Frame Length (m)",
                min=0.01,
                max=0.5,
                step=0.01,
                initial_value=self.frame_length,
            )
            self.frame_radius_slider = self.server.gui.add_slider(
                "Frame Radius (m)",
                min=0.001,
                max=0.05,
                step=0.001,
                initial_value=self.frame_radius,
            )
            self.update_rate_slider = self.server.gui.add_slider(
                "Update Rate (Hz)",
                min=1.0,
                max=60.0,
                step=1.0,
                initial_value=self.update_rate,
            )

            # Callbacks for GUI updates
            @self.frame_length_slider.on_update
            def _(_):
                self.frame_length = self.frame_length_slider.value

            @self.frame_radius_slider.on_update
            def _(_):
                self.frame_radius = self.frame_radius_slider.value

            @self.update_rate_slider.on_update
            def _(_):
                self.update_rate = self.update_rate_slider.value

        # Add info text
        self.info_text = self.server.gui.add_text(
            "Detected Frames: 0",
            initial_value="Detected Frames: 0",
        )
        
        # Add frame list display
        self.frame_list_text = self.server.gui.add_text(
            "Frame List",
            initial_value="No frames detected",
        )
    
    def _setup_camera_feed(self):
        """Set up camera feed display in GUI."""
        # Add image display in GUI
        try:
            # Create a test pattern initially (checkerboard) to verify display works
            # Create as RGB directly (3 channels)
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            # Add a simple test pattern - set RGB values directly
            test_image[::20, :] = [128, 128, 128]  # Horizontal lines (gray)
            test_image[:, ::20] = [128, 128, 128]  # Vertical lines (gray)
            # Add some colored squares to make it more visible
            test_image[100:200, 100:200] = [255, 0, 0]  # Red square
            test_image[300:400, 300:400] = [0, 255, 0]  # Green square
            test_image[200:300, 500:600] = [0, 0, 255]  # Blue square
            
            # add_image signature: image (positional), label (keyword-only)
            self.camera_image_handle = self.server.gui.add_image(
                test_image,  # image (positional argument)
                label="Camera Feed",  # label (keyword argument)
            )
            self.get_logger().info("Camera feed GUI element created")
            # Timer to update camera feed
            self.camera_update_timer = self.create_timer(0.033, self._update_camera_feed)  # ~30 FPS
        except Exception as e:
            # If add_image is not available, log warning and continue without camera feed
            self.get_logger().warning(f"Camera feed display not available: {e}")
            import traceback
            self.get_logger().warning(traceback.format_exc())
            self.camera_image_handle = None
    
    def image_callback(self, msg: Image):
        """Callback for camera image messages."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = cv_image
                self.image_count += 1
                # Log first few images received
                if self.image_count <= 3:
                    self.get_logger().info(f'Received camera image #{self.image_count}: shape={cv_image.shape}, dtype={cv_image.dtype}')
        except Exception as e:
            self.get_logger().warning(f'Failed to convert image: {e}')
            import traceback
            self.get_logger().warning(traceback.format_exc())
    
    def _update_camera_feed(self):
        """Update camera feed in GUI."""
        if self.camera_image_handle is None:
            return
            
        with self.image_lock:
            if self.latest_image is not None:
                try:
                    # Resize if needed for display
                    display_image = self.latest_image.copy()
                    max_height, max_width = 480, 640
                    if display_image.shape[0] > max_height or display_image.shape[1] > max_width:
                        scale = min(max_height / display_image.shape[0], max_width / display_image.shape[1])
                        new_width = int(display_image.shape[1] * scale)
                        new_height = int(display_image.shape[0] * scale)
                        display_image = cv2.resize(display_image, (new_width, new_height))
                    
                    # Ensure image is contiguous array
                    if not display_image.flags['C_CONTIGUOUS']:
                        display_image = np.ascontiguousarray(display_image)
                    
                    # Convert BGR to RGB for display
                    display_image_rgb = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
                    
                    # Ensure uint8 type
                    if display_image_rgb.dtype != np.uint8:
                        display_image_rgb = display_image_rgb.astype(np.uint8)
                    
                    # Ensure values are in valid range [0, 255]
                    display_image_rgb = np.clip(display_image_rgb, 0, 255).astype(np.uint8)
                    
                    # Update the image in GUI
                    # Use the 'image' property (not 'value') to update the displayed image
                    try:
                        self.camera_image_handle.image = display_image_rgb
                        # Log first successful update
                        if not hasattr(self, '_first_update_logged'):
                            self.get_logger().info(f'Successfully updated camera feed: {display_image_rgb.shape}')
                            self._first_update_logged = True
                    except (AttributeError, TypeError) as e:
                        # Fallback: try 'value' property if 'image' doesn't work
                        try:
                            self.camera_image_handle.value = display_image_rgb
                            if not hasattr(self, '_first_update_logged'):
                                self.get_logger().info(f'Updated camera feed using value property: {display_image_rgb.shape}')
                                self._first_update_logged = True
                        except Exception as e2:
                            self.get_logger().warning(f'Failed to update image handle: {e2}')
                            import traceback
                            self.get_logger().warning(traceback.format_exc())
                except Exception as e:
                    self.get_logger().warning(f'Failed to update camera feed: {e}')
                    import traceback
                    self.get_logger().warning(traceback.format_exc())
            else:
                # Log if no image received yet (only once)
                if not hasattr(self, '_no_image_logged'):
                    self.get_logger().info('Waiting for camera images...')
                    self._no_image_logged = True
    
    def _generate_color_palette(self) -> Dict[int, tuple]:
        """Generate a color palette for different ArUco markers."""
        # Generate distinct colors using HSV color space
        colors = {}
        for i in range(51):  # Support up to 51 markers
            hue = int((i * 180 / 51) % 180)  # Vary hue
            saturation = 200 + (i % 3) * 20  # Vary saturation slightly
            value = 200 + (i % 2) * 55  # Vary brightness
            
            # Convert HSV to RGB (for display purposes, we'll use BGR for OpenCV)
            hsv = np.uint8([[[hue, saturation, value]]])
            rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0][0]
            colors[i] = tuple(int(c) for c in rgb)
        
        return colors

    def _quaternion_to_rotation_matrix(self, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        """Convert quaternion to 3x3 rotation matrix."""
        # Normalize quaternion
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm > 0:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

        # Convert to rotation matrix
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        return R

    def _update_frame_visualization(self, frame_name: str, transform):
        """Update or create visualization for a single ArUco frame."""
        # Extract position and orientation
        pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        quat = transform.transform.rotation
        R = self._quaternion_to_rotation_matrix(
            quat.x, quat.y, quat.z, quat.w
        )

        # Extract marker ID from frame name (e.g., "aruco_5" -> 5)
        try:
            marker_id = int(frame_name.split('_')[1])
        except (IndexError, ValueError):
            marker_id = 0
        
        # Get color for this marker
        marker_color = self.frame_colors.get(marker_id, (255, 255, 255))
        
        # Use Viser's add_frame for coordinate frame visualization
        # Viser expects wxyz quaternion format (w, x, y, z)
        self.server.scene.add_frame(
            name=f"/{frame_name}",
            wxyz=(quat.w, quat.x, quat.y, quat.z),  # wxyz format as tuple
            position=tuple(pos),  # position as tuple
        )
        
        # Add colored sphere/box at frame origin to distinguish markers
        marker_size = self.frame_length * 0.1
        marker_pos = pos + R @ np.array([0, 0, marker_size * 2])
        
        # Create a small colored marker to distinguish this frame
        # Convert RGB color to tuple format expected by Viser
        try:
            # Try using add_box for a colored marker
            self.server.scene.add_box(
                name=f"{frame_name}/marker",
                position=tuple(marker_pos),
                half_size=(marker_size, marker_size, marker_size),
                color=marker_color,
            )
        except (AttributeError, TypeError):
            # If add_box doesn't work or color format is wrong, try sphere or skip
            try:
                # Try sphere if available
                self.server.scene.add_sphere(
                    name=f"{frame_name}/marker",
                    position=tuple(marker_pos),
                    radius=marker_size,
                    color=marker_color,
                )
            except (AttributeError, TypeError):
                # Skip marker if neither method works
                pass
        
        # Add text label using GUI (if supported)
        try:
            # Try to add label in GUI panel
            label_name = f"label_{frame_name}"
            label_text = f"{frame_name}\n(ID: {marker_id})"
            # Note: We'll add this to the info display instead
        except:
            pass

    def _update_origin_frame(self):
        """Update or create visualization for the origin/base frame."""
        # Origin frame is at (0, 0, 0) with identity rotation
        origin_name = f"origin_{self.base_frame}"
        
        # Identity quaternion (w=1, x=0, y=0, z=0)
        self.server.scene.add_frame(
            name=f"/{origin_name}",
            wxyz=(1.0, 0.0, 0.0, 0.0),  # Identity quaternion
            position=(0.0, 0.0, 0.0),  # Origin position
        )
        
        # Note: Frame names are typically displayed in Viser's UI automatically
        # Text labels are skipped to avoid API compatibility issues

    def _discover_aruco_frames(self) -> list:
        """Discover all available ArUco frames from TF tree."""
        aruco_frames = []
        
        # Try to lookup ArUco frame names directly
        # Check for frames aruco_0 through aruco_50 (common range)
        # This is more reliable than parsing TF tree strings
        for i in range(51):
            frame_name = f"aruco_{i}"
            try:
                # Try to get transform - if it exists, add to list
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                if frame_name not in aruco_frames:
                    aruco_frames.append(frame_name)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                # Frame doesn't exist or not available, skip
                pass
        
        return aruco_frames

    def _update_loop(self):
        """Main update loop running in separate thread."""
        while self.running:
            try:
                # Discover ArUco frames
                aruco_frames = self._discover_aruco_frames()
                
                # Update info text with frame list
                frame_list = ", ".join(aruco_frames) if aruco_frames else "None"
                self.info_text.value = f"Detected Frames: {len(aruco_frames)}"
                
                # Update frame list with marker IDs
                if aruco_frames:
                    frame_details = []
                    for frame_name in sorted(aruco_frames):
                        try:
                            marker_id = int(frame_name.split('_')[1])
                            frame_details.append(f"{frame_name} (ID: {marker_id})")
                        except:
                            frame_details.append(frame_name)
                    self.frame_list_text.value = "\n".join(frame_details)
                else:
                    self.frame_list_text.value = "No frames detected"

                # Update origin frame visualization
                self._update_origin_frame()

                # Update visualization for each frame
                for frame_name in aruco_frames:
                    try:
                        # Get transform
                        transform = self.tf_buffer.lookup_transform(
                            self.base_frame,
                            frame_name,
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        
                        # Update visualization
                        self._update_frame_visualization(frame_name, transform)
                        self.known_frames[frame_name] = True

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException) as e:
                        # Frame not available, but keep it in known_frames
                        # so we don't remove it immediately
                        pass
                    except Exception as e:
                        self.get_logger().warning(f"Error updating frame {frame_name}: {e}")

                # Remove frames that are no longer detected
                frames_to_remove = []
                for frame_name in self.known_frames.keys():
                    if frame_name not in aruco_frames:
                        frames_to_remove.append(frame_name)
                
                for frame_name in frames_to_remove:
                    # Remove visualization elements
                    try:
                        self.server.scene.remove(f"/{frame_name}")
                        self.server.scene.remove(f"{frame_name}/marker")
                    except Exception:
                        pass
                    del self.known_frames[frame_name]

                # Sleep based on update rate
                time.sleep(1.0 / self.update_rate)

            except Exception as e:
                self.get_logger().error(f"Error in update loop: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoViserVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
