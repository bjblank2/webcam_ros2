#!/usr/bin/env python3

"""
ROS2 Webcam Node
This node publishes camera image messages from webcams.
Supports multiple cameras and can publish to different topics.
"""

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import time


class Ros2CamNode(Node):
    """
    ROS2 node for publishing camera images from webcams.
    """

    def __init__(self):
        super().__init__('webcam_ros2_node')

        # Parameters
        self.declare_parameter('camera_ids', [0])  # List of camera device IDs
        self.declare_parameter('frame_rate', 30.0)  # FPS
        self.declare_parameter('camera_names', ['camera'])  # Topic names for each camera
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('publish_compressed', True)  # Publish compressed images
        self.declare_parameter('compression_format', 'jpeg')  # 'jpeg' or 'png'
        self.declare_parameter('jpeg_quality', 80)  # JPEG quality (1-100)

        # Handle string-to-number/list conversion from launch files
        camera_ids_val = self.get_parameter('camera_ids').value
        if isinstance(camera_ids_val, str):
            # Try to evaluate string as Python list/array
            import ast
            try:
                camera_ids = ast.literal_eval(camera_ids_val)
            except:
                camera_ids = [int(camera_ids_val)]
        else:
            camera_ids = camera_ids_val or [0]
            
        frame_rate_val = self.get_parameter('frame_rate').value
        self.frame_rate = float(frame_rate_val) if frame_rate_val is not None else 30.0
        
        camera_names_val = self.get_parameter('camera_names').value
        if isinstance(camera_names_val, str):
            import ast
            try:
                camera_names = ast.literal_eval(camera_names_val)
            except:
                camera_names = [camera_names_val]
        else:
            camera_names = camera_names_val or ['camera']
            
        width_val = self.get_parameter('width').value
        self.width = int(width_val) if width_val is not None else 640
        
        height_val = self.get_parameter('height').value
        self.height = int(height_val) if height_val is not None else 480
        
        publish_compressed_val = self.get_parameter('publish_compressed').value
        self.publish_compressed = bool(publish_compressed_val) if publish_compressed_val is not None else True
        
        compression_format_val = self.get_parameter('compression_format').value
        self.compression_format = str(compression_format_val) if compression_format_val is not None else 'jpeg'
        
        jpeg_quality_val = self.get_parameter('jpeg_quality').value
        self.jpeg_quality = int(jpeg_quality_val) if jpeg_quality_val is not None else 80

        # Ensure camera_ids is a list
        if not isinstance(camera_ids, list):
            camera_ids = [camera_ids]
        if not isinstance(camera_names, list):
            camera_names = [camera_names]

        # Ensure we have a name for each camera
        while len(camera_names) < len(camera_ids):
            camera_names.append(f'camera_{len(camera_names)}')

        self.camera_ids = camera_ids
        self.camera_names = camera_names

        # CV bridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Dictionary to store camera objects, publishers, and threads
        self.cameras = {}
        self.camera_locks = {}
        self.image_publishers = {}
        self.compressed_image_publishers = {}
        self.camera_threads = {}
        self.running = True

        # Initialize cameras
        self.init_cameras()

        # Timer for checking camera status
        self.status_timer = self.create_timer(5.0, self.check_camera_status)

        self.get_logger().info(f'Camera Node initialized with {len(camera_ids)} camera(s)')

    def init_cameras(self):
        """Initialize all cameras and start publishing threads."""
        for i, camera_id in enumerate(self.camera_ids):
            camera_name = self.camera_names[i] if i < len(self.camera_names) else f'camera_{i}'
            
            try:
                # Open camera
                cap = cv2.VideoCapture(camera_id)
                
                if not cap.isOpened():
                    self.get_logger().error(
                        f'Failed to open camera {camera_id} ({camera_name})'
                    )
                    continue

                # Set camera properties
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

                # Store camera object
                self.cameras[camera_name] = cap
                self.camera_locks[camera_name] = threading.Lock()

                # Create publishers
                if self.publish_compressed:
                    # Compressed image publisher
                    compressed_topic_name = f'/{camera_name}/image_raw/compressed'
                    self.compressed_image_publishers[camera_name] = self.create_publisher(
                        CompressedImage,
                        compressed_topic_name,
                        10
                    )
                    self.get_logger().info(
                        f'Camera {camera_id} ({camera_name}) will publish compressed images to {compressed_topic_name}'
                    )
                else:
                    # Raw image publisher
                    topic_name = f'/{camera_name}/image_raw'
                    self.image_publishers[camera_name] = self.create_publisher(
                        Image,
                        topic_name,
                        10
                    )
                    self.get_logger().info(
                        f'Camera {camera_id} ({camera_name}) will publish raw images to {topic_name}'
                    )

                # Start publishing thread
                thread = threading.Thread(
                    target=self.publish_camera_loop,
                    args=(camera_name,),
                    daemon=True
                )
                thread.start()
                self.camera_threads[camera_name] = thread

            except Exception as e:
                self.get_logger().error(
                    f'Error initializing camera {camera_id} ({camera_name}): {e}'
                )

    def publish_camera_loop(self, camera_name):
        """Publishing loop for a single camera (runs in separate thread)."""
        if camera_name not in self.cameras:
            return

        frame_time = 1.0 / self.frame_rate if self.frame_rate > 0 else 0.033

        while self.running:
            try:
                # Re-fetch the current camera handle (and hold its lock) on every
                # iteration — check_camera_status() may release() the old handle
                # and swap in a new VideoCapture on reconnect; reading a captured
                # reference from thread start would silently read a dead camera
                # forever after any transient disconnect.
                lock = self.camera_locks[camera_name]
                with lock:
                    cap = self.cameras[camera_name]
                    ret, frame = cap.read()

                if not ret:
                    self.get_logger().warn(
                        f'Failed to read frame from camera {camera_name}'
                    )
                    time.sleep(frame_time)
                    continue

                # Get current timestamp
                timestamp = self.get_clock().now().to_msg()
                
                try:
                    if self.publish_compressed:
                        # Publish compressed image
                        if camera_name in self.compressed_image_publishers:
                            compressed_msg = CompressedImage()
                            compressed_msg.header.stamp = timestamp
                            compressed_msg.header.frame_id = camera_name
                            
                            # Compress image directly (OpenCV uses BGR, which is preserved in compressed format)
                            if self.compression_format.lower() == 'jpeg':
                                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                                result, encimg = cv2.imencode('.jpg', frame, encode_param)
                                compressed_msg.format = 'jpeg'
                            elif self.compression_format.lower() == 'png':
                                encode_param = [int(cv2.IMWRITE_PNG_COMPRESSION), 9]
                                result, encimg = cv2.imencode('.png', frame, encode_param)
                                compressed_msg.format = 'png'
                            else:
                                # Default to JPEG
                                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                                result, encimg = cv2.imencode('.jpg', frame, encode_param)
                                compressed_msg.format = 'jpeg'
                            
                            if result:
                                compressed_msg.data = encimg.tobytes()
                                self.compressed_image_publishers[camera_name].publish(compressed_msg)
                            else:
                                self.get_logger().warn(
                                    f'Failed to compress image from camera {camera_name}'
                                )
                    else:
                        # Publish raw image
                        if camera_name in self.image_publishers:
                            # Convert BGR to RGB (OpenCV uses BGR, ROS typically expects RGB)
                            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            
                            ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, 'rgb8')
                            ros_image.header.stamp = timestamp
                            ros_image.header.frame_id = camera_name
                            
                            self.image_publishers[camera_name].publish(ros_image)
                except Exception as e:
                    self.get_logger().error(
                        f'Error converting/publishing image from {camera_name}: {e}'
                    )

                # Control frame rate
                time.sleep(frame_time)

            except Exception as e:
                self.get_logger().error(
                    f'Error in camera loop for {camera_name}: {e}'
                )
                time.sleep(1.0)

    def check_camera_status(self):
        """Periodically check camera status and attempt to reconnect if needed."""
        for camera_name, cap in list(self.cameras.items()):
            if not cap.isOpened():
                self.get_logger().warn(
                    f'Camera {camera_name} is not open, attempting to reconnect...'
                )
                # Find the camera ID for this camera
                idx = self.camera_names.index(camera_name) if camera_name in self.camera_names else 0
                camera_id = self.camera_ids[idx] if idx < len(self.camera_ids) else 0
                
                try:
                    # Hold the same lock the publish thread uses so we never
                    # release() a handle it's concurrently read()-ing, and so it
                    # never reads through a released handle after the swap below.
                    with self.camera_locks[camera_name]:
                        cap.release()
                        new_cap = cv2.VideoCapture(camera_id)
                        reconnected = new_cap.isOpened()
                        if reconnected:
                            new_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                            new_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                            self.cameras[camera_name] = new_cap
                    if reconnected:
                        self.get_logger().info(f'Camera {camera_name} reconnected')
                    else:
                        self.get_logger().error(f'Failed to reconnect camera {camera_name}')
                except Exception as e:
                    self.get_logger().error(
                        f'Error reconnecting camera {camera_name}: {e}'
                    )

    def on_shutdown(self):
        """Cleanup on shutdown."""
        self.running = False
        
        # Wait for threads to finish
        for camera_name, thread in self.camera_threads.items():
            if thread.is_alive():
                thread.join(timeout=1.0)

        # Release all cameras
        for camera_name, cap in self.cameras.items():
            try:
                cap.release()
                self.get_logger().info(f'Released camera {camera_name}')
            except Exception as e:
                self.get_logger().error(
                    f'Error releasing camera {camera_name}: {e}'
                )


def main(args=None):
    rclpy.init(args=args)

    node = Ros2CamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

