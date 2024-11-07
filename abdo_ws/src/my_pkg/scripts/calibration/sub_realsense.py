import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import open3d as o3d
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Camera intrinsic parameters
        self.fx, self.fy = 642.72, 642.66
        self.cx, self.cy = 647.91, 373.42
        self.scale = 1000.0  # Scale for depth image conversion

        # Set workspace limits for point cloud filtering
        self.xmin, self.xmax = -0.19, 0.12
        self.ymin, self.ymax = 0.02, 0.15
        self.zmin, self.zmax = 0.0, 1.0
        self.lims = [self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax]

        # Subscribers for color and depth images
        self.color_subscriber = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_subscriber = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.sync = ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.listener_callback)

        self.br = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.image_count = 0
        self.save_directory = 'captured_images'

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        self.get_logger().info('ImageSubscriber initialized and synchronized.')

    def listener_callback(self, color_msg, depth_msg):
        self.get_logger().info('Receiving synchronized image frames.')

        # Convert color and depth images using CvBridge
        try:
            self.color_image = self.br.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            self.depth_image = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.get_logger().info('Images successfully converted.')
        except Exception as e:
            self.get_logger().error(f'Failed to process images: {e}')
            return

        # Process and visualize the point cloud within the callback
        self.process_and_visualize_point_cloud()

    def process_and_visualize_point_cloud(self):
        if self.color_image is not None and self.depth_image is not None:
            # Normalize color image for visualization
            color_image_rgb = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

            # Create point cloud from depth image
            xmap, ymap = np.meshgrid(np.arange(self.depth_image.shape[1]), np.arange(self.depth_image.shape[0]))
            points_z = self.depth_image / self.scale
            points_x = (xmap - self.cx) / self.fx * points_z
            points_y = (ymap - self.cy) / self.fy * points_z

            # Filter points based on workspace limits
            mask = (points_z > 0) & (points_z < self.zmax)
            points = np.stack([points_x, points_y, points_z], axis=-1)
            points = points[mask].astype(np.float32)
            colors = color_image_rgb[mask]

            # Create Open3D point cloud
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            cloud.colors = o3d.utility.Vector3dVector(colors)

            # Apply transformation if necessary (for correct orientation)
            trans_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            cloud.transform(trans_mat)

            # Visualize the point cloud and handle key events
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window(window_name='Point Cloud Visualization')

            vis.add_geometry(cloud)

            # Add key callback to save images when 's' is pressed
            def save_callback(vis):
                self.get_logger().info('Key "s" pressed. Saving images...')
                self.save_images()

            vis.register_key_callback(ord('S'), save_callback)

            # Add key callback to close visualization when 'q' is pressed
            def close_callback(vis):
                self.get_logger().info('Key "q" pressed. Exiting visualization...')
                vis.destroy_window()

            vis.register_key_callback(ord('Q'), close_callback)

            self.get_logger().info('Visualizing point cloud with Open3D...')
            vis.run()
            vis.destroy_window()

    def save_images(self):
        if self.color_image is not None and self.depth_image is not None:
            # Determine the next available filenames for color and depth images
            color_image_filename = self._get_next_filename('color_image')
            depth_image_filename = self._get_next_filename('depth_image')

            self.get_logger().info(f'Saving images: {color_image_filename}, {depth_image_filename}')

            # Save color and depth images
            try:
                cv2.imwrite(color_image_filename, self.color_image)
                cv2.imwrite(depth_image_filename, self.depth_image)
                self.get_logger().info(f'Saved RGB image as {color_image_filename}')
                self.get_logger().info(f'Saved depth image as {depth_image_filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to save images: {e}')
        else:
            self.get_logger().warning('Images are not ready for saving.')

    def _get_next_filename(self, prefix):
        """Generate the next available filename with the format: prefix_XX.png"""
        i = 0
        while True:
            filename = os.path.join(self.save_directory, f'{prefix}_{i:02d}.png')
            if not os.path.exists(filename):
                return filename
            i += 1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('Shutting down...')
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import open3d as o3d
# from sensor_msgs_py import point_cloud2
# import os

# class ViewAndCaptureNode(Node):
#     def __init__(self):
#         super().__init__('view_and_capture_node')
#         self.bridge = CvBridge()

#         # Create directory for saving images
#         self.image_dir  = 'captured_images'

#         if not os.path.exists(self.image_dir):
#             os.makedirs(self.image_dir)


#         # Camera parameters
#         self.fx, self.fy = 642.72, 642.66
#         self.cx, self.cy = 647.91, 373.42
#         self.scale = 1000.0  # Scale for depth image (depends on your camera)

#         # Set workspace limits for point cloud filtering
#         self.xmin, self.xmax = -0.19, 0.12
#         self.ymin, self.ymax = 0.02, 0.15
#         self.zmin, self.zmax = 0.0, 1.0
#         self.lims = [self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax]
#         self.image_count = 4  

#         # Subscribers for color image, depth image, and point cloud
#         self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
#         self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
#         self.pointcloud_sub = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.pointcloud_callback, 10)

#         self.color_image = None
#         self.depth_image = None
#         self.point_cloud_received = False

#         self.get_logger().info('ViewAndCaptureNode initialized. Subscribed to color, depth images, and point cloud.')

#     def color_callback(self, msg):
#         try:
#             self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f'Failed to process color image: {e}')

#     def depth_callback(self, msg):
#         try:
#             self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f'Failed to process depth image: {e}')

#     # def display_images(self):
#     #     if self.color_image is not None:
#     #         cv2.imshow('Color Stream', self.color_image)

#     #     if self.depth_image is not None:
#     #         depth_grayscale = cv2.convertScaleAbs(self.depth_image, alpha=0.03)
#     #         cv2.imshow('Depth Stream (Grayscale)', depth_grayscale)

#     def pointcloud_callback(self, msg):
#         self.get_logger().info('Received point cloud')

#         # Convert ROS PointCloud2 to Open3D PointCloud
#         points = []
#         for point in point_cloud2.read_points(msg, skip_nans=True):
#             points.append([point[0], point[1], point[2]])

#         if len(points) == 0:
#             self.get_logger().warning('Empty point cloud received!')
#             return

#         # Convert point cloud to a NumPy array
#         points = np.array(points, dtype=np.float32)

#         # Simulate the use of the color image as color information
#         if self.color_image is not None:
#             colors = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
#             colors = colors.astype(np.float32) / 255.0  # Normalize color values

#         if self.depth_image is not None:
#             xmap, ymap = np.meshgrid(np.arange(self.depth_image.shape[1]), np.arange(self.depth_image.shape[0]))
#             points_z = self.depth_image / self.scale
#             points_x = (xmap - self.cx) / self.fx * points_z
#             points_y = (ymap - self.cy) / self.fy * points_z

#             # Filter the point cloud based on workspace limits
#             mask = (points_z > 0) & (points_z < self.zmax)
#             points = np.stack([points_x, points_y, points_z], axis=-1)
#             points = points[mask].astype(np.float32)
#             colors = colors[mask].astype(np.float32)

#             self.get_logger().info(f'Point cloud workspace limits: {self.lims}')
#             self.get_logger().info(f'Filtered points min: {points.min(axis=0)}, max: {points.max(axis=0)}')

#             # Create Open3D point cloud
#             cloud = o3d.geometry.PointCloud()
#             cloud.points = o3d.utility.Vector3dVector(points)
#             cloud.colors = o3d.utility.Vector3dVector(colors)

#             # Visualize the point cloud
#             self.get_logger().info('Visualizing point cloud with Open3D...')
#             o3d.visualization.draw_geometries([cloud])

#             self.point_cloud_received = True

#         if self.color_image is not None and self.depth_image is not None:
#             self.get_logger().info('Color and depth images are ready for capture.')

#     def save_images(self):
#             color_image_path = os.path.join(self.image_dir, f'color_image_{self.image_count:04d}.png')
#             depth_image_path = os.path.join(self.image_dir, f'depth_image_{self.image_count:04d}.png')
            
            
#             self.image_count += 1
#             # Save color and depth images
#             cv2.imwrite(color_image_path, self.color_image)
#             cv2.imwrite(depth_image_path, self.depth_image)

#             self.get_logger().info(f'Saved color image as {color_image_path}')
#             self.get_logger().info(f'Saved depth image as {depth_image_path}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ViewAndCaptureNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down...')
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()





















# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import os

# # Create a pipeline
# pipeline = rs.pipeline()

# # Configure the pipeline to stream color and depth with identical resolution and format
# config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# # Start streaming
# pipeline.start(config)

# def save_images(color_frame, depth_frame, folder_path):
#     # Convert depth frame to numpy array
#     depth_image = np.asanyarray(depth_frame.get_data())
    
#     # Convert depth image to grayscale for display
#     depth_image_grayscale = cv2.convertScaleAbs(depth_image, alpha=0.03)  # For visualization
    
#     # Ensure depth is saved as 16-bit unsigned integer
#     depth_image_16bit = depth_image.astype(np.uint16)

#     # Color image is saved without changes (no BGR to RGB conversion)
#     color_image = np.asanyarray(color_frame.get_data())

#     # Create the folder if it doesn't exist
#     if not os.path.exists(folder_path):
#         os.makedirs(folder_path)

#     # Save depth image in PNG format (16-bit)
#     depth_image_path = os.path.join(folder_path, 'depth_image.png')
#     color_image_path = os.path.join(folder_path, 'color_image.png')

#     # Save the depth and color images (color is unchanged)
#     cv2.imwrite(depth_image_path, depth_image_16bit)
#     cv2.imwrite(color_image_path, color_image)

#     # Optionally, display the grayscale depth image for visualization
#     depth_display_path = os.path.join(folder_path, 'depth_grayscale.png')
#     cv2.imwrite(depth_display_path, depth_image_grayscale)

# def main():
#     folder_path = "captured_images"
    
#     print("Press 's' to save both RGB and Depth images as 'color_image.png' and 'depth_image.png'. Press 'q' to exit.")

#     try:
#         while True:
#             # Wait for coherent pair of frames: depth and color
#             frames = pipeline.wait_for_frames()
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
            
#             if not depth_frame or not color_frame:
#                 continue

#             # Display color image for reference
#             color_image = np.asanyarray(color_frame.get_data())
#             cv2.imshow('Color Stream', color_image)

#             # Show depth image as grayscale (this does not affect saving)
#             depth_image = np.asanyarray(depth_frame.get_data())
#             depth_grayscale = cv2.convertScaleAbs(depth_image, alpha=0.03)  # Grayscale for display
#             cv2.imshow('Depth Stream (Grayscale)', depth_grayscale)

#             # Check for key presses
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('s'):
#                 # Save both the RGB and Depth images in the required format
#                 save_images(color_frame, depth_frame, folder_path)
#                 print(f"Saved RGB and Depth images as 'color_image.png' and 'depth_image.png'")
#             elif key == ord('q'):
#                 break

#     finally:
#         # Stop streaming
#         pipeline.stop()
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()































# import pyrealsense2 as rs
# import numpy as np
# import cv2

# # Create a pipeline
# pipeline = rs.pipeline()

# # Create a configuration for the pipeline
# config = rs.config()

# # Enable both the RGB and Depth streams at 1280x720 resolution
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # RGB stream
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)   # Depth stream

# # Start streaming
# pipeline.start(config)

# try:
#     while True:
#         # Wait for a coherent pair of frames: depth and color
#         frames = pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()

#         if not depth_frame or not color_frame:
#             continue

#         # Convert depth and color images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())  # Depth in 16-bit format
#         color_image = np.asanyarray(color_frame.get_data())  # RGB image in 8-bit format

#         # Ensure depth image is in 16-bit unsigned integer format
#         depth_image_uint16 = depth_image.astype(np.uint16)

#         # Display the color and depth images for preview (depth as grayscale for visualization)
#         depth_image_display = cv2.convertScaleAbs(depth_image_uint16, alpha=0.03)  # For preview only
#         cv2.imshow('RGB Image', color_image)
#         cv2.imshow('Depth Image (Preview)', depth_image_display)

#         # Press 's' to save both images
#         key = cv2.waitKey(1)
#         if key == ord('s'):
#             # Save color image (RGB in 8-bit format)
#             cv2.imwrite('color_image.png', color_image)

#             # Save depth image (16-bit unsigned integer format)
#             cv2.imwrite('depth_image.png', depth_image_uint16 )

#             print("Images saved: color_image.png and depth_image.png")

#         # Break the loop by pressing 'q'
#         if key == ord('q'):
#             break

# finally:
#     # Stop streaming
#     pipeline.stop()
#     cv2.destroyAllWindows()
