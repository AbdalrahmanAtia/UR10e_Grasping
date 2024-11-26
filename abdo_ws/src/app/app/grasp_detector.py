
# grasp_detector.py

import argparse
import os
import time

import cv2
import torch
import numpy as np
import open3d as o3d
from PIL import Image
import random

# Ensure these modules are available in your PYTHONPATH
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup



# Initialize configuration similar to argparse
parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--max_gripper_width', type=float, default=0.1, help='Maximum gripper width (<=0.1m)')
parser.add_argument('--gripper_height', type=float, default=0.03, help='Gripper height')
parser.add_argument('--top_down_grasp', action='store_true', help='Output top-down grasps.')
parser.add_argument('--debug', action='store_true', help='Enable debug mode')

        

class GraspDetector:
    def __init__(self, coordinator):
        """
        Initializes the GraspDetector with the given configurations.

        Args:
            checkpoint_path (str): Path to the AnyGrasp model checkpoint.
            max_gripper_width (float): Maximum gripper width in meters (default: 0.1m).
            gripper_height (float): Gripper height in meters (default: 0.03m).
            top_down_grasp (bool): Whether to output top-down grasps (default: False).
            debug (bool): Enable debug mode with visualizations (default: True).
        """

        self.coordinator = coordinator  # Just to use the coordinator node for logging

        self.cfgs = parser.parse_args()
        self.cfgs.max_gripper_width = max(0, min(0.1, self.cfgs.max_gripper_width))

        self.anygrasp = AnyGrasp(self.cfgs)
        self.anygrasp.load_net()
        
        self.coordinator.get_logger().info('AnyGrasp model loaded.\n')

        # Camera intrinsics
        self.fx = 642.4262770792711
        self.fy = 642.3461744750167
        self.cx = 647.5434733474444
        self.cy = 373.3602344467871

        # self.fx, self.fy = 927.17, 927.37
        # self.cx, self.cy = 651.32, 349.62

        self.scale = 1000.0


        #original
        # self.xmin, self.xmax = -0.19, 0.12
        # self.ymin, self.ymax = 0.02, 0.15
        # self.zmin, self.zmax = 0.0, 1.0


        self.xmin, self.xmax = -0.25, 0.20
        self.ymin, self.ymax = -.2, 0.4
        self.zmin, self.zmax = -0.1, 1.0

        # self.xmin, self.xmax = -0.26, 0.26  # x-axis limits (half the width of the table)
        # self.ymin, self.ymax = -0.325, 0.325  # y-axis limits (half the length of the table)
        # self.zmin, self.zmax = 0.0, 1.0    # z-axis limits (from the table surface to 1 meter above)

        
        # Workspace limits (in meters)
        #self.lims = [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0]
        self.lims = [self.xmin, self.xmax, self.ymin, self.ymax ,self.zmin, self.zmax]
        

    def detect_grasps(self, color_image, depth_image):
        """
        Processes the color and depth images to generate grasp poses.

        """

        #self.process_and_visualize_point_cloud(color_image, depth_image)


        # x = 345
        # y = 160
        # w = 605
        # h = 330
        # color_image = color_image[y:y + h, x:x + w]
        # depth_image = depth_image[y:y + h, x:x + w]
        #self.process_and_visualize_point_cloud(color_image, depth_image)
        
        # Normalize color image
        colors = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        # Ensure depth image is float32
        depths = depth_image

        # get point cloud
        xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depths / self.scale
        points_x = (xmap - self.cx) / self.fx * points_z
        points_y = (ymap - self.cy) / self.fy * points_z

        # set your workspace to crop point cloud
        mask = (points_z > 0) & (points_z < 1)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)
        #print(points.min(axis=0), points.max(axis=0))

        gg, cloud = self.anygrasp.get_grasp(points, colors, lims=self.lims, apply_object_mask=True, dense_grasp=False, collision_detection=False)

        selected_grasp = None


        if (gg is not None):
            if len(gg) == 0:
                self.coordinator.get_logger().warn('No Grasp detected after collision detection!')                

            else: 
                gg = gg.nms().sort_by_score()
                gg_pick = gg[0:20]
                if gg_pick[0].score > 0.3:
                  selected_grasp = gg_pick[0]
                  self.coordinator.get_logger().info(f'\nSelected Grasp score: {selected_grasp.score}\n')


        # # visualization
        # if self.cfgs.debug:
        #     trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        #     cloud.transform(trans_mat)
        #     grippers = gg.to_open3d_geometry_list()
        #     for gripper in grippers:
        #         gripper.transform(trans_mat)
        #     o3d.visualization.draw_geometries([*grippers, cloud])
        #     o3d.visualization.draw_geometries([grippers[0], cloud])
            #time.sleep(2)

        
        return selected_grasp
    






































            # if self.cfgs.debug:
        #     # Ensure cloud is a valid Open3D PointCloud
        #     if not isinstance(cloud, o3d.geometry.PointCloud):
        #         cloud_o3d = o3d.geometry.PointCloud()
        #         cloud_o3d.points = o3d.utility.Vector3dVector(points)
        #         cloud_o3d.colors = o3d.utility.Vector3dVector(colors)
        #         cloud = cloud_o3d

        #     # Check cloud properties

        #     print("I am heeeere")
        #     print("Number of points in cloud:", len(cloud.points))
        #     print("Cloud bounds:", np.asarray(cloud.points).min(axis=0), np.asarray(cloud.points).max(axis=0))

        #     # Visualize point cloud
        #     if len(cloud.points) > 0:
        #         o3d.visualization.draw_geometries([cloud], "Point Cloud Visualization", width=800, height=600)
        #     else:
        #         print("No points to visualize in the cloud.")



        # gg = gg.nms().sort_by_score()
        # gg_pick = gg[0:20]
        #print(f'Number of grasps detected: {len(gg_pick)}')
        #print(f'Selected Grasp: {gg_pick[0]}')

        # Select a random grasp from the top picks
        #selected_grasp = random.choice(gg_pick)














    def process_and_visualize_point_cloud(self, color_image, depth_image):
        if color_image is not None and depth_image is not None:

            # Normalize color image for visualization
            color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

            # Create point cloud from depth image
            xmap, ymap = np.meshgrid(np.arange(depth_image.shape[1]), np.arange(depth_image.shape[0]))
            points_z = depth_image / self.scale
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

            # Add key callback to close visualization when 'q' is pressed
            def close_callback(vis):
                self.get_logger().info('Key "q" pressed. Exiting visualization...')
                vis.destroy_window()

            vis.register_key_callback(ord('Q'), close_callback)

            #self.get_logger().info('Visualizing point cloud with Open3D...')
            vis.run()
            vis.destroy_window()






    def generate_point_cloud(self, depth_image):
        """
        Generates a point cloud from the depth image using camera intrinsics.

        Args:
            depth_image (numpy.ndarray): Depth image as a NumPy array (H x W).

        Returns:
            numpy.ndarray: Array of 3D points (N x 3).
        """
        H, W = depth_image.shape
        xmap, ymap = np.meshgrid(np.arange(W), np.arange(H))
        points_z = depth_image / self.scale
        points_x = (xmap - self.cx) / self.fx * points_z
        points_y = (ymap - self.cy) / self.fy * points_z

        # Apply mask for valid depth
        mask = (points_z > 0) & (points_z < 1)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)

        return points











    def transform_grasp_to_base(self, grasp):
        """
        Transforms the grasp pose to the robot's base frame.

        Args:
            grasp (Grasp): The detected grasp pose.

        Returns:
            list: Transformed grasp pose [x, y, z, roll, pitch, yaw].
        """
        # TODO: Implement the transformation logic based on TF or other methods
        # For now, return the grasp pose as is
        return grasp.grasp_pose

    # def visualize_grasps(self, cloud, grasps):
    #     """
    #     Visualizes the point cloud and grasp poses using Open3D.

    #     Args:
    #         cloud (numpy.ndarray): Point cloud array (N x 3).
    #         grasps (GraspGroup): Group of detected grasps.
    #     """
    #     trans_mat = np.array([[1,0,0,0],
    #                           [0,1,0,0],
    #                           [0,0,-1,0],
    #                           [0,0,0,1]])
    #     cloud_o3d = o3d.geometry.PointCloud()
    #     cloud_o3d.points = o3d.utility.Vector3dVector(cloud.points)
    #     cloud_o3d.colors = o3d.utility.Vector3dVector(cloud.colors)
    #     cloud_o3d.transform(trans_mat)

    #     grippers = grasps.to_open3d_geometry_list()
    #     for gripper in grippers:
    #         gripper.transform(trans_mat)

    #     o3d.visualization.draw_geometries([*grippers, cloud_o3d])
    #     # Optionally, visualize only the first grasp
    #     # o3d.visualization.draw_geometries([grippers[0], cloud_o3d])








































# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose
# from cv_bridge import CvBridge
# import cv2
# import threading
# import numpy as np
# import random

# from camera_controller import CameraController

# class GraspDetector(Node):
#     def __init__(self):
#         super().__init__('grasp_detector')

#         # Initialize CameraController
#         self.camera_controller = CameraController(self)

#         # Publisher for grasp poses
#         self.grasp_pose_publisher = self.create_publisher(Pose, '/grasp_detector/grasp_pose', 10)

#         # Timer to trigger grasp detection periodically (e.g., every 1 second)
#         self.timer_period = 1.0  # seconds
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)

#         self.get_logger().info('GraspDetector initialized and timer started.')

#     def timer_callback(self):
#         """Periodic callback to detect and publish a grasp pose."""
#         # Retrieve the latest images from CameraController
#         color_image = self.camera_controller.get_color_image()
#         depth_image = self.camera_controller.get_depth_image()

#         if color_image is None or depth_image is None:
#             self.get_logger().warn('Waiting for color and depth images...')
#             return

#         self.get_logger().info('Received color and depth images. Processing for grasp detection.')

#         # Placeholder for grasp detection logic
#         translation, rotation_matrix = self.generate_grasp_pose(color_image, depth_image)

#         if translation is None or rotation_matrix is None:
#             self.get_logger().warn('No grasp pose detected.')
#             return

#         # Convert rotation matrix to quaternion
#         #quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
#         quaternion = [
#         0.2594052377364779,
#         0.6452711484893212,
#         0.4281667286508839,
#         0.5770678643266786
#     ]
#         # Create Pose message
#         pose = Pose()
#         pose.position.x = translation[0]
#         pose.position.y = translation[1]
#         pose.position.z = translation[2]
#         pose.orientation.x = quaternion[0]
#         pose.orientation.y = quaternion[1]
#         pose.orientation.z = quaternion[2]
#         pose.orientation.w = quaternion[3]

#         self.get_logger().info(f'Publishing Grasp Pose: Position={translation}, Orientation={quaternion}')

#         # Publish the grasp pose
#         self.grasp_pose_publisher.publish(pose)

#     def generate_grasp_pose(self, color_image, depth_image):
#         """
#         A method for generating a single grasp pose.
#         Need to replace this method with actual grasp detection logic using AnyGrasp SDK.
#         Returns:
#             translation (np.ndarray): [x, y, z] in meters
#             rotation_matrix (np.ndarray): 3x3 rotation matrix
#         """
#         # For demonstration, we'll create a dummy grasp pose
#         height, width = color_image.shape[:2]

#         # Generate random pixel coordinates
#         x_pixel = random.randint(0, width - 1)
#         y_pixel = random.randint(0, height - 1)

#         # Retrieve depth at the pixel
#         depth = depth_image[y_pixel, x_pixel]

#         if depth == 0:
#             self.get_logger().warn(f'Depth at ({x_pixel}, {y_pixel}) is zero. Skipping this grasp.')
#             return None, None

#         # Convert pixel coordinates to real-world coordinates (simple pinhole model)
#         fx = 600  # Focal length in x (replace with actual)
#         fy = 600  # Focal length in y (replace with actual)
#         cx = width / 2
#         cy = height / 2

#         z = depth / 1000.0  # Convert mm to meters if depth is in mm
#         x = (x_pixel - cx) * z / fx
#         y = (y_pixel - cy) * z / fy

#         translation = np.array([x, y, z])

#         # Define a dummy rotation matrix (identity)
#         rotation_matrix = np.eye(3)

#         return translation, rotation_matrix

#     # def rotation_matrix_to_quaternion(self, rotation_matrix):
#     #     """
#     #     Convert a 3x3 rotation matrix to a quaternion.
#     #     Args:
#     #         rotation_matrix (np.ndarray): 3x3 rotation matrix
#     #     Returns:
#     #         tuple: (x, y, z, w) quaternion
#     #     """
#     #     # Convert the rotation matrix to a 4x4 homogeneous matrix
#     #     homogeneous_matrix = np.eye(4)
#     #     homogeneous_matrix[:3, :3] = rotation_matrix

#     #     # Convert to quaternion
#     #     quaternion = quaternion_from_matrix(homogeneous_matrix)

#     #     return quaternion

#     # def quaternion_to_euler(self, q):
#     #     """Convert quaternion to Euler angles (roll, pitch, yaw)."""
#     #     x, y, z, w = q.x, q.y, q.z, q.w

#     #     # Using tf_transformations for conversion
#     #     from tf_transformations import euler_from_quaternion
#     #     roll, pitch, yaw = euler_from_quaternion([x, y, z, w])

#     #     return [roll, pitch, yaw]

# def main(args=None):
#     rclpy.init(args=args)
#     grasp_detector = GraspDetector()

#     try:
#         rclpy.spin(grasp_detector)
#     except KeyboardInterrupt:
#         grasp_detector.get_logger().info('Shutting down GraspDetector node.')
#     finally:
#         grasp_detector.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
