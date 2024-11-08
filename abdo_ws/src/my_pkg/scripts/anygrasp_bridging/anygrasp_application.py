#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import logging
import random
import threading
import numpy as np
import cv2
import sys
import os
import math

import torch
# ROS2 message types
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ROS2 image message to CV2 image conversion
from cv_bridge import CvBridge, CvBridgeError

# AnyGrasp SDK imports
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup


# RTDE imports
# import rtde_control
# import rtde_receive
# import rtde_io
import tf2_ros

# Configure logging
logging.basicConfig(level=logging.DEBUG)


                    ### A class to deal with camera image capturing using our realsense  ###


class CameraInterface(Node):
    def __init__(self):
        super().__init__('camera_interface')
        self.get_logger().info("Initializing CameraInterface...")
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image,
            '/Realsense_D455/color/image_raw',  ## Subscribe to our rgb image publisher
            self.image_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/Realsense_D455/aligned_depth_to_color/image_raw', ## Subscribe to our aligned depth image publisher
            self.depth_callback,
            10)
        
        self.color_image = None
        self.depth_image = None
        self.image_event = threading.Event()
        self.depth_event = threading.Event()

    def image_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.image_event.set()
            self.get_logger().debug("Received color image.")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_event.set()
            self.get_logger().debug("Received depth image.")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def get_latest_data(self):
        self.get_logger().info("Waiting for color and depth images...")
        self.image_event.wait()
        self.depth_event.wait()
        self.image_event.clear()
        self.depth_event.clear()
        self.get_logger().info("Color and depth images received.")
        return self.color_image, self.depth_image



################################################################################

                        ### A class to deal with Grasp Detection using anyGrasp SDK ###


class GraspDetector(Node):
    def __init__(self, camera_interface):
        super().__init__('grasp_detector')
        self.camera_interface = camera_interface
        self.get_logger().info("Initializing GraspDetector...")
        
        # AnyGrasp configuration parameters (these were taken from the anygrasp detection demo.py)
        self.checkpoint_path = 'log/checkpoint_detection.tar'  # Adjust the path as needed
        self.max_gripper_width = 0.1  # Maximum gripper width in meters
        self.gripper_height = 0.03  # Gripper height in meters
        self.top_down_grasp = True  # Enable top-down grasps
        self.debug = False  # Disable debug mode (set True to enable)
        
        # Initialize AnyGrasp with the above parameters
        self.anygrasp = AnyGrasp(
            checkpoint_path=self.checkpoint_path,
            max_gripper_width=self.max_gripper_width,
            gripper_height=self.gripper_height,
            top_down_grasp=self.top_down_grasp,
            debug=self.debug
        )
        self.anygrasp.load_net()
        self.camera_frame = 'camera_color_optical_frame'

        # Camera intrinsic parameters (taken from our camera calibration results)
        self.fx, self.fy = 642.72, 642.66
        self.cx, self.cy = 647.91, 373.42
        self.scale = 1000.0  # Depth scale factor

    def detect_grasps(self):

        ## These lines of code are taken from angrasp detection demo.py

        self.get_logger().info("Detecting grasps using AnyGrasp SDK...")
        color_image, depth_image = self.camera_interface.get_latest_data()

        # Process images as required by AnyGrasp
        colors = color_image.astype(np.float32) / 255.0
        depths = depth_image.astype(np.float32)

        # Generate point cloud
        points = self.generate_point_cloud(depths)

        # Apply mask to filter valid points
        mask = (points[:, 2] > 0) & (points[:, 2] < 1)
        points = points[mask]
        colors = colors.reshape(-1, 3)[mask]

        if len(points) == 0:
            self.get_logger().warning("No valid points found in the point cloud.")
            return []

        # Set workspace limits (might be adjusted later)
        xmin, xmax = -0.5, 0.5
        ymin, ymax = -0.5, 0.5
        zmin, zmax = 0.0, 1.0
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        # Get grasps from AnyGrasp
        gg, _ = self.anygrasp.get_grasp(
            points, colors, lims=lims,
            apply_object_mask=True,
            dense_grasp=False,
            collision_detection=False
        )

        if gg is None or len(gg.grasp_group_array) == 0:
            self.get_logger().warning("No grasps detected by AnyGrasp.")
            return []

        gg = gg.nms().sort_by_score()
        grasps = gg.grasp_group_array

        self.get_logger().info(f"Detected {len(grasps)} grasps.")
        return grasps

    def get_random_grasp(self):
        grasps = self.detect_grasps()
        if len(grasps) > 0:
            grasp = random.choice(grasps) ## Choose just a random grasp
            self.get_logger().info(f"Selected random grasp with score {grasp[-1]}.")
            # Convert grasp to PoseStamped
            grasp_pose = self.grasp_to_pose(grasp)
            return grasp_pose
        else:
            self.get_logger().error("No grasps detected.")
            raise Exception("No grasps detected.")

    def grasp_to_pose(self, grasp):
        # Grasp is a numpy array with grasp parameters
        # Extract position and rotation matrix
        position = grasp[:3]
        rotation_matrix = grasp[3:12].reshape(3, 3)

        # Convert rotation matrix to quaternion
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def generate_point_cloud(self, depths):
        h, w = depths.shape
        xmap, ymap = np.meshgrid(np.arange(w), np.arange(h))
        points_z = depths / self.scale
        points_x = (xmap - self.cx) / self.fx * points_z
        points_y = (ymap - self.cy) / self.fy * points_z

        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points.reshape(-1, 3)
        return points

    def rotation_matrix_to_quaternion(self, R):
        # Convert rotation matrix to quaternion
        qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        return [qx, qy, qz, qw]



################################################################################3

                        ### A class to control and move our UR10e Robot using moveit ###


class RobotController:
    def __init__(self):
        # Not inheriting from Node since we're using RTDE directly
        self.logger = logging.getLogger('RobotController')
        self.logger.info("Initializing RobotController with RTDE...")
        # Initialize RTDE control interface
        # self.rtde_c = rtde_control.RTDEControlInterface("192.168.56.101")  
        # self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.101")  
        self.home_joint_positions = [0, -math.pi/2, 0, -math.pi/2, 0, 0]
        self.speed = 0.1 # Speed for robot movements
        self.acceleration = 0.05  # Acceleration for robot movements

    def move_to_pose(self, pose_stamped):
        self.logger.info(f"Moving to pose:\n{pose_stamped}")
        # Convert PoseStamped to robot pose in base frame
        robot_pose = self.pose_stamped_to_robot_pose(pose_stamped)
        # Move the robot to the desired pose using RTDE
        success = self.rtde_c.moveL(robot_pose, self.speed, self.acceleration)
        if success:
            self.logger.info("Robot successfully moved to the target pose.")
        else:
            self.logger.error("Failed to move the robot to the target pose.")

    def pose_stamped_to_robot_pose(self, pose_stamped):
        # Extract position and orientation
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z
        qx = pose_stamped.pose.orientation.x
        qy = pose_stamped.pose.orientation.y
        qz = pose_stamped.pose.orientation.z
        qw = pose_stamped.pose.orientation.w

        # Convert quaternion to rotation vector (axis-angle)
        rotation_vector = self.quaternion_to_rotation_vector([qx, qy, qz, qw])

        # Create the robot pose vector [X, Y, Z, Rx, Ry, Rz]
        robot_pose = [x, y, z] + rotation_vector
        return robot_pose

    def quaternion_to_rotation_vector(self, quaternion):
        # Convert quaternion to rotation vector (Rodrigues vector)
        qx, qy, qz, qw = quaternion
        angle = 2 * math.acos(qw)
        s = math.sqrt(1 - qw * qw)
        if s < 1e-8:
            rx = qx
            ry = qy
            rz = qz
        else:
            rx = qx / s * angle
            ry = qy / s * angle
            rz = qz / s * angle
        return [rx, ry, rz]

    def stop_motion(self):
        self.logger.info("Stopping robot motion.")
        self.rtde_c.stopScript()

    def move_home(self):
        self.logger.info("Moving robot to home position.")
        self.rtde_c.moveJ(self.home_joint_positions, self.speed, self.acceleration)



################################################################################3

            ### A class to control and move (open and close) our 2f-140 gripper using robotiq controller ###


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.get_logger().info("Initializing GripperController...")
        # Initialize gripper control interfaces
        self.gripper_command_publisher = self.create_publisher(String, '/gripper_command', 10) #using Robotiq_f2-140 (make sure the command name is correct)

    def open_gripper(self):
        self.get_logger().info("Opening gripper.")
        msg = String()
        msg.data = 'open'
        self.gripper_command_publisher.publish(msg)

    def close_gripper(self):
        self.get_logger().info("Closing gripper.")
        msg = String()
        msg.data = 'close'
        self.gripper_command_publisher.publish(msg)

    def set_grip_force(self, force):
        self.get_logger().info(f"Setting grip force to {force}.")
        # I need to implement setting grip force if needed
        pass




################################################################################3

                    ### A class to bridge, integrate, and coordinate all the previous services ###



class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.get_logger().info("Initializing Coordinator...")
        self.camera_interface = CameraInterface()
        self.grasp_detector = GraspDetector(self.camera_interface)
        self.robot_controller = RobotController()
        self.gripper_controller = GripperController()
        self.get_logger().info("Coordinator initialized.")

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    def execute_grasp_sequence(self):
        try:
            self.get_logger().info("Starting grasp sequence.")
            # Open gripper before moving
            self.gripper_controller.open_gripper()
            grasp_pose = self.grasp_detector.get_random_grasp()
            # Transform grasp pose to robot base frame
            grasp_pose_base = self.transform_pose_to_base_frame(grasp_pose)
            self.get_logger().info(f"Moving to grasp pose in base frame:\n{grasp_pose_base}")
            self.robot_controller.move_to_pose(grasp_pose_base)
            self.gripper_controller.close_gripper()
            self.get_logger().info("Grasp executed successfully.")
            # Optionally, we might lift the object or perform additional actions
        except Exception as e:
            self.handle_errors(e)

    def transform_pose_to_base_frame(self, pose):
        self.get_logger().info("Transforming pose to robot base frame using TF2.")
        target_frame = 'base'  # Replace with our robot's base frame name
        source_frame = pose.header.frame_id

        try:
            # Wait for the transform to become available
            self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            # Transform the pose
            transformed_pose = self.tf_buffer.transform(pose, target_frame, timeout=rclpy.duration.Duration(seconds=1.0))
            return transformed_pose
        except tf2_ros.LookupException as ex:
            self.get_logger().error(f"Transform lookup failed: {ex}")
            raise
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f"Transform extrapolation failed: {ex}")
            raise
        except Exception as ex:
            self.get_logger().error(f"Failed to transform pose: {ex}")
            raise

    def handle_errors(self, error):
        self.get_logger().error(f"Error occurred: {error}")
        self.robot_controller.stop_motion()
        self.gripper_controller.open_gripper()


################################################################################3

                                     ### Our main unction ##



def main(args=None):
    rclpy.init(args=args)

    # Create the Coordinator node
    coordinator = Coordinator()

    # Spin in a separate thread to handle callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(coordinator)
    executor.add_node(coordinator.camera_interface)
    executor.add_node(coordinator.grasp_detector)
    # The RobotController is no longer a ROS2 Node since it uses RTDE directly
    executor.add_node(coordinator.gripper_controller)

    try:
        coordinator.get_logger().info("Running Coordinator...")
        coordinator.execute_grasp_sequence()
        executor.spin()
    except KeyboardInterrupt:
        coordinator.get_logger().info("Shutting down Coordinator...")
    finally:
        executor.shutdown()
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
