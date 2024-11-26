# grasp_system_pkg/grasp_system_pkg/robot_controller_node.py

import math
import select
import subprocess
import sys
import time
import threading
import queue
import numpy as np
import cv2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
import rtde_receive
import rtde_control
import open3d as o3d
import tf2_ros
from camera_controller import ImageSubscriber
from grasp_pose_transformer import GraspPoseTransformer
from grasp_detector import GraspDetector
from gripper_controller import GripperController
from myframe import Myframe
import sys
import termios
import tty
import keyboard  # Import the keyboard module


def to_radians(degrees):
    convertedRadians = [np.radians(angle) for angle in degrees]
    return convertedRadians



class Coordinator(Node):
    def __init__(self):
        super().__init__('Robot_Grasp_Coordinator_node')  # Initialize the ROS node with the name 'robot_controller_node'

        # Our services
        self.robot_ip = '192.168.56.101'  # IP address of the robot (replace with your robot's IP)
        self.tcp = [0, 0, 0.241, 0, 0, 0]  # TCP (Tool Center Point) offset
        self.rtde_r, self.rtde_c = None, None
        self.camera_controller = None
        self.grasp_pose_transformer = None
        self.gripper_controller = None
        self.grasp_detector = None

        # Define the home position for the robot (replace with your desiredv robot's home pose)
        home_pose = [-8.5, -93.16, 85.48, -80.68, -90.24, -17]
        self.home_pose_in_rad = to_radians(home_pose)  # Convert degrees to radians

        locationForRelease = [-69, -80.7, 85, -96.6, -90, 9.7]
        self.locationForRelease_in_rad = to_radians(locationForRelease)

        # Initialize TF Buffer and Listener for coordinate frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.buffer = tf2_ros.StaticTransformBroadcaster(self)
        self.base_to_camera_transformation_done = False
        
        self.base_to_camera = np.zeros((4,4), dtype=float)  # Transformation matrix from base to camera
        self.count = 0  # Counter to keep track of failed grasp attempts
        self.previous_grasp = None  # To store the previous grasp to avoid repetition
        self.target_pose = None     # To store the current target pose for moving safety

        self.executionType = "Visualize"

        self.initialize_RTDE()
        self.initialize_camera_controller()
        self.initialize_grasp_pose_transformer()
        self.initialize_gripper_controller()
        self.initialize_grasp_detector()

        # Flag to control the iterative grasping loop
        self.iterative = True

        # Start the iterative grasping loop in a separate thread
        self.grasp_thread = threading.Thread(target=self.run_grasp_loop)
        self.grasp_thread.start()

        # Start the key press listener in a separate thread for graceful shutdown
        self.key_listener_thread = threading.Thread(target=self.listen_for_exit_key)
        self.key_listener_thread.start()

        self.get_logger().info('Robot Grasp Coordinator node initialized and iterative grasping started.\n')


#################################################################


    def ask_for_grasp_action_type(self):

        # Ask if the user wants to execute the grasp or just visualize
        action = input("Press 'e' to execute the grasp or 'v' to visualize: ").strip().lower()
        if action == "e":
            self.executionType = "Execute"
        elif action == "v":
            self.executionType = "Visualize"
        else:
            print("Invalid input! Please press 'e' to execute or 'v' to visualize.")
            self.ask_for_grasp_action_type()  # Ask again if input is invalid


##################################################################

    def run_grasp_loop(self):
        """
        Runs the iterative grasping workflow:
        1. Wait for the next image pair.
        2. Execute grasp with the received images.
        3. Repeat until no grasps are detected or an error occurs.
        """
        self.move_to_home()  # Move to home position at the start

        self.ask_for_grasp_action_type()

        while self.iterative:
            try:

                self.get_logger().info('\n\n#################### New grasp cycle ... #######################\n')

                self.get_logger().info('Waiting for the next image pair for grasping...\n')
                time.sleep(1)  # Wait for a second before checking for images

                color_image, depth_image = self.camera_controller.get_current_images()

                # Check if both color and depth images are available
                if color_image is None or depth_image is None:
                    self.get_logger().info('No valid image pair received, retrying...\n')
                    continue  # Skip to the next iteration if images are not available

                self.get_logger().info('Image pair received. Executing grasp...\n')

                # Execute the grasp with the current images
                grasp_success = self.execute_grasp(color_image, depth_image)

                if not grasp_success:
                    self.get_logger().info('Grasp execution failed or no grasp detected.\n')
                    self.count += 1  # Increment the failure count
                    if self.count > 20:
                        self.get_logger().info('Maximum grasp attempts reached. Terminating iterative loop.\n')
                        self.iterative = False  # Stop the loop after too many failures
                else:
                    self.count = 0  # Reset the failure count on success
                    self.get_logger().info('\n\n#################### This Grasp executed successfully ... #######################\n')


            except Exception as e:
                self.get_logger().error(f'Unexpected error in grasp loop: {e}\n')
                self.iterative = False  # Stop the loop on unexpected errors


###################################################################

    def execute_grasp(self, color_image, depth_image):
        """
        Executes a single grasp cycle with the provided color and depth images.

        Args:
            color_image (numpy.ndarray): The latest color image.
            depth_image (numpy.ndarray): The latest depth image.

        Returns:
            bool: True if grasp was executed successfully, False otherwise.
        """
        try:
            self.get_logger().info('Detecting grasps...\n')
            selected_grasp = self.grasp_detector.detect_grasps(color_image, depth_image)  # Detect possible grasps and return one

            if selected_grasp is not None:
                if selected_grasp == self.previous_grasp:
                    self.get_logger().info('Same grasp as previous detected. Skipping to avoid repetition.\n')
                    return False
                self.previous_grasp = selected_grasp  # Update the previous grasp
            else:
                self.get_logger().warn('No valid grasp detected.\n')
                return False

            self.target_pose = self.grasp_pose_transformer.trasform_graspPose_to_Base(selected_grasp)
            self.get_logger().info(f'\n\nTarget grasp pose is: {self.target_pose} \n')

            if self.target_pose is None:
                return False
            
            if self.executionType == "Execute":

                self.move_to_grasp()  

                self.pick_object()

                self.move_to_release_position()

                self.release_object()

                self.move_to_home()


            return True

        except Exception as e:
            self.get_logger().error(f'Error during grasp execution: {e}\n')
            return False



###################################################################

    def move_to_grasp(self):

        try:
            self.get_logger().info('Commanding robot to move to the grasp pose.\n')

            self.move_just_above_target_pose(0.2) # Move 20 cm above the target in the z-axis
            self.rtde_c.moveL(self.target_pose, 0.08, 0.05)  # Move down to the target pose

            # Implement a waiting mechanism to confirm movement completion
            while True:
                current_pose = self.rtde_r.getActualTCPPose()  # Get the current pose of the robot
                distance = np.linalg.norm(np.array(current_pose[:3]) - np.array(self.target_pose[:3]))  # Calculate distance to target
                if distance < 0.01:  # If within 1 cm, assume target reached
                    self.get_logger().info('Robot reached the grasp pose.\n')
                    break
                time.sleep(0.5)  # Wait before checking again

        except Exception as e:
            self.get_logger().error(f'Failed to move the robot: {e}\n')


####################################################################

    def move_to_home(self):
        """
        Commands the robot to move to the predefined home position.
        """
        try:
            self.rtde_c.moveJ(self.home_pose_in_rad, 0.3, 0.2)  # Move to home position with specified speed and acceleration
            self.get_logger().info('Command sent to move the robot to the home position.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to move to home position: {e}\n')

        self.get_logger().info('Robot returned to home position successfully.\n')


    def move_to_release_position(self):
        self.get_logger().info('Command sent to move the robot to the release position.\n')

        self.move_just_above_target_pose(0.4) # Move 40 cm above the target in the z-axis
        # Define a specific release location (replace with your desired release location)

        self.rtde_c.moveJ(self.locationForRelease_in_rad, 0.3, 0.2)  # Move to the release location

####################################################################

    def move_just_above_target_pose(self, height):
        # Move above the target pose for safer movement
        try:
            just_above_target_pose = list(self.target_pose)
            just_above_target_pose[2] += height  # Move 'height' cm above the target in the z-axis
            just_above_target_pose = tuple(just_above_target_pose)
            self.rtde_c.moveL(just_above_target_pose, 0.2, 0.2)  # Move above the target pose
            self.get_logger().info('Command sent to move the robot just above the target position.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to move above target position: {e}\n')

########################################################################

    def release_object(self):
        try:
            self.get_logger().info('Releasing the object...\n')
            self.gripper_controller.open_gripper()  # Open the gripper to release the object
        except Exception as e:
            self.get_logger().error(f'Failed to release the object: {e}\n')

########################################################################

    def pick_object(self):
        try:
        # Control the gripper to grasp the object
            self.gripper_controller.open_gripper() # Just to make sure the gripper is open
            self.gripper_controller.close_gripper()
        except Exception as e:
            self.get_logger().error(f'Failed to pick the object: {e}\n')

#########################################################################

    def moveHome_after_grasping(self):
        """
        Moves the robot back to the home position after grasping.

        Returns:
            bool: True if movement was successful, False otherwise.
        """
        try:
            self.get_logger().info('Commanding robot to move back to the home position after grasping.\n')

            self.move_just_above_target_pose(0.4) # Move 40 cm above the target in the z-axis
            time.sleep(1)  # Wait for 2 seconds before moving back to home

            self.moveHome()

            self.get_logger().info('Robot returned to home position successfully.\n')

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to move the robot to home position: {e}\n')
            return False
        
###########################################################################

    def listener_callback(self, color_msg, depth_msg):
        """
        Callback function to handle incoming synchronized color and depth images.
        Converts images from ROS messages to OpenCV format.
        """
        try:
            self.color_image = self.br.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')  # Convert color image
            self.depth_image = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')  # Convert depth image
            #self.get_logger().info('Images successfully converted.')
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert images: {e}\n')

############################################################################

    def initialize_RTDE(self):

        # Initialize RTDE interfaces for robot communication
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.get_logger().info('RTDE interfaces initialized successfully.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RTDE interfaces: {e}\n')
            sys.exit(1)  # Exit the program if RTDE interfaces cannot be initialized

        self.rtde_c.setTcp(self.tcp)  # Set the TCP offset for the robot

############################################################################

    def initialize_camera_controller(self):
        # Initialize Camera Controller to handle grasp pose transformations
        try:
            self.camera_controller = ImageSubscriber(self)
            self.get_logger().info(f'Camera Controller initialized successfully.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Camera Controller: {e}\n')
            sys.exit(1)

##############################################################################

    def initialize_grasp_pose_transformer(self):
        # Initialize Grasp Pose Transformer to handle grasp pose transformations
        try:
            self.grasp_pose_transformer = GraspPoseTransformer(self)
            self.get_logger().info(f'Grasp Pose Transformer initialized with robot IP {self.robot_ip}.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GraspPoseTransformer: {e}\n')
            sys.exit(1)

##############################################################################

    def initialize_gripper_controller(self):
        # Initialize GripperController to manage the robot's gripper
        try:
            self.gripper_controller = GripperController(self)
            self.get_logger().info('Gripper Controller initialized successfully.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GripperController: {e}\n')
            sys.exit(1)

##############################################################################

    def initialize_grasp_detector(self):
        # Initialize GraspDetector to detect graspable objects
        try:
            self.grasp_detector = GraspDetector(self)
            self.get_logger().info('Grasp Detector initialized successfully.\n')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Grasp Detector: {e}\n')
            sys.exit(1)


###############################################################################

    def listen_for_exit_key(self):
        """
        Listens for the 'q' key press to gracefully shut down the program.
        Uses termios and sys.stdin for non-blocking key press detection on Linux.
        """
        self.get_logger()('Press "q" to safely shut down the program.\n')

        # Set up terminal settings for non-blocking key press detection
        old_settings = termios.tcgetattr(sys.stdin)  # Save current terminal settings
        try:
            tty.setcbreak(sys.stdin.fileno())  # Set terminal to non-blocking mode

            while self.iterative:
                # Non-blocking check for input using select.select
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)  # Read a single character
                    if key.lower() == 'q':  # If 'q' is pressed, initiate shutdown
                        self.get_logger()('"q" pressed. Initiating shutdown...\n')
                        self.iterative = False  # Stop the iterative loop
                        self.move_to_home()  # Move the robot back to the home position
                        break
        finally:
            # Restore original terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

################################################################################

    def cleanup(self):
        """
        Cleans up resources and safely shuts down the robot controller node.
        """
        try:
            self.iterative = False  # Stop the iterative loop
            if self.grasp_thread.is_alive():
                self.grasp_thread.join()  # Wait for the grasp thread to finish
            if self.key_listener_thread.is_alive():
                self.key_listener_thread.join()  # Wait for the key listener thread to finish

            # Disconnect RTDE interfaces
            self.rtde_r.disconnect()
            self.rtde_c.disconnect()

            self.get_logger().info('Robot Grasp Coordinator node shut down and resources cleaned up.\n\n')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}\n\n')


#################################################################################

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library
    robot_grasp_coordinator_node = Coordinator()  # Create an instance of the Robot Grasp Coordinator node

    try:
        rclpy.spin(robot_grasp_coordinator_node)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        robot_grasp_coordinator_node.get_logger().info('KeyboardInterrupt received. Shutting down Robot Grasp Coordinator node...\n')
    finally:
        robot_grasp_coordinator_node.cleanup()  # Clean up resources before shutting down
        robot_grasp_coordinator_node.destroy_node()  # Destroy the ROS node
        rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main()
