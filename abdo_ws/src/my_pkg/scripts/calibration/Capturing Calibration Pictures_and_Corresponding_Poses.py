from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


import math
import time
from scipy.spatial.transform import Rotation as R
import json  # Added to save JSON data
from myframe import Myframe
import pyrealsense2 as rs  # For RealSense camera support
import cv2  # For saving the captured image
import numpy as np
import os  # For handling directories
from scipy.spatial.transform import Rotation as R
import threading
import keyboard  # For capturing keyboard events
import sys




# Replace with your robot's IP address
robot_ip = "192.168.56.101"

# List of trajectories with positions in degrees
trajectories = {
    "traj0": [{"positions": [-35.4, -120.8, 115.5, -100, -79.9, -31.6]}],
    "traj1": [{"positions": [-26, -91, 84, -81, -87, -22]}],
    "traj2": [{"positions": [-26, -91, 85, -83, -87, 7]}],
    "traj3": [{"positions": [-26, -91, 84, -80, -89, -49]}],
    "traj4": [{"positions": [-26, -88, 82, -74, -91, -22]}],
    # "traj5": [{"positions": [-27, -93.5, 89, -93, -83, -23.5]}],
    # "traj6": [{"positions": [-29, -90, 82, -69, -72, -30.3]}],
    # "traj7": [{"positions": [-29, -89, 81, -66.6, -73.3, -36.5]}],
    # "traj8": [{"positions": [-29.3, -92.5, 84, -82, -69.5, 25]}],
    # "traj9": [{"positions": [14, -83, 72, -69, -112, 29]}],
    # "traj10": [{"positions": [13.6, -81.3, 68.5, -60, -111, 28]}],
    # "traj11": [{"positions": [29.3, -69, 68, -66, -124, 50]}],
    # "traj12": [{"positions": [29, -67, 63.4, -55.3, -119, 47]}],
    # "traj13": [{"positions": [29, -69, 67, -62.6, -127, 68.3]}],
    # "traj14": [{"positions": [-64.71, -85, 71, -75.5, -81, -59]}],
    # "traj15": [{"positions": [-70, -64.5, 41.7, -46, -74.5, -61]}],
    # "traj16": [{"positions": [-65.5, -59, 55.3, -61.3, -85, -53.5]}],
    # "traj17": [{"positions": [-65.71, -52.7, 44, -47.5, -82, -56.3]}],
    # "traj18": [{"positions": [-64.5, -57, 31, -27.5, -76.6, -57.2]}],
    # "traj19": [{"positions": [-65.5, -60, 36.3, -33, -68.5, -89]}],
    # "traj20": [{"positions": [-81, -73.5, 55.4, -34.4, -77.6, -102.3]}],
    # "traj21": [{"positions": [-84, -80, 66.5, -53.4, -57.7, -66.8]}],
    # "traj22": [{"positions": [-84, -65.8, 46.2, -50.8, -87.5, -58.7]}],
    # "traj23": [{"positions": [-19, -87.6, 74.8, -63.6, -76, -21]}],
    # "traj24": [{"positions": [-18.1, -82.8, 57.2, -65.8, -76, 34.5]}],
    # "traj25": [{"positions": [-17.7, -84, 60, -72.5, -78.4, 61]}],
    # "traj26": [{"positions": [-7, -87, 63.5, -75.2, -100.5, 81.5]}],
    # "traj27": [{"positions": [-8.2, -85, 59.3, -65.2, -93.5, -93.2]}],
    # "traj28": [{"positions": [6, -117.6, 98.7, -84.7, -104, -45]}],
    # "traj29": [{"positions": [-35.4, -120.8, 115.5, -100, -79.9, -31.6]}]
}


# Collect poses to save to JSON
poses_data = {
    "pose_frame_id": "base",
    "image_frame_id": "Realsense_D455_color_optical_frame",
    "poses": []
}

# Ensure images2 folder exists
os.makedirs("imagesFINAL2", exist_ok=True)

def take_picture(pipeline, picture_name):
    try:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("No frame captured")
            return

        # Convert image to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Save the image to the images2 folder
        full_path = os.path.join("imagesFINAL2", picture_name)
        cv2.imwrite(full_path, color_image)
        print(f"Picture saved as {full_path}")

    except Exception as e:
        print("An error occurred while taking picture:", e)

def move_robot_with_trajectory(rtde_control, rtde_receive, trajectory_dict, traj_name):
    try:
        # Get positions from the dictionary
        trajectory = trajectory_dict[0]["positions"]
        
        # Convert degrees to radians
        joint_positions_radians = [math.radians(angle) for angle in trajectory]

        # Move to the specified joint positions at a slow speed
        speed = 0.2  # Adjust speed (0.2 is slow)
        acceleration = 0.1  # Default acceleration

        rtde_control.moveJ(joint_positions_radians, speed, acceleration)

        # Get the tool position in the base coordinate system
        tool_position = rtde_receive.getActualTCPPose()

        # Convert RX, RY, RZ from rotation vector to quaternion
        rotation_vector = tool_position[3:]  # RX, RY, RZ in radians
        quaternion = R.from_rotvec(rotation_vector).as_quat()  # [x, y, z, w]
        et_version = Myframe.from_UR_pose(tool_position).as_xyzquat()
        print("elvin's function result:")
        print(et_version)
        # Create 7D pose (x, y, z, quaternion x, y, z, w)
        pose_7d = tool_position[:3] + quaternion.tolist()

        # Add 7D pose to poses_data
        poses_data["poses"].append(pose_7d)

        # Print 7D pose for reference
        print("7D Pose (x, y, z, qx, qy, qz, qw):", pose_7d)

    except Exception as e:
        print("An error occurred:", e)


# Main function to iterate over trajectories and allow stopping
def main():
    # Connect to the UR10e robot
    rtde_control = RTDEControlInterface(robot_ip)
    rtde_receive = RTDEReceiveInterface(robot_ip)

    #Initialize RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # Set to 1280x720 resolution
    pipeline.start(config)
    
    try:
        for index, (traj_name, positions_list) in enumerate(trajectories.items()):
            print(f"Moving robot to {traj_name}...")
            move_robot_with_trajectory(rtde_control, rtde_receive, positions_list, traj_name)

            # # Take a picture after each move
            picture_name = f"{str(index).zfill(2)}.png"
            take_picture(pipeline, picture_name)
            time.sleep(1)  # Adjust delay if needed

    finally:
        # Ensure the camera and robot connection are released
        # pipeline.stop()  # Stop the RealSense pipeline
        rtde_control.stopScript()
        print("Robot movement completed or interrupted.")

        #Save poses to a JSON file
        with open("poses.json", "w") as json_file:
            json.dump(poses_data, json_file, indent=4)
        print("7D Poses saved to 'poses.json'.")

if __name__ == "__main__":
    main()