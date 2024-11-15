# RealSense D455 Camera Calibration Steps

This guide is a summary of my trials to do camera calibration, providing step-by-step instructions to perform hand-eye calibration with a RealSense camera mounted on a URxe robotic arm.

## Prerequisites
- A table with a chessboard calibration pattern (11x8)
- A URxe robot with a RealSense camera attached.
- ROS 2 environment set up with `realsense2_camera` and the custom `my_ur_driver` package.

## Calibration Steps

### 1. Prepare the Setup
   - Place the chessboard on a table.
   - Position the robot arm to ensure the camera has a clear view of the chessboard with all corners visible.
   - Adjust the robot arm to different positions, directions, and rotations. Try zooming in and out to vary perspectives.
   - **Record Joint Angles**: For each position, write down the joint angles (in degrees) of the robot arm. Aim for 15-20 diverse positions.

### 2. Adjust Trajectories in the Code
   - Open the `capturing_calibration_pictures&poses.py` file.
   - Update the trajectory positions to match your recorded joint angles (in degrees).

### 3. Launch the RealSense Camera
   - Launch the default RealSense camera with the following command:
     ```bash
     ros2 launch realsense2_camera rs_launch.py
     ```
   - Verify that the ROS topic names from the camera match the topic names in the `capturing_calibration_pictures&poses.py` code to be able to subscribe correctly.
   - Use the command below to list the topics:
     ```bash
     ros2 topic list
     ```

### 4. Run the Capture Script
   - Adjust the movement speed in `capturing_calibration_pictures&poses.py` if needed.
   - Start the script, and stay ready to press the **E-STOP** button on the robot’s control board (Teach Pendant) if necessary.
   - The robot will move to each specified position, pause briefly, take a picture, and save the corresponding position pose (tool position: x, y, z, rx, ry, rz, w).
   
### 5. Check Output Folder
   - After running the capture script, a folder containing captured pictures names `Calibration_Chessboard_Images` and a `poses.json` file will be generated.

### 6. Organize Captured Data
   - Move the contents of the generated folder along with the 'poses.json' file to the following directory: `data/20241007_184458H/`.
   - Replace the existing pictures in that folder or back them up as needed.

### 7. Run the First Calibration Step
   - Execute the `00p_calib_step_01.py` script:
     ```bash
     python3 00p_calib_step_01.py
     ```
   - Press **Enter** when the first picture appears. The script will generate a `results.json` file in `data/20241007_184458H/`.

### 8. Complete the Calibration
   - Run the `00p_calib_step_02.py` script:
     ```bash
     python3 00p_calib_step_02.py
     ```
   - The `results.json` file in `data/20241007_184458H/` will be updated with the final hand-eye calibration data.

### 9. Finalize Calibration
   - Move the contents of the `data/20241007_184458H/` folder or only the `results.json` file to your configuration folder:
     ```bash
     mv data/20241007_184458H/results.json abdo_ws/src/my_ur_driver/Realsense Calibration Data/
     ```
   - and Nowww, ooooooh .. Calibration is complete!

### 10. Launch the Calibrated Camera
   - From now on, use the custom RealSense launch file with your calibrated settings:
     ```bash
     ros2 launch my_ur_driver camera.launch.py
     ```
   - This will load your `results.json` calibration data and apply it for accurate camera positioning.

## Notes
- Be cautious while the robot is moving; always be ready to use the **E-STOP** if anything goes wrong.
- Ensure your `results.json` file is backed up for future use.

Thankss!
