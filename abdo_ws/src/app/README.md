# Robot Grasp Coordinator

This project is a ROS2-based application designed to coordinate the grasping and manipulation of objects using a robotic arm. The system uses a combination of vision-based grasp detection and robot control commands to execute tasks such as detecting objects, grasping them, and performing various actions like releasing the object and returning the robot to a home position.

The `RobotGraspCoordinator` node is the central part of this application, handling all the stages of the grasping workflow, including image capture, grasp detection, pose transformation, robot movement, and gripper control. The system also includes key functionalities like visualizing grasps and gracefully shutting down the program using a keyboard press.

## **Features**

- **Robot Control**: Integrates with a robotic arm using the RTDE (Real-Time Data Exchange) interface to move the robot, execute grasp poses, and control the gripper.
- **Grasp Detection**: Uses computer vision (via OpenCV) to detect potential grasps from RGB-D camera images.
- **Grasp Execution**: Allows execution of grasp actions on detected objects.
- **Visualization**: Visualizes the detected grasps in real-time using Open3D.
- **Key-Based Control**: Includes keyboard inputs (`'e'`, `'v'`, `'q'`) for controlling the system and shutting it down.
- **Graceful Shutdown**: Listens for a `'q'` key press to safely shut down the program.

## Dependencies

This application requires the following dependencies to function correctly:

- ROS 2 (Humble or later)
- `rtde_receive`, `rtde_control` for robot communication
- `open3d` for 3D visualization
- `opencv-python` for image processing
- `cv_bridge` for converting ROS images to OpenCV format
- `keyboard` for listening to keyboard events
- `numpy` and other utility libraries for calculations and transformations

You can install the necessary dependencies using `pip`:

```bash
pip install open3d opencv-python numpy keyboard
```

## ROS Dependencies

The following ROS packages are required for the proper functioning of this system:

- **rclpy**: ROS 2 Python client library, which allows the node to communicate with the ROS network.
- **geometry_msgs**: Contains message types for geometry (such as poses, transforms, etc.).
- **sensor_msgs**: Contains message types for sensor data, including images.
- **message_filters**: Helps synchronize messages (such as color and depth images) for processing.
- **tf2_ros**: A package for managing coordinate frame transformations in ROS.
- **cv_bridge**: A ROS package to convert images between ROS image messages and OpenCV image format.

These dependencies can be installed as part of the ROS 2 package setup. Ensure that you have ROS 2 installed and set up the appropriate workspace.


## Setup and Usage

**Run the Robot Grasp Coordinator Node**

You can run the node using this command:

```bash
cd src/app/app
```

```bash
sh app.sh
```

This will initialize the RobotGraspCoordinator node, which will start the iterative grasping process.


**Interact with the System**

- Press 'e' to execute the grasp action (if a valid grasp is detected).
- Press 'v' to visualize the detected grasps.
- Press 'q' to gracefully shut down the system.


**Camera Setup**

Ensure that the robot's camera system is properly configured and launched to publish RGB and depth images. The camera system should be able to provide synchronized color and depth images, which will be used for grasp detection.

To launch the camera with point cloud and depth-algined enabled:

```bash
ros2 launch my_ur_driver my_camera.launch.py
```

**Grasp Detection**

The system uses an image-based approach to detect potential grasps using AnyGrasp SDK. It identifies the most likely positions where the robot can grasp an object based on visual cues. The detected grasp poses are then transformed into robot-specific poses that can be executed by the robot arm.


# Key Components

This section outlines the main components that drive the functionality of the Robot Grasp Coordinator system.

## 1. **RobotGraspCoordinator Node**

The central ROS node that orchestrates the entire grasping process.

- **Initialization:** Sets up RTDE communication, camera controller, grasp detector, grasp pose transformer, and gripper controller. It defines home and release poses, initializes TF buffers for coordinate transformations, and starts the grasping loop and key listener in separate threads.
- **Grasp Loop:** Continuously captures image pairs, detects grasps, and executes them based on user input. Implements retry mechanisms and counts failed grasp attempts to prevent endless looping.
- **Graceful Shutdown:** Listens for a 'q' key press to safely terminate all operations, ensuring that the robot returns to its home position and all resources are properly cleaned up.
- **Logging:** Provides detailed logs for each step, aiding in diagnostics and monitoring the system's state.

## 2. **Camera Controller**

Captures synchronized color and depth images from the robot's camera.

- **Image Subscription:** Subscribes to ROS topics publishing color and depth images.
- **Image Conversion:** Uses `CvBridge` to convert ROS image messages into OpenCV-compatible formats for processing.
- **Image Retrieval:** Provides methods to retrieve the latest synchronized image pairs for grasp detection.

## 3. **Grasp Detector**

Analyzes color and depth images to detect potential grasp poses.

- **AnyGrasp Integration:** Utilizes the AnyGrasp SDK to process point clouds derived from RGB-D images and identify feasible grasp poses.
- **Workspace Definition:** Defines the operational workspace based on bounding boxes and depth ranges to focus grasp detection on relevant areas.
- **Grasp Filtering:** Applies Non-Maximum Suppression (NMS) and score-based filtering to select the best grasp candidates.
- **Visualization:** Integrates with Open3D to visualize detected grasps for user verification.

## 4. **Grasp Pose Transformer**

Converts detected grasp poses from the camera frame to the robot's base frame using coordinate transformations.

- **TF Integration:** Uses ROS TF2 to listen for and apply coordinate transformations between different frames.
- **Pose Transformation:** Transforms grasp poses to the robot's base frame, ensuring accurate movement commands.
- **Error Handling:** Implements retry mechanisms for transformation lookups and logs errors if transformations fail.

## 5. **Gripper Controller**

Controls the robot's gripper to grasp and release objects.

- **Gripper Commands:** Sends open and close commands to the robot's gripper using the RTDE control interface.
- **State Monitoring:** Ensures that the gripper state is correctly managed to avoid failed grasps or object drops.
- **Feedback Logging:** Monitors gripper actions and logs the outcomes for each grasp attempt.

## 6. **RTDE Communication**

Facilitates communication between the ROS node and the robotic arm.

- **RTDE Receive Interface:** Receives real-time data from the robot, such as current poses and states.
- **RTDE Control Interface:** Sends movement and action commands to the robot, controlling its end-effector and gripper.
- **TCP Configuration:** Sets the Tool Center Point (TCP) offset to align the robot's movements with the grasping actions.

---

# How it Works

This section explains the key workflows of the Robot Grasp Coordinator system.

## **Initialization**

When the `RobotGraspCoordinator` node starts, it performs the following steps:

- **RTDE Communication Setup:** Initializes RTDE receive and control interfaces to communicate with the robot arm. Sets the TCP offset for accurate movement commands.
- **Camera Controller Initialization:** Sets up the camera controller to subscribe to color and depth image topics, enabling synchronized image capture for grasp detection.
- **Grasp Detector Initialization:** Loads the AnyGrasp model and sets up the grasp detector with predefined workspace limits. Configures camera intrinsic parameters for accurate point cloud generation.
- **Grasp Pose Transformer Initialization:** Initializes the grasp pose transformer to handle coordinate frame transformations between the camera and robot base frames.
- **Gripper Controller Initialization:** Sets up the gripper controller to manage the robot's gripper actions, ensuring proper grasping and releasing of objects.
- **TF Buffer and Listener Setup:** Initializes TF2 buffers and listeners to manage coordinate transformations between different frames in ROS.
- **Thread Initialization:** Starts separate threads for the grasping loop (`grasp_thread`) and key listener (`key_listener_thread`) to handle operations concurrently without blocking the main ROS spinning.

## **Grasp Loop**

The system enters an iterative grasp loop where it continuously performs the following:

- **Image Capture:** Waits for and retrieves the latest synchronized color and depth image pairs from the camera.
- **Grasp Detection:** Processes the images using the grasp detector to identify potential grasp poses within the defined workspace. Applies filtering to select the most promising grasp candidates.
- **Grasp Execution:**
    - **User Input:** Based on user input ('e' to execute or 'v' to visualize), decides whether to perform the grasp or just visualize it.
    - **Movement Commands:** If executing, commands the robot to move to the grasp pose, close the gripper to grasp the object, move to a release position, and then release the object.
    - **Visualization:** If visualizing, displays the detected grasps using Open3D for user verification without executing the grasp.
- **Error Handling:** Monitors the success of each grasp attempt. Increments a failure counter if grasp execution fails, and terminates the loop after a predefined number of unsuccessful attempts to prevent endless retries.

## **Grasp Pose Transformation**

Once a grasp is detected, the grasp pose is transformed from the camera frame to the robot base frame to ensure accurate movement:

- **Coordinate Transformation:** Uses TF2 to obtain the transformation matrix between the camera and robot base frames.
- **Pose Conversion:** Converts the grasp pose to the robot's coordinate frame using the transformation matrix.
- **Pose Broadcasting:** Broadcasts the transformed grasp pose using TF2 for other ROS nodes or tools to reference.
- **Robot Movement:** Translates the transformed pose into movement commands sent via RTDE to the robot arm.

---

# Key Concepts

## **Grasp Pose Transformer**

- This class is responsible for transforming grasp poses (positions and orientations) from the camera frame to the robot's base frame.

## **Transformation Matrices**

- Grasp poses are defined relative to the camera frame, and the goal is to transform them into the robot's base frame.
- `T_grasp` is a 4x4 homogeneous transformation matrix combining translation and rotation for a grasp pose.
- The transformation from camera to base (`base_to_camera`) is stored as a 4x4 matrix.

## **Coordinate Frames**

- **Camera Frame:** The frame in which the grasp pose is initially defined.
- **Base Frame:** The robot’s base frame, which is the target frame for the transformation.

## **Myframe Class**

- A custom utility for handling transformations. It helps convert between transformation matrices and TF messages, and facilitates rotation operations.
- Methods like `pose_trans()` and `from_Tmat()` are used to apply transformations and convert between frames.

## **TF2 (Transformations)**

- `tf2_ros` is used for ROS-based frame transformations. The `lookup_transform` function is used to get the transform from the robot's base frame to the camera's optical frame.
- After acquiring the transformation, the grasp pose is transformed using matrix multiplication (`np.matmul`).

## **Workflow**

- **Defining the Grasp Pose:** The function `define_grasp_pose()` takes a `grasp_pose` object and converts it into a homogeneous transformation matrix `T_grasp`, combining translation and rotation.
- **Getting the Transformation from Camera to Base:** `get_base_to_camera_transformation()` retrieves the transformation from the base frame to the camera frame using `tf_buffer.lookup_transform()`. The transformation is stored in the `base_to_camera` matrix.
- **Transforming Grasp Pose to Base:** The grasp pose is first defined with respect to the camera frame. Then, the pose is transformed to the base frame by multiplying the camera-to-base transformation matrix (`base_to_camera`) with the grasp pose.

---

# Troubleshooting

Here are some common issues you may encounter, and suggestions on how to address them:

- **No Grasp Detected:**
    - Ensure that the camera is positioned correctly and providing high-quality RGB-D images. The grasp detector may fail if the images are unclear or the object is not in view.

- **RTDE Connection Issues:**
    - Check the network connection to the robot and ensure that the robot's IP address is correctly set in the script.

- **Key Press Not Detected:**
    - Ensure that the `keyboard` module is installed and that the script has the necessary permissions to listen for key events.

- **Visualization Issues:**
    - Ensure that Open3D is properly installed.
    - Check if the point cloud and grasps are correctly transformed before visualization.
    - Verify that the camera intrinsics are correctly set for accurate point cloud generation.

- **Robot Movement Errors:**
    - Verify that the robot is powered on and connected to the network.
    - Ensure that the home and release poses are correctly defined and reachable.
    - Check for any physical obstructions that might prevent the robot from moving to the target poses.

---

# License

This project is using AnyGrasp detection algorithm which is licensed under the AnyGrasp SDK License - see the LICENSE here [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk) for details.

---

# Acknowledgments

- The robot control is based on the RTDE interface provided by Universal Robots.
- The grasp detection and pose transformation are based on standard image processing techniques and ROS libraries.
- Special thanks to the developers of the AnyGrasp SDK and Open3D for their invaluable tools and resources.
