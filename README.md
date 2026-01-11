
# UR10e Grasping Project

This repository contains a ROS-based project for controlling and performing grasping tasks with a UR10e robotic arm. The setup includes configurations and dependencies for working with the UR10e, a Robotiq gripper, and additional custom packages.


https://github.com/user-attachments/assets/3b38dc54-bd81-4e59-a324-d1ab38bdfbc6



https://github.com/user-attachments/assets/10fceca6-a3b9-4606-b4ec-ae415e48d024



## Table of Contents

- [Project Overview](#project-overview)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Submodules](#submodules)
- [License](#license)

## Project Overview

This project is designed to provide control, automation, and precision grasping capabilities for a UR10e robotic arm equipped with a Robotiq gripper. The primary goal is to enable easy setup and execution of grasping tasks in a ROS2 environment. The project includes essential ROS packages for control, sensor data processing, and motion planning.

## Repository Structure

The repository structure is as follows:

```plaintext
UR10e_Grasping/
├── abdo_ws/
│   ├── src/
│   │   ├── app/        # Contains the app for the robot grasping system
│   │   ├── gripper/    # Submodule for the Robotiq gripper drivers
│   │   ├── serial/     # Submodule or custom package for serial communication
│   │   └── my_pkg/     # Has the Realsense camera calibration code and data
│   │   └── my_ur_driver/ # Has the ur10e launch and configuration files 
│   │   └── ur_calibration/ # Has the correct calibration code for extracting the robot calibration (Kinematics)
├── .gitmodules         # Configuration for submodules
├── README.md           # Project overview and setup instructions
└── ...
```

### Key Folders

- **`abdo_ws/src/app`**: Includes all parts of the system (camera, grasp detector, robot, gripper) and a coordinator to integrate all of them.
- **`abdo_ws/src/my_pkg`**: Contains the Realsense camera calibration code and data.
- **`abdo_ws/src/my_ur_driver`**: Contains the ur10e launch and configuration files.
- **`abdo_ws/src/gripper`**: Contains drivers and configuration for controlling the Robotiq gripper. This folder is managed as a submodule and links to an external repository.
- **`abdo_ws/src/serial`**: Contains either serial communication libraries or specific configurations needed for the UR10e’s serial communication. This folder is also managed as a submodule.
- **`.gitmodules`**: Contains submodule configurations with URLs linking to external repositories for dependencies.

## Getting Started

### Prerequisites

- **Ubuntu 22.04**
- **ROS 2 (Humble)**: This project is designed for ROS 2. Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) to set it up. Running "sudo apt-get install ros-humble-ur" and then "sudo apt install ros-humble-ur-robot-driver is recommended for installing UR robots packages.
- **Moveit 2 (Humble)**: This project is designed for Moveit Humble. Follow the [official Moveit Humble installation guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) to set it up. You better skip this guide and install moveit humble using "sudo apt install ros-humble-moveit" and then "sudo apt install ros-humble-ur-moveit-config".
- **Git**: Make sure Git is installed to manage submodules and version control.
  
  ```bash
  sudo apt update
  sudo apt install git
  ```

### Installation

1. **Clone the Repository**:

   Clone this repository and initialize submodules.

   ```bash
   git clone --recurse-submodules https://github.com/AbdalrahmanAtia/UR10e_Grasping.git
   cd UR10e_Grasping
   ```

2. **Initialize and Update Submodules**:

   If you cloned without `--recurse-submodules`, initialize and update the submodules manually.

   ```bash
   git submodule update --init --recursive
   ```

3. **Build the Workspace**:

   Navigate to the workspace and build the ROS packages.

   ```bash
   cd abdo_ws
   colcon build
   ```

## Usage

### Default ROS Commands

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
  ```

```bash
# Launching the default UR robots from ROS:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101   # Adjust ur_type and IP
  ```

```bash
# Launching the default MoveIt from ROS:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e  # Adjust ur_type
  ```

```bash
# Launching the robot on Simulation:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 use_fake_hardware:=true fake_execution:=true
  ```

```bash
# Launching MoveIt on Simulation:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e use_fake_hardware:=true fake_execution:=true
  ```

```bash
# Running the default Realsense camera:
ros2 launch realsense2_camera rs_launch.py
  ```

```bash
# Running the Realsense camera with aligned depth with color and point cloud enabled:
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
  ```
### My Custom UR10e Launch Commands

```bash
# Source the workspace
cd ~/abdo_ws
source install/setup.bash
  ```

```bash
# Launching the UR10e:
ros2 launch my_ur_driver my_ur.launch.py  # Modify the launch code to adjust ur_type and IP
  ```

```bash
# Launching MoveIt:
ros2 launch my_ur_driver my_moveit.launch.py  # Modify to adjust ur_type
  ```

```bash
# Launching the UR10e with the gripper:
ros2 launch my_ur_driver my_ur_with_gripper.launch.py  # Modify the launch code to adjust ur_type and IP
  ```

```bash
# Launching MoveIt with the gripper:
ros2 launch my_ur_driver my_moveit_with_gripper.launch.py  # Modify to adjust ur_type
  ```

```bash
# Launching the robot on Simulation:
ros2 launch my_ur_driver my_ur.launch.py use_fake_hardware:=true fake_execution:=true
  ```

```bash
# Launching MoveIt on Simulation:
ros2 launch my_ur_driver my_moveit.launch.py use_fake_hardware:=true fake_execution:=true
  ```

```bash
# Running the Realsense camera with point cloud and depth aligned enabled:
ros2 launch my_ur_driver my_camera.launch.py
  ```

### Gripper Commands

```bash
# The gripper launch (NO NEED to launch again if already launched the my_ur_with_gripper.launch.py)
ros2 launch robotiq_description robotiq_control.launch.py
  ```

##### Gripper limits: 0 or 0.1 (fully open) - 0.8 (fully closed)

```bash
# Closing the gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.8, max_effort: 50.0}}"
  ```

```bash
# Opening the gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.1, max_effort: 50.0}}"
  ```



### Running Python Files

```bash
# Run Python files directly
/usr/bin/python3.10 calibration.py
  ```

```bash
# Or using ROS2
ros2 run my_pkg sub_realsense.py
  ```

### Network Connection Issue Solution

To make sure the robot is connected to the PC:
```bash
ping 192.168.56.101   # adjust the IP based on your robot's IP.
  ```
if it pings and you can see information are continuously recieved, the PC is connected to the robot.

```bash
# Adjust the connection name and robot IP
sudo nmcli con mod "Wired connection 2" ipv4.addresses 192.168.56.66/24
sudo nmcli con mod "Wired connection 2" ipv4.method manual
sudo nmcli con up "Wired connection 2"
  ```

### Install All Required ROS Packages

```bash
# Ensure all required ROS packages are installed
rosdep update
rosdep install --from-paths /opt/ros/humble/share --ignore-src -r -y
  ```

## Submodules

This project uses Git submodules for external dependencies. Ensure the submodules are correctly initialized and updated as described above.

- **`abdo_ws/src/gripper`**: This submodule points to the [Robotiq gripper driver repository](https://github.com/AbdalrahmanAtia/ros2_robotiq_gripper.git).
- **`abdo_ws/src/serial`**: This submodule points to a repository containing serial communication packages or configurations necessary for the gripper driver. If you need to change or update this submodule, modify the URL in the `.gitmodules` file which now contains this repository [Serial Repository](https://github.com/AbdalrahmanAtia/serial.git).

### Updating Submodules

If there are updates in the submodules, you can pull those updates using:

```bash
git submodule update --remote
```

# Robot Grasp Coordinator

This project is a ROS2-based application designed to coordinate the grasping and manipulation of objects using a robotic arm. The system uses a combination of vision-based grasp detection and robot control commands to execute tasks such as detecting objects, grasping them, and performing various actions like releasing the object and returning the robot to a home position.

The `RobotGraspCoordinator` node is the central part of this application, handling all the stages of the grasping workflow, including image capture, grasp detection, pose transformation, robot movement, and gripper control. The system also includes key functionalities like visualizing grasps and gracefully shutting down the program using a keyboard press.

## ------ For more details, please check the README file in abdo_ws/src/app. -------

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


## License

Not yet
