
# UR10e Grasping Project

This repository contains a ROS-based project for controlling and performing grasping tasks with a UR10e robotic arm. The setup includes configurations and dependencies for working with the UR10e, a Robotiq gripper, and additional custom packages.

## Table of Contents

- [Project Overview](#project-overview)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Submodules](#submodules)
- [Contributing](#contributing)
- [License](#license)

## Project Overview

This project is designed to provide control, automation, and precision grasping capabilities for a UR10e robotic arm equipped with a Robotiq gripper. The primary goal is to enable easy setup and execution of grasping tasks in a ROS2 environment, facilitating robotics research and industrial applications. The project includes essential ROS packages for control, sensor data processing, and motion planning.

## Repository Structure

The repository structure is as follows:

```plaintext
UR10e_Grasping/
├── abdo_ws/
│   ├── src/
│   │   ├── gripper/    # Submodule for the Robotiq gripper drivers
│   │   ├── serial/     # Submodule or custom package for serial communication
│   │   └── my_pkg/
│   │   └── my_ur_driver/
│   │   └── ur_calibration/
├── .gitmodules         # Configuration for submodules
├── README.md           # Project overview and setup instructions
└── ...
```

### Key Folders

- **`abdo_ws/src/gripper`**: Contains drivers and configuration for controlling the Robotiq gripper. This folder is managed as a submodule and links to an external repository.
- **`abdo_ws/src/serial`**: Contains either serial communication libraries or specific configurations needed for the UR10e’s serial communication. This folder is also managed as a submodule.
- **`.gitmodules`**: Contains submodule configurations with URLs linking to external repositories for dependencies.

## Getting Started

### Prerequisites

- **Ubuntu 22.04**
- **ROS 2 (e.g., Humble)**: This project is designed for ROS 2. Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) to set it up.
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

1. **Source the Workspace**:

   ```bash
   source install/setup.bash
   ```

2. **Launching the ur10e robot control Node**:

   Launch the ur10e robot control node, which should start the UR10e robot, if configured correctly. Adjust the command based on your launch files and package names.

   ```bash
   ros2 launch my_ur_robot my.launch.py
   ```
2. **Launching the ur10e robot moveit control Node**:

   Launch the ur10e robot moveit control node, which should start the UR10e robot moveit, if configured correctly. Adjust the command based on your launch files and package names.

   ```bash
   ros2 launch my_ur_robot my_moveit.launch.py
   ```

## Submodules

This project uses Git submodules for external dependencies. Ensure the submodules are correctly initialized and updated as described above.

- **`abdo_ws/src/gripper`**: This submodule points to the [Robotiq gripper driver repository](https://github.com/AbdalrahmanAtia/ros2_robotiq_gripper.git).
- **`abdo_ws/src/serial`**: This submodule points to a repository containing serial communication packages or configurations necessary for this project. If you need to change or update this submodule, modify the URL in the `.gitmodules` file.

### Updating Submodules

If there are updates in the submodules, you can pull those updates using:

```bash
git submodule update --remote
```

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork this repository.
2. Create a new branch for your feature or bug fix.
3. Make your changes and ensure they are properly tested.
4. Commit your changes with clear commit messages.
5. Push your branch to your fork and open a pull request.

## License

Not yet
