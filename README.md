# ROS2 Walker Simulation and ROS Bag Recording

## Overview

This repository contains a ROS2-based walker simulation, built using Gazebo and ROS2 Humble. The walker node is a simple robot that interacts with a simulated environment. You can launch the walker simulation, enable or disable ROS bag recording, and inspect the recorded bag files. The bag files contain sensor data and robot state information that can be replayed for further analysis.

## Assumptions and Dependencies

- **ROS2 Humble**: This repository assumes that you are using **ROS2 Humble**. It is required for the system to work as expected.
- **Gazebo**: Gazebo is used for the simulation of the robot. Ensure that Gazebo is installed and set up correctly.
- **Colcon**: The build tool used in ROS2 projects.
- **Clang-Tidy**: For static code analysis and code style checks (optional but recommended).
- **Python**: Python is required for running the launch files and ROS2 commands.

To install the required dependencies, use the following commands (for ROS2 Humble):

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Directory Structure

├── results/ # Contains outputs from clang-tidy, build logs, and other results. 
├── rosbag/ # Directory where recorded bag files are stored. 
├── src/ 
│ └── walker/
|    └── src/
│       └── walker_node.cpp # Main walker node implementation. 
|     └── launch/ │
|       └── walker_launch.py # ROS2 launch file to start the walker node and simulation.

## Build Instructions

Clone the repository:

```bash
git clone <repository_url>
cd <repository_directory>
```

Build the workspace with colcon:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Source the setup file: After building the workspace, source the environment:

```bash
source install/setup.bash
```

## Running the Simulation

To run the walker simulation, use following command:

```bash
ros2 launch walker walker_launch.py
```
### Running the simulation with bag file recording

```bash
ros2 launch walker walker_launch.py enable_rosbag:=true
```

### Inspecting bag files

After running the simulation with recording enabled, you can inspect the recorded bag files located in the rosbag/ directory. You can check the contents of a bag file using the following command:

```bash
ros2 bag info rosbag
```

### Playing Back Bag Files

To play back a recorded bag file, first ensure that Gazebo is not running, as Gazebo will conflict with the playback. Then, use the following command:
```bash
ros2 bag play rosbag
```

### Clang-Tidy Static Analysis

To ensure code quality and consistency, you can run clang-tidy to check your C++ files for issues. Before running clang-tidy, ensure the workspace has been built with the following command:

After building, run clang-tidy on your C++ files (for example, walker_node.cpp):
```bash
clang-tidy -p ./build $( find . -name *.cpp | grep -v "/build/" )
```

If no issues are found, clang-tidy will exit with a 0 code. You can redirect the output to a results file as follows:

```bash
clang-tidy -p ./build $( find . -name *.cpp | grep -v "/build/" ) > results/clang_tidy_output.txt; \
if [ $? -eq 0 ]; then echo "0" > results/clang_tidy_output.txt; fi
```

This will either log any issues found or write 0 to results/clang_tidy_output.txt if no issues were detected.

### Cpplint

```bash
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -v "/build/" )
```