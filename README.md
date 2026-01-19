# Project Name
custom bridging sotware to run a Unity <-> ROS2 pipeline thorugh local UDP ports and JSON data structures.

---

## Table of Contents
1. [Prerequisites]
2. [Installation]
   - [1. Clone the Repository]
   - [2. Build ROS2 Workspace]
   - [3. Open Unity Project]
3. [Running the Project)
   - [1. Launch ROS2 Nodes]
   - [2. Start Unity Simulation]
4. [Usage](#usage)


---

## Prerequisites
List all software/tools that need to be installed beforehand:
- [Unity 2022.3.32f1](https://unity.com/)
- ROS2 Humble
- Python 3.10+
- Git

---

## Installation

### 1. Clone the Repository
```bash
git clone (https://github.com/HenryDalrymple53/custom_msg_unity_ros_bridge.git)
cd custom_msg_unity_ros_bridge
```
### 2. Build ROS2 Workspace
```
cd unity_udp_ros_bridge
colcon build
. install/setup.bash
```
### 3. Open Unity Project
From the Unity launcher, create a project form directory and select the root cloned directory.


## Running the Project

### 1. Launch ROS2 nodes
In the terminal where you built and ran . install/setup.bash,
```
ros2 launch unity_udp_ros_bridge udp_bridge_launch.py 

```

### 2. Start Unity Simulation
WHen running, should produce test output on /drive_topic that it both subscribes and publishes to


## Usage
This is meant to be a module that can be merged with other modules, such as the DAMRC Mars Rover Unity Groundstation

This is just a demo to be built off of / used in another project.

