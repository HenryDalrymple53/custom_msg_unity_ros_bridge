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

Example usage

#### Message generation
After sourcing all relevant directories for wanted messages, run jsonGenGui.py. This may require you to install PyQt6. You can do this with 
```
pip install -r requirements.txt
```
in the same directory.
Then, run 
```
python3 jsonGenGUI.py
```

![Message Library Selection](https://github.com/user-attachments/assets/23b358d3-0d01-4271-b6b0-033b659c23fe)

1. Select the **message libraries** you want to define.
2. Choose a **save directory**.
   - Recommended: `Unity/Assets/Templates` (but any directory works).
3. Once everything is configured, click **“Process Messages”**.

> **Note:** The application may freeze for a while during processing. This is expected—just let it run.

---

## Unity Examples

### Subscription Node Example

![Subscription Node Example](https://github.com/user-attachments/assets/4dd4751e-2c98-4498-9fe9-e3f10343ea25)

Example of a **subscription node** set up in Unity.

---

### Publishing Node Example

![Publishing Node Example](https://github.com/user-attachments/assets/c384c43f-ddfd-4eaf-b4cc-e6fb6fcad239)

Example of a **publishing node** set up in Unity.

---

## UDP Controller Setup

![UDP Controller Object](https://github.com/user-attachments/assets/b29be359-a8a0-484e-bfb4-6026ab4efb8c)

You need to create a **UDP Object** with the **UDP Controller** script attached.

- Set the **Host** to `localhost`.

---

## Test Object Configuration

![Test Object Configuration](https://github.com/user-attachments/assets/d4182165-1fa7-4d32-8cbf-2ae18d674bf3)

The **test object** manages both publishing and subscribing.

- It contains **two scripts**:
  - Each script has an attached `.json` text file
- It also includes a reference to the **UDPController** object
