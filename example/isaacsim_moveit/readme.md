# IsaacSim MoveIt Integration for Agilebot Robot

<div align="center">

**[🔙 Back to Main Documentation](../../README.md)**

---

**[🇨🇳 中文文档](./README_CN.md)** | English

</div>

This example demonstrates how to use **MoveIt** to control Agilebot robot with gripper in IsaacSim.
The gripper uses **Robotiq** open-source URDF.

---

## Principle Overview

`topic_based_ros2_control` sends MoveIt's planning results through a specified **topic** and receives feedback from the specified **topic**, thereby achieving control of robot actions.

Required software package:
```bash
sudo apt install ros-humble-topic-based-ros2-control
```

---

## Quick Start Guide

## Configure ROS 2 with Python 3.11

Reference: https://github.com/camopel/isaacsim-ros2-python3.11

> ros2-humble installs Python 3.10 version, while IsaacSim uses Python 3.11 version, therefore need to configure Python 3.11 version of ROS 2

### 1. Add Project to ROS 2 Workspace (Terminal 1)

```bash
source /opt/ros/humble/setup.bash # Python 3.10 version of ROS 2
cp -r isaacsim_moveit {your_workspace}/src
cd {your_workspace}/
colcon build
source install/setup.bash
```

### 2. Launch MoveIt 2

```bash
ros2 launch gbt_gripper_moveit_config demo.launch.py
```

### 3. Launch IsaacSim (Terminal 2)

```bash
source /opt/ros/humble_ws/install/setup.bash # Python 3.11 version of ROS 2
cd ~/isaacsim
./isaac-sim.sh
```

#### Load Scene in IsaacSim

1. Click `File -> Open`
2. Select the `isaacsim_moveit` folder in your workspace
3. Open `./gbt_c7a_moveit.usd` in the above folder
4. Click **Play** button to start simulation

---

## 4. Control Robot

After completing the above steps, you can control the Agilebot robot in IsaacSim through **MoveIt** in **RViz**.
