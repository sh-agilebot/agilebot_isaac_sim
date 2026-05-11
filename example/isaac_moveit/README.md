# Isaac MoveIt C5A Project

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Isaac Sim](https://img.shields.io/badge/Simulation-Isaac%20Sim-orange.svg)](https://developer.nvidia.com/isaac-sim)

English | **[中文说明](README.zh-CN.md)**

---

**Demonstration of controlling a GBT C5A robot with a Robotiq 2F-140 gripper in Isaac Sim using MoveIt 2.**

![Robot Demonstration](https://via.placeholder.com/800x450.png?text=GBT+C5A+Isaac+Sim+MoveIt+2+Demonstration)

</div>

---

## Overview

This repository provides a complete ROS 2 and Isaac Sim integration for the GBT C5A arm. It uses `topic_based_ros2_control` to bridge MoveIt 2 and Isaac Sim, allowing seamless motion planning and execution in simulation.

---

## Repository Structure

- **[gbt_c5a_gripper_description](./gbt_c5a_gripper_description/README.md)**: Robot description package. Contains URDF, meshes, and Isaac Sim USD conversion scripts.
- **[gbt_c5a_gripper_moveit_config](./gbt_c5a_gripper_moveit_config/README.md)**: MoveIt 2 configuration package. Handles SRDF, controllers, and planning parameters.

---

## Getting Started

### 1. Prerequisites

Install the required control bridge plugin:
```bash
sudo apt install ros-humble-topic-based-ros2-control
```

### 2. Build the Workspace

```bash
# Clone into your workspace src/ and build
colcon build --packages-select gbt_c5a_gripper_description gbt_c5a_gripper_moveit_config
source install/setup.bash
```

### 3. Generate USD Asset

Before running the simulation, you need to convert the URDF to a USD file suitable for Isaac Sim.

Refer to the **[USD Conversion Guide](./gbt_c5a_gripper_description/README.md#quick-start)** for detailed steps.

### 4. Setup ActionGraph in Isaac Sim

To enable ROS 2 communication, follow the **[ActionGraph Setup Guide](./gbt_c5a_gripper_description/README.md#ros-2-control-setup-actiongraph---shortcut-method)** to configure the joint state publisher and subscriber.

### 5. Launch Simulation

1.  **Launch MoveIt 2**:
    ```bash
    ros2 launch gbt_c5a_gripper_moveit_config demo.launch.py
    ```
2.  **Launch Isaac Sim**:
    - Open Isaac Sim and load `gbt_c5a_gripper_description/urdf/gbt_c5a.usd`.
    - Press **Play**.

---

## Technical Details

### Topic Based Control
The system uses the following ROS 2 topics for communication:
- **Joint States**: `/isaac_joint_states`
- **Joint Commands**: `/isaac_joint_commands`

### USD Generation
The conversion tool handles URDF import, camera mounting, physics configuration, and collision mesh refinement.

---

## Reference
- [Isaac Sim ROS 2 Manipulation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/index.html)

---
