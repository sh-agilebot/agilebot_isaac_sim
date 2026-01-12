

# 🤖 Agilebot Isaac Sim

<div align="left">

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-Latest-orange)](https://developer.nvidia.com/isaac-sim)

---

 **[中文](./README_CN.md)** | [English](./README.md)

</div>



---

## 📋 Project Overview

**agilebot_isaac_sim** is the simulation integration repository in the Agilebot Robot Isaac ecosystem, providing a complete solution for integrating Agilebot series robots into NVIDIA IsaacSim simulation environment, including core control code, motion policy configurations, and rich example demonstrations.

This project is developed and maintained by [Shanghai Agilebot Robotics Ltd.](https://www.sh-agilebot.com/). Agilebot is a high-tech enterprise specializing in industrial robot R&D, manufacturing, and intelligent manufacturing solutions. With proprietary **Single-Chip Multi-Axis Drive-Control Integrated Motion Controller (SCIMC)** technology at its core, Agilebot provides high-performance, cost-effective, and user-friendly robot products and intelligent manufacturing solutions for various industries.

### ✨ Key Features

- 🦾 **Multi-Model Support**: Supports multiple Agilebot robot models including gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a
- 🎮 **RMPflow Motion Control**: Integrated RMPflow (Robot Motion Policy Flow) motion policy framework
- 🎯 **Rich Function Demos**: Complete task examples including target following, pick-and-place, stacking
- 🔧 **MoveIt Integration**: Support for robot control through ROS 2 + MoveIt
- 📷 **Vision Grasping**: Wrist-mounted camera integrated grasping task implementation
- 🔍 **Kinematics Verification**: Forward/inverse kinematics verification tools

---

## 🚀 Quick Start

### Prerequisites

- NVIDIA IsaacSim (latest version recommended)
- Python 3.11+
- Robot USD model files ([Download Link](https://github.com/sh-agilebot/agilebot_isaac_usd_assets/tree/main))

> 📖 For detailed installation and configuration instructions, see **[Isaac Sim Environment Configuration Guide](./docs/isaacsim_environment_setup.md)**

### Installation Configuration

#### 1. Configure RMPflow Motion Policy

Copy the `motion_policy_configs/Agilebot` directory to your IsaacSim configuration directory:

```bash
cp -r agilebot_integration/motion_policy_configs/Agilebot \
  ~/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/
```

#### 2. Configure Agilebot Integration Code

Copy the `code/agilebot` directory to your IsaacSim extensions directory:

```bash
cp -r agilebot_integration/code/agilebot \
  ~/isaacsim/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/
```

> ⚠️ **Note**: This path may cause package prefix conflicts in VS Code, preventing code hints. Users can adjust the path according to actual needs.

#### 3. Prepare Robot USD Files

Download robot USD model files from the [agilebot_isaac_usd_assets](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) repository and place them in the specified path, then configure the correct USD path in the demo files.

> 💡 **Isaac Asset Setup Tip**: For Isaac Sim asset caching configuration, please refer to [Isaac Sim Official Documentation - Setup Tips - Asset](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#setup-tips) section.

Supported robot models:
- gbt-c5a
- gbt-c7a
- gbt-c12a
- gbt-c16a

---

## 🗂️ Agilebot Isaac Simulation Repository Series

This project is part of Agilebot Robot Isaac ecosystem, providing a complete solution for simulation, training, and asset management.

| Repository Name | Function | Main Content | Link |
|---------|---------|---------|------|
| **agilebot_isaac_sim** | Simulation Integration | IsaacSim integration for Agilebot robots, including simulation configurations, setup files, and demo examples. No robot digital assets are included. | [Current Repository](#) |
| **agilebot_isaac_lab** | Training & Learning | IsaacLab environments and training examples for Agilebot robots, including task definitions and learning pipelines. No robot digital assets are included. | [View Repository](https://github.com/sh-agilebot/agilebot_isaac_lab) |
| **agilebot_isaac_usd_assets** | Asset Management | Centralized repository maintaining USD files, meshes, and textures for Agilebot robots. | [View Repository](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) |

> 💡 **Usage Note**: Before using this repository, please download the required robot USD digital asset files from **agilebot_isaac_usd_assets**.

---

## 💡 Usage Examples

This repository provides comprehensive demonstration examples covering multiple robot tasks and integration scenarios:

### 📖 Demo Categories

| Demo Category | Description | Robot Models | Key Features |
|---------------|-------------|--------------|--------------|
| **Stacking** | Cube stacking tasks | gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a | Adjustable gripper force, multi-height stacking |
| **Pick & Place** | Object grasping and placement | gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a | Precise grasping control, trajectory planning |
| **Follow Target** | End-effector trajectory tracking | gbt-c5a (with/without gripper) | Real-time IK, smooth motion |
| **FK Verification** | Forward kinematics validation | gbt-c5a (with/without gripper) | Real-time pose verification, accuracy analysis |
| **MoveIt Control** | ROS 2 + MoveIt integration | gbt-c7a | ROS 2 interface, MoveIt planning |
| **Camera Grasping** | Vision-based grasping with recording | Full asset (arm+gripper+camera) | Camera feedback, video recording (30fps) |

**Detailed Documentation:**
- 📖 **[GBT Robot Examples Documentation](./agilebot_integration/demos/README.md)** - Complete guide for all demonstration examples
- 🔧 **[MoveIt Integration Guide](./example/isaacsim_moveit/readme.md)** - ROS 2 + MoveIt control tutorial
- 📷 **[Camera Grasping Task Guide](./example/pick_place_agilebot_camera_gripper/readme.md)** - Vision-based grasping implementation

---

## 💡 Advanced Tips

### Completely Independent Project Setup

> 💡 **Tip**: If you want your project to run completely independently without relying on copying code to the Isaac Sim installation directory, you can refer to the implementation of the **[Camera Grasping Example](./example/pick_place_agilebot_camera_gripper)**:
>
> 1. **Localize Configuration Files**: Copy the required `motion_policy_configs` configuration files to your project's local directory (like `rmpflow/` in the example)
> 2. **Copy Core Code**: Copy core classes from `agilebot_integration/code/agilebot/` (controllers, tasks, etc.) to your project's local directory (like `controllers/` and `tasks/` in the example)
> 3. **Adjust Import Paths**: Modify Python import statements to use local relative paths instead of Isaac Sim extension paths
>
> Benefits of this approach:
> - ✅ Complete project independence for easier version control and team collaboration
> - ✅ No dependency on specific Isaac Sim installation paths
> - ✅ Easier deployment and migration
> - ✅ Avoid package path conflicts causing code hint issues

### Isaac Sim GUI Unresponsive Issue

> 💡 **Tip**: If the Isaac Sim GUI is unresponsive and there are no error messages, this is usually because resources are being loaded from the network. Please ensure your network connection is stable, or use a VPN to accelerate the resource loading process. It is recommended to set up Isaac Sim's resource path in advance to avoid delays caused by network resource loading.

---

## 📁 Project Structure

```
agilebot_isaac_sim/
├── agilebot_integration/          # Agilebot integration code and demos
│   ├── code/agilebot/            # Core code
│   │   ├── controllers/          # Controllers (grasping, stacking, etc.)
│   │   └── tasks/                # Task definitions
│   ├── demos/                    # Demo examples
│   │   ├── follow_target.py
│   │   ├── follow_target_with_fk_verification.py
│   │   ├── pick_place.py
│   │   └── stacking.py
│   └── motion_policy_configs/    # RMPflow motion control configs
│       └── Agilebot/
│           ├── gbt_c5a/
│           ├── gbt_c7a/
│           ├── gbt_c12a/
│           └── gbt_c16a/
├── assets/                       # Resource files
├── example/                      # Project examples
│   ├── isaacsim_moveit/          # MoveIt integration example
│   │   └── gbt_c7a_moveit.usd
│   └── pick_place_agilebot_camera_gripper/  # Wrist camera grasping demo
│       ├── pick_place_example.py
│       └── video_recorder.py
├── LICENSE                       # License file
└── README.md                     # This file
```

---

## 🚀 Upcoming Features

> This section is automatically synchronized from [CHANGELOG.md](./docs/CHANGELOG.md)

- 2D grasping demo
- 3D grasping demo



---

## 🔗 Related Resources

### Official Resources
- [Agilebot Website](https://www.sh-agilebot.com/)
- [IsaacSim Official Documentation](https://docs.omniverse.nvidia.com/isaacsim/)

### Agilebot Isaac Ecosystem Repositories
- [agilebot_isaac_usd_assets](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) - Robot digital assets repository (USD files, meshes, textures)
- [agilebot_isaac_lab](https://github.com/sh-agilebot/agilebot_isaac_lab) - IsaacLab environments and training examples
- [agilebot_isaac_sim](https://github.com/sh-agilebot/agilebot_isaac_sim) - Current repository, IsaacSim integration

---

## 🤝 Contributing

Issues and Pull Requests are welcome!

---

## 📄 License

This project is licensed under the [Apache License 2.0](LICENSE). See LICENSE file for details.

---

## 🙏 Acknowledgments

This project references camera mount design from [MetaIsaacGrasp](https://github.com/YitianShi/MetaIsaacGrasp), licensed under MIT License.

---

<div align="center">

**[Shanghai Agilebot Robotics Ltd.](https://www.sh-agilebot.com/)**

© 2025 Shanghai Agilebot Robotics Ltd. All rights reserved.

</div>
