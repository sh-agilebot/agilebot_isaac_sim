# 🤖 Agilebot Isaac Sim



[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-Latest-orange)](https://developer.nvidia.com/isaac-sim)
![GitHub Release](https://img.shields.io/github/v/release/sh-agilebot/agilebot_isaac_sim)
![GitHub Stars](https://img.shields.io/github/stars/sh-agilebot/agilebot_isaac_sim)
![GitHub Forks](https://img.shields.io/github/forks/sh-agilebot/agilebot_isaac_sim)
![GitHub Issues](https://img.shields.io/github/issues/sh-agilebot/agilebot_isaac_sim)


---

[中文](./README_CN.md) | **[English](./README.md)**


---

## 📋 Project Overview

**agilebot_isaac_sim** is the simulation integration repository in the Agilebot Robot Isaac ecosystem, providing a complete solution for integrating Agilebot series robots into NVIDIA IsaacSim simulation environment, including core control code, motion policy configurations, and rich example demonstrations.

This project is developed and maintained by [Shanghai Agilebot Robotics Ltd.](https://www.sh-agilebot.com/). Agilebot is a high-tech enterprise specializing in industrial robot R&D, manufacturing, and intelligent manufacturing solutions. With proprietary **Single-Chip Multi-Axis Drive-Control Integrated Motion Controller (SCIMC)** technology at its core, Agilebot provides high-performance, cost-effective, and user-friendly robot products and intelligent manufacturing solutions for various industries.

### ✨ Key Features

- 🦾 **Multi-Model Support**: Supports multiple Agilebot robot models including gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a
- 🎮 **RMPflow Motion Control**: Integrated RMPflow (Robot Motion Policy Flow) motion policy framework
- 🎯 **Rich Function Demos**: Complete task examples including target following, pick-and-place, stacking
- 🔧 **MoveIt Integration**: Support for robot control through ROS 2 + MoveIt
- 📷 **Video Recording**: Wrist-mounted camera integrated video recording functionality
- 🔍 **Kinematics Verification**: Forward/inverse kinematics verification tools

### 🛠️ Environment Requirements

| Component | Minimum Version | Recommended Version | Status |
|-----------|-----------------|---------------------|--------|
| IsaacSim | 5.0 | 5.1 | ✅ Verified |
| CUDA |  | 12.x | ⚠️ Optional |
| ROS 2 |  | Humble | ⚠️ Optional |
| Ubuntu | | 22.04 LTS | ✅ Verified |

> Nvidia driver is recommended to use the latest version for best performance.

---

## 📁 Project Structure

```
agilebot_isaac_sim/
├── agilebot_integration/          # Agilebot core integration module
│   ├── code/agilebot/            # Robot controllers and task definitions
│   │   ├── controllers/          # Gripper control, stacking algorithms, etc.
│   │   └── tasks/                # Reinforcement learning task configurations
│   ├── demos/                    # Function demonstration examples
│   │   ├── follow_target.py      # Target following demo
│   │   ├── follow_target_with_fk_verification.py  # Kinematics verification
│   │   ├── pick_place.py         # Pick and place task
│   │   └── stacking.py           # Stacking task
│   └── motion_policy_configs/    # RMPflow motion policy configurations
│       └── Agilebot/
│           ├── gbt_c5a/          # C5A robot configuration
│           ├── gbt_c7a/          # C7A robot configuration
│           ├── gbt_c12a/         # C12A robot configuration
│           └── gbt_c16a/         # C16A robot configuration
├── assets/                       # USD assets, meshes, and texture files
├── docs/                         # Documentation, installation guides, and tutorials
├── example/                      # Standalone example projects
│   ├── isaacsim_moveit/          # ROS 2 + MoveIt integration example
│   │   └── gbt_c7a_moveit.usd
│   └── pick_place_agilebot_camera_gripper/  # Wrist camera grasping and video recording
│       ├── pick_place_example.py
│       └── video_recorder.py
├── tests/                        # Unit tests and integration tests
├── scripts/                      # Utility scripts and tools
├── LICENSE                       # Apache 2.0 License
├── README.md                     # Main documentation (English)
└── README_CN.md                  # Main documentation (Chinese)
```

---

## 🚀 Quick Start

### Step 1: Prepare NVIDIA Driver

Ensure your system has NVIDIA driver installed. It is recommended to use the latest version. You can download and install the driver from the NVIDIA official website.


### Step 2: Install IsaacSim and IsaacLab


Download and install [NVIDIA IsaacSim and IsaacLab](https://isaac-sim.github.io/IsaacLab/main/index.html)

> 📖 For detailed installation and configuration instructions, please also refer to **[Isaac Sim Environment Configuration Guide](./docs/isaacsim_environment_setup.md)**

### Step 3: Clone the Project

```bash
# Clone the repository
git clone https://github.com/sh-agilebot/agilebot_isaac_sim.git
cd agilebot_isaac_sim
```

### Step 4: Configure Environment

#### 4.1 Configure RMPflow Motion Policy

Copy the `motion_policy_configs/Agilebot` directory to your IsaacSim configuration directory:

```bash
cp -r agilebot_integration/motion_policy_configs/Agilebot \
  ~/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/
```

#### 4.2 Configure Agilebot Integration Code

Copy the `code/agilebot` directory to your IsaacSim extensions directory:

```bash
cp -r agilebot_integration/code/agilebot \
  ~/isaacsim/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/
```

> ⚠️ **Note**: This path may cause package prefix conflicts in VS Code, preventing code hints. Users can adjust the path according to actual needs.

#### 4.3 Prepare Robot USD Files

Download robot USD model files from the [agilebot_isaac_usd_assets](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) repository and place them in the specified path, then configure the correct USD path in the demo files.

> 💡 **Isaac Asset Setup Tip**: For Isaac Sim asset caching configuration, please refer to [Isaac Sim Official Documentation - Setup Tips - Asset](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#setup-tips) section.

### Step 5: Run Demos

```bash
# Run target following example
python -m agilebot_integration.demos.follow_target

# Run pick and place example
python -m agilebot_integration.demos.pick_place

# Run stacking example
python -m agilebot_integration.demos.stacking
```

> ⚠️ **Note**: First run may require downloading USD resources, please ensure stable network connection.

---

### 📦 Supported Robot Models

| Model | Reach | Payload | Repeatability | Product Link |
|-------|-------|---------|------------|---------|
| gbt-c5a | 933mm | 5kg | 0.02mm | [View Details](https://www.sh-agilebot.com/74/6/?type=0) |
| gbt-c7a | 785mm | 7kg | 0.02mm | [View Details](https://www.sh-agilebot.com/74/60/?type=0) |
| gbt-c12a | 1303mm | 12kg | 0.03mm | [View Details](https://www.sh-agilebot.com/74/61/?type=0) |
| gbt-c16a | 980mm | 16kg | 0.03mm | [View Details](https://www.sh-agilebot.com/74/69/?type=0) |

---

## 🗂️ Agilebot Isaac Simulation Repository Series

This project is part of the Agilebot Robot Isaac ecosystem, providing complete solutions for simulation, training, and asset management.

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
| **Pick & Place with Video Recording** | Pick and place task with video recording | Full asset (arm+gripper+wrist camera) | Video capture (30fps) |

**Detailed Documentation:**
- 📖 **[GBT Robot Examples Documentation](./agilebot_integration/demos/README.md)** - Complete guide for all demonstration examples
- 🔧 **[MoveIt Integration Guide](./example/isaacsim_moveit/README.md)** - ROS 2 + MoveIt control tutorial
- 📷 **[Wrist Camera Grasping Task Guide](./example/pick_place_agilebot_camera_gripper/README.md)** - Pick & place task and video recording implementation

---

## ❓ FAQ

### Completely Independent Project Setup

> 💡 **Tip**: If you want your project to run completely independently without relying on copying code to the Isaac Sim installation directory, you can refer to the implementation of the **[Wrist Camera Grasping Example](./example/pick_place_agilebot_camera_gripper)**:

> 1. **Localize Configuration Files**: Copy the required `motion_policy_configs` configuration files to your project's local directory (like `rmpflow/` in the example)
> 2. **Copy Core Code**: Copy core classes from `agilebot_integration/code/agilebot/` (controllers, tasks, etc.) to your project's local directory (like `controllers/` and `tasks/` in the example)
> 3. **Adjust Import Paths**: Modify Python import statements to use local relative paths instead of Isaac Sim extension paths

> Benefits of this approach:
> - ✅ Complete project independence for easier version control and team collaboration
> - ✅ No dependency on specific Isaac Sim installation paths
> - ✅ Easier deployment and migration
> - ✅ Avoid package path conflicts causing code hint issues

### Isaac Sim GUI Unresponsive

**Symptoms**: Window freezes, no error messages
**Cause**: Resources are being loaded from network
**Solution**:
1. Ensure stable network connection
2. Use VPN to accelerate resource downloading
3. Set up IsaacSim resource path in advance

> 📖 For detailed instructions, refer to [Isaac Sim Official Documentation - Setup Tips - Asset](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#setup-tips)

### URDF Import Collision Body Lost

**Symptoms**: Robot cannot perform collision detection after import
**Environment**: Isaac Sim 5.1
**Solution**: Use Isaac Sim **5.0 or lower version** for URDF import, then upgrade the generated USD file to version 5.1 for use



---

## 🐛 Report Issues

When encountering problems, please:

1. Check [FAQ](#-faq)
2. Search [GitHub Issues](https://github.com/sh-agilebot/agilebot_isaac_sim/issues)
3. Create a new Issue with:
   - Environment information (OS, Python, IsaacSim version)
   - Reproduction steps
   - Error logs
   - Screenshots/videos (if available)

---

## 🤝 Contact & Support

- 📧 **Email**: info@agilebot.com.cn
- 🐛 **Issue Reporting**: [GitHub Issues](https://github.com/sh-agilebot/agilebot_isaac_sim/issues)
- � **Service Hotline**: 400-996-7588
- 🏢 **Shanghai Headquarters**: 7th Floor, Building T1, Hongqiao Wanchuang Center, Lane 500, Xinlong Road, Minhang District, Shanghai
- 🌐 **Website**: https://www.sh-agilebot.com/

> 💡 It is recommended to report issues through GitHub Issues for better tracking and community solution sharing.

---



## 🗺️ Development Roadmap

> This section is automatically synchronized from [CHANGELOG.md](./docs/CHANGELOG.md)






### ✅ Released Features (v0.0.1)

- [x] Agilebot IsaacSim Integration Framework Initial Release
- [x] Agilebot Robot Core Control Code
- [x] RMPflow Motion Policy Configuration for gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a
- [x] Example Demos:
- [x] MoveIt 2 Integration with ROS 2 Humble
- [x] Pick and Place with Wrist Camera Demo (complete workflow and video recording)
- [x] Bilingual Documentation (English/Chinese)

### 🚧 In Development

- [ ] 2D grasping demo: Simulation grasping based on 2D vision
- [ ] 3D grasping demo: Simulation grasping based on 3D vision

## � License

This project is licensed under the Apache 2.0 License. See [LICENSE](./LICENSE) file for details.

---

