# Isaac MoveIt C5A 项目

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Isaac Sim](https://img.shields.io/badge/Simulation-Isaac%20Sim-orange.svg)](https://developer.nvidia.com/isaac-sim)

**[English](README.md)** | 中文说明

---

**演示如何使用 MoveIt 2 控制 Isaac Sim 中带有 Robotiq 2F-140 夹爪的捷勃特 (GBT) C5A 机器人。**

![机器人演示](https://via.placeholder.com/800x450.png?text=GBT+C5A+Isaac+Sim+MoveIt+2+Demonstration)
*占位图：请在此处添加机器人运行的 GIF 或截图。*

</div>

---

## 概述

本仓库提供了 GBT C5A 机械臂的完整 ROS 2 与 Isaac Sim 集成方案。系统通过 `topic_based_ros2_control` 插件桥接 MoveIt 2 与 Isaac Sim，实现仿真环境下的无缝路径规划与执行。

---

## 仓库结构

- **[gbt_c5a_gripper_description](./gbt_c5a_gripper_description/README.zh-CN.md)**: 机器人描述包。包含 URDF、网格模型以及 Isaac Sim USD 转换脚本。
- **[gbt_c5a_gripper_moveit_config](./gbt_c5a_gripper_moveit_config/README.md)**: MoveIt 2 配置包。负责 SRDF、控制器和规划参数设置。

---

## 前期准备 (资产生成)

在启动仿真之前，如果您是第一次运行，需要先完成网格安装和 USD 转换。

### 1. 安装 Robotiq 网格
请从 [Robotiq 官方仓库](https://github.com/ros-industrial-attic/robotiq/tree/kinetic-devel/robotiq_2f_140_gripper_visualization) 下载 2F-140 的 STL 文件，然后运行：
```bash
bash gbt_c5a_gripper_description/scripts/setup_robotiq_meshes.sh <STL文件夹路径>
```

### 2. 转换为 USD 资产
**注意：此步骤需要使用 Isaac Sim 的 Python 环境（如 Isaac Lab）。必须添加 `--skip-camera`。**

请在 `isaac_moveit` 根目录下运行以下一键转换命令（注意输入输出路径是相对于描述包根目录的）：

```bash
python gbt_c5a_gripper_description/scripts/convert_urdf_to_usd.py \
  urdf/gbt_c5a.urdf \
  urdf/gbt_c5a.usd \
  --skip-camera \
  --drive-type force \
  --arm-natural-frequency 300.0 \
  --gripper-natural-frequency 300.0 \
  --mimic-natural-frequency 2500.0 \
  --damping-ratio 0.005 \
  --finger-max-force 200.0 \
  --solver-position-iterations 64 \
  --solver-velocity-iterations 16
```

---

## 快速开始

在完成上述资产准备和 MoveIt 配置之后，请按照以下步骤启动：

### 1. 启动 MoveIt 2 和 Isaac Sim

分别在两个独立的终端中执行：

#### **终端一：启动 MoveIt 2**
```bash
# Source Python 3.10 的 ROS 环境
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gbt_c5a_gripper_moveit_config demo.launch.py
```

#### **终端二：启动 Isaac Sim**
```bash
# Source Python 3.11 的 ROS 2 环境
source /opt/ros/humble_ws/install/setup.bash # 请根据实际安装路径修改
# 启动 Isaac Sim
./isaac-sim.sh
```

### 2. Isaac Sim 配置与运行（Graph Shortcut）

1.  在 Isaac Sim 中打开准备好的机器人 `.usd` 文件（路径：`gbt_c5a_gripper_description/urdf/gbt_c5a.usd`）。
2.  在菜单中选择 **Tools > Robotics > ROS 2 OmniGraphs > JointStates**。
3.  在弹窗中配置：
    - `Articulation Prim`：选择 `base_link`（通常为 `/GBT_C5A_gripper/base_link`）。
    - `Publish Topic`：`/isaac_joint_states`
    - `Subscribe Topic`：`/isaac_joint_commands`
    - `Add Articulation Controller`：勾选
4.  点击 **OK** 自动生成 ActionGraph，然后点击 **Play**。

## 3. 使用 MoveIt 控制
此时 ROS 2 与 Isaac Sim 已连接，MoveIt 可以控制 Isaac Sim 中的机械臂与夹爪。
---

## 技术细节

### 基于话题的控制
系统使用以下 ROS 2 话题进行通信：
- **关节状态**: `/isaac_joint_states`
- **关节指令**: `/isaac_joint_commands`

### USD 生成
转换工具负责 URDF 导入、碰撞体补齐和物理参数优化。

---

## 参考资料
- [Isaac Sim ROS 2 Manipulation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/index.html)

---


