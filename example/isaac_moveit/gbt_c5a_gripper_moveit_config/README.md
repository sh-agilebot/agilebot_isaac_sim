# GBT Gripper MoveIt Config | 捷勃特夹爪 MoveIt 配置

This package contains the MoveIt 2 configuration for the GBT C5A robot arm with a Robotiq 2F-140 gripper.
本包包含了带有 Robotiq 2F-140 夹爪的 GBT C5A 机械臂的 MoveIt 2 配置。

---

## Features | 特性
- MoveIt 2 motion planning for `gbt_arm` and `hand` groups. | 为 `gbt_arm` 和 `hand` 规划组提供 MoveIt 2 运动规划。
- Configured for Isaac Sim via `topic_based_ros2_control`. | 通过 `topic_based_ros2_control` 为 Isaac Sim 进行配置。
- Includes SRDF, controllers, and joint limits. | 包含 SRDF、控制器和关节限制。

---

## Usage | 使用方法

### Demo Mode (RViz only) | 演示模式 (仅 RViz)
```bash
ros2 launch gbt_c5a_gripper_moveit_config demo.launch.py
```

### With Isaac Sim | 配合 Isaac Sim 使用
1. Start Isaac Sim and load the GBT C5A USD asset. | 启动 Isaac Sim 并加载 GBT C5A USD 资产。
2. Run the MoveIt core: | 启动 MoveIt 核心节点：
```bash
ros2 launch gbt_c5a_gripper_moveit_config demo.launch.py
```
*(Note: `demo.launch.py` includes `move_group` and `rviz` by default)*

---

## Configuration | 配置说明
- **SRDF**: `config/gbt_c5a.srdf`
- **Controllers | 控制器**: `config/moveit_controllers.yaml` & `config/ros2_controllers.yaml`
- **Joint Limits | 关节限制**: `config/joint_limits.yaml`

---

## Integration | 集成
This package is part of the [Isaac MoveIt C5A Project](../README.md).
本包是 [Isaac MoveIt C5A 项目](../README.md) 的一部分。
