# IsaacSim MoveIt 控制捷勃特机器人示例

<div align="center">

**[🔙 返回主文档](../../README.md)**

---

English | **[🇨🇳 中文文档](#)**

</div>

本案例演示了如何使用 **MoveIt** 控制 IsaacSim 中带有夹爪的捷勃特机器人。
夹爪使用 **Robotiq** 开源 URDF。

---

## 原理简述

`topic_based_ros2_control` 将 MoveIt 的规划结果通过指定 **topic** 发送，并接收指定 **topic** 的反馈，从而实现对机器人动作的控制。

需要安装的软件包：
```bash
sudo apt install ros-humble-topic-based-ros2-control
```

---

## 快速使用指南

## 配置python3.11版本的ros2

参考：https://github.com/camopel/isaacsim-ros2-python3.11
> ros2-humble安装python3.10版本,isaacsim安装python3.11版本,因此需要配置python3.11版本的ros2

### 1. 将项目添加到 ROS 2 工作空间（终端1）

```bash
source /opt/ros/humble/setup.bash # python3.10的ros2
cp -r isaacsim_moveit {你的工作空间}/src
cd {你的工作空间}/
colcon build
source install/setup.bash
```

### 2. 启动 MoveIt 2

```bash
ros2 launch gbt_gripper_moveit_config demo.launch.py
```

### 3. 启动 IsaacSim(终端2)

```bash
source /opt/ros/humble_ws/install/setup.bash # python3.11的ros2
cd ~/isaacsim
./isaac-sim.sh
```

#### 在 IsaacSim 中加载场景

1. 点击 `File -> Open`
2. 选择工作空间中的 `isaacsim_moveit` 文件夹
3. 打开上述文件夹下的 `./gbt_c7a_moveit.usd`
4. 点击 **Play** 按钮启动模拟

---

## 4. 控制机器人

完成上述步骤后，即可通过 **RViz** 中的 **MoveIt** 控制 IsaacSim 中的捷勃特机器人。
