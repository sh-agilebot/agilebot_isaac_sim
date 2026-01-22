# AGILEBOT Examples

<div align="center">

**[🔙 返回主文档](../../README.md)**

---

English | **[🇨🇳 中文文档](#)**

</div>

Isaac Sim 机器人示例集，演示 AgileBot 机器人的各种操作功能。

---

## ⚠️ 运行前必读

### 运行环境要求

在运行任何示例之前，请确保使用正确的 Python 环境：

**方法一：使用 Isaac Sim 的 Python 环境**
```bash
~/isaacsim/python.sh <script_name>.py
```

**方法二：使用 IsaacLab 环境**
```bash
conda activate isaaclab
python <script_name>.py
```

> ⚠️ **重要提示**：请确保在正确的环境中运行脚本，否则可能会出现导入错误或模块找不到的问题。

### GUI 无响应问题处理

如果在运行脚本后，没有出现错误信息但 GUI 窗口长时间无响应，这通常是 Isaac Sim 正在从网络加载资源。

**解决方案**：
1. **耐心等待**：首次运行时需要下载资源，请耐心等待 1-2 分钟
2. **使用 VPN**：如果网络连接不稳定或速度较慢，建议使用 VPN 加速资源下载
3. **配置资源缓存**：建议提前配置 Isaac Sim 的资源路径，避免每次都从网络加载资源（详见[环境配置指南](../../docs/isaacsim环境配置.md)）

---

## 示例列表

### 1. Stacking (堆垛示例)
**文件**: `stacking.py`

演示机器人堆叠立方体的功能。

**运行命令**:
```bash
# 默认使用 gbt-c5a
python stacking.py

# 指定机器人类型
python stacking.py --robot_type gbt-c7a
python stacking.py --robot_type gbt-c12a
python stacking.py --robot_type gbt-c16a

# 设置夹爪最大力矩
python stacking.py --max_force 3.0
```

**支持的机器人类型**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**可选参数**:
- `--max_force`: 夹爪最大力矩，单位N (默认: 2.0)

---

### 2. Pick and Place (抓取放置示例)
**文件**: `pick_place.py`

演示机器人抓取和放置立方体的功能。

**运行命令**:
```bash
# 默认使用 gbt-c5a
python pick_place.py

# 指定机器人类型
python pick_place.py --robot_type gbt-c7a
python pick_place.py --robot_type gbt-c12a
python pick_place.py --robot_type gbt-c16a

# 设置夹爪最大力矩
python pick_place.py --max_force 3.0
```

**支持的机器人类型**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**可选参数**:
- `--max_force`: 夹爪最大力矩，单位N (默认: 2.0)

---

### 3. Follow Target (跟随目标示例)
**文件**: `follow_target.py`

演示机器人末端执行器跟随目标移动的功能，支持可选的夹爪配置。拖动名为`Targetcube`的虚拟物体，机器人将跟随其移动。

**运行命令**:
```bash
# 默认使用 gbt-c5a，不带夹爪
python follow_target.py

# 指定机器人类型
python follow_target.py --robot_type gbt-c7a
python follow_target.py --robot_type gbt-c12a
python follow_target.py --robot_type gbt-c16a

# 使用夹爪
python follow_target.py --robot_type gbt-c5a --attach_gripper
```

**支持的机器人类型**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**可选参数**:
- `--attach_gripper`: 是否为机器人安装夹爪 (默认: False)

---

### 4. Follow Target with FK Verification (跟随目标+正运动学验证示例)
**文件**: `follow_target_with_fk_verification.py`

演示机器人末端执行器跟随目标移动的功能，并实时进行正运动学验证。该示例在跟踪目标的同时，通过正运动学计算末端位姿，并与API获取的实际位姿进行对比，验证IK和FK的准确性。拖动名为`Targetcube`的虚拟物体，机器人将跟随其移动。

**运行命令**:
```bash
# 默认使用 gbt-c5a，不带夹爪
python follow_target_with_fk_verification.py

# 指定机器人类型
python follow_target_with_fk_verification.py --robot_type gbt-c7a
python follow_target_with_fk_verification.py --robot_type gbt-c12a
python follow_target_with_fk_verification.py --robot_type gbt-c16a

# 使用夹爪
python follow_target_with_fk_verification.py --robot_type gbt-c5a --attach_gripper
```

**支持的机器人类型**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**可选参数**:
- `--attach_gripper`: 是否为机器人安装夹爪 (默认: False)

**功能特性**:
- 实时目标跟踪（使用逆运动学）
- 正运动学计算（从关节位置计算末端位姿）
- 位姿对比验证：FK vs API（验证正运动学准确性）
- 位姿对比验证：FK vs Target（验证跟踪精度）
- 周期性输出验证结果（1秒间隔）

**输出格式**:
```
[Frame XXXXX] Target: [x, y, z] | FK: [x, y, z] | API: [x, y, z] | FK-API Error: X.XXXXXX m | FK-Target Error: X.XXXXXX m
```

---

## 通用参数

所有示例都支持以下命令行参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--robot_type` | 机器人类型 | gbt-c5a |

## 前置条件

1. 确保已正确安装 Isaac Sim

## 路径设置

运行示例前需要正确配置以下路径：

### 1. RMPflow 运动控制配置

将项目中的 `motion_policy_configs/Agilebot` 目录复制到 IsaacSim 的配置目录：

```bash
cp -r agilebot_integration/motion_policy_configs/Agilebot \
  ~/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/
```

### 2. Agilebot 集成代码

将项目中的 `code/agilebot` 目录复制到 IsaacSim 扩展目录：

```bash
cp -r agilebot_integration/code/agilebot \
  ~/isaacsim/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/
```

### 3. 机器人 USD 文件

每个 demo 文件中 USD 路径的设置行号：

| Demo文件 | 行号 | USD路径 |
|---------|------|---------|
| [follow_target.py](agilebot_integration/demos/follow_target.py#L117) | 第117行 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |
| [follow_target_with_fk_verification.py](agilebot_integration/demos/follow_target_with_fk_verification.py#L126) | 第126行 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |
| [pick_place.py](agilebot_integration/demos/pick_place.py#L95) | 第95行 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |
| [stacking.py](agilebot_integration/demos/stacking.py#L97) | 第97行 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |

USD 模型可通过以下地址下载：
[agilebot_isaac_usd_assets](https://github.com/sh-agilebot/agilebot_isaac_usd_assets)

支持的机器人类型：
- gbt-c5a
- gbt-c7a
- gbt-c12a
- gbt-c16a

## 许可证

本项目遵循 Apache License 2.0 许可证。
