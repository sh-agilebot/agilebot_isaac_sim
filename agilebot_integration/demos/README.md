# AGILEBOT Examples

<div align="center">

**[🔙 Back to Main Documentation](../../README.md)**

---

**[English](#)** | 🇨🇳 中文文档

</div>

Isaac Sim robot demonstration set showcasing various operational functions of AgileBot robots.

---

## ⚠️ Important: Read Before Running

### Runtime Environment Requirements

Before running any examples, ensure you are using the correct Python environment:

**Method 1: Using Isaac Sim's Python Environment**
```bash
~/isaacsim/python.sh <script_name>.py
```

**Method 2: Using IsaacLab Environment**
```bash
conda activate isaaclab
python <script_name>.py
```

> ⚠️ **Important Note**: Make sure to run scripts in the correct environment, otherwise you may encounter import errors or missing module issues.

### GUI Unresponsive Issue

If after running a script there are no error messages but the GUI window remains unresponsive for an extended period, this is usually because Isaac Sim is loading resources from the network.

**Solutions**:
1. **Be Patient**: First-time runs require resource downloads; please wait 1-2 minutes
2. **Use VPN**: If network connection is unstable or slow, use a VPN to accelerate resource downloads
3. **Configure Resource Caching**: It's recommended to set up Isaac Sim's resource path in advance to avoid network resource loading delays (see [Environment Configuration Guide](../../docs/isaacsim_environment_setup.md))

---

## Example List

### 1. Stacking
**File**: `stacking.py`

Demonstrates the robot's cube stacking functionality.

**Run Commands**:
```bash
# Default uses gbt-c5a
python stacking.py

# Specify robot type
python stacking.py --robot_type gbt-c7a
python stacking.py --robot_type gbt-c12a
python stacking.py --robot_type gbt-c16a

# Set gripper maximum torque
python stacking.py --max_force 3.0
```

**Supported Robot Types**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**Optional Parameters**:
- `--max_force`: Gripper maximum torque in N (default: 2.0)

---

### 2. Pick and Place
**File**: `pick_place.py`

Demonstrates the robot's object grasping and placement functionality.

**Run Commands**:
```bash
# Default uses gbt-c5a
python pick_place.py

# Specify robot type
python pick_place.py --robot_type gbt-c7a
python pick_place.py --robot_type gbt-c12a
python pick_place.py --robot_type gbt-c16a

# Set gripper maximum torque
python pick_place.py --max_force 3.0
```

**Supported Robot Types**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**Optional Parameters**:
- `--max_force`: Gripper maximum torque in N (default: 2.0)

---

### 3. Follow Target
**File**: `follow_target.py`

Demonstrates the robot's end-effector following a moving target with optional gripper configuration. Drag the virtual object named `Targetcube` to make the robot follow its movement.

**Run Commands**:
```bash
# Default uses gbt-c5a without gripper
python follow_target.py

# Specify robot type
python follow_target.py --robot_type gbt-c7a
python follow_target.py --robot_type gbt-c12a
python follow_target.py --robot_type gbt-c16a

# Use gripper
python follow_target.py --robot_type gbt-c5a --attach_gripper
```

**Supported Robot Types**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**Optional Parameters**:
- `--attach_gripper`: Whether to attach gripper to robot (default: False)

---

### 4. Follow Target with FK Verification
**File**: `follow_target_with_fk_verification.py`

Demonstrates the robot's end-effector following a moving target with real-time forward kinematics verification. This example verifies the accuracy of IK and FK by comparing the forward kinematics computed end-effector pose with the actual pose obtained from the API while tracking the target. Drag the virtual object named `Targetcube` to make the robot follow its movement.

**Run Commands**:
```bash
# Default uses gbt-c5a without gripper
python follow_target_with_fk_verification.py

# Specify robot type
python follow_target_with_fk_verification.py --robot_type gbt-c7a
python follow_target_with_fk_verification.py --robot_type gbt-c12a
python follow_target_with_fk_verification.py --robot_type gbt-c16a

# Use gripper
python follow_target_with_fk_verification.py --robot_type gbt-c5a --attach_gripper
```

**Supported Robot Types**: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a

**Optional Parameters**:
- `--attach_gripper`: Whether to attach gripper to robot (default: False)

**Features**:
- Real-time target tracking (using inverse kinematics)
- Forward kinematics computation (calculating end-effector pose from joint positions)
- Pose verification: FK vs API (verifying forward kinematics accuracy)
- Pose verification: FK vs Target (verifying tracking precision)
- Periodic output of verification results (1-second interval)

**Output Format**:
```
[Frame XXXXX] Target: [x, y, z] | FK: [x, y, z] | API: [x, y, z] | FK-API Error: X.XXXXXX m | FK-Target Error: X.XXXXXX m
```

---

## Common Parameters

All examples support the following command-line parameters:

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `--robot_type` | Robot type | gbt-c5a |

## Prerequisites

1. Ensure Isaac Sim is correctly installed

## Path Configuration

Before running examples, ensure the following paths are correctly configured:

### 1. RMPflow Motion Control Configuration

Copy the `motion_policy_configs/Agilebot` directory from the project to your IsaacSim configuration directory:

```bash
cp -r agilebot_integration/motion_policy_configs/Agilebot \
  ~/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/
```

### 2. Agilebot Integration Code

Copy the `code/agilebot` directory from the project to your IsaacSim extensions directory:

```bash
cp -r agilebot_integration/code/agilebot \
  ~/isaacsim/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/
```

### 3. Robot USD Files

USD path setting line numbers in each demo file:

| Demo File | Line Number | USD Path |
|-----------|-------------|-----------|
| [follow_target.py](agilebot_integration/demos/follow_target.py#L117) | Line 117 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |
| [follow_target_with_fk_verification.py](agilebot_integration/demos/follow_target_with_fk_verification.py#L126) | Line 126 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |
| [pick_place.py](agilebot_integration/demos/pick_place.py#L95) | Line 95 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |
| [stacking.py](agilebot_integration/demos/stacking.py#L97) | Line 97 | `/home/gbt/ws/usd/{robot_type}/{robot_type}.usd` |

USD models can be downloaded from:
[agilebot_isaac_usd_assets](https://github.com/sh-agilebot/agilebot_isaac_usd_assets)

Supported robot types:
- gbt-c5a
- gbt-c7a
- gbt-c12a
- gbt-c16a

## License

This project is licensed under the Apache License 2.0.
