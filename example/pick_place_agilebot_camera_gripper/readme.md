# Wrist Camera Grasping Task Example

<div align="center">

**[🔙 Main Documentation](../../README.md)**

---

[English](./README.md) | **[🇨🇳 中文文档](./README_CN.md)**

---

</div>

This is a wrist camera grasping task example provided by Shanghai Agilebot Robotics Co., Ltd., containing complete digital assets for the robot, wrist camera, and gripper, along with a grasping and placing task implementation and video recording program.

## Features

- **Robot Digital Assets**: Complete robot model including robotic arm and gripper
- **Grasping Test Program**: Full grasping and placing task implementation based on Isaac Sim
- **Video Recording**: RGB video recording support (30fps) with non-blocking performance using multi-processing
- **Centralized Runtime Configuration**: Robot asset paths, robot prim names, end effector frame, and camera parameters are centrally managed via configuration files

## Key Files

| File | Description |
|------|-------------|
| `pick_place.py` | Main program entry point, starts simulation and executes grasping and placing task |
| `video_recorder.py` | Video recording module with non-blocking multi-process recording |
| `controllers/pick_place.py` | Grasping and placing controller |
| `tasks/pick_place.py` | Grasping and placing task definition |
| `config/robot_config.json` | Unified runtime configuration file for robot assets, prim names, end effector frame, and camera parameters |
| `config/runtime.py` | Configuration loading and helper logic shared by main program, task, IK, and RMPFlow controllers |
| `rmpflow/gbt_c5a_rmpflow_common.template.yaml` | RMPFlow template file, automatically generates temporary YAML at runtime based on `config/robot_config.json` |
| `usd/gbt-c5a_wrist_camera_gripper/gbt-c5a_wrist_camera_gripper.usd` | Robot digital asset file with priority for current configuration |

> **Tip:** Due to copyright reasons, third-party components have been removed and need to be converted by users via scripts. You can obtain the latest USD models from the [Agilebot USD Assets repository](https://github.com/sh-agilebot/agilebot_isaac_usd_assets/tree/main/gbt-c5a_wrist_camera_gripper).

## Configuration

The main runtime parameters previously scattered in the code are now centralized in `config/robot_config.json`, including:

- USD asset candidate paths
- Robot root prim path
- End effector frame name and prim name candidates
- Gripper joint names
- Camera prim name candidates
- Camera resolution

The RMPFlow controller now reads from `rmpflow/gbt_c5a_rmpflow_common.template.yaml` and automatically generates temporary YAML at runtime. Therefore, when adapting to new robot assets or prim hierarchies, prioritize modifying `config/robot_config.json`.

## Usage

### Running the Program

Run the main program to start the simulation and grasping/placing task:

```bash
conda activate isaaclab
python pick_place.py
```

### Customizing Robot Parameters

If you have replaced the robot USD asset or used a different prim hierarchy, it is recommended to modify `config/robot_config.json` first instead of changing the source code directly.

Recommended fields to check first:

1. `assets.usd_candidates`
   Lists candidate paths for robot USD assets in order. The program checks them sequentially and uses the first existing file.
2. `scene.robot_root_prim_path`
   The root prim path of the robot after loading into the scene. Subsequent searches for end effector and camera traverse downward from this path.
3. `robot.end_effector_frame_name`
   The end effector coordinate frame name passed to IK/RMPFlow. It must match the frame name in the URDF and robot description.
4. `robot.end_effector_prim_name_candidates`
   Candidate list of names used when searching for end effector prim in the USD scene. Suitable for compatibility with naming differences across different assets.
5. `camera.prim_name_candidates`
   Priority list of names for matching camera prim under the robot hierarchy. If none match, the program falls back to the first prim of Camera type.
6. `camera.resolution`
   Resolution of the wrist camera output image in format `[width, height]`. This also affects the recorded video size.

The program will automatically:
1. Start Isaac Sim simulation environment
2. Load robotic arm, wrist camera, and gripper
3. Execute grasping and placing task
4. Record RGB video to `saved_videos/captured_video_rgb.mp4`

## System Requirements

- Isaac Sim
- Python 3.x
- Dependencies: numpy, opencv-python

---

**Shanghai Agilebot Robotics Co., Ltd.** | Website: [https://www.sh-agilebot.com/](https://www.sh-agilebot.com/)
