# Pick and Place Task Example with Wrist Camera

<div align="center">

**[🔙 Back to Main Documentation](../../README.md)**

---

**[🇨🇳 中文文档](./README_CN.md)** | English

---

</div>

A wrist camera pick and place task example provided by Shanghai Agilebot Robotics Co., Ltd. This project includes a complete digital asset package featuring a robot arm with wrist-mounted camera and gripper, along with pick and place task implementation and video recording functionality.

## Features

- **Robot Digital Asset**: Complete robot model including manipulator arm and gripper
- **Pick and Place Demo**: Full implementation based on Isaac Sim
- **Video Recording**: RGB video recording support (30fps) using non-blocking multiprocessing

## Main Files

| File | Description |
|------|-------------|
| `pick_place.py` | Main entry point - launches simulation and executes pick and place task |
| `video_recorder.py` | Video recording module using multiprocessing for non-blocking operation |
| `controllers/pick_place.py` | Pick and place controller |
| `tasks/pick_place.py` | Pick and place task definition |
| `usd/gbt-c5a_camera_gripper/gbt-c5a_camera_gripper.usd` | Robot digital asset (complete model with robot, camera, and gripper) |

## Usage

### Running the Program

Launch the simulation and execute the pick and place task:

```bash
conda activate isaaclab
python pick_place.py
```

The program will automatically:
1. Launch the Isaac Sim simulation environment
2. Load the manipulator arm, wrist camera, and gripper
3. Execute the pick and place task
4. Save RGB video recording to `saved_images/captured_video_rgb.mp4`

> **Note:** The camera's field of view does not include the gripper. If you are training a VLA (Vision-Language-Action) model, please be aware of this limitation.

## System Requirements

- NVIDIA Isaac Sim
- Python 3.x
- Dependencies: `numpy`, `opencv-python`, `open3d`

## Third-Party Notice

This demo includes a camera mount adapted from [MetaIsaacGrasp](https://github.com/YitianShi/MetaIsaacGrasp), licensed under the MIT License.

---

**Shanghai Agilebot Robotics Co., Ltd.** | Website: [https://www.sh-agilebot.com/](https://www.sh-agilebot.com/)
