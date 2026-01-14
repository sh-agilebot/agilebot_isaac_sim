# Robot Digital Asset with Camera and Gripper

<div align="center">

**[🔙 Back to Main Documentation](../../README.md)**

---

**[🇨🇳 中文文档](./README_CN.md)** | English

</div>

This is a robot digital asset provided by Shanghai Agilebot Robotics Ltd., including a robot, camera, and gripper, along with a complete grasping test program.

---

## Features

- **Robot Digital Asset**: Complete robot model including robotic arm and gripper
- **Grasping Test Program**: Complete pick-and-place task implementation based on Isaac Sim
- **Video Recording**: Supports RGB video recording (30fps) using multiprocessing for non-blocking performance

## Main Files

- `pick_place_example.py` - Main program entry point, launches simulation and executes pick-and-place task
- `video_recorder.py` - Video recording module using multiprocessing for non-blocking recording
- `controllers/pick_place.py` - Pick-and-place controller
- `tasks/pick_place.py` - Pick-and-place task definition
- `usd/gbt_c5a_camera_gripper/gbt_c5a_camera_gripper.usd` - Robot digital asset file (complete model with robot, camera, and gripper)

## Usage

### Configuration

Before running the program, ensure the robot asset path is correctly configured in [tasks/pick_place.py](./tasks/pick_place.py#L65).

If the path is different, modify this absolute path to point to your actual `usd` folder location.

### Running the Program

Run the main program to start the simulation and pick-and-place task:

```bash
conda activate isaaclab
python pick_place_example.py
```

The program will automatically:
1. Launch Isaac Sim simulation environment
2. Load the robotic arm, wrist camera, and gripper
3. Execute the pick-and-place task
4. Record RGB video to `saved_images/captured_video_rgb.mp4`

## System Requirements

- Isaac Sim
- Python 3.x
- Dependencies: numpy, opencv-python, open3d

## Third-party Notice

This demo includes a camera mount adapted from
[MetaIsaacGrasp](https://github.com/YitianShi/MetaIsaacGrasp),
licensed under the MIT License.

---

**Shanghai Agilebot Robotics Ltd.** | Website: [https://www.sh-agilebot.com/](https://www.sh-agilebot.com/)
