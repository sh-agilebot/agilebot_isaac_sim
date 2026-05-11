# GBT C5A Wrist Camera Gripper

[中文说明 / Chinese](README.zh-CN.md)

This repository provides a combined URDF for the GBT C5A arm, a wrist camera mount, and a Robotiq 2F-140 gripper, plus a script-first workflow for generating USD assets for Isaac Sim.

The documented workflow in this repository is script-only. GUI import steps are intentionally omitted so the process stays reproducible for open-source users.

## Repository Layout

- `urdf/gbt_c5a.urdf`: Combined robot URDF (Arm + Gripper)
- `launch/display.launch.py`: Standard ROS 2 launch file for RViz preview
- `rviz/display.rviz`: RViz configuration for visualization
- `meshes/visual/`: Visual meshes used by the URDF
- `meshes/collision/`: Collision meshes used by the URDF
- `scripts/setup_robotiq_meshes.sh`: Installs required Robotiq STL files
- `scripts/convert_urdf_to_usd.py`: Isaac Sim conversion utility

## Prerequisites

- Isaac Sim or Isaac Lab Python environment
- Robotiq 2F-140 STL files from a lawful source

Important notes:

- This repository does not redistribute Robotiq STL assets.
- `scripts/convert_urdf_to_usd.py` is not a plain Python utility. It depends on Isaac runtime modules such as `isaacsim`, `isaaclab`, `omni.kit.commands`, and `pxr`.
- The camera postprocess step adds the Orbbec Gemini2 USD by remote asset URL. Your environment must be able to access that asset for the default workflow to succeed.

If you manage Isaac Lab with Miniforge, activate that environment before running any commands below:

```bash
source ~/miniforge3/bin/activate isaaclab
```

If your Miniforge installation lives somewhere else, replace `~/miniforge3` with the actual path.

## Quick Start

1. Install the required Robotiq meshes (Source: [ros-industrial-attic/robotiq](https://github.com/ros-industrial-attic/robotiq/tree/kinetic-devel/robotiq_2f_140_gripper_visualization)):
```bash
bash scripts/setup_robotiq_meshes.sh /path/to/robotiq_stl_dir
```

Required STL files:

- `robotiq_arg2f_base_link.stl`
- `robotiq_arg2f_coupling.stl`
- `robotiq_arg2f_140_outer_knuckle.stl`
- `robotiq_arg2f_140_outer_finger.stl`
- `robotiq_arg2f_140_inner_knuckle.stl`
- `robotiq_arg2f_140_inner_finger.stl`

2. Run the scripted import inside an Isaac Sim or Isaac Lab Python environment:

Recommended one-shot command: this makes the current project-recommended parameters explicit so the result is easier to reproduce.

```bash
python scripts/convert_urdf_to_usd.py \
  urdf/gbt_c5a.urdf \
  urdf/gbt_c5a.usd \
  --drive-type force \
  --arm-natural-frequency 300.0 \
  --gripper-natural-frequency 300.0 \
  --mimic-natural-frequency 2500.0 \
  --damping-ratio 0.005 \
  --finger-max-force 200.0 \
  --solver-position-iterations 64 \
  --solver-velocity-iterations 16
```

If you do not need the camera (e.g., in a network-isolated environment or if visual input is not required), use the `--skip-camera` flag:

```bash
python scripts/convert_urdf_to_usd.py \
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

This recommended command:

- runs headless
- keeps `fix_base=true`
- keeps `merge_fixed_joints=false`
- automatically attaches the camera, adds colliders, and updates the physics layer
- removes the camera light automatically during postprocess

```bash
python scripts/convert_urdf_to_usd.py
```

By default, this command:

- reads `urdf/gbt_c5a.urdf`
- writes `urdf/gbt_c5a.usd`
- runs headless
- keeps `fix_base=true`
- keeps `merge_fixed_joints=false`
- automatically runs postprocess to attach the camera, add colliders, and update the physics layer
- removes the camera `RectLight` automatically during postprocess

## Script Workflow

Generate USD with explicit input and output paths:

```bash
python scripts/convert_urdf_to_usd.py \
  urdf/gbt_c5a.urdf \
  urdf/gbt_c5a.usd
```

Postprocess an existing USD stage:

```bash
python scripts/convert_urdf_to_usd.py postprocess \
  urdf/gbt_c5a.usd
```

Postprocess always attaches the camera, removes the camera `RectLight`, adds colliders, and updates the nearby physics layer.

Add colliders to an existing USD stage without re-running URDF import:

```bash
python scripts/convert_urdf_to_usd.py postprocess \
  urdf/gbt_c5a.usd
```

Collider edits always redirect to the nearby `*_physics.usd` layer when one exists and prefer `/colliders` as the search root. Collider sync is always enabled and uses the built-in defaults.

Useful help commands:

```bash
python scripts/convert_urdf_to_usd.py --help
python scripts/convert_urdf_to_usd.py postprocess --help
```

## Key Options

Common import options:

- `--no-headless`: show the Isaac app window during import
- `--no-fix-base`: import the robot with a floating base
- `--merge-fixed-joints`: not recommended for this robot
- `--drive-type force|acceleration`
- `--arm-natural-frequency`
- `--gripper-natural-frequency`
- `--mimic-natural-frequency`
- `--damping-ratio`
- `--finger-max-force`

Common postprocess options:

- `--urdf <path>`: use an explicit URDF file when syncing colliders
- `--physics-stage <path>`: use an explicit physics layer
- `--no-remove-camera-rect-light`: keep the camera `RectLight` instead of deactivating it
- `--skip-camera`: skip camera attachment and related post-processing
- `--skip-finger-friction`: skip finger material creation and binding
- `--skip-articulation-config`: skip articulation solver tuning

Camera attachment, `RectLight` removal, collider sync, and physics-layer updates always run with the built-in workflow defaults.

Default script values:

- arm natural frequency: `300.0`
- gripper natural frequency: `300.0`
- mimic natural frequency: `2500.0`
- damping ratio: `0.02`
- finger max force: `5000.0`
- drive type: `force`
- static friction: `1.2`
- dynamic friction: `1.1`
- restitution: `0.0`
- solver position iterations: `96`
- solver velocity iterations: `8`

## Outputs

After a successful import, the generated files usually include:

- `urdf/gbt_c5a.usd`
- `urdf/configuration/gbt_c5a_base.usd`
- `urdf/configuration/gbt_c5a_physics.usd`
- `urdf/configuration/gbt_c5a_robot.usd`
- `urdf/configuration/gbt_c5a_sensor.usd`

## Validation

Minimum validation after changes:

- run `python3 -m py_compile scripts/convert_urdf_to_usd.py`
- run `python3 scripts/convert_urdf_to_usd.py --help`
- run `python3 scripts/convert_urdf_to_usd.py postprocess --help`
- confirm the top-level USD was generated
- confirm the physics layer exists
- (if not skipped) open the USD in Isaac Sim and check that the camera view is available
- (if not skipped) confirm the gripper is visible in `Stream_rgb`
- if the camera `RectLight` still appears unexpectedly, rerun `scripts/convert_urdf_to_usd.py postprocess` with `--no-remove-camera-rect-light` omitted and confirm the light prim is inactive

## Troubleshooting

- If the mesh setup script fails, verify that all 6 required STL files exist with the exact expected filenames.
- If the conversion script fails immediately, make sure you are running it inside an Isaac Sim or Isaac Lab Python environment.
- If camera attachment fails and you need the camera, verify that your Isaac Sim environment can access the remote Orbbec Gemini2 asset. If you do not need the camera, ensure you have used the `--skip-camera` flag.
- If the camera light is still visible after import, rerun `python scripts/convert_urdf_to_usd.py postprocess` and make sure `--no-remove-camera-rect-light` is not set.
- If the script cannot find the physics layer during postprocess, rerun with `--physics-stage <path>`.
- If collider sync cannot find your robot links, rerun `scripts/convert_urdf_to_usd.py postprocess` and inspect the generated stage hierarchy. The built-in sync always prefers the nearby physics layer and `/colliders` when present.
- Do not enable fixed-joint merging for this robot. Keeping `merge_fixed_joints=false` preserves the expected camera and gripper hierarchy.

## ROS 2 Control Setup (ActionGraph) - Shortcut Method

The easiest way to set up the ActionGraph in Isaac Sim for ROS 2 joint control is using the built-in "shortcut" tool. This automatically creates the necessary ActionGraph nodes.

### Setup Steps:

1.  **Open Tool**: From the top menu in Isaac Sim, select **Tools -> Robotics -> ROS 2 OmniGraphs -> JointStates**.
2.  **Configure Parameters**: In the popup window, set the following:
    *   **Articulation Prim**: Click the button and select `base_link` in the stage tree (usually at `/GBT_C5A_gripper/base_link`).
    *   **Publish Topic**: Set to `/isaac_joint_states`.
    *   **Subscribe Topic**: Set to `/isaac_joint_commands`.
    *   **Add Articulation Controller**: Check this box (required for arm control).
3.  **Generate Graph**: Click **OK**. An ActionGraph with all required nodes will be created.

### Verification & Running:

1.  Press **Play** in Isaac Sim.
2.  Run `ros2 topic list` in a terminal to confirm the following topics are active:
    *   `/isaac_joint_states`
    *   `/isaac_joint_commands`
3.  **MoveIt 2 Integration**: This project is configured with the `TopicBasedSystem` plugin in `gbt_c5a.ros2_control.xacro`, which automatically interfaces with these topics.

For more details, refer to the [official NVIDIA tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html).

## Known Limitations

- The workflow depends on Isaac runtime modules and cannot be fully executed in a generic Python environment.
- The default camera asset reference depends on remote Isaac asset availability.
## Integration
This package is part of the [Isaac MoveIt C5A Project](../README.md).
