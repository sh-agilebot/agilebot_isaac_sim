# GBT C5A Wrist Camera Gripper

[English README](README.md)

本仓库提供一个组合 URDF，包含 GBT C5A 机械臂、腕部相机安装支架和 Robotiq 2F-140 夹爪，并提供一套面向 Isaac Sim 的脚本化 USD 生成流程。

本文档只保留脚本方式，不再介绍 GUI 导入或手工挂载步骤，目的是让开源用户能够复现同一套流程。

## 仓库结构

- `urdf/gbt_c5a.urdf`：组合机器人 URDF
- `meshes/visual/`：URDF 使用的可视化网格
- `meshes/collision/`：URDF 使用的碰撞网格
- `scripts/setup_robotiq_meshes.sh`：把所需 Robotiq STL 安装到本仓库
- `scripts/convert_urdf_to_usd.py`：负责 URDF 导入、相机挂载、删除相机 `RectLight`、碰撞体补齐和 physics layer 更新

## 前置条件

- Isaac Sim 或 Isaac Lab 的 Python 环境
- 你已经从合法来源准备好 Robotiq 2F-140 STL 文件

请注意：

- 本仓库不分发 Robotiq STL 资源。
- `scripts/convert_urdf_to_usd.py` 不是普通 Python 脚本，它依赖 `isaacsim`、`isaaclab`、`omni.kit.commands`、`pxr` 等 Isaac 运行时模块。
- 相机后处理默认通过远程资产 URL 挂载 Orbbec Gemini2 相机。默认流程要求当前环境能够访问该远程资源。

如果你是通过 Miniforge 管理 Isaac Lab 环境，建议先激活环境再执行下面所有命令：

```bash
source ~/miniforge3/bin/activate isaaclab
```

如果你的 Miniforge 安装路径不是 `~/miniforge3`，请改成你自己的实际路径。

## 快速开始

1. 安装所需 Robotiq 网格：

> https://github.com/ros-industrial-attic/robotiq/tree/kinetic-devel/robotiq_2f_140_gripper_visualization
```bash
bash scripts/setup_robotiq_meshes.sh /path/to/robotiq_stl_dir
```

需要准备的 STL 文件：

- `robotiq_arg2f_base_link.stl`
- `robotiq_arg2f_coupling.stl`
- `robotiq_arg2f_140_outer_knuckle.stl`
- `robotiq_arg2f_140_outer_finger.stl`
- `robotiq_arg2f_140_inner_knuckle.stl`
- `robotiq_arg2f_140_inner_finger.stl`

2. 在 Isaac Sim 或 Isaac Lab 的 Python 环境中运行导入脚本：

推荐直接使用这条一键命令，它把当前项目默认建议保留的参数都显式写出来，便于复现：

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

如果你不需要挂载相机（例如在没有网络环境或不需要视觉输入的情况下），请添加 `--skip-camera` 参数：

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

这套推荐参数会：

- 以 headless 模式运行
- 保持 `fix_base=true`
- 保持 `merge_fixed_joints=false`
- 导入完成后自动挂载相机、补齐碰撞体，并更新 physics layer
- 会在后处理阶段自动去掉相机自带灯光

```bash
python scripts/convert_urdf_to_usd.py
```

默认情况下，这条命令会：

- 读取 `urdf/gbt_c5a.urdf`
- 生成 `urdf/gbt_c5a.usd`
- 以 headless 模式运行
- 保持 `fix_base=true`
- 保持 `merge_fixed_joints=false`
- 在导入完成后自动执行后处理，包括挂载相机、补齐碰撞体，以及更新 physics layer
- 会在后处理阶段自动去掉相机 `RectLight`

## 脚本工作流

显式指定输入 URDF 和输出 USD：

```bash
python scripts/convert_urdf_to_usd.py \
  urdf/gbt_c5a.urdf \
  urdf/gbt_c5a.usd
```

对已有 USD 单独执行后处理：

```bash
python scripts/convert_urdf_to_usd.py postprocess \
  urdf/gbt_c5a.usd
```

后处理会固定执行相机挂载、删除相机 `RectLight`、碰撞体补齐，以及附近 physics layer 更新。

不给 URDF 重新导入，单独给已有 USD 补碰撞体：

```bash
python scripts/convert_urdf_to_usd.py postprocess \
  urdf/gbt_c5a.usd
```

碰撞体编辑会自动切到附近的 `*_physics.usd` layer，并优先在 `/colliders` 下查找碰撞几何体。碰撞体补齐始终开启，并固定使用脚本内建默认值。

帮助命令：

```bash
python scripts/convert_urdf_to_usd.py --help
python scripts/convert_urdf_to_usd.py postprocess --help
```

## 关键参数

常用导入参数：

- `--no-headless`：导入时显示 Isaac 窗口
- `--no-fix-base`：以浮动底座方式导入
- `--merge-fixed-joints`：不建议对该机器人启用
- `--drive-type force|acceleration`
- `--arm-natural-frequency`
- `--gripper-natural-frequency`
- `--mimic-natural-frequency`
- `--damping-ratio`
- `--finger-max-force`

常用后处理参数：

- `--urdf <path>`：在补 collider 时显式指定 URDF 文件
- `--physics-stage <path>`：显式指定 physics layer
- `--no-remove-camera-rect-light`：保留相机 `RectLight`，不在后处理阶段自动禁用
- `--skip-camera`：跳过相机挂载与相关后处理
- `--skip-finger-friction`：跳过手指摩擦材质创建与绑定
- `--skip-articulation-config`：跳过 articulation 求解器参数更新

相机挂载、`RectLight` 删除、碰撞体补齐和 physics layer 更新都固定使用脚本内建工作流默认值。

脚本默认值：

- 机械臂 natural frequency：`300.0`
- 夹爪主动关节 natural frequency：`300.0`
- mimic joints natural frequency：`2500.0`
- damping ratio：`0.02`
- finger max force：`5000.0`
- drive type：`force`
- static friction：`1.2`
- dynamic friction：`1.1`
- restitution：`0.0`
- solver position iterations：`96`
- solver velocity iterations：`8`

## 输出文件

导入成功后，通常会生成这些文件：

- `urdf/gbt_c5a.usd`
- `urdf/configuration/gbt_c5a_base.usd`
- `urdf/configuration/gbt_c5a_physics.usd`
- `urdf/configuration/gbt_c5a_robot.usd`
- `urdf/configuration/gbt_c5a_sensor.usd`

## 验证建议

最少建议完成以下检查：

- 运行 `python3 -m py_compile scripts/convert_urdf_to_usd.py`
- 运行 `python3 scripts/convert_urdf_to_usd.py --help`
- 运行 `python3 scripts/convert_urdf_to_usd.py postprocess --help`
- 确认顶层 USD 已生成
- 确认 physics layer 文件存在
- （如果未跳过相机）在 Isaac Sim 中打开生成的 USD，确认相机视图可用
- （如果未跳过相机）确认 `Stream_rgb` 中能看到夹爪
- 如果 `RectLight` 仍然意外可见，可以重新执行 `scripts/convert_urdf_to_usd.py postprocess`，并确认没有启用 `--no-remove-camera-rect-light`

## 排障说明

- 如果网格安装脚本报错，先检查 6 个 STL 文件名是否完全匹配。
- 如果转换脚本一开始就失败，通常是因为没有在 Isaac Sim 或 Isaac Lab 的 Python 环境中运行。
- 如果相机挂载失败且你需要相机，先检查当前 Isaac Sim 环境是否能访问远程 Orbbec Gemini2 资产。如果你不需要相机，请确保启用了 `--skip-camera`。
- 如果导入后仍然能看到相机灯光，请重新运行 `python scripts/convert_urdf_to_usd.py postprocess`，并确认没有启用 `--no-remove-camera-rect-light`。
- 如果后处理阶段找不到 physics layer，请通过 `--physics-stage <path>` 显式传入。
- 如果补碰撞体找不到机器人 link，可以优先重跑 `scripts/convert_urdf_to_usd.py postprocess`，再检查生成出来的 stage 层级。内建补齐逻辑会优先使用附近的 physics layer 和 `/colliders` 路径。
- 不要为该机器人启用 fixed joint merge。保持 `merge_fixed_joints=false` 才能保留预期的相机和夹爪层级。

## ROS 2 控制设置 (ActionGraph) - 快捷方式

要在 Isaac Sim 中通过 ROS 2 控制机器人关节，最简单的“快捷方式”是使用内置的工具栏。这会自动创建所需的 ActionGraph 节点。

### 设置步骤：

1.  **打开工具**：在 Isaac Sim 顶部菜单栏中，选择 **Tools -> Robotics -> ROS 2 OmniGraphs -> JointStates**。
2.  **配置参数**：在弹出的窗口中进行以下设置：
    *   **Articulation Prim**：点击右侧的按钮，在场景树中选择 `base_link`（通常路径为 `/GBT_C5A_gripper/base_link`）。
    *   **Publish Topic**：设置为 `/isaac_joint_states`。
    *   **Subscribe Topic**：设置为 `/isaac_joint_commands`。
    *   **Add Articulation Controller**：勾选此项（这是控制机械臂所必须的）。
3.  **生成图**：点击 **OK**。系统会自动创建一个包含所有必要节点的 ActionGraph。

### 验证与运行：

1.  点击 **Play**。
2.  在终端运行 `ros2 topic list`，确认看到以下话题：
    *   `/isaac_joint_states`
    *   `/isaac_joint_commands`
3.  **MoveIt 2 适配**：本项目已在 `gbt_c5a.ros2_control.xacro` 中配置了 `TopicBasedSystem` 插件，它会自动对接这两个话题。

更多详细信息请参考 [NVIDIA 官方教程](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)。

## 已知限制

- 这套流程依赖 Isaac 运行时模块，不能在通用 Python 环境中完整执行。
- 默认相机引用依赖远程 Isaac 资产可访问。
- Robotiq 资源需要用户自行获取。
- 本文档只覆盖脚本工作流，不包含 GUI 操作说明。

## 项目集成
本包是 [Isaac MoveIt C5A 项目](../README.md) 的一部分。
