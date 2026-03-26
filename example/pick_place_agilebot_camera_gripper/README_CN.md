# 手腕相机抓取任务示例

<div align="center">

**[🔙 返回主文档](../../README_CN.md)**

---

[English](./README.md) | **[🇨🇳 中文文档](#)**

---

</div>

这是由上海捷勃特机器人有限公司提供的手腕相机抓取任务示例，包含机器人、手腕相机和夹爪的完整数字资产，以及抓取放置任务和视频记录程序。

## 功能特性

- **机器人数字资产**：包含机械臂和夹爪的完整机器人模型
- **抓取测试程序**：基于 Isaac Sim 的完整抓取和放置任务实现
- **视频录制**：支持 RGB 视频录制（30fps），使用多进程实现非阻塞性能
- **集中式运行配置**：机器人资产路径、机器人 prim 名称、末端执行器 frame 和相机参数统一由配置文件管理

## 主要文件

| 文件 | 描述 |
|------|------|
| `pick_place.py` | 主程序入口，启动仿真并执行抓取放置任务 |
| `video_recorder.py` | 视频录制模块，使用多进程实现非阻塞录制 |
| `controllers/pick_place.py` | 抓取放置控制器 |
| `tasks/pick_place.py` | 抓取放置任务定义 |
| `config/robot_config.json` | 统一管理机器人资产、prim 名称、末端执行器 frame 与相机参数的运行配置文件 |
| `config/runtime.py` | 主程序、task、IK 和 RMPFlow 控制器共用的配置加载与辅助逻辑 |
| `rmpflow/gbt_c5a_rmpflow_common.template.yaml` | RMPFlow 模板文件，运行时会根据 `config/robot_config.json` 自动生成临时 YAML |
| `usd/gbt-c5a_wrist_camera_gripper/gbt-c5a_wrist_camera_gripper.usd` | 当前配置优先使用的机器人数字资产文件 |

> **小贴士：** 由于版权原因，第三方组件已删除，需要用户通过脚本自行转换。您可以从 [Agilebot USD Assets 仓库](https://github.com/sh-agilebot/agilebot_isaac_usd_assets/tree/main/gbt-c5a_wrist_camera_gripper) 获取最新的 USD 模型。

## 配置说明

原来分散在代码中的主要运行参数，现在集中放在 `config/robot_config.json` 中，包括：

- USD 资产候选路径
- 机器人根 prim 路径
- 末端执行器 frame 名称和 prim 名称候选
- 夹爪关节名称
- 相机 prim 名称候选
- 相机分辨率

RMPFlow 控制器现在会读取 `rmpflow/gbt_c5a_rmpflow_common.template.yaml`，并在运行时自动生成临时 YAML，因此当你适配新的机器人资产或 prim 层级时，优先修改 `config/robot_config.json` 即可。

## 使用方法

### 运行程序

运行主程序以启动仿真和抓取放置任务：

```bash
conda activate isaaclab
python pick_place.py
```

### 自定义机器人参数

如果你替换了机器人 USD 资产，或者使用了不同的 prim 层级结构，建议先修改 `config/robot_config.json`，而不是直接改源码。

建议优先检查这些字段：

1. `assets.usd_candidates`
   按顺序列出机器人 USD 资产候选路径。程序会依次检查，找到第一个存在的文件就使用它。
2. `scene.robot_root_prim_path`
   机器人加载到场景后的根 prim 路径。后续查找末端执行器和相机时，都会从这个路径向下遍历。
3. `robot.end_effector_frame_name`
   传给 IK / RMPFlow 的末端执行器坐标系名称。它必须和 URDF、robot description 里的 frame 名称一致。
4. `robot.end_effector_prim_name_candidates`
   在 USD 场景里查找末端执行器 prim 时使用的名称候选列表。适合兼容不同资产里的命名差异。
5. `camera.prim_name_candidates`
   在机器人层级下查找相机 prim 时优先匹配的名称列表。如果都没匹配到，程序会退回到第一个 Camera 类型 prim。
6. `camera.resolution`
   手腕相机输出图像的分辨率，格式为 `[宽, 高]`。这也会影响录制视频的尺寸。

程序将自动：
1. 启动 Isaac Sim 仿真环境
2. 加载机械臂、手腕相机和夹爪
3. 执行抓取放置任务
4. 将 RGB 视频录制到 `saved_videos/captured_video_rgb.mp4`



## 系统要求

- Isaac Sim
- Python 3.x
- 依赖项：numpy, opencv-python



---

**上海捷勃特机器人有限公司** | 官网：[https://www.sh-agilebot.com/](https://www.sh-agilebot.com/)
