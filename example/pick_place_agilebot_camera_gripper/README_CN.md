# 带有相机和夹爪的机器人数字资产

<div align="center">

**[🔙 返回主文档](../../README.md)**

---

English | **[🇨🇳 中文文档](#)**

</div>

这是由上海捷勃特机器人有限公司提供的手腕相机抓取任务示例，包含机器人、手腕相机和夹爪的完整数字资产，以及抓取放置任务和视频记录程序。

## 功能特性

- **机器人数字资产**：包含机械臂和夹爪的完整机器人模型
- **抓取测试程序**：基于 Isaac Sim 的完整抓取和放置任务实现
- **视频录制**：支持 RGB 视频录制（30fps），使用多进程实现非阻塞性能

## 主要文件

- `pick_place_example.py` - 主程序入口，启动仿真并执行抓取放置任务
- `video_recorder.py` - 视频录制模块，使用多进程实现非阻塞录制
- `controllers/pick_place.py` - 抓取放置控制器
- `tasks/pick_place.py` - 抓取放置任务定义
- `usd/gbt_c5a_camera_gripper/gbt_c5a_camera_gripper.usd` - 机器人数字资产文件（包含机器人、相机和夹爪的完整模型）

## 使用方法

### 配置

运行程序前，请确保在 [tasks/pick_place.py](./tasks/pick_place.py#L65) 中正确配置了机器人资源路径：



如果路径不同，请修改此绝对路径以指向您的实际 `usd` 文件夹位置。

### 运行程序

运行主程序以启动仿真和抓取放置任务：

```bash
conda activate isaaclab
python pick_place_example.py
```

程序将自动：
1. 启动 Isaac Sim 仿真环境
2. 加载机械臂、手腕相机和夹爪
3. 执行抓取放置任务
4. 将 RGB 视频录制到 `saved_images/captured_video_rgb.mp4`

## 系统要求

- Isaac Sim
- Python 3.x
- 依赖项：numpy, opencv-python, open3d

## 第三方声明

本演示包含改编自 [MetaIsaacGrasp](https://github.com/YitianShi/MetaIsaacGrasp) 的相机支架，
采用 MIT 许可证。

---

**上海捷勃特机器人有限公司** | 官网：[https://www.sh-agilebot.com/](https://www.sh-agilebot.com/)
