# 更新日志

本项目的所有重要更改都将在此文件中记录。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
本项目遵循 [语义化版本控制](https://semver.org/lang/zh-CN/)规范。

---

## [0.0.2] - 2026-02-05

### 修复
- **手腕相机抓取示例纹理丢失bug**：通过添加缺失的纹理文件（Image_Joint.png、Image_Link.png、Image_Logo.png）并更新USD资源路径结构，修复了手腕相机抓取放置示例中的纹理丢失问题
- 通过将USD路径从 `gbt_c5a_camera_gripper` 更新为 `gbt-c5a_camera_gripper` 并正确绑定纹理，改善了机器人资源加载

### 变更
- **文档链接**：修复了手腕相机抓取示例文档中的导航链接，使其指向正确的语言版本（README_CN.md现在正确链接到主中文文档）
- **代码组织**：将主入口脚本从 `pick_place_example.py` 重命名为 `pick_place.py`，以保持一致性
- **URDF配置**：将URDF文件从 `gbt-c5a.urdf` 重命名为 `gbt-c5a_camera_gripper.urdf`，以更好地反映机器人配置
- **文档结构**：改进README文件的格式和组织（添加表格结构、依赖项代码格式、区段层级）
- **代码注释**：增强 `tasks/pick_place.py` 中的代码注释，提高可维护性和可读性

### 新增
- **VLA模型说明**：在英文和中文文档中添加了VLA模型训练的相机视野限制说明

---

## [0.0.3] - 2026-03-11

### 变更
- **示例文档说明更新**：在 `isaacsim_moveit` 和 `pick_place_agilebot_camera_gripper` 两个示例的中英文 README 中补充版权提示，说明第三方组件已移除，并引导用户从 [Agilebot USD Assets 仓库](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) 获取最新 USD 模型

### 已移除
- **第三方夹爪相关资源**：由于第三方夹爪版权原因，移除了 `example/isaacsim_moveit` 中随仓库分发的 Robotiq 夹爪网格、USD 配置、场景 USD 及相关纹理/环境资源
- **手腕相机夹爪示例 USD 资源**：移除了 `example/pick_place_agilebot_camera_gripper` 中包含第三方夹爪的 USD 模型及关联纹理资源
- **changelog 自动同步脚本与工作流**：移除了 `.github/scripts/sync_changelog.py`、`.github/workflows/sync-changelog.yml` 及对应说明文档

---

## [未发布]

## [0.0.4] - 2026-03-26

### 新增
- **集中式运行时配置**：新增 `config/robot_config.json`，集中管理机器人资源路径、prim名称、末端执行器框架、夹爪关节名称及相机参数
- **配置加载模块**：新增 `config/runtime.py`，提供配置加载辅助函数，供主程序、任务、IK和RMPFlow控制器共享使用
- **RMPFlow XRDF配置**：新增 `gbt_c5a_camera_gripper_robot_description.xrdf`，用于RMPFlow运动策略配置
- **GitIgnore文件**：为手腕相机夹爪示例添加了 `.gitignore` 文件

### 变更
- **配置驱动架构**：重构 `pick_place.py`、`tasks/pick_place.py`、`controllers/ik_solver.py` 和 `controllers/rmpflow_controller.py`，改为从 `config/robot_config.json` 读取参数，而非硬编码
- **RMPFlow模板系统**：从静态的 `gbt_c5a_rmpflow_common.yaml` 改为动态模板系统，根据 `config/robot_config.json` 在运行时自动生成临时YAML文件
- **文档更新**：更新 README.md，添加配置管理说明，解释如何通过 `robot_config.json` 自定义机器人参数
- **IK求解器文档**：增强 `ik_solver.py` 的文档字符串，提高可维护性和可读性

### 已移除
- **静态RMPFlow YAML**：移除 `gbt_c5a_rmpflow_common.yaml`，已被动态模板系统取代

---

## [0.0.1] - 2025-01-09

### 新增
- Agilebot IsaacSim 集成初始发布
- Agilebot 机器人的核心集成代码
- gbt-c5a、gbt-c7a、gbt-c12a、gbt-c16a 的 RMPflow 运动控制配置
- 示例演示：
  - 堆垛任务
  - 抓取放置任务
  - 跟随目标任务
  - 带正运动学验证的跟随之目标任务
- 带 ROS 2 Humble 的 MoveIt 2 集成示例
- 带有手腕相机的抓取放置任务演示，演示完整的抓取放置任务流程和视频录制
- 双语种文档（英文/中文）


### 变更
- 更新了 Isaac Sim 环境配置文档至 5.1 版本：
  - Python 版本要求改为 3.11（原为 3.10）
  - 系统要求更新（推荐 Ubuntu 22.04/Windows 11）
  - NVIDIA 驱动版本更新至 580+ 系列（Linux: 580.65.06+, Windows: 580.88+）
  - 添加了 Isaac Lab 资源缓存配置
- 更新了演示脚本，移除 `--help` 命令（避免被 Isaac 拦截）
- 同步了中英文文档
- 更新了 README 和 README_CN.md，添加到环境配置指南的链接
- 增强了 README，添加环境设置前置条件

### 已弃用
- 无

### 已移除
- 无

### 修复
- 无

### 安全
- 无

---

## 变更类型

- `Added` 或 `新增` - 用于新功能
- `Changed` 或 `变更` - 对现有功能的更改
- `Deprecated` 或 `已弃用` - 即将移除的功能
- `Removed` 或 `已移除` - 已移除的功能
- `Fixed` 或 `修复` - 任何 bug 修复
- `Security` 或 `安全` - 安全漏洞相关
- `Planned` 或 `计划中` - 未来路线图项目

## 配置说明

要更新此更新日志，请遵循以下步骤：

1. 在适当的部分（新增/变更/修复等）下添加您的更改
2. 在 `[未发布]` 的 `### 计划中` 部分下添加计划功能
3. 发布时更新版本号和日期
4. 推送更改 - GitHub Actions 将自动同步到两个 README 文件
5. 本地测试时，运行：
   ```bash
   python .github/scripts/sync_changelog.py
   ```
6. 如果 GitHub Actions 失败或手动同步，一起提交两个 CHANGELOG 文件

## 自动化同步

仓库包含 GitHub Actions 工作流（`.github/workflows/sync-changelog.yml`），它：

- 推送 CHANGELOG.md 或 CHANGELOG_CN.md 时自动触发
- 从 CHANGELOG 文件中读取 `### 计划中` 部分
- 更新 README.md 中的"即将推出的功能"部分
- 更新 README_CN.md 中的"Upcoming Features"部分
- 支持英文和中文两个版本
- 保留现有格式
- 自动提交更改（跳过 CI 以避免递归触发）

详细信息，请参阅：
- [英文指南](./.github/workflows/README.md)
- [中文指南](./.github/workflows/README_CN.md)

### 手动同步

如果需要手动同步（例如，本地测试）：

```bash
python .github/scripts/sync_changelog.py
```
