# 🤖 Agilebot Isaac Sim



[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-Latest-orange)](https://developer.nvidia.com/isaac-sim)
![GitHub Release](https://img.shields.io/github/v/release/sh-agilebot/agilebot_isaac_sim)
![GitHub Stars](https://img.shields.io/github/stars/sh-agilebot/agilebot_isaac_sim)
![GitHub Forks](https://img.shields.io/github/forks/sh-agilebot/agilebot_isaac_sim)
![GitHub Issues](https://img.shields.io/github/issues/sh-agilebot/agilebot_isaac_sim)


---

[中文](#) | **[English](./README.md)**


---

## 📋 项目简介

**agilebot_isaac_sim** 是 Agilebot 机器人 Isaac 生态系统中的仿真集成仓库，提供了将 Agilebot 系列机器人集成到 NVIDIA IsaacSim 仿真环境的完整解决方案，包括核心控制代码、运动策略配置、以及丰富的示例演示。

本项目由 [上海捷勃特机器人有限公司](https://www.sh-agilebot.com/) 开发维护。捷勃特是一家专注于工业机器人研发、制造与智能制造解决方案的高科技企业，依托完全自主知识产权的 **单芯片多轴驱控一体运动控制器（SCIMC）** 核心技术，为各行业提供高性能、高性价比、高易用性的机器人产品及智能制造解决方案。

### ✨ 主要特性

- 🦾 **多型号支持**：支持 gbt-c5a、gbt-c7a、gbt-c12a、gbt-c16a 等多种 Agilebot 机器人型号
- 🎮 **RMPflow 运动控制**：集成 RMPflow (Robot Motion Policy Flow) 运动策略框架
- 🎯 **丰富的功能演示**：包含目标跟随、抓取放置、堆垛等完整任务示例
- 🔧 **MoveIt 集成**：支持通过 ROS 2 + MoveIt 控制机器人
- 📷 **视频记录**：提供手腕相机集成的视频记录功能
- 🔍 **运动学验证**：包含正/逆运动学验证工具

### 🛠️ 环境要求

| 组件 | 最低版本 | 推荐版本 | 状态 |
|------|---------|---------|------|
| IsaacSim | 5.0 | 5.1 | ✅ 验证 |
| CUDA |  | 12.x | ⚠️ 可选 |
| ROS 2 |  |Humble | ⚠️ 可选 |
| Ubuntu | | 22.04 LTS | ✅ 验证 |

> Nvidia driver 建议使用最新版本，以获得最佳性能。
---

## 📁 项目结构

```
agilebot_isaac_sim/
├── agilebot_integration/          # Agilebot 核心集成模块
│   ├── code/agilebot/            # 机器人控制器和任务定义
│   │   ├── controllers/          # 夹爪控制、堆叠算法等控制器
│   │   └── tasks/                # 强化学习任务配置
│   ├── demos/                    # 功能演示示例
│   │   ├── follow_target.py      # 目标跟随演示
│   │   ├── follow_target_with_fk_verification.py  # 运动学验证
│   │   ├── pick_place.py         # 抓取放置任务
│   │   └── stacking.py           # 堆垛任务
│   └── motion_policy_configs/    # RMPflow 运动策略配置
│       └── Agilebot/
│           ├── gbt_c5a/          # C5A 机器人配置
│           ├── gbt_c7a/          # C7A 机器人配置
│           ├── gbt_c12a/         # C12A 机器人配置
│           └── gbt_c16a/         # C16A 机器人配置
├── assets/                       # USD 资产、网格和纹理文件
├── docs/                         # 文档、安装指南和教程
├── example/                      # 独立示例项目
│   ├── isaacsim_moveit/          # ROS 2 + MoveIt 集成示例
│   │   └── gbt_c7a_moveit.usd
│   └── pick_place_agilebot_camera_gripper/  # 手腕相机抓取与视频记录
│       ├── pick_place_example.py
│       └── video_recorder.py
├── tests/                        # 单元测试和集成测试
├── scripts/                      # 辅助脚本和工具
├── LICENSE                       # Apache 2.0 许可证
├── README.md                     # 主文档（英文版）
└── README_CN.md                  # 主文档（中文版）
```

---

## 🚀 快速开始

### 第 1 步：准备NVIDIA Driver

确保您的系统已安装 NVIDIA 驱动程序，推荐使用最新版本。您可以从 NVIDIA 官方网站下载并安装驱动程序。


### 第 2 步：安装 IsaacSim和isaaclab


下载并安装 [NVIDIA IsaacSim 和 IsaacLab](https://isaac-sim.github.io/IsaacLab/main/index.html)

> 📖 详细的安装和配置说明，请也可以参考 **[Isaac Sim 环境配置指南](./docs/isaacsim环境配置.md)**

### 第 3 步：克隆项目

```bash
# 克隆仓库
git clone https://github.com/sh-agilebot/agilebot_isaac_sim.git
cd agilebot_isaac_sim
```

### 第 4 步：配置环境

#### 4.1 配置 RMPflow 运动控制策略

将 `motion_policy_configs/Agilebot` 目录复制到 IsaacSim 的配置目录：

```bash
cp -r agilebot_integration/motion_policy_configs/Agilebot \
  ~/isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/
```

#### 4.2 配置 Agilebot 集成代码

将 `code/agilebot` 目录复制到 IsaacSim 扩展目录：

```bash
cp -r agilebot_integration/code/agilebot \
  ~/isaacsim/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/
```

> ⚠️ **注意**：该路径可能由于包路径前缀冲突，在 VS Code 中无法解析，没有代码提示。用户可根据实际情况调整路径。

#### 4.3 准备机器人 USD 文件

从 [agilebot_isaac_usd_assets](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) 仓库下载机器人 USD 模型文件并放置在指定路径，在 demo 文件中配置正确的 USD 路径。

> 💡 **Isaac资产设置提示**：关于 Isaac Sim 资源缓存的配置，请参考 [Isaac Sim 官方文档 - Setup Tips - Asset](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#setup-tips) 部分。

### 第 5 步：运行演示

```bash
# 运行目标跟随示例
python -m agilebot_integration.demos.follow_target

# 运行抓取放置示例
python -m agilebot_integration.demos.pick_place

# 运行堆垛示例
python -m agilebot_integration.demos.stacking
```

> ⚠️ **注意**：首次运行可能需要下载 USD 资源，请确保网络连接正常。

---

### 📦 支持的机器人型号

| 型号 | 工作半径 | 负载 | 重复定位精度 | 产品链接 |
|------|-----|------|------------|---------|
| gbt-c5a | 933mm | 5kg | 0.02mm | [查看详情](https://www.sh-agilebot.com/74/6/?type=0) |
| gbt-c7a | 785mm | 7kg | 0.02mm | [查看详情](https://www.sh-agilebot.com/74/60/?type=0) |
| gbt-c12a | 1303mm | 12kg | 0.03mm | [查看详情](https://www.sh-agilebot.com/74/61/?type=0) |
| gbt-c16a | 980mm | 16kg | 0.03mm | [查看详情](https://www.sh-agilebot.com/74/69/?type=0) |

---

## 🗂️ Agilebot 机器人仿真仓库系列

本项目是 Agilebot 机器人 Isaac 生态系统的一部分，提供完整的仿真、训练和资产管理解决方案。

| 仓库名称 | 功能定位 | 主要内容 | 链接 |
|---------|---------|---------|------|
| **agilebot_isaac_sim** | 仿真集成 | Agilebot 机器人的 IsaacSim 集成，包括仿真配置、设置文件和演示示例。不包含机器人数字资产。 | [当前仓库](#) |
| **agilebot_isaac_lab** | 训练与学习 | Agilebot 机器人的 IsaacLab 环境和训练示例，包括任务定义和学习管道。不包含机器人数字资产。 | [查看仓库](https://github.com/sh-agilebot/agilebot_isaac_lab) |
| **agilebot_isaac_usd_assets** | 资产管理 | 维护 Agilebot 机器人的集中式 USD 文件、网格和纹理的中心化仓库。 | [查看仓库](https://github.com/sh-agilebot/agilebot_isaac_usd_assets) |

> 💡 **使用说明**：使用本仓库前，请先从 **agilebot_isaac_usd_assets** 下载所需的机器人 USD 数字资产文件。

---

## 💡 使用示例

本仓库提供丰富的功能演示示例，覆盖多种机器人任务和集成场景：

### 📖 示例分类

| 示例类别 | 功能描述 | 支持的机器人型号 | 核心特性 |
|---------|---------|----------------|---------|
| **堆垛任务** | 立方体堆叠 | gbt-c5a、gbt-c7a、gbt-c12a、gbt-c16a | 可调节夹爪力度、堆叠 |
| **抓取放置** | 物体抓取与放置 | gbt-c5a、gbt-c7a、gbt-c12a、gbt-c16a | 精确抓取控制、轨迹规划 |
| **目标跟随** | 末端执行器轨迹跟踪 | gbt-c5a（可带/不带夹爪） | 实时逆运动学、平滑运动 |
| **运动学验证** | 正运动学验证 | gbt-c5a（可带/不带夹爪） | 实时位姿验证、精度分析 |
| **MoveIt 控制** | ROS 2 + MoveIt 集成 | gbt-c7a | ROS 2 接口、MoveIt 规划 |
| **抓取放置与视频记录** | 抓取放置任务与视频记录 | 完整资产（机械臂+夹爪+手腕相机） | 视频采集（30fps） |

**详细文档：**
- 📖 **[GBT Robot Examples 文档](./agilebot_integration/demos/README_CN.md)** - 所有演示示例的完整指南
- 🔧 **[MoveIt 集成指南](./example/isaacsim_moveit/README_CN.md)** - ROS 2 + MoveIt 控制教程
- 📷 **[手腕相机抓取示例文档](./example/pick_place_agilebot_camera_gripper/README_CN.md)** - 抓取放置任务与视频记录的实现说明

---

## ❓ 常见问题

### 完全独立运行项目

> 💡 **小贴士**：如果您希望项目完全独立运行，不依赖于将代码复制到 Isaac Sim 安装目录，可以参考 **[手腕相机抓取示例](./example/pick_place_agilebot_camera_gripper)** 的实现方式：

> 1. **将配置文件本地化**：将所需的 `motion_policy_configs` 配置文件复制到项目本地目录（如示例中的 `rmpflow/`）
> 2. **复制核心代码**：将 `agilebot_integration/code/agilebot/` 中的核心类（控制器、任务等）复制到项目本地目录（如示例中的 `controllers/` 和 `tasks/`）
> 3. **调整导入路径**：修改 Python 文件的导入语句，使用本地相对路径而非 Isaac Sim 扩展路径

> 这种方式的优势：
> - ✅ 项目完全独立，便于版本控制和团队协作
> - ✅ 不依赖特定的 Isaac Sim 安装路径
> - ✅ 更容易部署和迁移
> - ✅ 避免包路径冲突导致的代码提示问题

### Isaac Sim GUI 无响应

**症状**：窗口卡住，无错误提示  
**原因**：正在从网络加载资源  
**解决**：
1. 确保网络连接通畅
2. 使用 VPN 加速资源下载
3. 提前设置好 IsaacSim 的资源路径

> 📖 详细说明请参考 [Isaac Sim 官方文档 - Setup Tips - Asset](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#setup-tips)

### URDF 导入碰撞体丢失

**症状**：导入后机器人无法进行碰撞检测  
**环境**：Isaac Sim 5.1  
**解决**：使用 Isaac Sim **5.0 或更低版本** 进行 URDF 导入，然后再将生成的 USD 文件升级到 5.1 版本使用



---

## 🐛 报告问题

遇到问题时，请：

1. 查看 [常见问题](#-常见问题)
2. 搜索 [GitHub Issues](https://github.com/sh-agilebot/agilebot_isaac_sim/issues)
3. 创建新 Issue，包含：
   - 环境信息（OS、Python、IsaacSim 版本）
   - 复现步骤
   - 错误日志
   - 截图/视频（如有）

---

## 🤝 联系与支持

- 📧 **邮箱**：info@agilebot.com.cn
- 🐛 **问题报告**：[GitHub Issues](https://github.com/sh-agilebot/agilebot_isaac_sim/issues)
- 📞 **服务热线**：400-996-7588
- 🏢 **上海总部**：上海市闵行区新龙路500弄虹桥万创中心T1栋7楼
- 🌐 **官网**：https://www.sh-agilebot.com/

> 💡 建议通过 GitHub Issues 报告问题，便于跟踪和社区共享解决方案。

---



## 🗺️ 开发路线图

> 本部分从 [CHANGELOG_CN.md](./docs/CHANGELOG_CN.md) 自动同步






### ✅ 已发布功能 (v0.0.1)

- [x] Agilebot IsaacSim 集成初始发布
- [x] Agilebot 机器人的核心集成代码
- [x] gbt-c5a、gbt-c7a、gbt-c12a、gbt-c16a 的 RMPflow 运动控制配置
- [x] 示例演示：
  - 堆垛任务
  - 抓取放置任务
  - 跟随目标任务
  - 带正运动学验证的跟随目标任务
- [x] 带 ROS 2 Humble 的 MoveIt 2 集成示例
- [x] 带有手腕相机的抓取放置任务演示，演示完整的抓取放置任务流程和视频录制
- [x] 双语种文档（英文/中文）

### 🚧 开发中

- [ ] 2D 抓取演示：基于2D视觉的仿真抓取
- [ ] 3D 抓取演示：基于3D视觉的仿真抓取

---

## 📄 许可证

本项目采用 Apache 2.0 许可证。详情请参阅 [LICENSE](./LICENSE) 文件。

---




