# Isaac Sim 5.1环境配置

## 安装方法选择

根据需求选择最适合的安装方法：

| 安装方法 | Isaac Sim 来源 | Isaac Lab 来源 | 适用场景 | 难度 |
| :--- | :--- | :--- | :--- | :--- |
| **推荐：Pip + 源码** | `pip install` | 源码（git） | **初学者、标准用户** | **简单** |
| 二进制 + 源码 | 官网下载二进制包 | 源码（git） | 偏好二进制安装 Isaac Sim 的用户 | 简单 |
| 全源码构建 | 源码（git） | 源码（git） | **需要修改 Isaac Sim 本身的开发者** | 高级 |
| 仅 Pip 包 | `pip install` | `pip install` | **仅用于开发外部扩展**（不包含训练/示例脚本） | 特殊情况 |
| Docker 容器 | Docker | 源码（git） | 需要在容器化环境中使用的用户 | 高级 |

**新手推荐**：使用 **Pip + 源码** 方式（本文档主要介绍的方式）

## 系统要求

### 操作系统
- **Ubuntu 22.04** (Linux x64) 或 **Windows 11** (x64)
- ⚠️ **Isaac Lab 不再支持 Isaac Sim 4.2.0 及更早版本**，必须使用 Isaac Sim 5.1.0+

### 硬件要求

| 元素    | 最低配置                                          | 推荐配置                                          | 理想配置                                         |
|---------|--------------------------------------------------|---------------------------------------------------|--------------------------------------------------|
| CPU      | Intel Core i7（7代） 或 AMD Ryzen 5               | Intel Core i7（9代） 或 AMD Ryzen 7               | Intel Core i9 X 系列或更高，AMD Ryzen 9 或 Threadripper  |
| 核心数   | 4                                                | 8                                                 | 16                                               |
| 内存     | 32 GB                                            | 64 GB                                             | 64 GB                                            |
| 存储     | 50 GB SSD                                        | 500 GB SSD                                        | 1 TB NVMe SSD                                    |
| GPU      | NVIDIA GeForce RTX 3070                          | NVIDIA GeForce RTX 4080                          | NVIDIA RTX Ada 6000                              |
| 显存     | 8 GB                                             | 16 GB                                             | 48 GB                                            |

### Python 版本要求
**必须与 Isaac Sim 版本严格匹配**：
- **Isaac Sim 5.X** → **Python 3.11**（必须使用）
- Isaac Sim 4.X → Python 3.10（已不建议使用）

> ⚠️ **重要提示**：Isaac Lab 和 Isaac Sim 正在快速迭代中，下载地址或版本可能会有变化。请按照官网的链接进行下载，以获取最新版本。

---

## NVIDIA 驱动安装

> **官方推荐版本**：Linux `580.65.06` 或更高，Windows `580.88` 或更高
>
> 因系统内核版本等因素，可能无法直接安装，推荐使用图形化方式。以下两种方式任选其一：

### 方法一：图形化界面安装

1. 打开 “软件和更新”（Software & Updates）
2. 切换到 “附加驱动程序”（Additional Drivers）标签页
3. 系统会自动检测 NVIDIA 显卡并列出可用驱动
4. 选择标有 “推荐” 的驱动版本，点击 “应用更改”（Apply Changes）
5. 输入密码并等待安装完成
6. 安装完成后重启电脑

---

### 方法二：命令行安装

#### 1. 更新系统并安装构建工具
```bash
sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r)
```

#### 2. 禁用 Nouveau 驱动
```bash
echo -e "blacklist nouveau\noptions nouveau modeset=0" \
  | sudo tee /etc/modprobe.d/blacklist-nouveau.conf
sudo update-initramfs -u
sudo reboot
```

#### 3. 安装 NVIDIA 驱动
- 自动安装推荐版本：
```bash
sudo ubuntu-drivers autoinstall
```  
- 或手动安装指定版本，如：
```bash
sudo apt install -y nvidia-driver-570
```
重启电脑后生效。

#### 4. 验证安装
```bash
nvidia-smi
```
如果能正常显示 GPU 信息，即安装成功。

> **注意**：
> - 如果启用了安全启动（Secure Boot），安装时可能需要配置 MOK（Machine Owner Key），请按提示设置密码并重启时进行确认。
> - 确保已安装与当前内核版本匹配的内核头文件（见第 1 步）。

---

## 安装 CUDA 12.8.1

1. 下载并安装：
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda_12.8.1_570.124.06_linux.run
sudo sh cuda_12.8.1_570.124.06_linux.run
```
2. 添加环境变量：
```bash
export PATH=/usr/local/cuda-12.8/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
3. 验证安装：
```bash
nvcc -V
```

---

## 安装 cuDNN 9.8.0

```bash
wget https://developer.download.nvidia.com/compute/cudnn/9.8.0/local_installers/cudnn-local-repo-ubuntu2204-9.8.0_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2204-9.8.0_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2204-9.8.0/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install -y cudnn-cuda-12
```

---

## 兼容性检查：Isaac Sim Compatibility Checker

1. 访问 Isaac Sim 官方文档：
   [https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

2. 下载 Isaac Sim Compatibility Checker 压缩包（页面第二项），解压后运行：
   - **Linux**: `omni.isaac.sim.compatibility_check.sh`
   - **Windows**: `omni.isaac.sim.compatibility_check.bat`

3. 验证结果示例：

![](assets/download.png)

![](assets/compatibility.png)
---

## 安装 Miniforge

```bash
mkdir -p ~/miniforge3
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O ~/miniforge3/miniforge.sh
bash ~/miniforge3/miniforge.sh -b -p ~/miniforge3
rm ~/miniforge3/miniforge.sh
source ~/miniforge3/bin/activate
conda init --all
```

---

## 安装 NVIDIA Isaac Sim

### 1. 二进制文件安装

#### 下载和安装步骤

1. 访问 Isaac Sim 下载页面：
   [https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

2. 下载第一项（Isaac Sim Standalone）

3. 解压并安装：
```bash
mkdir -p ~/isaacsim
unzip "isaac-sim-standalone-5.1.0-linux-x86_64.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
./isaac-sim.selector.sh
```

4. 在弹出的界面中选择 `Isaac Sim Full`，等待启动

![](assets/download.png)
### 2. Python 包安装（pip）【推荐】

> ⚠️ **要求**: Python 3.11，GLIBC ≥ 2.34

#### 创建并激活虚拟环境

```bash
conda create -n isaacsim python=3.11 -y
conda activate isaacsim
pip install --upgrade pip
pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
```

#### 安装 Isaac Sim

```bash
pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cu121
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com
```

#### 验证安装

> 首次运行会提示是否接受用户协议，输入 `yes` 即可。首次启动较慢，请耐心等待，直到出现 `Isaac Sim` 窗口中显示 GPU 型号和帧率信息。

```bash
isaacsim
# 或运行体验文件：
isaacsim isaacsim.exp.full.kit
```

**成功启动页面示例：**

![](assets/isaacsim.png)
**成功启动页面**

![](assets/isaacsim.png)
---

## 安装 VS Code

1. 访问官网下载：[https://code.visualstudio.com/](https://code.visualstudio.com/)

2. 使用以下命令安装：
```bash
sudo apt install ./<file>.deb
```

---

## 安装 ISAAC Lab

### 资源缓存配置（强烈推荐）

Isaac Lab 的资源存储在 AWS S3 上，首次加载或网络不佳时可能很慢。启用资源缓存可显著提升后续加载速度，并支持离线工作。

配置步骤：
1. 参考官方文档：[资源缓存配置](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/asset_caching.html)
2. 设置环境变量指定缓存目录
3. 首次运行时自动下载资源到本地缓存

### 1. 克隆仓库并安装依赖

> ⚠️ **注意**: 需要科学上网

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
sudo apt install -y cmake build-essential
cd IsaacLab
conda activate isaacsim
./isaaclab.sh --install  # 或 "./isaaclab.sh -i"
```

### 2. 验证安装

**选项一：使用脚本**
```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

**选项二：使用 Python**
```bash
python scripts/tutorials/00_sim/create_empty.py
```

> 出现一个空白的 Isaac Sim 窗口，即安装成功。

### 3. 训练验证

> ⚠️ **注意**: 训练前需要先安装 `rl_games` 模块，否则会报错。

```bash
# 安装 rl_games 模块
./isaaclab.sh -i rl_games

# 运行训练脚本（--headless 模式表示不显示前端页面，可加速训练过程）
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task Isaac-Ant-v0

# 如需使用 headless 模式，取消下面命令的注释：
# ./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task Isaac-Ant-v0 --headless
```

**Isaac Lab 界面示例：**

![](assets/isaaclab.png)

![alt text](assets/isaaclab.png)




---

## ROS 2 Bridge 冲突问题

### 问题描述
当需要使用 Isac Sim 的 ROS 2进行交互时，ROS2-Humble 与 Isaac Sim 的 Python 版本不兼容，导致 isaacsim ros2 bridge 无法正常加载。

### 冲突原因

Isaac Sim 基于 Python 3.11，而 ROS 2 Humble 基于 Python 3.10，因此 pybind 的依赖库无法识别。

### 解决方法

#### 方案一：使用预编译版本（推荐）

GitHub 上有已编译好的 ROS 2 Humble Python 3.11 版本，可直接使用。

**仓库地址**:
[https://github.com/camopel/isaacsim-ros2-python3.11](https://github.com/camopel/isaacsim-ros2-python3.11)

**安装步骤**：

1. 下载压缩包：
```bash
wget https://github.com/camopel/isaacsim-ros2-python3.11/releases/download/v1.0/ros_py311.tar.gz
```

2. 解压并移动到系统目录：
```bash
tar -xzvf ros_py311.tar.gz
sudo mv humble_ws /opt/ros/
```

3. 验证安装：
```bash
source /opt/ros/humble_ws/install/local_setup.bash
python3.11
>>> import rclpy
```

**环境配置**：
- **启动 Isaac Sim 时**：使用 `source /opt/ros/humble_ws/install/local_setup.bash` 配置环境
- **启动本地 ROS 2 时**：使用 `source /opt/ros/humble/setup.bash` 配置环境

> ⚠️ **注意**：两个环境变量不可以叠加，如果切换，请重新打开一个终端。

#### 方案二：自行编译

根据官方文档使用 Python 3.11 编译 rclpy，然后放置在 `/opt/ros/`。

**参考文档**:
[https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11)

### 遗留问题

- Python 3.11 编译的 ROS 2 无法正常在本地运行，只能作为 Isaac Sim 的扩展使用。

---

## ⚠️ 重要提示与技巧

### URDF 导入碰撞体丢失问题

> 💡 **小贴士**：Isaac Sim 5.1 版本在导入 URDF 文件时可能会出现碰撞体丢失的情况。
>
> **解决方法**：如果遇到此问题，建议使用 Isaac Sim **5.0 或更低版本** 进行 URDF 导入，然后再将生成的 USD 文件升级到 5.1 版本使用。

---

## 参考文档

- [NVIDIA CUDA 下载](https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda_12.8.1_570.124.06_linux.run)
- [cuDNN 本地安装包](https://developer.download.nvidia.com/compute/cudnn/9.8.0/local_installers/cudnn-local-repo-ubuntu2204-9.8.0_1.0-1_amd64.deb)
- [IsaacSim 安装指南](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)
