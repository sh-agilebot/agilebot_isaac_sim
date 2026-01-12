# Isaac Sim 5.1 Environment Configuration

## Installation Method Selection

Choose the installation method that best suits your needs:

| Installation Method | Isaac Sim Source | Isaac Lab Source | Use Case | Difficulty |
| :--- | :--- | :--- | :--- | :--- |
| **Recommended: Pip + Source** | `pip install` | Source (git) | **Beginners, Standard Users** | **Simple** |
| Binary + Source | Download from official site | Source (git) | Users preferring binary Isaac Sim installation | Simple |
| Full Source Build | Source (git) | Source (git) | **Developers needing to modify Isaac Sim** | Advanced |
| Pip Only | `pip install` | `pip install` | **External extension development only** (no training/examples) | Special |
| Docker Container | Docker | Source (git) | Users needing containerized environment | Advanced |

**Beginner Recommendation**: Use the **Pip + Source** method (covered in this document)

## System Requirements

### Operating System
- **Ubuntu 22.04** (Linux x64) or **Windows 11** (x64)
- ⚠️ **Isaac Lab no longer supports Isaac Sim 4.2.0 and earlier**, must use Isaac Sim 5.1.0+

### Hardware Requirements

| Element | Minimum Configuration | Recommended Configuration | Ideal Configuration |
|---------|-----------------------|----------------------------|---------------------|
| CPU | Intel Core i7 (7th gen) or AMD Ryzen 5 | Intel Core i7 (9th gen) or AMD Ryzen 7 | Intel Core i9 X series or higher, AMD Ryzen 9 or Threadripper |
| Cores | 4 | 8 | 16 |
| Memory | 32 GB | 64 GB | 64 GB |
| Storage | 50 GB SSD | 500 GB SSD | 1 TB NVMe SSD |
| GPU | NVIDIA GeForce RTX 3070 | NVIDIA GeForce RTX 4080 | NVIDIA RTX Ada 6000 |
| VRAM | 8 GB | 16 GB | 48 GB |

### Python Version Requirement
**Must strictly match Isaac Sim version**:
- **Isaac Sim 5.X** → **Python 3.11** (Required)
- Isaac Sim 4.X → Python 3.10 (Not recommended)

> ⚠️ **Important Note**: Isaac Lab and Isaac Sim are rapidly evolving. Download links and versions may change. Please follow official website links for the latest versions.

---

## NVIDIA Driver Installation

> **Official Recommended Version**: Linux `580.65.06` or higher, Windows `580.88` or higher
>
> Due to system kernel version factors, direct installation may not be possible. Graphical installation is recommended. Choose one of the following two methods:

### Method 1: Graphical Interface Installation

1. Open "Software & Updates"
2. Switch to the "Additional Drivers" tab
3. The system will automatically detect NVIDIA graphics card and list available drivers
4. Select the driver version marked as "Recommended", click "Apply Changes"
5. Enter password and wait for installation to complete
6. Restart computer after installation

---

### Method 2: Command Line Installation

#### 1. Update System and Install Build Tools
```bash
sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r)
```

#### 2. Disable Nouveau Driver
```bash
echo -e "blacklist nouveau\noptions nouveau modeset=0" \
  | sudo tee /etc/modprobe.d/blacklist-nouveau.conf
sudo update-initramfs -u
sudo reboot
```

#### 3. Install NVIDIA Driver
- Auto-install recommended version:
```bash
sudo ubuntu-drivers autoinstall
```  
- Or manually install specific version:
```bash
sudo apt install -y nvidia-driver-570
```
Restart computer to take effect.

#### 4. Verify Installation
```bash
nvidia-smi
```
If GPU information displays correctly, installation is successful.

> **Note**:
> - If Secure Boot is enabled, you may need to configure MOK (Machine Owner Key) during installation. Follow prompts to set password and confirm during reboot.
> - Ensure kernel headers matching the current kernel version are installed (see step 1).

---

## Install CUDA 12.8.1

1. Download and install:
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda_12.8.1_570.124.06_linux.run
sudo sh cuda_12.8.1_570.124.06_linux.run
```
2. Add environment variables:
```bash
export PATH=/usr/local/cuda-12.8/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
3. Verify installation:
```bash
nvcc -V
```

---

## Install cuDNN 9.8.0

```bash
wget https://developer.download.nvidia.com/compute/cudnn/9.8.0/local_installers/cudnn-local-repo-ubuntu2204-9.8.0_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2204-9.8.0_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2204-9.8.0/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install -y cudnn-cuda-12
```

---

## Compatibility Check: Isaac Sim Compatibility Checker

1. Visit Isaac Sim official documentation:
   [https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

2. Download Isaac Sim Compatibility Checker archive (second item on page), extract and run:
   - **Linux**: `omni.isaac.sim.compatibility_check.sh`
   - **Windows**: `omni.isaac.sim.compatibility_check.bat`

3. Verification result example:

![](assets/download.png)

![](assets/compatibility.png)
---

## Install Miniforge

```bash
mkdir -p ~/miniforge3
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O ~/miniforge3/miniforge.sh
bash ~/miniforge3/miniforge.sh -b -p ~/miniforge3
rm ~/miniforge3/miniforge.sh
source ~/miniforge3/bin/activate
conda init --all
```

---

## Install NVIDIA Isaac Sim

### 1. Binary File Installation

#### Download and Installation Steps

1. Visit Isaac Sim download page:
   [https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

2. Download the first item (Isaac Sim Standalone)

3. Extract and install:
```bash
mkdir -p ~/isaacsim
unzip "isaac-sim-standalone-5.1.0-linux-x86_64.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
./isaac-sim.selector.sh
```

4. In the popup interface, select `Isaac Sim Full` and wait for startup

![](assets/download.png)
### 2. Python Package Installation (pip) [Recommended]

> ⚠️ **Requirement**: Python 3.11, GLIBC ≥ 2.34

#### Create and Activate Virtual Environment

```bash
conda create -n isaacsim python=3.11 -y
conda activate isaacsim
pip install --upgrade pip
pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
```

#### Install Isaac Sim

```bash
pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cu121
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com
```

#### Verify Installation

> First run will prompt to accept user agreement, type `yes` to proceed. First startup is slow, please wait patiently until the `Isaac Sim` window displays GPU model and frame rate information.

```bash
isaacsim
# Or run experience file:
isaacsim isaacsim.exp.full.kit
```

**Successful startup page example:**

![](assets/isaacsim.png)
**Successful startup page**

![](assets/isaacsim.png)
---

## Install VS Code

1. Visit official website to download: [https://code.visualstudio.com/](https://code.visualstudio.com/)

2. Install using the following command:
```bash
sudo apt install ./<file>.deb
```

---

## Install ISAAC Lab

### Resource Caching Configuration (Strongly Recommended)

Isaac Lab resources are stored on AWS S3 and may be slow during initial loading or poor network conditions. Enabling resource caching significantly improves subsequent loading speed and supports offline operation.

Configuration steps:
1. Refer to official documentation: [Asset Caching Configuration](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/asset_caching.html)
2. Set environment variables to specify cache directory
3. Resources are automatically downloaded to local cache on first run

### 1. Clone Repository and Install Dependencies

> ⚠️ **Note**: Requires network access

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
sudo apt install -y cmake build-essential
cd IsaacLab
conda activate isaacsim
./isaaclab.sh --install  # or "./isaaclab.sh -i"
```

### 2. Verify Installation

**Option 1: Using Script**
```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

**Option 2: Using Python**
```bash
python scripts/tutorials/00_sim/create_empty.py
```

> A blank Isaac Sim window indicates successful installation.

### 3. Training Verification

> ⚠️ **Note**: Before training, install `rl_games` module, otherwise errors will occur.

```bash
# Install rl_games module
./isaaclab.sh -i rl_games

# Run training script (--headless mode disables frontend for faster training)
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task Isaac-Ant-v0

# To use headless mode, uncomment the following command:
# ./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task Isaac-Ant-v0 --headless
```

**Isaac Lab Interface Example:**

![](assets/isaaclab.png)

![alt text](assets/isaaclab.png)





---

## ROS 2 Bridge Conflict Issue

### Problem Description
When using Isaac Sim's ROS 2 for interaction, there is incompatibility between ROS2-Humble and Isaac Sim's Python version, causing the isaacsim ros2 bridge to fail loading normally, preventing normal use of ROS 2 for interaction.

### Conflict Cause

Isaac Sim is based on Python 3.11, while ROS 2 Humble is based on Python 3.10, so pybind dependency libraries cannot be recognized.

### Solution

#### Method 1: Use Precompiled Version (Recommended)

There is a precompiled ROS 2 Humble Python 3.11 version on GitHub that can be used directly.

**Repository Address**:
[https://github.com/camopel/isaacsim-ros2-python3.11](https://github.com/camopel/isaacsim-ros2-python3.11)

**Installation Steps**:

1. Download archive:
```bash
wget https://github.com/camopel/isaacsim-ros2-python3.11/releases/download/v1.0/ros_py311.tar.gz
```

2. Extract and move to system directory:
```bash
tar -xzvf ros_py311.tar.gz
sudo mv humble_ws /opt/ros/
```

3. Verify installation:
```bash
source /opt/ros/humble_ws/install/local_setup.bash
python3.11
>>> import rclpy
```

**Environment Configuration**:
- **When starting Isaac Sim**: Use `source /opt/ros/humble_ws/install/local_setup.bash` to configure environment
- **When starting local ROS 2**: Use `source /opt/ros/humble/setup.bash` to configure environment

> ⚠️ **Note**: Two environment variables cannot be combined. If switching, please open a new terminal.

#### Method 2: Self-Compilation

Compile rclpy using Python 3.11 according to official documentation, then place in `/opt/ros/`.

**Reference Documentation**:
[https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11)

### Remaining Issues

- ROS 2 compiled with Python 3.11 cannot run normally locally and can only be used as an Isaac Sim extension.

---

## ⚠️ Important Tips and Tricks

### URDF Import Collision Mesh Loss Issue

> 💡 **Tip**: Isaac Sim 5.1 may lose collision meshes when importing URDF files.
>
> **Solution**: If encountering this issue, it's recommended to use Isaac Sim **5.0 or earlier** for URDF import, then upgrade the generated USD file to 5.1 version.

---

## Reference Documentation

- [NVIDIA CUDA Download](https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda_12.8.1_570.124.06_linux.run)
- [cuDNN Local Installation Package](https://developer.download.nvidia.com/compute/cudnn/9.8.0/local_installers/cudnn-local-repo-ubuntu2204-9.8.0_1.0-1_amd64.deb)
- [IsaacSim Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)
