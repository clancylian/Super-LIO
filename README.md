<div align="center">
  <h1>⚡Super-LIO</h1>
  <h2>Super-LIO: 基于紧凑建图策略的鲁棒高效激光惯性里程计系统</h2>
  <p><strong>本工作已被 <i>IEEE Robotics and Automation Letters (RA-L 2026)</i> 录用。</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoWidth=16)](https://github.com/Liansheng-Wang/Super-LIO.git) [![arXiv](https://img.shields.io/badge/arXiv-blue?logo=arxiv&color=%23B31B1B)](https://arxiv.org/abs/2509.05723) [![IEEE](https://img.shields.io/badge/RAL2026-004088.svg)](https://ieeexplore.ieee.org/document/11347459) [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?style=flat-square&logo=bilibili&logoColor=white&logoWidth=16)](https://www.bilibili.com/video/BV11wBeBYEp6) [![YouTube](https://img.shields.io/badge/YouTube-FF0000?style=flat-square&logo=youtube&logoColor=white&logoWidth=16)](https://youtu.be/m9-hl8s5DDw)
</div>


<div align="center">
  <p>
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros1" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="Switch to ROS1"
      onmouseover="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-1e40af?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros2" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="ROS2 Active"
      onmouseover="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-166534?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="#" style="text-decoration: none; cursor: default;">
      <img src="https://img.shields.io/badge/🖥️ PLATFORM - X86 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22" alt="X86 and ARM Support"
      onmouseover="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-4f46e5?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"/>
    </a>
  </p>
</div>

## 概述

<p align="center">
  <img src="docs/system_overview.png" width="95%">
</p>

**核心特性：高效 · 鲁棒 · 跨平台 · 支持 ROS1/ROS2**

Super-LIO 是一个面向实时大规模自主导航的鲁棒高效激光-惯性里程计（LIO）系统。该系统提出了一种紧凑且结构化的建图策略，实现了可预测的对应搜索和稳定的状态估计。经过大量真实场景实验和与前沿方法的对比验证，Super-LIO 不仅取得了**优异的精度**，还保持了**更低的资源消耗**，并实现了约 **1.2–4 倍的实时处理速度**⚡。


**贡献者**：[Liansheng Wang](https://github.com/Liansheng-Wang), [Xinke Zhang](https://github.com/PSQzzzxk), [Chenhui Li](https://github.com/kermitLHH), [Dongjiao He](https://github.com/Joanna-HE), [Yihan pan](https://github.com/pyh3552), Jianjun Yi.


## 功能特性

### 支持的雷达类型

Super-LIO 支持多种主流激光雷达，包括：

| 雷达类型 | 枚举值 | 说明 |
|---------|-------|------|
| Livox 系列 | 1 | 通过 Livox 自定义消息接口接收数据 |
| 禾赛 HESAI16 | 2 | 标准 PointCloud2 接口，含 ring 和 timestamp 字段 |
| Velodyne VLP-16 | 3 | 标准 PointCloud2 接口 |
| Velodyne VLP-32 | 4 | 标准 PointCloud2 接口 |
| NCLT 数据集 | 5 | 用于 NCLT 数据集回放 |
| 镭神 LS16 | 6 | 标准 PointCloud2 接口 |
| Ouster | 7 | 标准 PointCloud2 接口，含 t 字段 |
| Gazebo 仿真 | 8 | 用于仿真环境 |
| RoboSense Airy | 9 | 标准 PointCloud2 接口，含 ring 和 timestamp 字段，自动处理 RBD→FLU 坐标系转换 |

> **RoboSense Airy 说明**：Airy 雷达使用 RBD（右-后-下）坐标系，与 ROS 标准的 FLU（前-左-上）坐标系不同。系统在数据接收阶段自动将点云和 IMU 数据从 RBD 转换为 FLU，确保输出结果符合 ROS 坐标系标准，外参矩阵只需配置物理安装关系即可。

### 动态点滤除

系统支持在建图保存时滤除动态物体（行人、车辆等），提供两种方法：

- **时序法（Temporal）**：基于前后帧对比，滤除在相邻帧中未被持续占据的点
- **射线法（Raycast）**：从传感器位置发射射线，滤除被射线穿透的点（需要里程计数据）

**配置参数**（在 yaml 文件中）：
```yaml
lio.dynamic_removal.enable: true        # 启用动态点滤除
lio.dynamic_removal.method: 0           # 0: 时序法, 1: 射线法
lio.dynamic_removal.grid_size: 0.2      # 体素网格大小（米）
lio.dynamic_removal.min_neighbors: 2    # 保留点所需的最小邻居网格数
lio.dynamic_removal.frame_window: 1     # 时序法的帧窗口大小
lio.dynamic_removal.raycast_min_hits: 2 # 射线法的最小命中次数
lio.dynamic_removal.isolated_removal: true
```

启用后，保存地图时会同时输出原始地图和滤除后的地图（`filtered_<map_name>`）。

### 重定位模式

系统支持基于预建地图的重定位，可在已保存的地图上恢复定位，无需重新建图。适用于长期部署、重复任务或跟踪丢失后的恢复场景。

运行重定位前，请确保已将地图保存到磁盘。

### 多机器人命名空间

系统支持通过命名空间（namespace）配置多机器人场景，在 launch 文件中通过 `ns` 参数指定，所有 TF 帧、话题均自动添加命名空间前缀。

### 异步地图保存

地图保存操作在独立线程中执行，避免阻塞主线程的实时计算。保存每帧点云时同时记录对应的里程计信息。

### 点云质量过滤

支持基于距离和强度的点云质量过滤，可配置盲区距离、最大范围及强度阈值，在数据接收阶段即剔除低质量点。

### CPU 亲和性优化

LIO 计算线程绑定高优先级 CPU 核心，确保关键路径的实时性能。


## 快速开始

**ROS1 用户**：请切换到 **ros1** 分支，参照 [ros1 分支说明](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)

### 环境要求

Ubuntu 24(22).04 · C++20 · ROS Jazzy(Humble) · Eigen · PCL 

### 依赖安装

glog · TBB

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```

### 编译与运行
```bash
git clone https://github.com/Liansheng-Wang/Super-LIO.git
cd Super-LIO
colcon build

source install/setup.bash
ros2 launch super_lio Livox_mid360.py

```

#### 🔁 重定位模式

```bash
cd PATH_2_Super-LIO
source install/setup.bash
ros2 launch super_lio relocation.py
```

#### 📡 RoboSense Airy 雷达

```bash
ros2 launch super_lio Robosense_airy.py
```

Airy 雷达的默认话题为 `/front_lidar`（点云）和 `/front_lidar/imu`（IMU），如需修改请在 `config/robosense_airy.yaml` 中调整。


## 数据集
<p align="center">
  <img src="docs/datasets_compressed.png" width="95%">
</p>

Super-LIO 在涵盖室内、室外和大规模场景的多个真实数据集上进行了评估验证。

> **待办**：数据集下载链接和详细说明将在后续提供。


---

## 论文引用

如果您喜欢我们的项目，请引用并给我们一个 star 🌟。
如果您发现本库有用，建议引用[我们的论文](https://ieeexplore.ieee.org/document/11347459)：

```latex
@article{wang2026superlio,
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy},
  author  = {Wang, Liansheng and Zhang, Xinke and Li, Chenhui and He, Dongjiao and Pan, Yihan and Yi, Jianjun},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {3},
  pages   = {2666--2673},
  doi     = {10.1109/LRA.2026.3653372}
}
```


## Update Logs

<details>
<summary>Click to expand <b>Update Logs</b> (click to collapse)</summary>

<br>

- 2026-01-04  
  - Separate ROS interface and algorithm.
  - Refactor SuperLIOReLoc to inherit from SuperLIO.
  - Code style aligned with ROS2 version.

- 2026-01-04
  - The main branch is renamed to ros1
  - add ros2 branch

- 2026-01-04 21:51
  - release ROS2 version

- 2026-06-07
 - [Important revisions]: Fixed some known errors and improved algorithm accuracy!
 
</details>
