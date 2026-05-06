# 基于ROS2的网球拾取机器人

## 项目简介

本项目是一个基于 ROS2 的网球自主拾取机器人系统。机器人搭载 Jetson Nano 作为下位机，运行 NVIDIA CSI 摄像头、YDLIDAR 激光雷达、IMU、电机驱动等硬件模块及 YOLOv5 网球检测算法；PC 端作为上位机，运行 SLAM 建图、路径规划、导航与可视化界面。通过 WiFi/以太网实现双机 ROS2 DDS 通信。

### 主要功能

- **键盘遥控**：WASD 控制机器人运动，空格停止
- **网球目标检测**：基于 YOLOv5 的实时网球识别与定位（距离 + 角度计算）
- **实时画面传输**：CSI 摄像头采集 + JPEG 压缩 + ROS2 传输 + PC 端可视化
- **SLAM 建图**：基于 Google Cartographer 的 2D 激光 SLAM
- **自主导航**：基于 Nav2 的全局路径规划、局部避障与 AMCL 定位

---

## 代码架构

### 目录结构

```
TennisRobot/
├── TennisRobot_PC/          # PC 端（上位机）ROS2 Humble 工作空间
│   └── src/
│       ├── msg_interfaces/      # 自定义 ROS2 消息定义（DetectResult, ImageResult, WheelSpeed）
│       ├── camera_pkg/          # 视频流显示节点（JPEG 解码、检测结果渲染）
│       ├── car_control/         # 键盘遥控节点（/cmd_vel 发布）
│       ├── robot_description/   # 机器人 URDF/XACRO 模型
│       ├── slam_pkg/            # Cartographer SLAM 建图
│       └── robot_navigation2/   # Nav2 自主导航（AMCL + 路径规划 + 避障）
│
├── TennisRobot_ws/          # Jetson 端（下位机）ROS2 Foxy 工作空间
│   └── src/
│       ├── msg_interfaces/      # 自定义 ROS2 消息定义（与 PC 端一致）
│       ├── car_pkg/             # 电机驱动（串口通信）、里程计（diff_tf）、键盘遥控
│       ├── car_launch/          # 启动文件 + EKF 配置
│       ├── imu_pkg/             # IMU 传感器驱动（串口）
│       ├── camera_pkg/          # CSI 摄像头驱动（GStreamer）
│       ├── ydlidar_ros2_driver/ # YDLIDAR 激光雷达驱动
│       ├── yolov5_pkg/          # YOLOv5 网球检测（GPU 推理）
│       └── robot_description/   # 机器人 URDF 模型
│
└── README.md
```

### 系统架构图

```
[CSI 摄像头] ──→ camera_pkg ──→ /camera/image_raw (JPEG)
        │
        └──→ yolov5_pkg ──→ /detection/results (DetectResult)
                    └──→ /detection/visualization (ImageResult + 网球框)

[电机驱动板] ←── car_pkg/car_serial (/dev/car_serial) ──→ /cmd_vel
                    └──→ /wheelspeed (编码器数据)
                    └──→ diff_tf ──→ TF: odom → base_link, /odom

[IMU] ──→ imu_pkg (/dev/imu) ──→ /imu_data

[YDLIDAR] ──→ ydlidar_ros2_driver (/dev/ydlidar) ──→ /scan

        ┌──────────────────────────────────┐
        │     ROS2 DDS (WiFi/以太网)       │
        │     RMW: CycloneDDS              │
        │     ROS_DOMAIN_ID: 28            │
        └──────────────────────────────────┘
                    │
             ┌─────┴─────┐
             │    PC 端    │
             ├────────────┤
             │ camera_pkg │──→ 显示检测画面
             │ car_control│──→ 键盘遥控 → /cmd_vel
             │ slam_pkg   │──→ Cartographer SLAM
             │ nav2       │──→ AMCL + 路径规划
             └────────────┘
```

### TF 坐标树

```
map ──→ odom ──→ base_link ──┬── imu_link
                              ├── laser_link (YDLIDAR)
                              └── camera_link
```

---

## 环境要求

| 机器 | 操作系统 | ROS2 版本 | 硬件 |
|------|---------|----------|------|
| PC | Ubuntu 22.04 | Humble | WiFi/以太网 |
| Jetson | Ubuntu 20.04 | Foxy | Jetson Nano, CSI 摄像头, YDLIDAR F2, 9 轴 IMU, 电机驱动板 |

### 关键依赖

- **两台机器都需安装：**
  ```bash
  # CycloneDDS（跨版本兼容性好的 RMW 实现）
  sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
  ```

- **PC 端额外安装：**
  ```bash
  # Nav2 导航栈
  sudo apt install ros-humble-nav2-bringup
  # Cartographer（SLAM 建图）
  sudo apt install ros-humble-cartographer-ros
  ```

- **Jetson 端额外安装：**
  ```bash
  # PyTorch + torchvision（YOLOv5 推理）
  # OpenCV + GStreamer（摄像头）
  # YOLOv5 源码（需在 ~/yolov5-7.0 目录）
  pip install memory_profiler
  pip install tf-transformations
  ```

---

## 使用步骤

### 1. 一次性环境配置

两台机器的 `~/.bashrc` 末尾添加：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=28
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

两台机器的 `~/cyclonedds.xml`：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65536</MaxMessageSize>
    </General>
  </Domain>
</CycloneDDS>
```

### 2. 编译工作空间

**Jetson 端：**
```bash
cd ~/TennisRobot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

**PC 端：**
```bash
cd ~/TennisRobot_PC
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3. 启动机器人

> 注意：必须先启动 Jetson 端，再启动 PC 端

**Jetson 端（先开）：**

```bash
# 终端 1 — 硬件驱动（电机、里程计、IMU、激光雷达）
source ~/TennisRobot_ws/install/setup.bash
ros2 launch car_launch slam_launch.py

# 终端 2 — YOLOv5 网球检测
source ~/TennisRobot_ws/install/setup.bash
ros2 run yolov5_pkg yolov5_detectV2
```

**PC 端（Jetson 启动后开）：**

```bash
# 验证通信
source /opt/ros/humble/setup.bash
ros2 topic list | grep scan   # 能看到 /scan 说明通信正常

# 终端 1 — 键盘遥控
source /opt/ros/humble/setup.bash
source ~/TennisRobot_PC/install/setup.bash
ros2 run car_control car_control
# W=前进 S=后退 A=左转 D=右转 空格=停止 Q=退出

# 终端 2 — 查看检测画面
source /opt/ros/humble/setup.bash
source ~/TennisRobot_PC/install/setup.bash
ros2 run camera_pkg jepg_detect

# 终端 3 — SLAM + 导航（先关闭终端 1 避免抢 /cmd_vel）
source /opt/ros/humble/setup.bash
source ~/TennisRobot_PC/install/setup.bash
ros2 launch robot_navigation2 navigation2.launch.py
# 在 RViz2 中使用 "2D Pose Estimate" 设置初始位姿
# 使用 "2D Goal Pose" 设置导航目标点
```

### 4. 关机

逐个终端 `Ctrl+C` 停止即可。

---

## 话题说明

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | Twist | PC → Jetson | 速度控制指令 |
| `/scan` | LaserScan | Jetson → PC | 激光雷达数据 |
| `/imu_data` | Imu | Jetson → PC | IMU 传感器数据 |
| `/odom_diff` | Odometry | Jetson → PC | 轮式里程计 |
| `/wheelspeed` | WheelSpeed | Jetson → PC | 编码器原始数据 |
| `/camera/image_raw` | Image | Jetson → PC | CSI 摄像头画面 |
| `/detection/results` | DetectResult | Jetson → PC | 网球检测结果 |
| `/detection/visualization` | ImageResult | Jetson → PC | 带检测框的压缩画面 |
| `/tf` | TFMessage | 双向 | 坐标变换 |
| `/map` | OccupancyGrid | PC → Jetson | 代价地图 |
| `/plan` | Path | PC → Jetson | 全局路径 |
| `/goal_pose` | PoseStamped | PC | 导航目标点 |

---

## 已知问题与改进方向

1. **转弯里程计漂移**：纯编码器里程计在转弯时因轮子打滑导致角度偏差，可启用 EKF 融合 IMU 数据改善
2. **WiFi 通信不稳定**：建议使用网线直连 Jetson 和 PC，或使用树莓派做中继
3. **检测帧率偏低**：WiFi 带宽限制，降低分辨率或使用网线可提升
4. **检测结果→导航点桥接**：`jepg_detect` 已计算网球距离/角度，但尚未写代码将结果转为 `/goal_pose` 发送给 Nav2
5. **自主探索建图**：目前仅支持手动设定导航目标点，自主探索功能尚未实现
