# 基于ROS2的网球拾取机器人

## 项目简介

本项目是一个基于 ROS2 的网球自主拾取机器人系统。采用三层架构：STM32F103 作为底层电机驱动板，负责 PWM 电机控制、编码器采集与 PID 闭环控制；Jetson Nano 作为中层下位机，运行 NVIDIA CSI 摄像头、YDLIDAR 激光雷达、IMU 等传感器驱动及 YOLOv5 网球检测算法；PC 端作为上层上位机，运行 SLAM 建图、路径规划、导航与可视化界面。Jetson 与 PC 通过 WiFi/以太网实现 ROS2 DDS 通信，Jetson 与 STM32 通过 UART 串口进行自定义协议通信。

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
├── TennisRobot_ws/          # Jetson 端（中层下位机）ROS2 Foxy 工作空间
│   └── src/
│       ├── msg_interfaces/      # 自定义 ROS2 消息定义（与 PC 端一致）
│       ├── car_pkg/             # 电机驱动（串口通信 STM32）、里程计（diff_tf）
│       ├── car_launch/          # 启动文件 + EKF 配置
│       ├── imu_pkg/             # IMU 传感器驱动（串口）
│       ├── camera_pkg/          # CSI 摄像头驱动（GStreamer）
│       ├── ydlidar_ros2_driver/ # YDLIDAR 激光雷达驱动
│       ├── yolov5_pkg/          # YOLOv5 网球检测（GPU 推理）
│       └── robot_description/   # 机器人 URDF 模型
│
├── stm32/                    # STM32 端（底层下位机）电机驱动固件
│   ├── Application/             # 应用层（OS 调度、GUI 显示）
│   ├── Middleware/              # 中间件（PID 控制、串口协议）
│   ├── Hardware/                # 硬件抽象层（电机 PWM、编码器、ADC、LCD）
│   └── Core/                    # STM32 HAL 库配置（CubeMX 生成）
│
└── README.md
```

### 三层架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                         PC 端（上位机）                          │
│  camera_pkg: 显示检测画面                                        │
│  car_control: 键盘遥控 → /cmd_vel                                │
│  slam_pkg: Cartographer SLAM 建图                                │
│  robot_navigation2: AMCL 定位 + Nav2 路径规划                     │
└────────────┬────────────────────────────────────────────────────┘
             │  ROS2 DDS (WiFi/以太网, Fast-DDS, Domain 28)
             │
┌────────────▼────────────────────────────────────────────────────┐
│                      Jetson 端（中层下位机）                      │
│  camera_pkg: CSI 摄像头 → /camera/image_raw (JPEG)              │
│  yolov5_pkg: YOLOv5 网球检测 GPU 推理                             │
│  ydlidar_ros2_driver: 激光雷达 → /scan                           │
│  imu_pkg: 9轴IMU → /imu_data                                    │
│  car_pkg/car_serial: 串口通信 STM32 ←→ /cmd_vel, /wheelspeed    │
│  car_pkg/diff_tf: 编码器 → 里程计 TF: odom → base_link           │
└────────────┬────────────────────────────────────────────────────┘
             │  UART Serial (115200 8N1, /dev/car_serial → ttyTHS1)
             │  自定义12字节二进制协议: 0xAA55 + 左右轮速 + XOR校验
             │
┌────────────▼────────────────────────────────────────────────────┐
│                      STM32 端（底层下位机）                       │
│  STM32F103RCT6 (Cortex-M3, 72MHz)                               │
│  PID 闭环控制: Kp=30, Ki=0.1, Kd=300, 10ms 控制周期              │
│  3路电机 PWM: 左轮/右轮/拾球滚轮 (10kHz, TIM1+TIM4)              │
│  4路编码器: 左轮/右轮/滚轮/旋钮      (TIM2+TIM5+TIM3+TIM8)       │
│  ADC 采样: 电池电压/电流/功率       (ADC1 + DMA)                 │
│  2.5寸 LCD 显示: ST7789V            (SPI2 + DMA)                │
└─────────────────────────────────────────────────────────────────┘
```

### TF 坐标树

```
map ──→ odom ──→ base_link ──┬── imu_link
                              ├── laser_link (YDLIDAR)
                              └── camera_link
```

---

## STM32 下位机详解

### 硬件参数

| 项目 | 参数 |
|------|------|
| 主控芯片 | STM32F103RCT6 (Cortex-M3, 72MHz, 256KB Flash, 48KB RAM) |
| 电机数 | 3 路（左轮、右轮、拾球滚轮） |
| PWM 频率 | 10kHz (TIM1 CH1/CH4, TIM4 CH1-4) |
| 编码器数 | 4 路（左轮 TIM2, 滚轮 TIM3, 右轮 TIM5, 旋钮 TIM8) |
| 控制周期 | 10ms (TIM7 中断) |
| 通信接口 | USART2, 115200bps, 8N1 |
| 显示 | 2.5寸 LCD (ST7789V, SPI2 + DMA) |
| 采样 | ADC1: 电池电压/电流/功率 (10x 过采样) |

### STM32 软件架构

```
Application/
 ├── OS/os.[ch]      # 主调度器（5状态轮询 + TIM7中断）
 └── GUI/GUI.[ch]    # LCD 界面（电压/电流/功率显示）

Middleware/
 ├── PID/pid.[ch]    # PID 控制器（Kp=30, Ki=0.1, Kd=300）
 └── Serial/serial.[ch]  # Jetson 通信协议（12字节二进制包）

Hardware/
 ├── Motor/Motor.[ch]    # H桥PWM电机驱动 + 编码器初始化
 ├── ADC_cs/ADC_cs.[ch]  # ADC 采样 + 自动校准（2.5V参考）
 ├── LCD/LCD_init.[ch]   # ST7789V 驱动
 ├── LCD/LCD_dma.[ch]    # DMA 图形缓冲（38400B 帧缓冲）
 └── KeyPad/KeyPad.[ch]  # 4 按键检测（去抖处理）

Core/               # STM32CubeMX 生成的 HAL 配置
 ├── Src/main.c          # 主入口
 ├── Src/tim.c           # 定时器（PWM/编码器/系统时钟）
 ├── Src/usart.c         # 串口（USART1 调试, USART2 通信）
 ├── Src/adc.c           # ADC 配置
 ├── Src/spi.c           # SPI2 LCD 配置
 └── Src/gpio.c          # GPIO（按键/复位/命令控制）
```

### Jetson ↔ STM32 通信协议

自定义 12 字节二进制包，UART 115200bps 8N1：

```
┌───────┬───────┬────────┬────────────┬─────────────┬──────────┐
│ Offset │ Size  │ 字段   │ 内容       │ 说明        │
├───────┼───────┼────────┼────────────┼─────────────┤
│  0    │  2    │ 帧头   │ 0xAA 0x55  │ 同步字       │
│  2    │  1    │ 长度   │ 0x09       │ 数据长度      │
│  3    │  4    │ 左轮   │ int32      │ 目标速度/编码器反馈 │
│  7    │  4    │ 右轮   │ int32      │ 目标速度/编码器反馈 │
│ 11    │  1    │ 校验   │ XOR        │ Byte2~10 异或  │
└───────┴───────┴────────┴────────────┴─────────────┘
```

- **下行** (Jetson→STM32)：目标左右轮速度，DMA 乒乓缓冲接收
- **上行** (STM32→Jetson)：每 100ms 反馈累积编码器脉冲数

### PID 控制流程

```
目标速度(Jetson) ──→ UART 解析 ──→ os.target_LeftSpeed
                                         │
                              ┌──────────▼──────────┐
                              │  TIM7 ISR (10ms)    │
                              │  读取编码器计数器     │
                              │  计算实际速度         │
                              │  PID 计算 PWM 输出   │
                              │  Motor_PWM() 驱动H桥 │
                              │  累积编码器值         │
                              └─────────────────────┘
                                         │
每100ms ──→ serial_communication() ──→ Jetson 里程计
```

---

## 环境要求

| 机器 | 操作系统 | ROS2 版本 | 硬件 |
|------|---------|----------|------|
| PC | Ubuntu 22.04 | Humble | WiFi/以太网 |
| Jetson | Ubuntu 20.04 | Foxy | Jetson Nano, CSI 摄像头, YDLIDAR F2, 9 轴 IMU, 电机驱动板 |

### 关键依赖

- **两台机器都需使用 Fast-DDS（ROS2 默认 RMW，无需额外安装）**

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
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=28
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
