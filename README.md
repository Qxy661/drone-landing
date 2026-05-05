# drone-landing

ROS2 精准视觉降落系统。ArUco 标记检测 + PnP 6DOF 位姿估计 + 级联 PID 控制。

## 功能

- ArUco 标记检测 (多字典支持)
- PnP 6DOF 位姿估计 (solvePnP + LM 迭代优化)
- 级联 PID 控制 (位置->速度->姿态)
- 5 阶段降落状态机 (COARSE/FINE/DESCENT/FLARE/LANDED)
- 视觉丢失处理 + 安全中止

## 架构

```
摄像头 -> landing_detector (ArUco + PnP) -> landing_controller (级联PID) -> MAVROS -> 飞控
                      mission_planner (任务编排)
```

## 算法原理

```
ArUco Marker (地面)
    |
  相机检测 2D 角点
    |
  solvePnP -> 6DOF 位姿 (x, y, z, roll, pitch, yaw)
    |
  级联 PID 控制:
  - 水平位置误差 -> 水平速度命令
  - 水平速度误差 -> 倾斜角命令
  - 高度误差 -> 垂直速度命令
```

## 降落阶段

| 阶段 | 高度 | 水平精度 | 下降速度 | 说明 |
|------|------|---------|---------|------|
| COARSE | >3m | ±0.5m | - | 粗定位, 大幅修正 |
| FINE | 1-3m | ±0.2m | - | 精对准, 小幅修正 |
| DESCENT | 0.3-1m | ±0.1m | 0.3 m/s | 稳定下降 |
| FLARE | <0.3m | ±0.05m | 0.1 m/s | 缓冲着陆 |
| LANDED | 0 | - | 0 | 完成 |

## 快速开始

```bash
# 安装依赖
pip install opencv-python numpy
sudo apt install ros-humble-mavros ros-humble-cv-bridge

# 编译
cd ros2_ws && colcon build --packages-select drone_landing
source install/setup.bash

# 运行测试
python3 src/drone_landing/test/test_aruco_pose.py

# 启动系统
ros2 launch drone_landing landing_test.launch.py
```

## License

MIT
