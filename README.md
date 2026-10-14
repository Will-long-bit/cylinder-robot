# 移动机器人轨迹跟踪仿真项目（ROS / Gazebo）

一个基于 **ROS（Robot Operating System）** 与 **Gazebo** 仿真平台的移动机器人轨迹跟踪实验工程。
以"圆柱型机器人"为控制对象，依次实现了：

- 机器人**动力学建模与离散化驱动**
- **卡尔曼滤波（KF）** 状态估计与定位
- **PID 控制** 与 **模型预测控制（MPC）** 两种轨迹跟踪算法
- 多组实验对比 KF 的定位效果，以及 PID 与 MPC 在不同场景下的跟踪性能

> 本项目为教学 / 课程实验用途，所有代码在 Gazebo 仿真中运行，无需真实硬件。

---

## 系统架构

系统由 3 类 ROS 节点通过话题（topic）串联成控制闭环：

```
┌────────────┐   /robot/control (Twist)   ┌────────┐
│ Controller │ ──────────────────────────▶ │ Driver │
│ (PID/MPC)  │                            └────────┘
└────────────┘                                │
      ▲                                       │ /gazebo/set_model_state
      │                                       ▼
      │                             ┌──────────────────┐
      │  /robot/esti_model_state    │   Gazebo 仿真     │
      │                             └──────────────────┘
      │                                       │
┌────────────┐  /robot/observe (LaserScan)   │
│ Perception │ ◀─────────────────────────────┘
│   (KF)     │
└────────────┘
```

| 节点 | 文件 | 职责 |
|------|------|------|
| **driver**（驱动） | `catkin_ws/src/cylinder_robot/script/driver.py` | 订阅 `/robot/control`，用双积分器动力学更新状态 `[vx, px, vy, py]`，向 Gazebo 发布 `/gazebo/set_model_state` 真正驱动机器人 |
| **perception**（感知 / 定位） | `catkin_ws/src/cylinder_robot/script/perception.py` | 卡尔曼滤波节点：融合控制指令（预测步）与激光雷达观测（校正步），发布滤波后的位姿 `/robot/esti_model_state` |
| **controller**（控制） | 见下节 5 个控制器 | 订阅滤波位姿，计算控制律，发布 `/robot/control` |

> 说明：`driver` 把控制输入的 `linear.x` 当作 X 方向力、`angular.z` 当作 Y 方向力（平面双积分模型，无角速度概念）。

---

## 目录结构

```
project/
├── README.md                 # 本文件
├── .gitignore
└── catkin_ws/                # ROS catkin 工作空间
    ├── readme.txt            # 运行环境 & 编译运行说明
    └── src/cylinder_robot/
        ├── launch/
        │   ├── runCylinder.launch       # 自动启动 driver + controller + perception
        │   └── teleopCylinder.launch    # 手动遥控 + RViz
        ├── script/                      # 核心 Python 源码
        ├── data/                        # 图像轨迹素材 (ikun.png, ufo.png)
        ├── worlds/  models/  config/    # Gazebo 世界 / 模型 / RViz 配置
        └── results/                     # 4 张 真实 vs 估计 轨迹对比图
```

---

## 控制器与实验说明

`script/` 目录下提供 5 种运行模式（同一套驱动 / 感知框架，仅控制器不同）：

| 模式 | 文件 | 算法 | 轨迹 |
|------|------|------|------|
| PID 矩形 | `pid_rectangle_controller.py` | 双 PID（X/Y 各一，积分分离 + 输出限幅 ±15） | 矩形 4 角点 (4,0)→(4,3)→(0,3)→(0,0) |
| PID 迷宫 | `pid_maze_controller.py` | 同上，14 航点状态机 | 多层嵌套迷宫脱逃 |
| MPC 迷宫 | `controller.py`（实为 mpc_maze） | cvxpy 二次优化，预测 15 步，力限 ±20 | 同一迷宫，前瞻优化走直角 |
| MPC 图像 | `mpc_image_controller.py` | cvxpy，预测 30 步；从 PNG 提取轮廓→重采样→三次样条插值生成轨迹 | 自定义图像轮廓（默认 `ufo.png`） |
| 手动遥控 | `teleopCylinder.launch` | 键盘遥控，配 RViz | — |

### 演示视频

演示视频体积较大（合计约 1.2GB），未纳入本仓库，请通过以下链接观看（占位，待补充网盘 / B 站链接）：

| 视频 | 对应控制器 | 链接 |
|------|-----------|------|
| PID 走矩形 | `pid_rectangle_controller.py` | _待补充_ |
| PID 走迷宫 | `pid_maze_controller.py` | _待补充_ |
| MPC 走迷宫 | `controller.py`（MPC maze） | _待补充_ |
| MPC 图片轨迹 | `mpc_image_controller.py` | _待补充_ |

---

## 核心算法要点

- **动力学模型**：双积分器，质量 m=10，`ẋ = A x + B u + w`，状态 `[vx, px, vy, py]`；离散化用直接积分得到精确 `Ã, B̃`，并推导过程噪声协方差闭式解。
- **卡尔曼滤波**：观测矩阵只观测位置 `(px, py)`；预测步 + 校正步；过程噪声 `1e-5·I`，观测噪声 `0.02²·I`。
- **MPC**：二次型代价 `Σ Q·(x−x_ref)² + R·u²` + 终端代价 `Q_f`，约束动力学等式与力上下限，cvxpy 实时求解取首步控制量。
- **PID 技巧**：积分分离（误差 < 0.5m 才积分，防饱和）、输出限幅、到位需连续 1.5s 稳定在 3cm 内才切换下一航点。

---

## 运行方式

详细的**运行环境、依赖安装与编译启动命令**见 `catkin_ws/readme.txt`（含 Ubuntu + ROS Melodic 环境说明）。
简要流程：

```bash
cd catkin_ws
catkin_make
export GAZEBO_MODEL_PATH=$PWD/src/cylinder_robot/models:$GAZEBO_MODEL_PATH
source devel/setup.bash
roslaunch cylinder_robot runCylinder.launch
```

启动后机器人自动执行对应轨迹；结束后 `Ctrl+C` 退出，感知节点会自动保存 `fig_x.png` 轨迹对比图。

> 提示：源码含中文注释，若在某些环境（如头歌）因编码报错，可手动删除中文注释后再运行。
