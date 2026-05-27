# 项目代码结构

> Kinova Gen3 Lite 三维物块连续绘图系统 — 每个文件的角色

## 目录总览

```
project/
├── dynamics_ros1.py          # [参考] 动力学验证 (关节力矩→驱动力矩, 经真机验证)
├── jacobian.py               # [参考] 运动学验证 (FK/Jacobian/IK, 经真机验证)
├── plan.md                   # 系统架构设计文档 + 全部可调参数清单
├── deployment.md             # 实机部署指南
├── structure.md              # 本文件 — 代码结构映射
│
└── catkin_ws/src/
    ├── block_drawing_msgs/       # 自定义 ROS 消息/服务/动作定义
    ├── trajectory_generator/     # 【模块A】2D图案 → 3D面轨迹
    ├── robot_executor/           # 【模块B】运动规划 + 绘画状态机
    ├── force_controller/         # 【模块C】柔顺力控 (恒压)
    └── system_integrator/        # 系统集成 launch + 实验脚本
```

## 数据流总览

```
   run_experiment.py
        │
        ├─(1) /set_block_pose ──→ trajectory_server.cpp
        │                          │ setBlockPose()
        │                          │ projector_.setBlockSize(L,W,H)
        │                          │ projector_.setFaceOffset(per-face)
        │                          ▼
        ├─(2) /generate_trajectory → trajectory_server.cpp
        │                          │ reader_.loadFromFile(svg)
        │                          │ projector_.project2DToFace(pl,face_id)
        │                          │ → P_flange = P_surface - R*(0,0,tip)
        │                          ▼ return SurfaceTrajectory[]
        │
        ├─(3) /force_controller/calibrate → controller_node.cpp
        │                          │ estimator_.calibrate(tau_no_load)
        │                          ▼
        ├─(4) /force_control/active ← publish true
        │                          │ controller_node.cpp controlLoop()
        │                          │ F_est = estimator_.estimateNormalForce(q,tau)
        │                          │ Δz = impedance_ctrl_.computeCorrection(F_est)
        │                          │ publish /force_correction
        │                          ▼
        └─(5) /execute_drawing (action) → executor_node.cpp
                                   │ DrawingExecutor state machine
                                   │ MotionPrimitives → /in/cartesian_velocity
                                   │ PenOrientation → SLERP 姿态插值
                                   └→ kortex_driver → 真实机械臂
```

---

## 一、block_drawing_msgs — 全系统共用接口定义

只定义数据格式，不含逻辑。

| 文件 | 角色 |
|------|------|
| `msg/SurfaceTrajectory.msg` | 单面轨迹: face_id + waypoints[] (Pose 数组) + arc_lengths[] (累计弧长) |
| `msg/ContactState.msg` | 笔尖接触状态: normal_force (法向力) + torque (力矩) + in_contact (是否接触) |
| `srv/SetBlockPose.srv` | 设置物块位姿 T_block_base + 尺寸 L/W/H + 各面投影偏移 face_offset_u/v[5] |
| `srv/GenerateTrajectory.srv` | 模块A 服务: svg_file + target_w/h_mm + faces[] → SurfaceTrajectory[] |
| `srv/ExecuteDrawing.srv` | 简单的启动/取消绘画 trigger |
| `action/DrawingExecution.action` | 绘画执行 action: goal (trajectories[]) → feedback (进度/力/状态) → result (是否完成) |

---

## 二、trajectory_generator — 【模块A】SVG → 3D 轨迹

**输入**: SVG 文件 + 物块位姿/尺寸 + 目标面列表
**输出**: 每个面的 `SurfaceTrajectory` (含法兰补偿 waypoints)
**核心类**: SvgReader (SVG 解析), SurfaceProjector (2D→3D 投影)

### 头文件

| 文件 | 核心内容 |
|------|---------|
| `include/trajectory_generator/types.h` | **基础数据结构**: Point2D (2D 点), Polyline (折线 + 是否闭合), FaceFrame (面坐标系: origin/u/v/normal) |
| `include/trajectory_generator/svg_reader.h` | SvgReader 接口: loadFromFile(), loadFromPoints(), generateTestSquare/Circle() |
| `include/trajectory_generator/surface_projector.h` | **核心投影类**: setBlockPose(), setBlockSize(), setFaceOffset(), project2DToFace(), generateTransitionWaypoints() |

### 实现文件

| 文件 | 核心逻辑 |
|------|---------|
| `src/svg_reader.cpp` | SVG path 解析 (M/L/C/S/Z/H/V 指令) → Polyline 序列; Bezier 曲线采样; 坐标归一化居中到 target_w×target_h (mm) |
| `src/surface_projector.cpp` | **核心投影**: 物块 5 面局部坐标系定义 → T_block_base 变换到基坐标系 → `P_on_surface = anchored_origin + x*u + y*v` → 笔姿态 `Z=-normal, X=tangent` → 法兰补偿 `P_flange = P_on_surface - R*(0,0,pen_tip_offset)` → 跨面过渡 waypoints 生成 |
| `src/trajectory_server.cpp` | **ROS 服务节点**: `/set_block_pose` (设置物块) + `/generate_trajectory` (生成轨迹) + RViz MarkerArray 可视化发布 `/trajectory_markers` |

### 辅助脚本

| 文件 | 角色 |
|------|------|
| `scripts/pattern_designer.py` | 生成 SVG 测试图案 (正方形/圆/星形)，不依赖机械臂 |

---

## 三、robot_executor — 【模块B】运动规划 + 绘画执行

**输入**: SurfaceTrajectory[] (来自模块A) + joint_states (来自 driver) + force_correction (来自模块C)
**输出**: `/in/cartesian_velocity` 或 MoveIt trajectory → 机械臂运动
**核心类**: DrawingExecutor (状态机), MotionPrimitives (基本动作), PenOrientation (笔姿态)

### 头文件

| 文件 | 核心内容 |
|------|---------|
| `include/robot_executor/drawing_executor.h` | **6 状态主状态机**: State 枚举 (IDLE/MOVING_TO_START/DRAWING/LIFTING/SWITCHING_FACE/HOMING/ERROR), DrawingParams 结构体, Action 服务器接口 |
| `include/robot_executor/motion_primitives.h` | **基本动作接口**: goHome(), liftPen(), lowerPen(), moveFreeSpace(), stop(), clearFaults(), publishCartesianVelocity(), getCurrentPose() |
| `include/robot_executor/pen_orientation.h` | **笔姿态计算**: computePenOrientation(surface_normal, travel_direction) → Z 轴沿 -normal, X 轴沿行进方向 + SLERP 姿态插值 |

### 实现文件

| 文件 | 核心逻辑 |
|------|---------|
| `src/drawing_executor.cpp` | **绘画状态机**: executeDrawingPlan() → enterMovingToStart()(空移到起点上方) → enterDrawing()(MoveIt CartesianPath 或逐点速度控制, 叠加 force_correction) → enterLifting()(沿面法向抬笔) → enterSwitchingFace()(换面) → enterHoming()(回 home)。通过 Action 服务器反馈进度 |
| `src/motion_primitives.cpp` | **基本动作实现**: goHome() → MoveIt JointTarget; liftPen() → 沿面法向移动 lift_height; **getCurrentPose() → 6-DOF 正运动学** (Standard DH 参数, = jacobian.py); publishCartesianVelocity() → `/in/cartesian_velocity` |
| `src/pen_orientation.cpp` | **笔姿态**: `Z = -surface_normal, X = travel_dir_proj_⊥_Z, Y = Z×X` + SLERP 插值 |
| `src/executor_node.cpp` | **主入口**: 创建 DrawingExecutor, 暴露 `/execute_drawing` action, ros::spin() |

### 配置

| 文件 | 角色 |
|------|------|
| `config/drawing_params.yaml` | speed(0.03), free_speed(0.15), lift_height(0.05), pen_tip_offset(0.12), eef_step(0.001), planning_time(2.0), planner_id, home_joints[6] |
| `launch/start_executor.launch` | 单独启动 executor_node (需要 MoveIt + driver 已运行) |

---

## 四、force_controller — 【模块C】柔顺力控

**输入**: joint_states (关节角 + effort 力矩) + base_feedback (40Hz)
**输出**: `/force_correction` (geometry_msgs/Vector3, 法向位置修正量) → 模块B 叠加
**核心类**: CurrentEstimator (力矩→末端力), ImpedanceController (力误差→Z 修正量)

### 头文件

| 文件 | 核心内容 |
|------|---------|
| `include/force_controller/current_estimator.h` | **力矩→力估计**: calibrate() (零偏标定), estimateNormalForce(q, tau, surface_normal), computeJacobian(q), forwardKinematics(q), getExternalTorques() |
| `include/force_controller/impedance_controller.h` | **虚拟阻抗**: computePoseCorrection(F_estimated, surface_normal, dt) → 一阶 (Δz=Kp*F_error) 或二阶 (M·Δz̈+D·Δż+K·Δz=F_error), reset(), setParams() |

### 实现文件

| 文件 | 核心逻辑 |
|------|---------|
| `src/current_estimator.cpp` | **标准 DH 正运动学 + Jacobian**: DH 参数与 jacobian.py 一致 ([α, a, d, θ_off]), dhTransformStandard() 用标准 DH 公式, computeJacobian() 返回 6×6 几何 Jacobian, estimateNormalForce() → F = pinv(J_lin^T) * τ_ext → 投影到表面法向 |
| `src/impedance_controller.cpp` | **力→位置修正**: 一阶 Δz = Kp * (F_desired - F_est), clamp(±dz_max); 二阶用前向欧拉离散, 输出 surface_normal * Δz |
| `src/controller_node.cpp` | **~100Hz 控制循环主节点**: 订阅 joint_states + base_feedback → 用 estimator_.estimateNormalForce() → impedance_ctrl_.computePoseCorrection() → 发布 `/force_correction`。服务: `/force_controller/calibrate` (记录零偏)。话题: `/force_control/active` (开关) |

### 配置

| 文件 | 角色 |
|------|------|
| `config/controller_params.yaml` | desired_force(1.0N), Kp(0.0003), Kd(0.00005), dz_max(0.0005m), control_rate(100Hz), contact_threshold(0.3N) |

---

## 五、system_integrator — 系统集成 + 实验工具

### Launch 文件

| 文件 | 角色 |
|------|------|
| `launch/full_system.launch` | **一键启动全系统**: kortex_driver + MoveIt move_group + trajectory_server (A) + executor_node (B) + controller_node (C) + RViz。参数: ip_address, robot_name, use_gazebo, start_rviz |
| `launch/simulation.launch` | Gazebo 仿真快捷启动 (调用 full_system.launch use_gazebo:=true) |

### 实验脚本

| 文件 | 角色 |
|------|------|
| `scripts/run_experiment.py` | **主实验脚本**: (1) 设置物块位姿/尺寸/面偏移 → (2) 生成轨迹 → (3) 力传感器标定 → (4) 激活力控 → (5) 发送 DrawingExecution action → (6) rosbag 录制。CLI: `--svg`, `--test-square`, `--width`, `--faces`, `--block-x/y/z/L/W/H`, `--face-offsets-u/v` |
| `scripts/evaluate_experiment.py` | **离线评估**: 读取 rosbag → 计算线条连续性 (连接点空间偏差)、力一致性 (CV)、数据统计 → 输出报告 |
| `scripts/plot_results.py` | **结果可视化**: 从 rosbag 生成 force_profile.png, joint_torques.png, progress.png, summary.png |

---

## 六、项目根目录参考文件

| 文件 | 角色 |
|------|------|
| `jacobian.py` | **运动学参考实现** (真机验证): Standard DH FK, 6×6 几何 Jacobian, IK (数值迭代)。我们的 C++ current_estimator.cpp 和 motion_primitives.cpp 的 DH 参数/变换公式均与此一致 |
| `dynamics_ros1.py` | **动力学验证** (真机验证): Newton-Euler 递推计算关节驱动力矩, 与真实 /joint_states/effort 对比验证。含各连杆质量/惯量参数 (可用于更精确的力估计) |
| `plan.md` | 系统架构设计文档 + 附录 A 全部可调参数清单 |
| `deployment.md` | 实机部署步骤 (环境准备/编译/网络配置/启动/实验/调试/安全) |

---

## 七、核心实现路线图

```
[用户图案]               [物块参数]              [力控策略]
   │                        │                      │
   ▼                        ▼                      ▼
svg_reader.cpp    surface_projector.cpp    current_estimator.cpp
  │  SVG→Polyline     │  setBlockPose()      │  calibrate(bias)
  │                   │  setFaceOffset()     │  estimateNormalForce()
  ▼                   │  project2DToFace()   │  forwardKinematics()
 Polyline[]           │  ↓                   │  computeJacobian()
                      │  P_flange waypoints  │  ↓
                      ▼                      │  pinv(J^T)*τ → F_est
              trajectory_server.cpp          │
                /set_block_pose (srv)        ▼
                /generate_trajectory (srv)   impedance_controller.cpp
                /trajectory_markers (topic)    │  Δz=Kp*(F_desired-F_est)
                ↓                              │  clamp(±dz_max)
        SurfaceTrajectory[]                    │  ↓
                ↓                              │  /force_correction (topic)
        drawing_executor.cpp      ◄────────────┘
          │ 6-state machine
          │ enterMovingToStart()
          │ enterDrawing() ─→ pen_orientation.cpp
          │   │                 computePenOrientation()
          │   │                 SLERP interpolate
          │   ▼
          │ motion_primitives.cpp
          │   moveFreeSpace() / lowerPen() / liftPen()
          │   getCurrentPose() (FK from DH)
          │   publishCartesianVelocity() → /in/cartesian_velocity
          │   stop() → /in/stop
          ▼
      executor_node.cpp
        /execute_drawing (action)
        feedback: progress + force + state
```
