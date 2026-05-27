# Kinova Gen3 Lite 三维物块连续绘图系统 — 代码架构与实现方案

## 0. 依赖: 官方 ros_kortex 驱动

官方仓库: `https://github.com/Kinovarobotics/ros_kortex` (branch: `noetic-devel`)

### ros_kortex 包含的包

| 包名 | 用途 | 我们的关系 |
|------|------|-----------|
| `kortex_driver` | ROS Noetic 驱动节点, 连接机械臂硬件 | **直接依赖, 不改动** |
| `kortex_description` | URDF/xacro/STL 模型文件 | **直接依赖, 不改动** |
| `kortex_moveit_config` / `kinova_gen3_lite_moveit_config` | MoveIt 配置 (SRDF, kinematics.yaml, ompl_planning.yaml) | **直接依赖, 可调参数** |
| `kortex_gazebo` | Gazebo 仿真 | 仿真测试用 |
| `kortex_examples` | C++ / Python 示例代码 | 参考学习 |
| `kortex_control` | ros_control 控制器配置 | 仿真用 |

### Gen3 Lite 关键参数
- 6 DOF, 手爪 `gen3_lite_2f`, 无视觉模块 (`vision=false`)
- 固件版本: v2.3.2, Kortex API: 通过 kortex_driver 封装
- 默认通信 IP: `192.168.1.10`, 反馈频率: 40 Hz

### kortex_driver 提供的核心 ROS 接口 (我们直接使用)

**控制 Topics (我们 publish)**:
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/<robot_name>/in/joint_velocity` | `kortex_driver/Base_JointSpeeds` | 关节速度控制 |
| `/<robot_name>/in/cartesian_velocity` | `kortex_driver/TwistCommand` | 末端 Cartesian 速度控制 |
| `/<robot_name>/in/stop` | `std_msgs/Empty` | 平滑停止 |
| `/<robot_name>/in/clear_faults` | `std_msgs/Empty` | 清除故障 |

**反馈 Topics (我们 subscribe)**:
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/<robot_name>/base_feedback` | `BaseCyclic_Feedback` | 完整反馈(含关节扭矩/电流, 40Hz) |
| `/<robot_name>/joint_state` | `sensor_msgs/JointState` | 关节状态 |

**启动命令:**
```bash
roslaunch kortex_driver kortex_driver.launch \
  arm:=gen3_lite dof:=6 gripper:=gen3_lite_2f \
  ip_address:=192.168.1.10 start_rviz:=true
```

> 我们的工作基于 ros_kortex **之上**开发应用层, 不动驱动层代码。

---

## 1. 系统总览

```
                        已知: T_block_base (物块→机械臂基座)
┌─────────────────────────────────────────────────────────────────┐
│                      系统数据流                                  │
│                                                                 │
│  SVG/图案 ──► [模块A] 2D→3D轨迹生成 ──► 末端位姿序列(waypoints)  │
│                   (人员1)                   │                   │
│                                            ▼                    │
│                               [模块B] 运动规划与执行              │
│                    ┌─────  (人员2, C++)  ───────┐               │
│                    │ MoveGroupInterface (MoveIt)│               │
│                    │   → plan + execute 轨迹     │               │
│                    │   或 Cartesian velocity     │               │
│                    │   直接控制 topic            │               │
│                    └────────────┬───────────────┘               │
│                                 │                               │
│  [模块C] 柔顺控制 ◄── /joint_state + /base_feedback (40Hz)      │
│    (人员3)          │                                           │
│       │             │                                           │
│       └──高度修正──►│  修正末端Z向位置(叠加到模块B输出)           │
│                     │                                           │
│                     ▼                                           │
│              /in/cartesian_velocity 或 MoveIt trajectory        │
│                     │                                           │
│                     ▼                                           │
│              kortex_driver → 真实机械臂                          │
│                                                                 │
│  已知前提: 物块在机械臂基坐标系下的位姿 T_block_base 已给定       │
└─────────────────────────────────────────────────────────────────┘
```

## 2. ROS Package 结构 (我们开发的包)

```
catkin_ws/src/
├── ros_kortex/                   # [外部依赖] 官方驱动 — 不动
│   ├── kortex_driver/
│   ├── kortex_description/
│   ├── kortex_moveit_config/
│   ├── kinova_gen3_lite_moveit_config/
│   ├── kortex_gazebo/
│   └── ...
│
├── block_drawing_msgs/           # [自定义] 消息包 — 全组共用接口
│   ├── msg/
│   │   ├── SurfaceTrajectory.msg     # 单面轨迹 (face_id + Pose[] waypoints + 弧长)
│   │   └── ContactState.msg          # 接触状态 (法向力估计值, 力矩)
│   ├── srv/
│   │   ├── SetBlockPose.srv          # 设置物块位姿 (T_block_base)
│   │   ├── GenerateTrajectory.srv    # 模块A服务: SVG → 轨迹序列
│   │   └── ExecuteDrawing.srv        # 模块B服务: 启动绘画执行
│   └── action/
│       └── DrawingExecution.action   # 执行过程可暂停/取消/反馈进度
│
├── trajectory_generator/         # [人员1] 图形→3D轨迹
│   ├── src/
│   │   ├── svg_reader.cpp             # SVG读取 + 离散化为折线
│   │   ├── surface_projector.cpp      # ★核心: 2D折线→面3D坐标投影
│   │   └── trajectory_server.cpp      # ROS Service 节点
│   ├── include/trajectory_generator/
│   │   ├── svg_reader.h
│   │   ├── surface_projector.h
│   │   └── types.h                    # Point2D, Polyline, FaceFrame 等
│   ├── scripts/
│   │   └── pattern_designer.py        # 辅助: 可视化编辑2D图案
│   └── CMakeLists.txt
│
├── robot_executor/               # [人员2] 运动规划与执行
│   ├── src/
│   │   ├── drawing_executor.cpp       # ★绘画状态机 (6状态)
│   │   ├── pen_orientation.cpp        # 笔尖姿态: 保持Z轴垂直于当前面
│   │   ├── motion_primitives.cpp      # 基本动作: goHome, liftPen, lowerPen
│   │   └── executor_node.cpp          # ROS Node 主程序
│   ├── include/robot_executor/
│   │   ├── drawing_executor.h
│   │   ├── pen_orientation.h
│   │   └── motion_primitives.h
│   ├── config/
│   │   └── drawing_params.yaml        # 速度/加速度/笔偏移/抬笔高度
│   ├── launch/
│   │   └── start_executor.launch      # 启动执行器 (需 MoveIt 已运行)
│   └── CMakeLists.txt
│
├── force_controller/             # [人员3] 恒压控制
│   ├── src/
│   │   ├── current_estimator.cpp      # 关节力矩→末端力估计 (Jacobian转置)
│   │   ├── impedance_controller.cpp   # 虚拟阻抗: 力误差→Z修正量
│   │   └── controller_node.cpp        # ROS Node, ~100Hz 控制循环
│   ├── include/force_controller/
│   │   ├── current_estimator.h
│   │   └── impedance_controller.h
│   ├── config/
│   │   └── controller_params.yaml     # 期望力, 刚度/阻尼系数
│   └── CMakeLists.txt
│
└── system_integrator/            # [人员3] 系统集成 + 实验
    ├── launch/
    │   ├── full_system.launch          # ★一键启动: 驱动+MoveIt+执行器+控制器
    │   └── simulation.launch           # Gazebo仿真启动
    ├── scripts/
    │   ├── run_experiment.py           # 主实验脚本
    │   ├── evaluate_experiment.py      # 实验评价: 线条连续性/压力方差
    │   └── plot_results.py             # 结果可视化
    └── CMakeLists.txt
```

## 3. 核心模块详细设计

### 3.1 模块A: trajectory_generator (人员1)

#### 输入与输出
- **输入**: SVG 文件路径 + 物块位姿(T_block_base) + 物块尺寸(L,W,H) + 要画的面的列表
- **输出**: `SurfaceTrajectory[]` — 每个元素对应一个面, 包含该面上的所有 waypoints

#### 关键类

**SvgReader** — SVG/图案读取
```cpp
// 轻量实现: 不依赖 librsvg, 使用 nanosvg 头文件库 (单头文件, ~2000行)
// 或直接用 tinyxml2 解析 SVG path 的 M/L/C 指令
class SvgReader {
public:
    // 读取 SVG 并输出有序折线段序列 (单位: mm, 坐标系原点在图案中心)
    std::vector<Polyline> loadFromFile(const std::string& path,
                                        double target_width_mm,
                                        double target_height_mm);
    // 也支持代码直接定义图案点列
    std::vector<Polyline> loadFromPoints(
        const std::vector<std::vector<Point2D>>& paths);
};

struct Polyline {
    std::vector<Point2D> points;  // 连续折线顶点
    bool isClosed;                // true=首尾相连闭合轮廓
};
```

**SurfaceProjector** — 核心投影算法 (论文核心)
```cpp
class SurfaceProjector {
public:
    void setBlockPose(const Eigen::Isometry3d& T_block_base);
    void setBlockSize(double L, double W, double H);

    // 将2D图案的 Polyline 投影到物块第 face_id 个面
    // face_id: 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面
    SurfaceTrajectory project2DToFace(
        const Polyline& polyline,
        int face_id,
        double pen_tip_offset  // 法兰→笔尖长度(m)
    );

    struct FaceFrame {
        Eigen::Vector3d origin;   // 面中心(在基坐标系下)
        Eigen::Vector3d u_axis;   // 面内水平方向 (←→)
        Eigen::Vector3d v_axis;   // 面内竖直方向 (↑↓, 侧面v朝上)
        Eigen::Vector3d normal;   // 外法向量 (笔尖应指向-normal)
    };
    FaceFrame getFaceFrame(int face_id) const;

private:
    // 物块各面局部坐标系预设 (物块坐标系原点=底面中心)
    //   顶面: origin=(0,0,H/2), u=(1,0,0), v=(0,1,0), normal=(0,0,1)
    //   前面: origin=(0,W/2,0), u=(1,0,0), v=(0,0,1), normal=(0,1,0)
    //   右面: origin=(L/2,0,0), u=(0,1,0), v=(0,0,1), normal=(1,0,0)
    //   ...
    // 全部经 T_block_base 变换到基坐标系
};
```

**投影算法逻辑 (伪代码):**
```
project2DToFace(polyline, face_id):
  1. FaceFrame F = getFaceFrame(face_id)  // 已在基坐标系
  2. waypoints = []
  3. for each 2D point p_2d=(x_mm, y_mm) in polyline:
       // 2D映射: x→u_axis方向, y→v_axis方向
       P_on_surface = F.origin + x_mm*F.u_axis + y_mm*F.v_axis
       // 姿态: 笔尖Z沿-normal(指向物块), 笔尖X沿切线
       R_pen = lookAt(tangent_direction, -F.normal)
       // 法兰位置补偿笔尖偏移
       P_flange = P_on_surface - R_pen * (0, 0, tip_length)
       waypoints.push_back({P_flange, R_pen})
  4. return SurfaceTrajectory{face_id, waypoints}
```

**跨面过渡处理 (在 trajectory_server 中实现):**
```
对相邻两面轨迹做连接:
  - face_i 最后一个 waypoint → 抬笔(沿face_i.normal方向退后5cm)
  - 空中移动到 face_{i+1} 第一个 waypoint 上方(同样沿face_{i+1}.normal退后5cm)
  - 下笔到 face_{i+1} 第一个 waypoint
  - 在序列中插入 lift/drop action 标记
```

#### 输出格式 (block_drawing_msgs/SurfaceTrajectory.msg)
```
int32   face_id                    # 0~4: 顶/前/右/后/左
geometry_msgs/Pose[] waypoints    # 末端期望位姿(已含法兰补偿)
float64[] arc_lengths             # 累计弧长, 用于速度规划
```

### 3.2 模块B: robot_executor (人员2)

#### 与 ros_kortex 的接口关系

```
robot_executor 运行在我们的 executor_node 中, 通过下列接口与外部通信:

  输入:
    ← SurfaceTrajectory[]  (从模块A的 service 获取)
    ← /joint_state          (从 kortex_driver 订阅, 关节位置/速度)
    ← 高度修正量 Δz         (从模块C topic 订阅)

  输出:
    → MoveGroupInterface   (MoveIt C++ API: computeCartesianPath + execute)
    → 或 /in/cartesian_velocity  (直接速度控制, 实时跟随)
    → /in/stop              (紧急停止)
```

#### 双模式执行策略

**模式1 (推荐用于绘画) — MoveIt computeCartesianPath:**
```python
# 伪代码
waypoints = module_A_output[i].waypoints
# 逐面规划
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, eef_step=0.001, jump_threshold=0.0)
if fraction >= 0.99:
    move_group.execute(plan)
```

**模式2 (实时跟随, 力控必需) — Cartesian velocity 直接控制:**
- 订阅 `/joint_state` 获取当前位姿
- 计算当前位置 → 下一waypoint 的方向
- 发布 `/in/cartesian_velocity` 控制末端运动
- 模块C的Δz叠加到法线方向的速度分量上
- 适合需要实时力反馈修正的场景

#### 关键类

**DrawingExecutor** — 主状态机
```cpp
class DrawingExecutor {
public:
    enum State { IDLE, MOVING_TO_START, DRAWING, LIFTING,
                 SWITCHING_FACE, HOMING, ERROR };

    // 接收轨迹序列并启动执行
    void executeDrawingPlan(
        const std::vector<SurfaceTrajectory>& trajectory_sequence,
        const DrawingParams& params
    );

    void pause();   void resume();   void abort();

private:
    State current_state_;
    ros::Subscriber sub_joint_state_;    // 订阅 /joint_state
    ros::Subscriber sub_z_correction_;   // 订阅模块C的Z修正量
    ros::Publisher  pub_cart_vel_;       // /in/cartesian_velocity
    ros::Publisher  pub_stop_;           // /in/stop
    ros::ServiceClient srv_clear_faults_;// /in/clear_faults

    // moveit::planning_interface::MoveGroupInterface
    // moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    std::unique_ptr<MoveGroupInterface> move_group_;

    size_t current_face_idx_;
    size_t current_waypoint_idx_;
};
```

**状态机流程:**
```
IDLE → MOVING_TO_START → DRAWING(逐waypoint) → LIFTING(抬笔) →
  ├─ 下一面还有轨迹 → SWITCHING_FACE → MOVING_TO_START → DRAWING
  └─ 全部完成 → HOMING → IDLE

每个 State 的 action:
  MOVING_TO_START : MoveIt 自由运动到第一面的起点(抬笔高度)
  DRAWING         : computeCartesianPath + execute / 或 cart_vel 逐点跟随
                    每到达一个 waypoint, 检查是否需要 force correction
  LIFTING         : 沿当前面法线方向抬高 lift_height
  SWITCHING_FACE  : 空中移动到下一面起点
  HOMING          : goHome (预定义关节位姿)
```

**MotionPrimitives** — 基本动作 (基于 MoveGroupInterface)
```cpp
class MotionPrimitives {
public:
    bool goHome();    // 回预定义 Home 关节位姿
    bool liftPen(const Eigen::Vector3d& surface_normal, double height);
    bool lowerPen(const Eigen::Isometry3d& target_pose);
    bool moveFreeSpace(const Eigen::Isometry3d& target);  // 自由空间运动(TCP)
    void stop();      // 发布到 /in/stop
    void clearFaults();
};
```

**PenOrientation** — 笔尖姿态控制
```cpp
class PenOrientation {
public:
    // 给定行进方向和面法向量, 计算笔尖姿态四元数
    // Z轴 = -surface_normal(指向物块), X轴 = travel_direction
    Eigen::Quaterniond computePenOrientation(
        const Eigen::Vector3d& surface_normal,
        const Eigen::Vector3d& travel_direction
    );

    // 转角处用SLERP平滑
    std::vector<Eigen::Quaterniond> interpolateOrientation(
        const Eigen::Quaterniond& from, const Eigen::Quaterniond& to, int steps
    );
};
```

#### 配置文件 drawing_params.yaml
```yaml
drawing:
  speed: 0.03          # 绘画速度 m/s
  free_speed: 0.15     # 空移速度
  lift_height: 0.05    # 抬笔高度 m
  pen_tip_offset: 0.12 # 法兰→笔尖 m
  eef_step: 0.001      # Cartesian path 离散步长

planner:
  planning_time: 2.0   # MoveIt 规划时间上限
  planner_id: "RRTConnectkConfigDefault"

robot:
  name: "my_gen3_lite"
  home_joints: [0.0, 0.26, 3.14, -1.96, 0.0, 1.57]  # 示例
```

### 3.3 模块C: force_controller (人员3)

#### 与 ros_kortex 的接口关系

```
force_controller 运行在我们的 controller_node 中:

  输入:
    ← /joint_state          (sensor_msgs/JointState, 来自 kortex_driver, 40Hz)
       .position[] → 关节角度 (rad)
       .effort[]   → 关节力矩 (Nm, 内置扭矩传感器)
    ← /base_feedback        (BaseCyclic_Feedback, 来自 kortex_driver, 40Hz)
       .actuators[].current_motor → 各关节电机电流 (A)
       .actuators[].torque        → 各关节扭矩

  输出:
    → /force_correction     (自定义 geometry_msgs/Vector3) 发给模块B
       .x/.y/.z: 在面法线方向上的位置修正量 (m)
```

#### 关键类

**CurrentEstimator** — 关节力矩→末端力估计
```cpp
class CurrentEstimator {
public:
    // 核心公式: τ_external = J^T(q) * F_tip
    // → F_tip ≈ pinv(J^T) * τ_external  (只用末端3个关节简化)
    //
    // 简化策略: 监测末端关节(腕部)力矩变化即可判断是否有接触力
    //   因为接触力主要通过腕部关节力矩体现
    double estimateNormalForce(
        const std::vector<double>& q,           // 关节角度
        const std::vector<double>& tau_measured  // 实测关节力矩
    );

    void calibrate(const std::vector<double>& tau_no_load);
        // 无负载状态下记录 bias, 以后测的减去 bias 即得到外力矩

private:
    std::vector<double> bias_;  // 空载力矩基线
    Eigen::MatrixXd jacobian_;  // 当前关节角的 Jacobian
};
```

**ImpedanceController** — 虚拟阻抗控制
```cpp
class ImpedanceController {
public:
    // 一阶力补偿 (最简单可用):
    //   Δz = Kp * (F_desired - F_estimated)
    //   clamp(Δz, -dz_max, +dz_max)  // 限制修正幅度
    //
    // 二阶虚拟阻抗 (更平滑):
    //   M * Δz̈ + D * Δż + K * Δz = F_error
    Eigen::Vector3d computePoseCorrection(
        double F_estimated,
        double dt  // 控制周期
    );

    void reset();  // 每面开始前清零历史
    void setParams(double Kp, double F_desired, double dz_max);

private:
    double Kp_, Kd_;          // 阻抗参数
    double F_desired_;        // 期望接触力 (0.5 ~ 2.0 N)
    double dz_max_;          // 单步最大修正量 (0.5 mm)
    double prev_error_, prev_z_correction_;
};
```

**控制循环流程 (在 controller_node 中, ~100Hz):**
```cpp
void controlLoop(const ros::TimerEvent&) {
    // 1. 从最新 JointState 获取 q, tau
    // 2. F_est = current_estimator_.estimateNormalForce(q, tau)
    // 3. if (当前在绘画状态):
    //       Δz = impedance_controller_.computePoseCorrection(F_est, dt)
    //       // 只在法线方向修正, 不影响横向轨迹
    //       correction = face_normal * Δz
    //       pub_correction_.publish(correction)
    //    else:
    //       impedance_controller_.reset()
}
```

#### 配置文件 controller_params.yaml
```yaml
force_control:
  desired_force: 1.0      # 期望笔尖压力 (N)
  Kp: 0.0003              # 比例增益 (m/N)
  Kd: 0.00005             # 阻尼
  dz_max: 0.0005          # 单步最大修正 (0.5 mm)
  control_rate: 100       # Hz
  contact_threshold: 0.3  # 判定接触的最小力 (N)
```

### 3.4 系统集成 (人员3): launch 文件 + 实验脚本

#### full_system.launch 结构
```xml
<launch>
  <!-- 1. 启动 kortex_driver (连接真实机械臂) -->
  <include file="$(find kortex_driver)/launch/kortex_driver.launch">
    <arg name="arm" value="gen3_lite"/>
    <arg name="dof" value="6"/>
    <arg name="gripper" value="gen3_lite_2f"/>
    <arg name="ip_address" value="$(arg ip_address)"/>
    <arg name="start_rviz" value="true"/>
  </include>

  <!-- 2. 启动 MoveIt move_group -->
  <include file="$(find kinova_gen3_lite_moveit_config)/launch/move_group.launch"/>

  <!-- 3. 启动模块A trajectory_generator (service) -->
  <node name="trajectory_server" pkg="trajectory_generator"
        type="trajectory_server" output="screen"/>

  <!-- 4. 启动模块B executor -->
  <node name="executor_node" pkg="robot_executor"
        type="executor_node" output="screen">
    <rosparam file="$(find robot_executor)/config/drawing_params.yaml"/>
  </node>

  <!-- 5. 启动模块C force controller -->
  <node name="controller_node" pkg="force_controller"
        type="controller_node" output="screen">
    <rosparam file="$(find force_controller)/config/controller_params.yaml"/>
  </node>
</launch>
```

#### 实验流程脚本 run_experiment.py 伪代码
```python
# 完整实验流程:
# 1. roslaunch full_system.launch           ← 启动全部节点
# 2. rosrun system_integrator run_experiment.py ← 运行实验

def run_experiment(svg_file, block_pose, block_size, faces, output_dir):
    rospy.init_node("experiment_runner")

    # Step 1: 设置物块位姿
    set_block = rospy.ServiceProxy("/set_block_pose", SetBlockPose)
    set_block(block_pose, block_size)

    # Step 2: 调用模块A生成轨迹
    gen_traj = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)
    resp = gen_traj(svg_file, faces)

    # Step 3: 标定力传感器零位
    calib = rospy.ServiceProxy("/force_controller/calibrate", Calibrate)
    calib()

    # Step 4: 开始记录数据
    bag = rosbag.Bag(f"{output_dir}/experiment.bag", 'w')

    # Step 5: 执行绘画 (Action 模式, 可暂停/反馈)
    client = actionlib.SimpleActionClient("/execute_drawing", DrawingExecutionAction)
    client.wait_for_server()
    goal = DrawingExecutionGoal(trajectories=resp.trajectories)
    client.send_goal(goal,
        feedback_cb=lambda fb: save_progress(fb),
        done_cb=lambda state, result: save_result(result, bag))

    # Step 6: 保存实验数据
    bag.close()
    evaluate_experiment(f"{output_dir}/experiment.bag")

# 实验评价指标:
#  - 线条连续性: 两段轨迹衔接处的空间距离 (mm)
#  - 压力一致性: 力估计值的标准差 / 均值
#  - 换面偏移: 跨面后实际落笔点与期望点的偏差
#  - 轨迹跟踪误差: 期望位姿与实际位姿的 RMS
```

## 4. 关键技术决策

### 4.1 两种执行模式的选择

| 场景 | 推荐模式 | 原因 |
|------|---------|------|
| 仿真 + 无模块C | MoveIt computeCartesianPath | 简单, 不需要自己写轨迹跟踪 |
| 真机 + 有模块C力控 | Cartesian velocity 直接控制 | 可以每帧叠加力修正量 |
| 真机 + 无模块C (纯位置) | MoveIt computeCartesianPath | 路径预规划, MoveIt保证无碰撞 |

**建议**: 先用 MoveIt 模式跑通基础功能, 再加模块C时切换为 Cartesian velocity 模式。

### 4.2 笔尖垂直表面策略
- **面内绘画时**: 笔尖Z轴 = -face_normal (笔尖指向物块内部)
- **行进方向**: 笔尖X轴 = normalize(current_waypoint - prev_waypoint)
- **Y轴由cross(X,Z)自动确定** (右手系)
- **转角处**: 姿态提前10个点开始 SLERP 过渡
- **换面**: 抬笔后姿态跟随新面法向量, SLERP过渡(20个插值点)

### 4.3 换面策略 (假定物块不动)
- 第i面画完 → 抬笔(沿当前面法向量退后 5cm) → 空中直线运动到第i+1面起点上方(同样退后5cm) → 沿第i+1面法向量方向下移5cm到触点
- 需要预先检查每个面的可达性, 侧面可能需要调整基座位置

### 4.4 力控策略
- 基于 Gen3 Lite **内置关节扭矩传感器** (不是电机电流, 精度更好)
- 从 `/base_feedback` 读取 `actuators[].torque` (Nm)
- τ_external = τ_measured - τ_bias (无负载标定值)
- 简化: 只用腕部关节力矩, 因为接触力主要体现在末端
- 控制在 0.5~2N 范围内即可保证笔尖接触而不过压

### 4.5 奇异位形处理
- 绘画前用 MoveIt 预检查整条轨迹的 IK 可解性
- 若当前 IK 求解失败: 微调末端绕笔尖Z轴旋转 ±10°, 重试
- Gen3 Lite 6DOF 注意: 完全伸展姿态可能到达关节限位

## 5. 各模块间通信接口总结

```
                      Service 调用
  模块A (trajectory_server) ←─ SetBlockPose ── 实验脚本
        │
        └─── GenerateTrajectory.srv ──→ 模块B (executor_node)
                                             │
                      Topic 订阅/发布          │
  模块B ←─ /joint_state ────────────── kortex_driver
  模块B ←─ /force_correction ───────── 模块C (controller_node)
  模块B ──→ /in/cartesian_velocity ──→ kortex_driver
  模块B ──→ /in/stop ────────────────→ kortex_driver
        │
  模块C ←─ /joint_state ────────────── kortex_driver
  模块C ←─ /base_feedback ──────────── kortex_driver
  模块C ──→ /force_correction ────────→ 模块B
        │
  Action 接口
  实验脚本 ── DrawingExecution.action ──→ 模块B
```

## 6. 依赖汇总

| 依赖 | 来源 | 备注 |
|------|------|------|
| ros_kortex | `https://github.com/Kinovarobotics/ros_kortex` noetic-devel 分支 | 驱动层, 不动 |
| MoveIt | `apt install ros-noetic-moveit` | 运动规划 |
| trac_ik | `apt install ros-noetic-trac-ik-kinematics-plugin` | IK优化(可选) |
| Eigen3 | `apt install libeigen3-dev` | 线性代数 |
| nanosvg | `src/nanosvg.h` (单头文件) | SVG解析, 放trajectory_generator/include |
| yaml-cpp | `apt install libyaml-cpp-dev` | 参数文件解析 |
| rosbag | ROS 自带 | 实验数据记录 |
| matplotlib | `pip install matplotlib` | 实验可视化 |

## 7. 验证方案

1. **模块A独立测试**:
   ```bash
   rosrun trajectory_generator trajectory_server
   rosservice call /generate_trajectory "svg_file: 'test.svg' faces: [0,1,2]"
   # 在 RViz 中查看 MarkerArray 可视化生成的3D轨迹
   ```

2. **模块B独立测试 (Gazebo仿真)**:
   ```bash
   roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite
   roslaunch robot_executor start_executor.launch
   # 用简单 waypoints 测试, 不夹笔空跑
   ```

3. **模块C独立测试**:
   - 机械臂保持静止, 手持物块轻触笔尖
   - 观察 `/force_correction` 的输出是否随接触力变化

4. **集成测试**:
   - 先用简单图案(正方形边框跨顶面+1个侧面), 在纸上画出评估
   - 逐步增加复杂度: 圆 → SVG校徽 → 多面连续图案

5. **实验指标采集**:
   - 线条连续性: 计算两段轨迹衔接处偏差 (mm)
   - 换面偏移量: 第一笔期望vs实际位置
   - 压力方差: 力估计值的 σ/μ
   - 全程录制 rosbag, 离线分析

---

## 附录A: 全部可调参数清单

> 以下列出代码中所有可调数值。大部分参数在多个文件中重复出现（C++ 默认值 / YAML 文件 / launch 文件 / argparse 默认值），修改时需要**所有位置保持一致**。
>
> **推荐修改顺序**: 先改 YAML 配置文件（运行时通过 `<rosparam>` 加载）和 launch 文件中的 `<param>`，最后改 C++ 源码中的构造函数初始化值（仅在未加载 yaml 或 ROS param 时作为兜底）。

### A.1 物块位姿与尺寸

| 参数 | 默认值 | 单位 | 说明 | 位置 |
|------|--------|------|------|------|
| `--block-x` | **0.4** | m | 物块底面中心 X 坐标（基坐标系） | `run_experiment.py:229` (argparse) |
| `--block-y` | **0.0** | m | 物块底面中心 Y 坐标 | `run_experiment.py:231` (argparse) |
| `--block-z` | **0.075** | m | 物块底面中心 Z 坐标（=H/2，底面在桌面） | `run_experiment.py:233` (argparse) |
| `--block-roll` | **0.0** | rad | 物块绕 X 轴旋转 | `run_experiment.py:235` (argparse) |
| `--block-pitch` | **0.0** | rad | 物块绕 Y 轴旋转 | `run_experiment.py:236` (argparse) |
| `--block-yaw` | **0.0** | rad | 物块绕 Z 轴旋转 | `run_experiment.py:237` (argparse) |
| `--block-L` | **0.20** | m | 物块长度 (X 方向) | `run_experiment.py:238`, `trajectory_server.cpp:32`, `full_system.launch:63` |
| `--block-W` | **0.20** | m | 物块宽度 (Y 方向) | `run_experiment.py:240`, `trajectory_server.cpp:33`, `full_system.launch:64` |
| `--block-H` | **0.15** | m | 物块高度 (Z 方向) | `run_experiment.py:242`, `trajectory_server.cpp:34`, `full_system.launch:65` |

> **如何改物块位姿**: 通过命令行参数传入，例如:
> ```bash
> rosrun system_integrator run_experiment.py \
>   --block-x 0.5 --block-y 0.1 --block-z 0.10 \
>   --block-L 0.30 --block-W 0.20 --block-H 0.10
> ```
> 物块坐标系原点 = **底面中心**。`block-z` 应设为 H/2（使底面在 z=0 桌面）。

### A.2 绘画参数

| 参数 | 默认值 | 单位 | 说明 | 位置 |
|------|--------|------|------|------|
| `speed` | **0.03** | m/s | 绘画行进速度 | `drawing_params.yaml:4`, `drawing_executor.cpp:22` |
| `free_speed` | **0.15** | m/s | 自由空间移动速度 | `drawing_params.yaml:5`, `drawing_executor.cpp:23`, `motion_primitives.cpp:16` |
| `lift_height` | **0.05** | m | 抬笔高度（沿面法向退后距离） | `drawing_params.yaml:6`, `drawing_executor.cpp:24`, `trajectory_server.cpp:31`, `full_system.launch:62` |
| `pen_tip_offset` | **0.12** | m | 法兰端面到笔尖距离 | `drawing_params.yaml:7`, `drawing_executor.cpp:25`, `trajectory_server.cpp:30`, `full_system.launch:61` |
| `eef_step` | **0.001** | m | Cartesian path 离散步长 | `drawing_params.yaml:8`, `drawing_executor.cpp:26` |

> 改法: 直接修改 `robot_executor/config/drawing_params.yaml`，然后重启 executor_node。

### A.3 运动规划参数

| 参数 | 默认值 | 单位 | 说明 | 位置 |
|------|--------|------|------|------|
| `planning_time` | **2.0** | s | MoveIt 规划时间上限 | `drawing_params.yaml:11`, `motion_primitives.cpp:17` |
| `planner_id` | **"RRTConnectkConfigDefault"** | — | MoveIt 规划器 ID | `drawing_params.yaml:12`, `motion_primitives.cpp:18` |
| `max_velocity_scale` | **0.5** | — | MoveIt 最大速度缩放比例 | `motion_primitives.cpp:43` |
| `max_acceleration_scale` | **0.5** | — | MoveIt 最大加速度缩放比例 | `motion_primitives.cpp:44` |

### A.4 机械臂 home 位姿

| 参数 | 默认值 | 单位 | 说明 | 位置 |
|------|--------|------|------|------|
| `home_joints[0]` | **0.0** | rad | 关节1 (底座旋转) | `drawing_params.yaml:16`, `motion_primitives.cpp:35` |
| `home_joints[1]` | **0.26** | rad | 关节2 | 同上 |
| `home_joints[2]` | **3.14** | rad | 关节3 | 同上 |
| `home_joints[3]` | **-1.96** | rad | 关节4 | 同上 |
| `home_joints[4]` | **0.0** | rad | 关节5 | 同上 |
| `home_joints[5]` | **1.57** | rad | 关节6 (腕部旋转) | 同上 |

> **如何找合适的 home 位姿**: 在 RViz 中手动拖动机器人到期望的收起姿态，然后用 `rostopic echo /my_gen3_lite/joint_states/position` 读取当前关节角，填入 `drawing_params.yaml:16`。

### A.5 力控参数

| 参数 | 默认值 | 单位 | 说明 | 位置 |
|------|--------|------|------|------|
| `desired_force` | **1.0** | N | 期望笔尖法向压力 | `controller_params.yaml:5`, `controller_node.cpp:40`, `impedance_controller.cpp:11` |
| `Kp` | **0.0003** | m/N | 力控比例增益 | `controller_params.yaml:6`, `controller_node.cpp:41`, `impedance_controller.cpp:8` |
| `Kd` | **0.00005** | m.s/N | 力控微分阻尼 | `controller_params.yaml:7`, `controller_node.cpp:42`, `impedance_controller.cpp:9` |
| `M` | **0.1** | kg | 二阶阻抗虚拟质量 | `impedance_controller.cpp:10` |
| `dz_max` | **0.0005** | m | 单步最大 Z 修正量 (0.5mm) | `controller_params.yaml:8`, `controller_node.cpp:43`, `impedance_controller.cpp:12` |
| `control_rate` | **100** | Hz | 力控循环频率 | `controller_params.yaml:9`, `controller_node.cpp:44` |
| `contact_threshold` | **0.3** | N | 判定接触的最小力 | `controller_params.yaml:10`, `controller_node.cpp:45` |
| `use_second_order` | **false** | — | true=二阶阻抗, false=一阶比例 | `impedance_controller.cpp:16` |

> 改法: 直接修改 `force_controller/config/controller_params.yaml`，重启 controller_node。

### A.6 DH 参数 (Gen3 Lite 运动学)

采用**标准 DH 约定** (Standard DH)，与 `jacobian.py` 一致。参数顺序: `[alpha, a, d, theta_offset]`。

变换公式: `T = Rot_z(θ + offset) * Trans_z(d) * Trans_x(a) * Rot_x(α)`

| 关节 | alpha (rad) | a (m) | d (m) | theta_offset (rad) | 位置 |
|------|-------------|-------|-------|-------------------|------|
| Joint 1 | **0** | **0.0** | **0.2433** | **0** | `current_estimator.cpp:16` |
| Joint 2 | **π/2** | **0.0** | **0.010** | **π/2** | `current_estimator.cpp:17` |
| Joint 3 | **π** | **0.280** | **0.0** | **π/2** | `current_estimator.cpp:18` |
| Joint 4 | **π/2** | **0.0** | **0.245** | **π/2** | `current_estimator.cpp:19` |
| Joint 5 | **π/2** | **0.0** | **0.057** | **0** | `current_estimator.cpp:20` |
| Joint 6 | **-π/2** | **0.0** | **0.235** | **-π/2** | `current_estimator.cpp:21` |

> **来源**: `jacobian.py` (项目根目录)，基于 Gen3 Lite 实测 DH 参数。
>
> 同样的 DH 参数也用于 `motion_primitives.cpp:186-193` 的正运动学计算 (`getCurrentPose()`).

### A.7 面坐标系定义 (5 个表面)

面局部坐标系在 `surface_projector.cpp:54-58` 中硬编码，原点 = 物块中心（底面中心偏移至该面中心），各轴经 `T_block_base` 变换到基坐标系：

| 面 ID | 面名 | origin (块坐标系) | u 轴 | v 轴 | normal |
|-------|------|-------------------|------|------|--------|
| 0 | 顶面 | (0, 0, **H/2**) | (1, 0, 0) | (0, 1, 0) | (0, 0, 1) |
| 1 | 前面 | (0, **W/2**, 0) | (1, 0, 0) | (0, 0, 1) | (0, 1, 0) |
| 2 | 右面 | (**L/2**, 0, 0) | (0, 1, 0) | (0, 0, 1) | (1, 0, 0) |
| 3 | 后面 | (0, **-W/2**, 0) | (-1, 0, 0) | (0, 0, 1) | (0, -1, 0) |
| 4 | 左面 | (**-L/2**, 0, 0) | (0, -1, 0) | (0, 0, 1) | (-1, 0, 0) |

> origin 中的 H/2, W/2, L/2 由 `setBlockSize()` 设置的动态值计算。方向向量 (u, v, normal) 是硬编码常量，一般不需要修改。

### A.8 其它常量及 magic numbers

| 参数 | 默认值 | 单位 | 说明 | 位置 |
|------|--------|------|------|------|
| 状态机控制频率 | **0.01s** (100Hz) | s | State machine timer period | `drawing_executor.cpp:47` |
| 下笔前接近距离 | **0.01** (1cm) | m | approach_pose 的抬升偏移 | `drawing_executor.cpp:210` |
| 到达目标判定阈值 | **0.001** (1mm) | m | "already there" 判定 | `drawing_executor.cpp:255` |
| 接近后暂停 | **0.1** | s | 下笔前暂停 | `drawing_executor.cpp:213` |
| 落笔后暂停 | **0.2** | s | 落笔后暂停 | `drawing_executor.cpp:217` |
| 力标定前等待 | **1.0** | s | 力传感器标定前稳定等待 | `run_experiment.py:114` |
| Action 超时 | **300.0** (5min) | s | 绘画执行 action 超时 | `run_experiment.py:138` |
| Service 等待超时 | **10.0** | s | 各 ServiceProxy 等待时间 | `run_experiment.py:48-60` |
| Bezier 采样点数 | **20** | — | 三次贝塞尔曲线离散段数 | `svg_reader.h:72` |
| 圆形近似点数 | **72** | — | 正多边形近似圆的边数 | `svg_reader.h:52` |
| 方向向量零范数阈值 | **1e-10** | — | 笔朝向叉积退化解判断 | `pen_orientation.cpp:15`, `surface_projector.cpp:193` |
| 闭合折线判定阈值 | **1e-6** | — | 首尾 hypot < eps 判定闭合 | `svg_reader.cpp:92` |
| RViz 线宽 | **0.002** (2mm) | m | Marker LINE_STRIP 宽度 | `trajectory_server.cpp:146` |
| RViz 姿态箭头间隔 | **每20点** | — | 方向箭头显示密度 | `trajectory_server.cpp:170` |
| Twist 参考坐标系 | **0** (base frame) | — | 速度指令参考系 | `motion_primitives.cpp:200` |
| Twist 持续时间 | **0** (连续) | — | 0=连续发送直到新指令 | `motion_primitives.cpp:207` |

### A.9 如何统一修改参数

推荐按场景分三层修改，避免漏改:

| 优先级 | 修改位置 | 适用场景 |
|--------|---------|---------|
| **第1优先** | YAML 配置文件 (`*_params.yaml`) | 绘画参数、力控参数、home 位姿 — 运行时通过 `<rosparam>` 加载 |
| **第2优先** | 命令行参数 (argparse) | 物块位姿/尺寸 — 每次实验不同，通过 `--block-x` 等传入 |
| **第3优先** | launch 文件 `<param>` | 笔偏移等不变参数 — 在 `full_system.launch` 中启动时设置 |
| **兜底值** | C++ 构造函数初始化 | 仅在未加载 ROS param 时使用，一般不用改 |

> **关键**: `run_experiment.py` 的 argparse 默认值与 `trajectory_server.cpp` 的 ROS param 默认值、`full_system.launch` 的 `<param>` 值是**三份独立的默认值**。当某个值在 argparse 中未指定时，Python 脚本使用 argparse 默认值；当 ROS param 未设置时，C++ 节点使用 `nh_.param("name", val, default)` 中的 default。修改参数时需要确保三处一致，或只信赖其中一处（推荐以 YAML + launch param 为准）。
