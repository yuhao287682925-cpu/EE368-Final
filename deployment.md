# Kinova Gen3 Lite 三维物块连续绘图系统 — 部署指南

---

## 0. 三种运行模式

本系统支持三种运行模式，按硬件依赖从低到高排列：

| 模式 | 需要硬件 | 需要 Gazebo | 模块A | 模块B | 模块C | 适用场景 |
|------|---------|------------|-------|-------|-------|---------|
| **离线模式** | 无 | 否 | ✅ 轨迹生成 | ❌ | ❌ | 开发 SVG 图案、验证投影算法、RViz 可视化 |
| **Gazebo 仿真** | 无 | 是 | ✅ | ✅ MoveIt+速度控制 | ⚠️ 仿真力矩(逻辑验证) | 验证运动规划、状态机、换面逻辑 |
| **实机部署** | 机械臂 | 否 | ✅ | ✅ | ✅ 真机扭矩传感器 | 真正在物块上绘画 |

> 应用代码（5个包）全部通过 ROS topic/service/action 与 robot 通信。kortex_driver 负责把指令翻译成硬件协议——无论背后是真实机械臂还是 Gazebo 仿真，对我们透明。



### 前置条件

#### 硬件
- **实机模式**: Kinova Gen3 Lite 机械臂 (6 DOF) + gen3_lite_2f 手爪 + 网线连接
- **仿真模式**: 无硬件需求
- 物块 (长方体, 尺寸已知) + 固定夹具 + 画笔 + 法兰夹具 (实机模式需)

#### 软件
- Ubuntu 20.04 LTS (ROS Noetic 唯一支持的系统)
- ROS Noetic 完整安装 (`ros-noetic-desktop-full`)
- catkin 编译工具

---

## 1. 环境准备

### 1.1 安装 ROS Noetic

```bash
# 添加 ROS 源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

# 安装完整桌面版
sudo apt install ros-noetic-desktop-full

# 安装依赖
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### 1.2 安装系统依赖

```bash
sudo apt install -y \
    ros-noetic-moveit \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    libeigen3-dev \
    libyaml-cpp-dev \
    python3-matplotlib \
    python3-numpy
```

### 1.3 配置 ROS 环境

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. 创建工作空间并编译

### 2.1 创建工作空间

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2.2 克隆官方 ros_kortex 驱动

```bash
git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
```

### 2.3 安装 ros_kortex 的 Python 依赖

```bash
sudo apt install python3 python3-pip
pip3 install --user conan==1.59.0
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
```

### 2.4 放置我们的应用包

将以下 5 个包复制到 `~/catkin_ws/src/` 下：

```
block_drawing_msgs/
trajectory_generator/
robot_executor/
force_controller/
system_integrator/
```

**从 Windows 传到 Linux (SCP)**:
```bash
# 在 Windows Git Bash 中执行 (替换 IP 为你的 Linux 机器 IP)
scp -r catkin_ws/src/block_drawing_msgs 192.168.xxx.xxx:~/catkin_ws/src/
scp -r catkin_ws/src/trajectory_generator 192.168.xxx.xxx:~/catkin_ws/src/
scp -r catkin_ws/src/robot_executor 192.168.xxx.xxx:~/catkin_ws/src/
scp -r catkin_ws/src/force_controller 192.168.xxx.xxx:~/catkin_ws/src/
scp -r catkin_ws/src/system_integrator 192.168.xxx.xxx:~/catkin_ws/src/
```

> **注意**: 不要拷贝 `src/CMakeLists.txt`（顶层 workspace 文件，Linux 上已存在）。也不要拷贝 `src/ros_kortex/`（已在 Linux 上单独 clone）。

> 如果是从 Windows 上开发的代码, 注意检查换行符: `dos2unix` 或 `sed -i 's/\r$//'` 处理所有 `.cpp`, `.h`, `.py`, `.launch`, `.yaml` 文件。

```bash
# 在 Linux 上执行: 批量转换换行符 (如有 Windows 换行符)
find ~/catkin_ws/src -path "*/ros_kortex/*" -prune -o -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.py" -o -name "*.launch" -o -name "*.yaml" -o -name "*.xml" -o -name "*.txt" \) -exec sed -i 's/\r$//' {} \;
```

### 2.5 修复权限并编译

ros_kortex 中的 Python 脚本可能缺少执行权限，需先修复：

```bash
# 修复 Gazebo 相关脚本的可执行权限
chmod +x ~/catkin_ws/src/ros_kortex/kortex_gazebo/scripts/home_robot.py

# 编译
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

### 2.6 加载工作空间

```bash
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 3. Gazebo 仿真（无硬件模式）

仿真模式不需要任何硬件。Gazebo 提供物理引擎模拟机械臂运动，kortex_gazebo 提供 Gen3 Lite 仿真模型。

### 3.1 启动 Gazebo 仿真全系统

```bash
roslaunch system_integrator simulation.launch
```

这会依次启动：
1. Gazebo + Gen3 Lite 模型
2. kortex_driver (连接仿真)
3. MoveIt move_group
4. 模块A/B/C
5. RViz

### 3.2 分步启动 Gazebo（调试用）

```bash
# 终端1: 启动 Gazebo 仿真世界
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite dof:=6

# 终端2: 启动 driver (连接仿真)
roslaunch kortex_driver kortex_driver.launch \
    arm:=gen3_lite dof:=6 gripper:=gen3_lite_2f \
    ip_address:=192.168.1.10 start_rviz:=true

# 终端3: 启动 MoveIt
roslaunch gen3_lite_gen3_lite_2f_move_it_config move_group.launch

# 终端4: 启动我们的模块
rosrun trajectory_generator trajectory_server
rosrun robot_executor executor_node _robot_name:=my_gen3_lite
rosrun force_controller controller_node _robot_name:=my_gen3_lite
```

### 3.3 Gazebo 仿真下运行实验

```bash
# Gazebo 仿真下测试正方形 (无笔无物块，仅验证运动)
rosrun system_integrator run_experiment.py \
    --test-square --faces 0 \
    --output-dir ~/experiments/sim_test

# 多面换面测试
rosrun system_integrator run_experiment.py \
    --test-square --faces 0 1 2 \
    --output-dir ~/experiments/sim_multi
```

### 3.4 Gazebo 仿真限制

| 能力 | 仿真支持 | 说明 |
|------|---------|------|
| 模块A 轨迹生成 | ✅ | RViz 中可查看 MarkerArray |
| 模块B MoveIt 规划 | ✅ | 完整碰撞检测和路径规划 |
| 模块B 速度控制 | ✅ | 通过 /in/cartesian_velocity 驱动仿真机械臂 |
| 模块C 力控逻辑 | ⚠️ | 代码可运行，但 Gazebo 算的 joint torque 是物理仿真值，非真机扭矩传感器读数 |
| 绘画质量评估 | ❌ | 仿真中无真实笔尖接触力，力控效果无法评估 |
| 换面可达性 | ✅ | 可验证侧面是否在机械臂工作空间内 |

### 3.5 离线模式（无需 Gazebo）

仅运行模块A 轨迹生成，在 RViz 中查看结果：

```bash
# 终端1
rosrun trajectory_generator trajectory_server

# 终端2: 设置假想的物块
rosservice call /set_block_pose "
block_pose:
  position: {x: 0.4, y: 0.0, z: 0.075}
  orientation: {x: 0, y: 0, z: 0, w: 1}
L: 0.20
W: 0.20
H: 0.15
face_offset_u: [0.0, 0.0, 0.0, 0.0, 0.0]
face_offset_v: [0.0, 0.0, 0.0, 0.0, 0.0]"

# 终端3: 生成轨迹
rosservice call /generate_trajectory "
svg_file: ''
target_width_mm: 80.0
target_height_mm: 80.0
faces: [0, 1, 2]"

# 在 RViz 中查看: Add → By topic → /trajectory_markers/MarkerArray
```

---

## 4. 网络配置（实机模式）

### 4.1 设置 PC 有线网卡 IP

机械臂默认 IP 为 `192.168.1.10`, 需要将 PC 有线网卡设为同一网段:

```bash
# 图形界面: Settings → Network → Wired → IPv4 → Manual
#   Address: 192.168.1.100
#   Netmask: 255.255.255.0

# 或命令行:
sudo ip addr add 192.168.1.100/24 dev eth0
```

### 4.2 验证连接

```bash
ping 192.168.1.10
# 应有响应, 延迟 <1ms
```

---

## 5. 启动系统（实机模式）

### 5.1 一键启动 (完整系统)

```bash
roslaunch system_integrator full_system.launch ip_address:=192.168.1.10 robot_name:=my_gen3_lite
```

这会依次启动:
1. **kortex_driver** — 连接机械臂硬件, 发布 `base_feedback` (40Hz) 和 `joint_states`
2. **MoveIt move_group** — 运动规划引擎
3. **trajectory_server** (模块A) — 轨迹生成服务
4. **executor_node** (模块B) — 绘画执行器, 暴露 `/execute_drawing` action
5. **controller_node** (模块C) — 力控制器, 发布 `/force_correction`
6. **RViz** — 可视化界面

### 5.2 分步启动 (调试用)

如果需要逐步调试:

```bash
# 第1步: 启动驱动和 MoveIt (实机模式)
roslaunch kortex_driver kortex_driver.launch \
    arm:=gen3_lite dof:=6 gripper:=gen3_lite_2f \
    ip_address:=192.168.1.10 start_rviz:=true
roslaunch gen3_lite_gen3_lite_2f_move_it_config move_group.launch

# 第2步: 启动我们的模块 (move_group 已由 spawn_kortex_robot 启动)
rosrun trajectory_generator trajectory_server
rosrun robot_executor executor_node _robot_name:=my_gen3_lite
rosrun force_controller controller_node _robot_name:=my_gen3_lite
```

---

## 6. 实验流程

### 6.1 快速自检

```bash
# 确保所有 topic 正常
rostopic list | grep my_gen3_lite
# 应看到:
#   /my_gen3_lite/base_feedback
#   /my_gen3_lite/joint_states
#   /my_gen3_lite/in/cartesian_velocity
#   /my_gen3_lite/in/stop

# 检查力控和轨迹服务
rosservice list | grep -E "force_controller|generate_trajectory|set_block_pose"
# 应看到:
#   /force_controller/calibrate
#   /generate_trajectory
#   /set_block_pose
```

### 6.2 运行实验

```bash
# 基础实验: 顶面画正方形
rosrun system_integrator run_experiment.py \
    --test-square \
    --width 80 --height 80 \
    --faces 0 \
    --block-x 0.4 --block-y 0.0 --block-z 0.075 \
    --block-L 0.20 --block-W 0.20 --block-H 0.15 \
    --output-dir ~/experiments/test1

# 顶面+前面画正方形 (换面实验)
rosrun system_integrator run_experiment.py \
    --test-square \
    --width 80 --height 80 \
    --faces 0 1 \
    --block-x 0.4 --block-y 0.0 --block-z 0.075 \
    --output-dir ~/experiments/test2

# SVG 文件实验
rosrun system_integrator run_experiment.py \
    --svg ~/patterns/circle.svg \
    --width 100 --height 100 \
    --faces 0 1 2 3 4 \
    --output-dir ~/experiments/all_faces
```

### 6.3 实验参数说明

| 参数 | 含义 | 典型值 |
|------|------|--------|
| `--test-square` | 使用内置正方形测试图案 (无需 SVG 文件) | flag, 默认开启 |
| `--svg` | SVG 图案文件路径 (与 `--test-square` 互斥) | `~/patterns/star.svg` |
| `--width/--height` | 图案实际尺寸 [mm] | `80` |
| `--faces` | 要画的面 (0顶/1前/2右/3后/4左) | `0 1` |
| `--block-x/y/z` | 物块底面中心在机械臂基坐标系下的位置 [m] | `0.4, 0, 0.075` |
| `--block-L/W/H` | 物块长/宽/高 [m] | `0.20, 0.20, 0.15` |
| `--face-offsets-u` | 各面图案沿 u 轴偏移, 5个逗号分隔值 [mm] | `"0,0,0,0,0"` |
| `--face-offsets-v` | 各面图案沿 v 轴偏移, 5个逗号分隔值 [mm] | `"0,0,0,0,0"` |
| `--output-dir` | 实验数据保存目录 | `~/experiments/test1` |

#### 面内偏移量说明

5 个值对应 `顶面, 前面, 右面, 后面, 左面`。默认全 0 = 图案中心对齐面中心。

各面 u/v 轴方向：
| 面 | u 轴正向 | v 轴正向 |
|----|---------|---------|
| 顶面 (0) | 块右方 → | 块前方 ↑ |
| 前面 (1) | 块右方 → | 块上方 ↑ |
| 右面 (2) | 块前方 → | 块上方 ↑ |
| 后面 (3) | 块左方 → | 块上方 ↑ |
| 左面 (4) | 块后方 → | 块上方 ↑ |

示例：
```bash
# 顶面画在偏右上方 (u=+30mm, v=+20mm)
--faces 0 --face-offsets-u "30,0,0,0,0" --face-offsets-v "20,0,0,0,0"

# 前面画在左下角 (u=-40mm, v=-30mm)
--faces 1 --face-offsets-u "0,-40,0,0,0" --face-offsets-v "0,-30,0,0,0"

# 顶面和前面各自不同位置
--faces 0 1 \
  --face-offsets-u "20,-30,0,0,0" \
  --face-offsets-v "10,-20,0,0,0"
```

---

## 7. 分模块独立测试

### 7.1 模块A独立测试 (轨迹生成)

不需要机械臂, 离线运行:

```bash
# 终端1: 启动轨迹服务器
rosrun trajectory_generator trajectory_server

# 终端2: 设置物块位姿 (含面内偏移)
rosservice call /set_block_pose "
block_pose:
  position: {x: 0.4, y: 0.0, z: 0.075}
  orientation: {x: 0, y: 0, z: 0, w: 1}
L: 0.20
W: 0.20
H: 0.15
face_offset_u: [0.0, 0.0, 0.0, 0.0, 0.0]
face_offset_v: [0.0, 0.0, 0.0, 0.0, 0.0]"

# 终端3: 生成轨迹
rosservice call /generate_trajectory "
svg_file: ''
target_width_mm: 80.0
target_height_mm: 80.0
faces: [0, 1]"

# 在 RViz 中查看: 添加 Marker topic → /trajectory_markers
```

### 7.2 模块B独立测试 (Gazebo 空跑)

**仿真模式**（推荐先跑）:
```bash
# 先启动 Gazebo 仿真 (见第3节)
roslaunch system_integrator simulation.launch

# 模块A + 模块B 会自动启动
# 然后运行实验脚本空跑:
rosrun system_integrator run_experiment.py \
    --test-square --faces 0 \
    --output-dir ~/experiments/module_b_test
```

**实机模式**:
```bash
roslaunch robot_executor start_executor.launch robot_name:=my_gen3_lite
# 用 action 发送简单绘画指令 (需模块A先运行)
# 在另一个终端中通过 Python 发送 goal, 机械臂空跑(不夹笔)
```

在 RViz 中观察机械臂是否按预期轨迹运动。

### 7.3 模块C独立测试 (力控标定)

```bash
# 启动力控制器
rosrun force_controller controller_node _robot_name:=my_gen3_lite

# 标定零位 (手臂自由悬挂, 无接触)
rosservice call /force_controller/calibrate

# 激活力控
rostopic pub /force_control/active std_msgs/Bool "data: true"

# 手持物块轻触笔尖, 观察 /force_correction 输出
rostopic echo /force_correction
# 应看到 z 分量随接触力变化
```

---

## 8. 调试与故障处理

### 8.1 机械臂无响应

```bash
# 检查驱动状态
rostopic echo /my_gen3_lite/base_feedback -n 1
# 如果无输出, 检查物理连接和 IP 配置

# 清除故障
rostopic pub /my_gen3_lite/in/clear_faults std_msgs/Empty
```

### 8.2 MoveIt 规划失败

```bash
# 检查 MoveIt 是否在运行
rosnode list | grep move_group

# 查看规划失败原因
rostopic echo /move_group/status

# 尝试增大规划时间
rosparam set /move_group/planning_time 5.0
```

### 8.3 编译错误

```bash
# 清理后重新编译
cd ~/catkin_ws
rm -rf build devel
catkin_make 2>&1 | tee build.log
```

常见错误及解决:

| 错误信息 | 原因 | 解决 |
|---------|------|------|
| `fatal error: block_drawing_msgs/SurfaceTrajectory.h: No such file or directory` | catkin 消息生成与目标编译之间缺少依赖声明 | 确保各包 CMakeLists.txt 中每个 target 都有 `add_dependencies(target ${catkin_EXPORTED_TARGETS})` |
| `'sensor_msgs' does not name a type` | 头文件缺少 `#include <sensor_msgs/JointState.h>` | 在报错的 `.h` 文件中添加对应 include |
| `no match for call to ... lambda ... candidate expects 1 argument, 0 provided` | actionlib 回调签名误用, `registerGoalCallback` 要求 `void()` | 改为 `[this]() { auto goal = action_server_.acceptNewGoal(); ... }` |
| `No rule to make target kortex_driver/TwistCommand.h` | ros_kortex 未克隆或未编译 | 确认 `~/catkin_ws/src/ros_kortex/` 存在 |
| `cannot find -lforce_controller` 或其他链接错误 | 包间依赖顺序混乱 | `catkin_make --force-cmake` |

### 8.4 路径规划超限/碰撞警告

```bash
# 在 RViz 中手动拖动末端到目标位置, 检查是否可达
# 如果侧面不可达, 调整物块相对机械臂的摆放位置
# 修改 block-x/y/z 参数重新实验
```

### 8.5 力控不生效

```bash
# 检查 calibration 是否完成
rosservice call /force_controller/calibrate

# 检查 force_control 是否激活
rostopic pub /force_control/active std_msgs/Bool "data: true"

# 检查 joint_states 中 effort 是否有值
rostopic echo /my_gen3_lite/joint_states -n 1 | grep effort
```

---

### 8.6 Gazebo 仿真问题

| 错误 | 原因 | 解决 |
|------|------|------|
| `Cannot locate node of type [home_robot.py] in package [kortex_gazebo]` | Python 脚本缺少执行权限 | `chmod +x ~/catkin_ws/src/ros_kortex/kortex_gazebo/scripts/home_robot.py` + 重新 `catkin_make` |
| `new node registered with same name` (robot_state_publisher) | `spawn_kortex_robot.launch` 与 `move_group.launch` 各自启动了一个 robot_state_publisher | 这是 ros_kortex 已知问题，ROS 会自动关闭重复节点，不影响运行 |
| `The node could not connect to the arm` / `not connected !!!` | Gazebo 仿真驱动没有正确启动。可能是 `sim:=true` 参数未加载 | 1. 确认通过 `full_system.launch use_gazebo:=true` 启动；2. 如果仍然崩溃，直接用 ros_kortex 自带的 `roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite dof:=6 gripper:=gen3_lite_2f` 测试 |
| `[move_group.launch] is neither a launch file in package [kinova_gen3_lite_moveit_config]` | 包名错误，Gen3 Lite + 2F 手爪的 MoveIt 包叫 `gen3_lite_gen3_lite_2f_move_it_config` | 使用正确包名: `roslaunch gen3_lite_gen3_lite_2f_move_it_config move_group.launch` |
| `Could not identify parent group for end-effector 'end_effector'` | MoveIt SRDF 配置小问题 | 警告，不影响运行 |

---

## 9. 实验数据收集与分析

### 9.1 rosbag 录制 (自动)

`run_experiment.py` 会自动保存 rosbag 到 `--output-dir/experiment.bag`。

手动录制:
```bash
rosbag record -O experiment_manual.bag \
    /my_gen3_lite/joint_states \
    /my_gen3_lite/base_feedback \
    /force_correction \
    /execute_drawing/feedback
```

### 9.2 实验评估

```bash
rosrun system_integrator evaluate_experiment.py ~/experiments/test1/experiment.bag
```

输出指标:
- **线条连续性** (mean gap / max gap): 轨迹段连接处的空间偏差 [mm]
- **力的方差系数** (CV): 笔压的一致性 (越低越好)
- **数据采集统计**: EE 位姿态样本数、力读数样本数、关节状态样本数

### 9.3 结果可视化

```bash
rosrun system_integrator plot_results.py ~/experiments/test1/experiment.bag -o ~/experiments/test1/plots
```

生成图表:
- `force_profile.png` — Z方向力修正量随时间变化
- `joint_torques.png` — 各关节力矩曲线
- `progress.png` — 绘画进度随时间变化
- `summary.png` — 四合一汇总看板

---

## 10. 安全注意事项

1. **首次运行必须空跑**: 不夹笔、不接触物块, 只看运动轨迹是否正确
2. **急停开关**: 确保 Kinova 示教器上的急停按钮触手可及
3. **速度限制**: 首次测试用低速 (修改 `drawing_params.yaml` 中的 `speed: 0.01`)
4. **力限幅**: 力控的 `dz_max: 0.0005` (0.5mm/步) 确保不会突然下压
5. **运行前检查**:
   - 机械臂工作范围内无人员/障碍物
   - 物块固定牢固, 不会因笔压移动
   - 笔与法兰夹具拧紧, 不会松脱
   - 电缆不会与运动部件干涉
6. **异常处理**: 出现异常声音或震动, 立即按下急停按钮, 或执行:
   ```bash
   rostopic pub /my_gen3_lite/in/stop std_msgs/Empty
   ```

---

## 11. 参数调优指南

### 11.1 绘画质量调参

| 参数文件 | 参数 | 默认值 | 说明 | 调大效果 | 调小效果 |
|---------|------|--------|------|---------|---------|
| `drawing_params.yaml` | `speed` | 0.03 m/s | 绘画速度 | 更快但精度低 | 更平滑更精确 |
| `drawing_params.yaml` | `lift_height` | 0.05 m | 抬笔高度 | 更安全 | 换面更快 |
| `controller_params.yaml` | `desired_force` | 1.0 N | 期望笔压 | 线条更深 | 线条更浅 |
| `controller_params.yaml` | `Kp` | 0.0003 m/N | 力控刚度 | 响应更快 | 更稳定 |
| `controller_params.yaml` | `dz_max` | 0.0005 m | 单步最大修正 | 适应更大误差 | 更平滑 |

### 11.2 物块位姿标定

机械臂基座中心为原点, 需要测量物块底面中心在基坐标系下的位置:

```
基座在地面, 机械臂根部中心 = (0, 0, 0)
物块: 桌面上的长方体
block_x = 物块中心到臂根部的前后距离
block_y = 物块中心到臂根部的左右距离
block_z = 物块半高 (底面在桌面上, 中心在高度 H/2 处)
```

---

## 12. 快速启动检查清单 (每次实验前)

- [ ] PC 网线连接机械臂底座
- [ ] PC 有线网卡 IP 设为 `192.168.1.100`
- [ ] `ping 192.168.1.10` 成功
- [ ] 机械臂工作范围清空
- [ ] 急停按钮未按下
- [ ] 笔与夹具紧固
- [ ] 物块紧固在工作台上
- [ ] 启动 `roslaunch system_integrator full_system.launch ip_address:=192.168.1.10 robot_name:=my_gen3_lite`
- [ ] RViz 中看到机械臂模型与实际姿态一致
- [ ] 先用 `--test-square --faces 0` 空跑测试
- [ ] 确认轨迹正确后再夹笔画在物块上
- [ ] 实验完成后停止 rosbag, 关闭机械臂驱动
