# 机械臂自动化接触式绘图系统 (Automated Contact Drawing System)
—— 顶面与侧边绘制、力觉感知及轨迹误差分析

## Slide 1: 项目概览与核心挑战 (Overview & Challenges)
- **项目目标**: 实现机械臂（Kortex Gen3 Lite）对未知平面的自动化接触式轨迹绘制。
- **核心挑战**:
  1. 无末端六维力传感器，如何实现精准的表面接触检测？
  2. 如何在未知平面（水平面 / 垂直侧面）上建立准确的轨迹映射？
  3. 接触作业中的动摩擦力、卡顿如何解决？轨迹的执行精度如何评估？
- **解决方案**: 基于雅可比矩阵的力矩-力学转换、安全泄压控制策略、理论与实际轨迹的 3D/2D 对比误差分析系统。

## Slide 2: 轨迹工程与离线生成 (Trajectory Engineering)
- **主要文件**: `trajectory/cube_circle_trajectory.py` (及各类形状脚本)
- **核心思想**: 独立于物理执行的 UV 坐标系轨迹生成，将轨迹拆分为 `hover`、`touch_down`、`draw` 等相态。
- **核心代码片段**:
```python
# 状态机分层：剥离物理执行，仅生成二维逻辑航点
for angle in np.linspace(0, 2 * math.pi, steps):
    x_m = center_x + radius_m * math.cos(angle)
    y_m = center_y + radius_m * math.sin(angle)
    # 为每一个点赋予相态，便于执行器动态调整高度
    waypoints.append({"x_m": x_m, "y_m": y_m, "phase": "draw"})
```

## Slide 3: 核心力觉感知与寻面 (Force Sensing & Auto-Touchdown)
- **主要文件**: `jacobian.py` (正运动学与雅可比计算)
- **核心思想**: 利用公式 $F = (J^T)^{+} \tau$ 将关节力矩转换为末端力估计，并进行动态零点去偏置。
- **核心代码片段**:
```python
# 在 joint_states_callback 中实时运算
thetas = msg.position[0:6]
torques = msg.effort[0:6]

# 计算基础雅可比矩阵，并通过伪逆将关节力矩转换为末端力
J = self.arm_model.basic_jacobian(thetas)
tool_force = np.linalg.pinv(J.T).dot(torques)

# 提取法向力并去除预先校准的静力偏置
raw_fz = tool_force[2]
self.current_fz = abs(raw_fz - self.fz_bias)
```

## Slide 4: 绘制准备与手腕位姿对齐 (Preparation & Wrist Alignment)
- **主要文件**: `tools/align_wrist.py`
- **核心思想**: 在执行复杂的接触式力控绘图前，首先通过 MoveIt! 将机械臂的第 6 关节（手腕）以及整体姿态调整为标准的水平下探姿势。
- **作用**: 消除初始位姿带来的奇异点风险，并确保向下探面时，$F = (J^T)^{+} \tau$ 中的 Z 轴阻力能够最纯粹地反映在特定的关节扭矩上，降低假力干扰。

## Slide 5: 顶面绘制与双重探面寻面策略 (Top-Down Drawing & Double-Probe)
- **主要文件**: `tools/auto_contact_draw_flat_traj.py`
- **核心思想（与上一版对比）**:
  - **上一版方案**：采用单次粗探 + 画图时连续动态泄压 (`z_offset_relief`)。存在问题：控制频繁触发导致目标轨迹 Z 轴漂移，且易产生“锯齿状”震荡。
  - **本版全新方案**：抛弃了画图中的动态高度微调，采用**“双重探面 + 纯位置死区绘制”**。先通过重压 (12N/15N) 克服初始形变，随后物理微抬并彻底悬空去皮，最后通过极慢速轻触 (1.2N) 精准锁定真实零点，最后基于该零点进行绝对平稳的位置跟随。
- **核心代码片段 (不再进行力控偏移，锁定平面)**:
```python
# 动态轨迹原点对齐 (彻底废弃了画图中的辅助 z_offset 漂移)
aligned_waypoints = []
for wp in raw_waypoints:
    aligned_wp = {
        'x': contact_pose.position.x + (wp['x'] - u_ref_x),
        'y': contact_pose.position.y + (wp['y'] - u_ref_y),
        # 严格基于二次高精度探面的接触点 Z 高度生成后续轨迹
        'z_nominal': contact_pose.position.z + (wp['z_nominal'] - u_ref_z)
    }
    aligned_waypoints.append(aligned_wp)
```

## Slide 6: 轨迹对比工程与误差可视化 (Error Analysis & Visualization)
- **主要文件**: `tools/analyze_error.py`
- **核心思想**: 读取主程序高频记录的实际执行轨迹与理论生成轨迹，利用统计学自适应识别降维平面，展示物理执行与理论规划的闭合误差。
- **核心代码片段**:
```python
# 通过理论目标点的方差，自动推断所在平面
std_x, std_y, std_z = np.std(theo_x), np.std(theo_y), np.std(theo_z)
stds = [('X', std_x, theo_x, act_x), ('Y', std_y, theo_y, act_y), ('Z', std_z, theo_z, act_z)]
stds.sort(key=lambda item: item[1])
plane_axes = stds[1:] # 剔除方差最小的法向，取出平面的两个主轴

# 计算平移轨迹均方根误差 (RMSE)
error = np.sqrt(np.mean((act_x - theo_x)**2 + (act_y - theo_y)**2 + (act_z - theo_z)**2))
```

## Slide 7: 结论与未来展望 (Conclusion & Future Work)
- 成功在极有限的硬件条件（无末端独立力传感器）下，利用纯运动学和本体关节电流，实现了鲁棒性极强的自动化寻面和绝对平整的定深绘制。
- 建立了一套涵盖“轨迹生成 -> 力矩转换 -> 自适应安全控制 -> 数据记录 -> 自动误差可视化”的完整工程闭环。
- **误差分析结论**: 可视化中出现的闭环开口问题，本质是由于基础比例控制器 (P-Control) 在对抗相反方向的动摩擦力时产生的**方向性稳态滞后误差**。
- **Future Work**: 计划在未来工作中探讨引入积分控制 (Integral Control) 以消除稳态误差彻底闭合图案，并进一步研究空间 3D 合力感知以应对复杂曲面接触。
