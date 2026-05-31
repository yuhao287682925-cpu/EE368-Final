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

## Slide 4: 顶面绘制与底层关节刚性伺服 (Top-Down Drawing & Joint Velocity Control)
- **主要文件**: `tools/auto_contact_draw.py`
- **核心痛点**: 机械臂自带的笛卡尔控制器 (Cartesian Twist) 会在接近障碍时因为内部的安全限位或姿态奇异导致不明原因的减速和卡阻，形成假阳性“卡死”。并且力控受到运动中的动摩擦力干扰，极易正反馈“飞天”。
- **创新解决方案**: 彻底抛弃力控，拥抱**直接关节速度控制**与**运动学主动悬挂**。通过雅可比矩阵伪逆 ($J^{\dagger}$)，将笛卡尔空间速度完全映射为底层的六关节角速度下发。一旦机械臂末端失速，意味着 100% 发生了真正的物理碰撞！
- **核心代码片段**:
```python
# 伪逆解算目标关节速度并进行底层硬限幅
J_pinv = np.linalg.pinv(J, rcond=1e-3)
q_dot = J_pinv.dot(V_cart)

# 绕过驱动器直接下发关节转速
for j in range(6):
    speed.joint_identifier = j
    speed.value = np.clip(q_dot[j], -0.5, 0.5)
```

## Slide 4.5: 运动学啄木鸟避障与一维软着陆 (Kinematic Woodpecker & Soft-Landing)
- **核心思想**: 寻面时，利用感受到的接触力作为阻抗成比例放慢速度（蜻蜓点水）；绘制时，基于刚性伺服下的“失速”判定，遇微小坑洼瞬间拔高 3.5mm，如同主动悬挂般锯齿状贴合起伏表面。
- **核心代码片段**:
```python
# 一维阻抗软着陆
speed_factor = max(0.0, (10.0 - current_net_fz) / 8.0)
down_speed = -0.005 * speed_factor

# 啄木鸟：物理受阻 (失速) 触发极速抬笔泄压
if actual_speed < 0.25 * expected_speed:
    stuck_cnt += 1
    
if stuck_cnt > 8: 
    z_offset_relief += 0.0035 # 瞬时抬升 3.5mm 越过障碍
```

## Slide 5: 侧边绘制 - 姿态变换与逆向重映射 (Side-Wall Drawing)
- **主要文件**: `tools/align_wrist_side.py`，`tools/side_contact_draw.py`
- **核心思想**: 控制末端姿态法向对齐 -Y 轴；并将原平面 XY 轨迹重映射至立面 XZ。
- **核心代码片段**:
```python
# 坐标系倒转与重映射 (-dv)
# Z 轴控制上下：取反原轨迹的 y 位移 (-dv)，实现“扎下后永远从下往上画”
# 这避免了笔尖由于重力和自锁效应导致的向下卡死
aligned_waypoints.append({
    'x': contact_x - du, 
    'y': contact_y, 
    'z': contact_z - dv, 
    'phase': wp['phase']
})
```

## Slide 6: 侧边绘制 - 单轴法向力控与动摩擦对抗 (Force Control & Friction)
- **主要文件**: `tools/side_contact_draw.py` 
- **核心思想**: 侧面受到重力与垂直面的综合动摩擦，通过实时读取目标面法向 (Y轴) 的受力来调节下压深度。
- **核心代码片段**:
```python
# 提取侧边绘制的法向受力 (Y轴)
raw_fy = tool_force[1]
self.current_fy = abs(raw_fy - self.fy_bias)

# 沿法向 (-Y) 的安全泄压退缩机制
if fy_filtered > 10.0 or stuck_cnt > 8:
    y_offset_relief += 0.005 * dt # 阻力过大，向 +Y 泄压
elif fy_filtered < 5.0:
    y_offset_relief -= 0.002 * dt # 恢复定深
```

## Slide 7: 轨迹对比工程与误差可视化 (Error Analysis & Visualization)
- **主要文件**: `tools/side_contact_draw_with_log.py`，`tools/analyze_error.py`
- **核心思想**: 高频记录闭环实际轨迹，利用统计学自适应识别降维平面，展示物理执行与理论规划的闭合误差。
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

## Slide 8: 结论与未来展望 (Conclusion & Future Work)
- 成功在极有限的硬件条件（无末端独立力传感器）下，利用纯运动学和本体关节电流，实现了鲁棒性极强的自动化寻面和绘制。
- 建立了一套涵盖“轨迹生成 -> 力矩转换 -> 自适应安全控制 -> 数据记录 -> 自动误差可视化”的完整工程闭环。
- **误差分析结论**: 可视化中出现的闭环开口问题，本质是由于基础比例控制器 (P-Control) 在对抗相反方向的动摩擦力时产生的**方向性稳态滞后误差**。
- **Future Work**: 计划在未来工作中探讨引入积分控制 (Integral Control) 以消除稳态误差彻底闭合图案，并进一步研究空间 3D 合力感知以应对复杂曲面接触。
