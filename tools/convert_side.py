import re

with open('tools/side_contact_draw_with_log.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace class name
content = content.replace('AutoContactDrawer', 'SideContactDrawer')
content = content.replace("rospy.init_node('auto_contact_draw'", "rospy.init_node('side_contact_draw'")

# Replace fz with fy
content = content.replace('fz_bias', 'fy_bias')
content = content.replace('current_fz', 'current_fy')
content = content.replace('raw_fz', 'raw_fy')
content = content.replace('force_fz_pub', 'force_fy_pub')
content = content.replace('estimated_fz', 'estimated_fy')
content = content.replace('Fz:', 'Fy:')
content = content.replace('z_offset', 'y_offset')

# tool_force[2] -> tool_force[1]
content = content.replace('tool_force[2]', 'tool_force[1]')

# Coordinate mappings for side drawing:
# 2D X -> 3D X
# 2D Y -> 3D Z (inverted)
# Approach axis is Y
# So in execute_and_draw:
# target_z is derived from wp['y']
# target_x is derived from wp['x']
# The fixed y depth is derived from contact_y
# Let's replace the trajectory alignment part

align_src = """        u_ref_x = raw_waypoints[first_draw_idx]['x']
        u_ref_y = raw_waypoints[first_draw_idx]['y']
        u_ref_z = raw_waypoints[first_draw_idx]['z_nominal']
        
        rospy.loginfo("🔄 正在基于实际物理接触点在线重生成轨迹...")
        aligned_waypoints = []
        for wp in raw_waypoints:
            aligned_wp = {
                'x': contact_pose.position.x + (wp['x'] - u_ref_x),
                'y': contact_pose.position.y + (wp['y'] - u_ref_y),
                'z_nominal': contact_pose.position.z + (wp['z_nominal'] - u_ref_z),
                'nx': wp['nx'],
                'ny': wp['ny'],
                'nz': wp['nz'],
                'phase': wp['phase'],
                'stroke_id': wp['stroke_id']
            }
            aligned_waypoints.append(aligned_wp)"""

align_dst = """        u_ref = raw_waypoints[first_draw_idx]['x']
        v_ref = raw_waypoints[first_draw_idx]['y']
        
        rospy.loginfo("🔄 正在基于实际物理接触点在线重生成轨迹...")
        aligned_waypoints = []
        for wp in raw_waypoints:
            du = wp['x'] - u_ref
            dv = wp['y'] - v_ref
            aligned_wp = {
                'x': contact_pose.position.x - du,
                'y_nominal': contact_pose.position.y,
                'z': contact_pose.position.z - dv,
                'nx': wp['nx'] if 'nx' in wp else 0.0,
                'ny': wp['ny'] if 'ny' in wp else 0.0,
                'nz': wp['nz'] if 'nz' in wp else 0.0,
                'phase': wp['phase'],
                'stroke_id': wp['stroke_id']
            }
            aligned_waypoints.append(aligned_wp)"""
content = content.replace(align_src, align_dst)

# Replace Z logic with Y logic in auto touchdown
content = content.replace('linear_z = -0.010', 'linear_y = -0.010')
content = content.replace('linear_z = 0.010', 'linear_y = 0.010')
content = content.replace('linear_z = -0.003', 'linear_y = -0.003')

# In execute_and_draw loop, we need to swap z/y
loop_src = """            target_x = wp['x']
            target_y = wp['y']
            
            # 动态刚度：空中极速，落笔放缓
            k_pos = 1.5 if phase == 'draw' else 3.5
            k_pos_z = 2.0  # 核心改动：Z轴使用距离/高度刚度控制
            
            # === 阶段 1：以距离和高度为主控的平滑移动 ===
            while not rospy.is_shutdown():
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                
                # 高度主控：计算绝对目标高度
                if phase not in ['draw', 'touch_down']:
                    target_z = wp['z_nominal']
                else:
                    target_z = wp['z_nominal'] + self.y_offset
                    
                dz = target_z - self.current_z
                
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.0015: 
                    break
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                
                v_x = k_pos * dx
                v_y = k_pos * dy
                if 0 < abs(v_x) < 0.008 and dist_to_target > 0.0015: v_x = math.copysign(0.008, v_x)
                if 0 < abs(v_y) < 0.008 and dist_to_target > 0.0015: v_y = math.copysign(0.008, v_y)
                
                cmd.twist.linear_x = np.clip(v_x, -0.04, 0.04)
                cmd.twist.linear_y = np.clip(v_y, -0.04, 0.04)
                
                if phase not in ['draw', 'touch_down']:
                    # 抬笔移动阶段：用位置误差主控 Z
                    dz = wp['z_nominal'] - self.current_z
                    cmd.twist.linear_z = np.clip(k_pos_z * dz, -0.025, 0.025)
                    if dz > 0.002:
                        cmd.twist.linear_x = 0.0
                        cmd.twist.linear_y = 0.0
                else:
                    # 绘制阶段：锁定 Z 高度，防止摩擦力和位置控制器抛起Z导致抬升
                    cmd.twist.linear_z = 0.0"""

loop_dst = """            target_x = wp['x']
            target_z = wp['z']
            
            # 动态刚度：空中极速，落笔放缓
            k_pos = 1.5 if phase == 'draw' else 3.5
            k_pos_y = 2.0  # 核心改动：Y轴使用距离/高度刚度控制
            
            # === 阶段 1：以距离和高度为主控的平滑移动 ===
            while not rospy.is_shutdown():
                dx = target_x - self.current_x
                dz = target_z - self.current_z
                
                # 法向(Y)主控：计算绝对目标高度
                if phase not in ['draw', 'touch_down']:
                    target_y = wp['y_nominal']
                else:
                    target_y = wp['y_nominal'] + self.y_offset
                    
                dy = target_y - self.current_y
                
                dist_to_target = math.hypot(dx, dz)
                if dist_to_target < 0.0015: 
                    break
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                
                v_x = k_pos * dx
                v_z = k_pos * dz
                if 0 < abs(v_x) < 0.008 and dist_to_target > 0.0015: v_x = math.copysign(0.008, v_x)
                if 0 < abs(v_z) < 0.008 and dist_to_target > 0.0015: v_z = math.copysign(0.008, v_z)
                
                cmd.twist.linear_x = np.clip(v_x, -0.04, 0.04)
                cmd.twist.linear_z = np.clip(v_z, -0.04, 0.04)
                
                if phase not in ['draw', 'touch_down']:
                    # 抬笔移动阶段：用位置误差主控 Y
                    dy = wp['y_nominal'] - self.current_y
                    cmd.twist.linear_y = np.clip(k_pos_y * dy, -0.025, 0.025)
                    if dy > 0.002:
                        cmd.twist.linear_x = 0.0
                        cmd.twist.linear_z = 0.0
                else:
                    # 绘制阶段：锁定 Y 法向，防止摩擦力抛起导致悬空
                    cmd.twist.linear_y = 0.0"""
content = content.replace(loop_src, loop_dst)

# Update log target names
content = content.replace("actual_executed_trajectory_position", "side_actual_executed_trajectory")
content = content.replace("theo_mapped_trajectory_position", "side_theo_mapped_trajectory")

# Replace target_y with target_z in theo_log
log_src = """                # 记录高频日志
                if phase == 'draw':
                    actual_log.append({'x': self.current_x, 'y': self.current_y, 'z': self.current_z})
                    theo_log.append({'x': target_x, 'y': target_y, 'z': target_z})"""
log_dst = """                # 记录高频日志
                if phase == 'draw':
                    actual_log.append({'x': self.current_x, 'y': self.current_y, 'z': self.current_z})
                    theo_log.append({'x': target_x, 'y': target_y, 'z': target_z})"""
# Wait, target_x, target_y, target_z are correct, but target_y is y_nominal. It's fine.

# Replace Z exit logic with Y exit logic
content = content.replace('for speed in [0.005, 0.015, 0.030]:\n            lift_cmd.twist.linear_z = speed', 'for speed in [0.005, 0.015, 0.030]:\n            lift_cmd.twist.linear_y = speed')
content = content.replace('高度 Z:', '深度 Y:')

# Replace auto_contact_draw with side_contact_draw in prints
content = content.replace('auto_contact_draw.py', 'side_contact_draw_with_log.py')

with open('tools/side_contact_draw_with_log.py', 'w', encoding='utf-8') as f:
    f.write(content)
