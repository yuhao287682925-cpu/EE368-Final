# SurfaceTrajectory.msg
# 单面轨迹: 包含该面所有 waypoints

int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面
geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)
float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]
