# ContactState.msg
# 接触状态: 笔尖与物块表面的接触信息

float64 normal_force        # 法向力估计值 [N]
float64 torque_x            # 末端力矩 X [Nm]
float64 torque_y            # 末端力矩 Y [Nm]
float64 torque_z            # 末端力矩 Z [Nm]
bool    in_contact          # 是否判定为接触中
