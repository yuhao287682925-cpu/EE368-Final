import math

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class Link:
    """
    定义一个连杆类，用于表示机械臂的一个连杆并计算其相关变换与雅可比矩阵
    """
    def __init__(self, dh_params):
        self.dh_params_ = dh_params #根据dh_params进行初始化，包括连杆的长度、偏置等信息。

    def transformation_matrix(self, theta):
        """
        计算相邻坐标系之间的齐次变换矩阵
        """
        alpha = self.dh_params_[0]
        a = self.dh_params_[1]
        d = self.dh_params_[2]
        # 加上固有的关节角度偏置
        theta = theta+self.dh_params_[3]
        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        # 变换矩阵的经典构造公式
        trans = np.array([[ct, -st, 0, a],
                          [st*ca, ct * ca, - sa, -sa * d],
                          [st*sa, ct * sa,   ca,  ca * d],
                          [0, 0, 0, 1]])
        return trans

    @staticmethod
    def basic_jacobian(trans, ee_pos):
        """
        计算该连杆关节对应的基础(几何)雅可比矩阵的列向量
        """
        # 提取当前关节坐标系原点的位置 p_{i-1}
        pos = np.array(
            [trans[0, 3], trans[1, 3], trans[2, 3]])

        # 提取当前关节坐标系 z 轴的方向向量 z_{i-1}
        z_axis = np.array(
            [trans[0, 2], trans[1, 2], trans[2, 2]])

        # 旋转关节的雅可比列向量构造
        basic_jacobian = np.hstack(
            (np.cross(z_axis, ee_pos - pos), z_axis))
        return basic_jacobian


class NLinkArm:
    """
    N 自由度串联机械臂
    """
    def __init__(self, dh_params_list) -> None:
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

    def transformation_matrix(self, thetas):
        """
        计算末端执行器相对于基座的总体齐次变换矩阵
        """
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            # 转移矩阵连乘
            trans = np.dot(
                trans, self.link_list[i].transformation_matrix(thetas[i]))
        return trans

    def forward_kinematics(self, thetas):
        """
        计算正向运动学，返回末端位姿
        """
        trans = self.transformation_matrix(thetas)
        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        
        # 转换为欧拉角
        alpha, beta, gamma = self.euler_angle(thetas)
        return [x, y, z, alpha, beta, gamma]

    def euler_angle(self, thetas):
        """
        转换为欧拉角
        """
        trans = self.transformation_matrix(thetas)

        # 计算欧拉角
        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return alpha, beta, gamma

    def inverse_kinematics(self, ref_ee_pose):
        """
        求解逆向运动
        """
        thetas = [0, 0, 0, 0, 0, 0] # 初始猜测值
        for cnt in range(500): # 最大迭代次数 500
            ee_pose = self.forward_kinematics(thetas)
            # 当前位姿与目标位姿之间的误差
            diff_pose = np.array(ref_ee_pose) - ee_pose

            # 获取当前构型下的基础雅可比矩阵
            basic_jacobian_mat = self.basic_jacobian(thetas)
            alpha, beta, gamma = self.euler_angle(thetas)

            # K_zyz 将基础几何雅可比关联到解析雅可比的矩阵，是将末端角速度映射为欧拉角速度的逆过程
            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            # 根据伪逆迭代公式计算关节角度修正
            theta_dot = np.dot(
                np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha),
                np.array(diff_pose))
            # 更新关节角度，引入缩放因子1/100以稳定迭代
            thetas = thetas + theta_dot / 100.
        return thetas
    

    def basic_jacobian(self, thetas):
        """
        计算 N 自由度机械臂的基础几何雅可比矩阵。
        """
        ee_pos = self.forward_kinematics(thetas)[0:3]
        basic_jacobian_mat = []
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            # 计算第 i 个连杆在基坐标系下的变换矩阵
            trans = np.dot(
                trans, self.link_list[i].transformation_matrix(thetas[i]))
            # 依次计算并保存对应连杆的雅可比列向量
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))
        # 将列表转换为 numpy 数组并进行转置
        return np.array(basic_jacobian_mat).T

if __name__ == "__main__":
    rospy.init_node("jacobian_test")
    # 初始化三个 Publisher，用于发布笛卡尔空间的末端位姿、速度、输出力
    tool_pose_pub = rospy.Publisher("/tool_pose_cartesian",Point,queue_size=1)
    tool_velocity_pub = rospy.Publisher("/tool_velocity_cartesian",Point,queue_size=1)
    tool_force_pub = rospy.Publisher("/tool_force_cartesian",Point,queue_size=1)

    # Kinova Gen3 Lite 机械臂的 Modified DH 参数表
    dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                               [math.pi/2, 0, 10/1000, 0+math.pi/2],
                               [math.pi, 280/1000, 0, 0+math.pi/2],
                               [math.pi/2, 0, 245/1000, 0+math.pi/2],
                               [math.pi/2, 0, 57/1000, 0],
                               [-math.pi/2, 0, 245/1000, 0-math.pi/2]])
    # 实例化机械臂
    gen3_lite = NLinkArm(dh_params_list)

    while not rospy.is_shutdown():
        # 阻塞等待关节状态话题发布消息
        feedback = rospy.wait_for_message("/my_gen3_lite/joint_states", JointState)
        # 获取当前六个关节的位置、速度和力矩
        thetas = feedback.position[0:6]
        velocities = feedback.velocity[0:6]
        torques = feedback.effort[0:6]

        #求解末端正运动学位置
        tool_pose = gen3_lite.forward_kinematics(thetas)
        
        #求解雅可比矩阵并映射末端笛卡尔速度: V = J * q_dot
        J = gen3_lite.basic_jacobian(thetas)
        tool_velocity = J.dot(velocities)
        
        # 3. 求解末端笛卡尔静力学
        tool_force = np.linalg.pinv(J.T).dot(torques)

        # 封装为 Point 消息并发布位姿数据
        tool_pose_msg = Point()
        tool_pose_msg.x = tool_pose[0]
        tool_pose_msg.y = tool_pose[1]
        tool_pose_msg.z = tool_pose[2]

        # 封装并发布速度数据
        tool_velocity_msg = Point()
        tool_velocity_msg.x = tool_velocity[0]
        tool_velocity_msg.y = tool_velocity[1]
        tool_velocity_msg.z = tool_velocity[2]

        # 封装并发布输出力数据
        tool_force_msg = Point()
        tool_force_msg.x = tool_force[0]
        tool_force_msg.y = tool_force[1]
        tool_force_msg.z = tool_force[2]

        tool_pose_pub.publish(tool_pose_msg)
        tool_velocity_pub.publish(tool_velocity_msg)
        tool_force_pub.publish(tool_force_msg)

        # 打印调试信息到终端
        print(f"joint position: {thetas}")
        print(f"joint velocity: {velocities}")
        print(f"joint torque: {torques}")

        print(f"tool position: {tool_pose}")
        print(f"tool velocity: {tool_velocity}")
        print(f"tool torque: {tool_force}")