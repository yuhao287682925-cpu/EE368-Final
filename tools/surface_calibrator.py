import numpy as np
import json
import os

def calculate_plane(p1, p2, p3):
    # 将输入转换为numpy数组
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    
    # 计算平面上的两个向量
    v1 = p2 - p1
    v2 = p3 - p1
    
    # 叉乘求法向量
    normal = np.cross(v1, v2)
    
    # 法向量归一化
    norm = np.linalg.norm(normal)
    if norm == 0:
        raise ValueError("三个点共线，无法构成平面！请重新选取三个散开的点。")
    normal = normal / norm
    
    # 启发式规则：通常物块的绘制面朝向外侧/上方，如果法向量朝下(朝向桌子内部)，则翻转它
    if normal[2] < -0.1:  
        normal = -normal
        
    # 计算平面中心（简单取3点几何中心作为该面的原点基准）
    center = (p1 + p2 + p3) / 3.0
    
    return normal, center

def main():
    print("=======================================")
    print("   🧱 物块绘制面 物理标定工具 (Member C)   ")
    print("=======================================")
    print("目的：将物理世界中真实方块的位姿转化为数学方程，供队友调用。")
    print("操作要求：请用手柄将笔尖分别触碰目标绘制面上的【三个不在同一直线的点】。")
    print("提示：这三个点最好能构成一个尽量大的三角形，这样计算出的平面误差最小。\n")
    
    points = []
    for i in range(1, 4):
        print(f"--- 📍 第 {i} 个点 ---")
        print("请移动机械臂笔尖触碰目标表面...")
        # 提示：成员C可以看自带网站的坐标，或者刚才写的记录脚本
        x = float(input("输入读取到的 X 坐标 (米): "))
        y = float(input("输入读取到的 Y 坐标 (米): "))
        z = float(input("输入读取到的 Z 坐标 (米): "))
        points.append([x, y, z])
        print("")
        
    try:
        normal, center = calculate_plane(points[0], points[1], points[2])
        
        surface_name = input("请输入该平面的名称 (例如: top_surface, side_a): ")
        if not surface_name.strip():
            surface_name = "surface_1"
            
        surface_data = {
            "surface_name": surface_name,
            "normal_vector": normal.tolist(),
            "center_point": center.tolist(),
            "raw_points": points
        }
        
        output_file = os.path.join(os.path.dirname(__file__), f'{surface_name}_config.json')
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(surface_data, f, indent=4, ensure_ascii=False)
            
        print("\n✅ 标定成功！")
        print(f"平面的 法向量(Normal) 为 : {np.round(normal, 4)}")
        print(f"平面的 中心点(Center) 为 : {np.round(center, 4)}")
        print(f"🗂️ 配置文件已自动导出至 : {output_file}")
        print("\n下一步：你可以直接把这个 JSON 文件发给成员A和B，他们的代码只需读取此文件，就不需要再痛苦地手动猜测和硬编码(Hardcode)平面的三维姿态了！")
        
    except Exception as e:
        print(f"\n❌ 标定失败: {e}")

if __name__ == "__main__":
    main()
