import json
import os

def generate_aligned_box_faces():
    print("=======================================")
    print("   📦 对齐方块 - 5面位姿快速生成工具   ")
    print("=======================================")
    print("前提：方块严格平行/垂直于机器人的基坐标系 (X, Y, Z轴)。")
    print("只需要输入方块的【几何中心点】和【三维尺寸】，即可瞬间生成5个面的配置文件！\n")
    
    try:
        # 获取方块中心
        print("--- 1. 方块的几何中心坐标 (如果已知底面中心，Z轴中心 = 底面Z + 高度/2) ---")
        cx = float(input("中心点 X (米): "))
        cy = float(input("中心点 Y (米): "))
        cz = float(input("中心点 Z (米): "))
        
        # 获取方块尺寸
        print("\n--- 2. 方块的物理尺寸 ---")
        lx = float(input("沿 X 轴的长度 (米): "))
        ly = float(input("沿 Y 轴的宽度 (米): "))
        lz = float(input("沿 Z 轴的高度 (米): "))
        
        # 定义5个面 (抛弃底面)
        # 约定：法向量指向物块外部
        faces = [
            {
                "name": "top",
                "normal": [0.0, 0.0, 1.0],
                "center": [cx, cy, cz + lz/2.0]
            },
            {
                "name": "front_x_plus",
                "normal": [1.0, 0.0, 0.0],
                "center": [cx + lx/2.0, cy, cz]
            },
            {
                "name": "back_x_minus",
                "normal": [-1.0, 0.0, 0.0],
                "center": [cx - lx/2.0, cy, cz]
            },
            {
                "name": "left_y_plus",
                "normal": [0.0, 1.0, 0.0],
                "center": [cx, cy + ly/2.0, cz]
            },
            {
                "name": "right_y_minus",
                "normal": [0.0, -1.0, 0.0],
                "center": [cx, cy - ly/2.0, cz]
            }
        ]
        
        out_dir = os.path.dirname(__file__)
        
        print("\n✅ 正在生成5个面的配置文件...")
        for face in faces:
            data = {
                "surface_name": face["name"],
                "normal_vector": face["normal"],
                "center_point": face["center"],
                "box_dimensions": [lx, ly, lz],
                "is_aligned": True
            }
            
            out_file = os.path.join(out_dir, f'box_{face["name"]}_config.json')
            with open(out_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            print(f"  -> 生成完毕: {out_file}")
            
        print("\n太棒了！所有面的数学模型已建立完毕。")
        print("因为是严格对齐的，队友写状态机(ROTATE/MOVE)时，连欧拉角都可以直接写死(比如旋转90度)！")

    except ValueError:
        print("\n❌ 输入有误，请输入纯数字。")

if __name__ == "__main__":
    generate_aligned_box_faces()
