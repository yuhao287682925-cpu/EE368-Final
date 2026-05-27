# trajectory — 立方体表面绘图轨迹生成模块

机械臂在已知位姿的立方体**每个面上各画一个图形**的轨迹生成器。
纯 Python，无 ROS / EGO-Planner 依赖，输出标准 CSV 给执行端使用。

---

## 功能

输入：物块顶面中心坐标 + 边长 + 每个面要画什么图形。
输出：一条 CSV 轨迹，每个轨迹点都带位置、表面法向量、笔尖姿态、抬笔/落笔标志、阶段标签。

支持的图形（每个面挑一个）：
`circle` / `triangle` / `square` / `star` / `sun` / `spiral`

支持的面（六个都建模）：
`top` / `front` / `right` / `back` / `left` / `bottom`

每个面独立绘制：approach → touch_down → draw → lift。面与面之间通过抬笔运动衔接，不会画出多余连线。

---

## 快速开始

需要 Python 3.8+。可视化额外需要 `matplotlib`。

```bash
# 在六个面上各画一个默认图形
python3 cube_circle_trajectory.py --faces all

# 渲染 3D 预览图
python3 plot_trajectory_3d.py \
  --input  generated/all_faces_patterns.csv \
  --output generated/all_faces_patterns_3d.png
```

默认的"每面一图"组合：

| 面 | 图形 |
|---|---|
| top | circle |
| front | triangle |
| right | star |
| back | spiral |
| left | sun |
| bottom | square |

想自己指定，用 `--face-patterns`：

```bash
python3 cube_circle_trajectory.py \
  --faces all \
  --face-patterns top:circle,front:triangle,right:star,left:sun,back:spiral,bottom:square \
  --output generated/all_faces_patterns.csv
```

只画一个面：

```bash
python3 cube_circle_trajectory.py --face top --pattern triangle
```

---

## 命令行参数

`cube_circle_trajectory.py` 的关键参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--top-center-x/y/z` | `0.280 / -0.032 / 0.119` | 物块**顶面中心**在机械臂基坐标系下的坐标，单位 m |
| `--side-length` | `0.060` | 立方体边长，单位 m |
| `--face` | `top` | 单面模式选哪个面 |
| `--faces` | — | 多面模式，逗号分隔或 `all`，覆盖 `--face` |
| `--pattern` | `circle` | 默认图形 |
| `--face-patterns` | `""` | 每个面独立指定图形，例如 `top:circle,front:triangle` |
| `--radius` | `0.020` | 图形半径，单位 m |
| `--samples` | `96` | 圆/螺旋的采样点数（最少 12） |
| `--point-spacing` | `0.002` | 重采样后相邻点的最大间距，单位 m |
| `--ray-count` | `8` | sun 图形的光芒数 |
| `--normal-offset` | `0.0` | 沿表面法向的额外偏移（凸出/凹陷） |
| `--approach-height` | `0.020` | 抬笔高度，沿法向，单位 m |
| `--roll/pitch/yaw-deg` | `22.688 / 175.755 / 83.736` | 末端姿态，度 |
| `--output` | 自动 | 输出 CSV 路径 |

> 如果手头的坐标是物块**几何中心**而不是顶面中心，把 `--top-center-z` 加上 `side-length / 2`。

---

## 输出格式

CSV 每行一个轨迹点：

| 字段 | 含义 |
|---|---|
| `index` | 全局序号 |
| `stroke_id` | 笔画编号（同一面内不同笔画之间会抬笔） |
| `phase` | `approach` / `touch_down` / `draw` / `lift` |
| `face` | 所在面 |
| `pattern` | 该面绘制的图形 |
| `x_m, y_m, z_m` | 机械臂基坐标系下的目标位置，单位 m |
| `nx, ny, nz` | 该点所在表面的单位法向量 |
| `roll_deg, pitch_deg, yaw_deg` | 末端姿态，度 |
| `roll_rad, pitch_rad, yaw_rad` | 末端姿态，弧度，可直接喂给 IK |
| `pen_down` | `1` 落笔，`0` 抬笔 |

执行端按 `pen_down` 切换墨水阀 / 夹爪即可。

---

## 3D 可视化

`plot_trajectory_3d.py` 把 CSV 渲染成 PNG：

```bash
python3 plot_trajectory_3d.py \
  --input  generated/all_faces_patterns.csv \
  --output generated/all_faces_patterns_3d.png
```

PNG 内容：

- 半透明立方体线框 + 六个面标签
- 每个笔画用对应面颜色画实线（落笔段）
- 灰色虚线表示 approach / lift 抬笔运动
- 每个笔画起点用圆点标出

无显示环境也能跑（默认用 `matplotlib` 的 `Agg` 后端）。

---

## 实现说明

模块覆盖以下几何工作：

- **物块表面建模**：`build_axis_aligned_cube_surfaces()` 给六个面各定义局部坐标系（`origin` / `u_axis` / `v_axis` / `normal`）和 uv 边界。
- **图形离散化**：`make_pattern()` 把每种图形转成一组 2D 折线。多笔画图形（例如 sun）天然支持，笔画间自动抬笔。
- **2D → 3D 投影**：`PlaneSurface.point_from_uv()` 把面内 uv 坐标 + 法向偏移映射到机械臂基坐标系。
- **等间距重采样**：`resample_polyline()` 按 `--point-spacing` 把折线打散成均匀点序列，便于执行端线性插值。
- **轨迹分阶段输出**：`generate_pattern_on_face()` 为每个笔画生成 approach / touch_down / draw / lift 四阶段，并附带表面法向量。
- **CSV 接口**：`write_waypoints_csv()` 写出标准 CSV，字段稳定，便于下游 IK / 控制节点直接读取。

---

## 目录结构

```
trajectory/
├── cube_circle_trajectory.py   # 主生成器：6 种图形 × 6 个面 → CSV
├── plot_trajectory_3d.py       # CSV → 3D PNG 可视化
├── generated/                  # 输出目录（运行时生成，已被 .gitignore 排除）
├── .gitignore
└── README.md
```
