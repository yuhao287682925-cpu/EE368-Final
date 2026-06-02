侧面绘制平面拟合演示

说明：本目录提供离线演示脚本，用于在不修改原始 `tools/` 代码的前提下，验证对侧面轨迹进行平面拟合并将轨迹投影/映射到拟合平面的方法。

文件：
- `plane_fit.py`：平面拟合与点投影工具函数。
- `map_trajectory.py`：离线示例脚本，读取包含 `x,y,z` 的 CSV，并输出投影后的轨迹 CSV。

示例运行：
```bash
python3 side_draw/map_trajectory.py side_theo_mapped_trajectory.csv mapped_out.csv
```

说明：此脚本仅作离线验证，你可以在确认拟合结果满意后，将思路迁移回 `tools/side_contact_draw_with_log.py` 中，按最小可行修改逐步替换沿全局 Y 的深度控制为沿估计法线方向的控制，并加入在线增量拟合。
