import numpy as np

def fit_plane(points):
    """
    使用 SVD 对点云拟合平面。
    输入: points: (N,3) 的 numpy 数组
    返回: normal (3,), centroid (3,), residual_rms
    """
    pts = np.asarray(points)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must be (N,3) array")

    centroid = pts.mean(axis=0)
    cov = np.dot((pts - centroid).T, (pts - centroid)) / pts.shape[0]
    U, S, Vt = np.linalg.svd(cov)
    # 最小奇异值对应的向量为法向
    normal = U[:, -1]
    # 确保法向朝向 z>=0 的半球，便于可视化和一致性
    if normal[2] < 0:
        normal = -normal

    # 计算残差 RMS
    diffs = pts - centroid
    dists = np.abs(np.dot(diffs, normal))
    residual_rms = np.sqrt(np.mean(dists**2))

    return normal, centroid, residual_rms

def project_point_to_plane(p, normal, centroid):
    """将点 p 投影到以 centroid 为点、normal 为法线的平面上"""
    p = np.asarray(p)
    n = np.asarray(normal)
    v = p - centroid
    d = np.dot(v, n)
    return p - d * n

def make_plane_frame(normal, ref_dir=None):
    """
    根据法线构造局部平面坐标系基向量 (x_axis, y_axis, normal)
    ref_dir: 可选参考方向，用于确定 x_axis 的朝向（默认使用世界 X 投影）
    返回 3x3 矩阵，列向量为基向量
    """
    n = np.asarray(normal) / np.linalg.norm(normal)
    if ref_dir is None:
        ref = np.array([1.0, 0.0, 0.0])
    else:
        ref = np.asarray(ref_dir)

    # x' 为 ref 在平面上的投影
    x_proj = ref - np.dot(ref, n) * n
    norm_x = np.linalg.norm(x_proj)
    if norm_x < 1e-6:
        # 若 ref 与法线近共线，使用 Y 方向作为备选
        ref = np.array([0.0, 1.0, 0.0])
        x_proj = ref - np.dot(ref, n) * n
        norm_x = np.linalg.norm(x_proj)
    x_axis = x_proj / norm_x
    y_axis = np.cross(n, x_axis)
    return np.column_stack((x_axis, y_axis, n))
