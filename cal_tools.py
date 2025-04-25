import math
import numpy as np


def calculate_circle_intersection(x1, y1, r1, x2, y2, r2):
    dx = x2 - x1
    dy = y2 - y1
    d_sq = dx**2 + dy**2
    if d_sq == 0:
        return []  # 同心圆，无解
    d = math.sqrt(d_sq)
    a = (r1**2 - r2**2 + d_sq) / (2 * d)
    h_sq = r1**2 - a**2
    if h_sq < 0:
        return []  # 无交点
    h = math.sqrt(h_sq)
    x_mid = x1 + a * dx / d
    y_mid = y1 + a * dy / d
    # 计算垂直方向的偏移
    dx_perp = -dy * h / d
    dy_perp = dx * h / d
    point1 = (x_mid + dx_perp, y_mid + dy_perp)
    point2 = (x_mid - dx_perp, y_mid - dy_perp)
    return [point1, point2]

def calculate_line_circle_intersection(line_x0, line_y0, dx_dir, dy_dir, circle_x, circle_y, radius):
    x_diff = line_x0 - circle_x
    y_diff = line_y0 - circle_y
    a = dx_dir**2 + dy_dir**2
    if a == 0:
        return []
    b = 2 * (x_diff * dx_dir + y_diff * dy_dir)
    c = x_diff**2 + y_diff**2 - radius**2
    delta = b**2 - 4 * a * c
    if delta < 0:
        return []
    sqrt_delta = math.sqrt(delta)
    t1 = (-b + sqrt_delta) / (2 * a)
    t2 = (-b - sqrt_delta) / (2 * a)
    point1 = (line_x0 + t1 * dx_dir, line_y0 + t1 * dy_dir)
    point2 = (line_x0 + t2 * dx_dir, line_y0 + t2 * dy_dir)
    return [point1, point2]

def add_gaussian_noise(data, sigma):
    """
    添加高斯噪声
    :param data: 输入数据
    :param sigma: 噪声标准差
    :return: 添加噪声后的数据
    """
    noise = np.random.normal(0, sigma, size=data.shape)
    return data + noise

def calculate_distance_3D(point1, point2):
    """
    计算三维空间中两点之间的距离
    :param point1: 第一个点的坐标 (x1, y1, z1)
    :param point2: 第二个点的坐标 (x2, y2, z2)
    :return: 两点之间的距离
    """
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)

def calculate_distance_2D(point1,point2):
    """
    计算二维空间中两点之间的距离
    :param point1: 第一个点的坐标 (x1, y1)
    :param point2: 第二个点的坐标 (x2, y2)
    :return: 两点之间的距离
    """
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def point_to_line_distance(point, start, end):
    """计算点到3D直线的距离"""
    if np.all(start == end):
        return np.linalg.norm(point - start)
    
    # 将所有点转换为NumPy数组
    point = np.array(point)
    start = np.array(start)
    end = np.array(end)

    # 计算向量
    line_vec = end - start
    point_vec = point - start
    
    # 计算点到直线的距离
    line_len = np.linalg.norm(line_vec)
    line_unitvec = line_vec / line_len
    
    # 向量投影
    projection = point_vec.dot(line_unitvec)
    
    # 如果投影小于0，则点在start之前；如果大于直线长度，则点在end之后
    if projection < 0:
        return np.linalg.norm(point - start)
    if projection > line_len:
        return np.linalg.norm(point - end)
    
    # 计算从点到直线的向量
    perp_vec = point_vec - projection * line_unitvec
    
    # 返回距离
    return np.linalg.norm(perp_vec)
