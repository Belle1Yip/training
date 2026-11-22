"""基础作业完成前两个函数，进阶作业完成第三个函数。距离 m，角度 rad。"""
import math
from robotics_demo import rotate2d


def optical_to_link(point_optical):
    """理想共原点安装：光学 X右Y下Z前 → link X前Y左Z上。"""
    # 基础 1：根据前、左、上对应的光学分量，返回三个坐标。
    optical_x, optical_y, optical_z = point_optical
    return optical_z, -optical_x, -optical_y


def aim_angles(point_reference):
    """输入固定瞄准参考系中的三维点，返回 (yaw, elevation)，均为 rad。

    参考系 X前Y左Z上，理想转轴共点。elevation 向上为正。
    如果输入有非有限值，或水平距离 <= 1e-12 m，应抛出 ValueError。
    """
    # 基础 2：使用 isfinite、hypot 和 atan2，实现上述约定。
    if not all(math.isfinite(value) for value in point_reference):
        raise ValueError("瞄准点坐标必须是有限数值")

    x, y, z = point_reference
    horizontal_distance = math.hypot(x, y)
    if horizontal_distance <= 1e-12:
        raise ValueError("目标位于正上方或正下方时，yaw 不唯一")

    yaw = math.atan2(y, x)
    elevation = math.atan2(z, horizontal_distance)
    return yaw, elevation


def small_yaw_to_base(point_small, big_yaw, small_yaw, small_offset, big_offset):
    """将小 yaw 中的三维点换到底盘中，返回 (x, y, z)。

    两根 yaw 轴均竖直，角度均为相对父坐标系的角度，向左为正。
    small_offset：小 yaw 原点在大 yaw 平台 L 中的位置，用 L 的轴表达。
    big_offset：大 yaw 原点在底盘 B 中的位置，用 B 的轴表达。
    p_L = Rz(small_yaw) p_S + small_offset
    p_B = Rz(big_yaw) p_L + big_offset
    """
    # 进阶：调用两次 rotate2d，分别得到 L、B 中的水平分量。
    # 每次旋转后加该段偏移，Z 只累加对应的高度偏移。
    x_large, y_large = rotate2d(
        point_small[0], point_small[1], small_yaw
    )
    x_large += small_offset[0]
    y_large += small_offset[1]
    z_large = point_small[2] + small_offset[2]

    x_base, y_base = rotate2d(x_large, y_large, big_yaw)
    x_base += big_offset[0]
    y_base += big_offset[1]
    z_base = z_large + big_offset[2]
    return x_base, y_base, z_base
