# from math import *
import numpy as np
import math
R_base = 60  # bán kính đế
R_platform = 42.62  # bán kính platform
re = 150  # chiều dài cánh tay nhỏ
rf = 350  # chiều dài cánh tay lớn
PI = math.pi

def forward_kinematic(theta1_deg, theta2_deg, theta3_deg):
    r = R_base - R_platform

    # Chuyển độ sang radian
    theta = [
        math.radians(theta1_deg),
        math.radians(theta2_deg),
        math.radians(theta3_deg)
    ]

    alpha = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0]

    # Tọa độ 3 khớp động
    P = [[0.0, 0.0, 0.0] for _ in range(3)] #biến P chứa tọa độ 3 khớp động
    # P = [[x1, y1, z1],  # khớp 1
    #   [x2, y2, z2],  # khớp 2
    #   [x3, y3, z3]]
    for i in range(3): # 3 vòng lặp tương ứng 3 cánh tay
        angle = theta[i]
        alpha_i = alpha[i]
        x = (r + re * math.cos(angle)) * math.cos(alpha_i)
        y = (r + re * math.cos(angle)) * math.sin(alpha_i)
        z = -re * math.sin(angle)  # Z hướng xuống

        P[i][0] = x
        P[i][1] = y
        P[i][2] = z

    # Vector v12 = P2 - P1, v13 = P3 - P1   ///thiếu vòng lặp
    v12 = [P[1][i] - P[0][i] for i in range(3)] #v12 = [x,y,z]
    v13 = [P[2][i] - P[0][i] for i in range(3)]

    # ex = v12 / |v12|
    norm_v12 = math.sqrt(sum([v**2 for v in v12])) #v**2 là square từng phần tử trong vector
    ex = [v / norm_v12 for v in v12]       #norm: tính độ dài vector

    # a = dot(ex, v13)
    a = sum([ex[i] * v13[i] for i in range(3)])

    # ey = (v13 - a*ex) / |...|
    ey_tmp = [v13[i] - a * ex[i] for i in range(3)]
    norm_ey = math.sqrt(sum([v**2 for v in ey_tmp]))
    ey = [v / norm_ey for v in ey_tmp]

    # ez = ex x ey (cross product)
    ez = [
        ex[1] * ey[2] - ex[2] * ey[1],
        ex[2] * ey[0] - ex[0] * ey[2],
        ex[0] * ey[1] - ex[1] * ey[0]
    ]

    # b = dot(ey, v13)
    b = sum([ey[i] * v13[i] for i in range(3)])

    d = norm_v12
    x = d / 2.0
    y = ((a*a + b*b) / (2.0 * b)) - (a * x / b)
    z_sq = rf*rf - x*x - y*y

    if z_sq < 0:
        # Không tồn tại vị trí hợp lệ
        return False, None, None, None

    z = math.sqrt(z_sq)

    # Vị trí end-effector
    Px = P[0][0] + x * ex[0] + y * ey[0] - z * ez[0]
    Py = P[0][1] + x * ex[1] + y * ey[1] - z * ez[1]
    Pz = P[0][2] + x * ex[2] + y * ey[2] - z * ez[2]

    return True, Px, Py, Pz

def inverse_kinematic(X_ee, Y_ee, Z_ee):
    R_Base = 60
    R_platform = 42.62  # Khoảng cách từ tâm end-effector tới khớp cầu
    r = R_Base - R_platform  # Khoảng cách từ tâm top tới động cơ

    re = 150  # Chiều dài cánh tay trên (mm)
    rf = 350  # Chiều dài cánh tay dưới (mm)

    # Z_ee = Z_ee+35.5
    # Góc xoay động cơ
    alpha = np.radians([0, 120, 240])
    J = np.zeros(3)
    threshold = 1e-3

    for i in range(3):
        cos_alpha = np.cos(alpha[i])
        sin_alpha = np.sin(alpha[i])
        A = -2 * re * (-r + X_ee * cos_alpha + Y_ee * sin_alpha)
        B = -2 * re * Z_ee
        C = (X_ee ** 2 + Y_ee ** 2 + Z_ee ** 2 + r ** 2 + re ** 2 - rf ** 2
             - 2 * r * (X_ee * cos_alpha + Y_ee * sin_alpha))

        denominator = np.sqrt(A ** 2 + B ** 2)
        if denominator == 0 or abs(C / denominator) > 1:
            raise ValueError("Không thể tính góc: giá trị trong acos không hợp lệ.")

        theta_1 = np.arctan2(B, A) + np.arccos(-C / denominator)
        theta_2 = np.arctan2(B, A) - np.arccos(-C / denominator)

        theta = theta_2 if theta_1 > theta_2 else theta_1

        if abs(theta) < threshold:
            theta = 0

        theta_deg = np.degrees(theta)
        J[i] = -theta_deg

    return J
def trajectory_planning_2_point(t, P0, Pf, tf):
    def compute_axis(p0, pf, t):
        delta = pf - p0
        a0 = p0
        a1 = a2 = 0
        a3 = 10 * delta / tf**3
        a4 = -15 * delta / tf**4
        a5 = 6 * delta / tf**5
        return a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5

    x = compute_axis(P0[0], Pf[0], t)
    y = compute_axis(P0[1], Pf[1], t)
    z = compute_axis(P0[2], Pf[2], t)
    return x, y, z