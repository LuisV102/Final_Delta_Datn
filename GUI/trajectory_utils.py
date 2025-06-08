import numpy as np
import math
import os

# --- Constants ---
DEG_PER_PULSE = 0.225
PULSE_PER_DEG = 1.0 / DEG_PER_PULSE
t_up_down = 0.2

# Robot Constants
R_Base = 60.0
R_platform = 42.62
r_const = R_Base - R_platform
re = 150.0
rf = 350.0
THRESHOLD = 0.001
ALPHA_DEG = [0, 120, 240]

# Output directory
OUTPUT_DIR = r"D:\TAI LIEU MON HOC\Do_an_tot_nghiep\Final_Code_Delta_4"

# Color destinations (E points)
COLOR_DESTINATIONS = {
    'R': (77.66, -113.09, -365),
    'G': (-22.54, -110.8, -365),
    'Y': (24.26, 103.62, -365)
}


def calculate_q(t, q0, delta_q, tf):
    if t < 0:
        return q0
    if t >= tf:
        return q0 + delta_q

    tau = t / tf
    q_t = q0 + delta_q * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5)
    return q_t


def inverse_kinematic(X_ee, Y_ee, Z_ee):
    J = [0.0, 0.0, 0.0]

    for i in range(3):
        alpha = math.radians(ALPHA_DEG[i])
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)

        A = -2.0 * re * (-r_const + X_ee * cos_alpha + Y_ee * sin_alpha)
        B = -2.0 * re * Z_ee
        C = (X_ee ** 2 + Y_ee ** 2 + Z_ee ** 2 + r_const ** 2 + re ** 2 - rf ** 2
             - 2 * r_const * (X_ee * cos_alpha + Y_ee * sin_alpha))

        denominator_val = A * A + B * B
        if denominator_val < 1e-12:
            return None

        denominator = math.sqrt(denominator_val)
        acos_arg = -C / denominator

        if abs(acos_arg) > 1.0 + 1e-6:
            return None

        theta1 = math.atan2(B, A) + math.acos(acos_arg)
        theta2 = math.atan2(B, A) - math.acos(acos_arg)
        theta = theta2

        if abs(theta) < THRESHOLD:
            theta = 0.0

        J[i] = -math.degrees(theta)

    return J[0], J[1], J[2]


def update_motor_parameters(time_per_point, angle_deg_1, angle_deg_2, angle_deg_3):
    """
    time_per_point: Thời gian dành cho mỗi điểm dữ liệu (tính từ tf_segment/num_points)
    """
    MIN_DELAY = 1000  # 100μs tối thiểu

    nPulse_1 = max(1, int(abs(angle_deg_1) * PULSE_PER_DEG))
    delay_1 = max(MIN_DELAY, int(time_per_point * 1e6 / nPulse_1))

    nPulse_2 = max(1, int(abs(angle_deg_2) * PULSE_PER_DEG))
    delay_2 = max(MIN_DELAY, int(time_per_point * 1e6 / nPulse_2))

    nPulse_3 = max(1, int(abs(angle_deg_3) * PULSE_PER_DEG))
    delay_3 = max(MIN_DELAY, int(time_per_point * 1e6 / nPulse_3))

    # Tính tương tự cho motor 2, 3
    return delay_1, delay_2, delay_3


def generate_segment(start_pos, end_pos, tf_segment, num_points):
    """Tạo dữ liệu delay và direction cho một đoạn chuyển động
    Args:
        start_pos: tuple (x,y,z) - vị trí bắt đầu
        end_pos: tuple (x,y,z) - vị trí kết thúc
        tf_segment: float - thời gian thực hiện đoạn (giây)
        num_points: int - số điểm dữ liệu
    Returns:
        list - dữ liệu cho 3 motor [(delay, direction), ...]
    """
    # Thiết lập thông số an toàn
    MIN_DELAY = 1000  # 100μs (tối thiểu)
    MAX_DELAY = 100000  # 100ms (tối đa)

    q0_x, q0_y, q0_z = start_pos
    qf_x, qf_y, qf_z = end_pos
    time_per_point = tf_segment / num_points  # Thời gian cho mỗi điểm

    segment_data = [[], [], []]  # Dữ liệu 3 motor
    prev_angles = inverse_kinematic(q0_x, q0_y, q0_z) or (0, 0, 0)

    for t in np.linspace(0, tf_segment, num_points):
        # Tính vị trí hiện tại
        x = calculate_q(t, q0_x, qf_x - q0_x, tf_segment)
        y = calculate_q(t, q0_y, qf_y - q0_y, tf_segment)
        z = calculate_q(t, q0_z, qf_z - q0_z, tf_segment)

        # Tính góc các motor
        curr_angles = inverse_kinematic(x, y, z)
        if not curr_angles:
            # Xử lý lỗi: thêm giá trị mặc định
            for motor in range(3):
                segment_data[motor].append((MAX_DELAY, 1))
            continue

        # Tính hướng quay và delay
        directions = []
        delays = []
        for i in range(3):
            # Hướng quay (1: thuận, 0: nghịch)
            direction = 1 if (curr_angles[i] >= prev_angles[i]) else 0
            directions.append(direction)

            # Tính số xung cần thiết
            angle_diff = abs(curr_angles[i] - prev_angles[i])
            n_pulse = max(1, int(angle_diff * PULSE_PER_DEG))

            # Tính delay (μs) và giới hạn trong khoảng an toàn
            delay = int(time_per_point * 1e6 / (n_pulse*2))
            delay = max(MIN_DELAY, min(delay, MAX_DELAY))
            delays.append(delay)

        # Lưu dữ liệu
        for motor in range(3):
            segment_data[motor].append((delays[motor], directions[motor]))

        # Cập nhật góc cho điểm tiếp theo
        prev_angles = curr_angles

    return segment_data
def trajectory_7point(point_A, point_C, tf_AC, color, num_points_per_segment=50):
    """
    Tạo quỹ đạo hoàn chỉnh từ A → B → C → D → E → F → G
    Args:
        point_A: tuple (x,y,z) - Vị trí hiện tại (home)
        point_C: tuple (x,y,z) - Vị trí vật thể cần gắp (z = -395)
        tf_AC: float - Thời gian thực hiện đoạn A → C (giây)
        color: str - Màu sắc vật thể ('R', 'G', hoặc 'Y')
        num_points_per_segment: int - Số điểm mỗi đoạn
    """
    # Xác định các điểm trung gian
    x_c, y_c, z_c = point_C

    # Điểm B: Phía trên vật thể (cùng x,y với C, z = -365)
    point_B = (x_c, y_c, -365)

    # Điểm D: Giống điểm B (phía trên vật thể sau khi gắp)
    point_D = point_B

    # Điểm E: Phụ thuộc vào màu sắc
    point_E = COLOR_DESTINATIONS.get(color.upper(), (0, 0, -365))

    # Điểm F: Cùng x,y với E, z = -395
    point_F = (point_E[0], point_E[1], -395)

    # Điểm G: Cùng x,y với E, z = -365
    point_G = (point_E[0], point_E[1], -365)

    # Danh sách các điểm
    points = [point_A, point_B, point_C, point_D, point_E, point_F, point_G]

    # Thời gian mặc định cho các đoạn (có thể điều chỉnh)
    time_AB = tf_AC - t_up_down  # thời gian đến trên vật thể
    time_BC = t_up_down  # thời gian ở vật thể
    time_CD = t_up_down  # Thời gian nhấc lên
    time_DE = tf_AC  # thời gian di chuyển tới điểm thả
    time_EF = t_up_down  # thời gian thả vật
    time_FG = t_up_down   # thời gian trên thả vật

    times = [time_AB, time_BC, time_CD, time_DE, time_EF, time_FG]

    # Tạo quỹ đạo
    all_delays = [[], [], []]  # Tổng hợp delay cho 3 motor
    magnet_states = []
    for i in range(len(points) - 1):
        start_pos = points[i]
        end_pos = points[i + 1]
        tf_segment = times[i]

        # Bật nam châm ở đoạn C→D (sau khi đến vật thể)
        if i == 2:  # C→D
            magnet_states.extend([1] * num_points_per_segment)  # Bật nam châm
        # Tắt nam châm ở đoạn E→F (trước khi thả vật thể)
        elif i == 4:  # E→F
            magnet_states.extend([0] * num_points_per_segment)  # Tắt nam châm
        else:
            magnet_states.extend([-1] * num_points_per_segment)  # Không thay đổi

        segment_delays = generate_segment(start_pos, end_pos, tf_segment, num_points_per_segment)

        for motor in range(3):
            all_delays[motor].extend(segment_delays[motor])

    # Lưu các file
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    motor_names = ["dlay_spd1_data.h", "dlay_spd2_data.h", "dlay_spd3_data.h"]
    array_names = ["DELAY_RUN_SPD_1", "DELAY_RUN_SPD_2", "DELAY_RUN_SPD_3"]

    for motor in range(3):
        save_delay_file(motor_names[motor], all_delays[motor], array_names[motor])
    # Lưu cả dữ liệu nam châm
    save_delay_file("magnet_data.h", magnet_states, "MAGNET_STATE")
    # print(f"Trajectory data saved to {OUTPUT_DIR}")
    # print(f"Trajectory points: A → B → C → D → E → F → G")
    # print(f"Color destination: {color} at {point_E}")
    # print(f"Time for A→C: {tf_AC}s | Total time: {sum(times):.2f}s")


def trajectory_2point(start_pos, end_pos, tf, num_points=50):
    """
    Generate trajectory data between two points with specified time
    Args:
        start_pos: tuple (x, y, z) - starting position
        end_pos: tuple (x, y, z) - ending position
        tf: float - total time for the movement (seconds)
        num_points: int - number of points in the trajectory
    Returns:
        Saves delay files for 3 motors (same format as trajectory_7point)
    """
    # Thiết lập thông số giống trajectory_7point
    MIN_DELAY = 100  # 100μs tối thiểu
    MAX_DELAY = 100000  # 100ms tối đa

    q0_x, q0_y, q0_z = start_pos
    qf_x, qf_y, qf_z = end_pos
    time_per_point = tf / num_points  # Tính thời gian mỗi điểm

    delays = [[], [], []]  # Dữ liệu 3 motor
    prev_angles = inverse_kinematic(q0_x, q0_y, q0_z) or (0, 0, 0)

    for t in np.linspace(0, tf, num_points):
        # Tính vị trí hiện tại (giống trajectory_7point)
        x = calculate_q(t, q0_x, qf_x - q0_x, tf)
        y = calculate_q(t, q0_y, qf_y - q0_y, tf)
        z = calculate_q(t, q0_z, qf_z - q0_z, tf)

        # Tính góc các motor
        curr_angles = inverse_kinematic(x, y, z)
        if not curr_angles:
            # Xử lý lỗi giống trajectory_7point
            for motor in range(3):
                delays[motor].append(MAX_DELAY)
            continue

        # Tính hướng quay và delay (giống trajectory_7point)
        for i in range(3):
            # Hướng quay
            direction = 1 if (curr_angles[i] >= prev_angles[i]) else 0

            # Tính số xung
            angle_diff = abs(curr_angles[i] - prev_angles[i])
            n_pulse = max(1, int(angle_diff * PULSE_PER_DEG))

            # Tính delay (μs) có giới hạn
            delay = int(time_per_point * 1e6 / n_pulse)
            delay = max(MIN_DELAY, min(delay, MAX_DELAY))

            delays[i].append(delay)  # Lưu delay thay vì (delay, direction)

        prev_angles = curr_angles

    # Lưu file cùng định dạng với trajectory_7point
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    motor_names = ["dlay_spd1_data.h", "dlay_spd2_data.h", "dlay_spd3_data.h"]
    array_names = ["DELAY_RUN_SPD_1", "DELAY_RUN_SPD_2", "DELAY_RUN_SPD_3"]

    for motor in range(3):
        # Chuyển delay thành tuple (delay, direction) nếu cần
        delay_data = [(d, 1) for d in delays[motor]]  # Giả sử direction=1
        save_delay_file(motor_names[motor], delay_data, array_names[motor])

def save_delay_file(filename, data, array_name):
    filepath = os.path.join(OUTPUT_DIR, filename)
    with open(filepath, "w", encoding='utf-8') as f:
        f.write(f"#ifndef {array_name}_H\n")
        f.write(f"#define {array_name}_H\n\n")
        f.write(f"#define NUM_POINTS_{array_name} {len(data)}\n\n")

        # Kiểm tra kiểu dữ liệu
        if isinstance(data[0], tuple):  # Dữ liệu motor
            f.write(f"const struct MotorData {array_name}_DATA[NUM_POINTS_{array_name}] = {{\n")
            for item in data:
                delay, direction = item
                f.write(f"  {{{delay}, {direction}}},\n")
        else:  # Dữ liệu nam châm
            f.write(f"const int {array_name}_DATA[NUM_POINTS_{array_name}] = {{\n")
            for val in data:
                f.write(f"  {val},\n")

        f.write("};\n")
        f.write(f"\n#endif // {array_name}_H\n")

# Example usage
if __name__ == "__main__":
    # Điểm hiện tại (home)
    point_A = (0.0, 0.0, -307.38)

    # Điểm vật thể cần gắp
    point_C = (0, 0, -392.5)

    # Tổng thời gian thực hiện (giây)
    tf = 0.5

    # Tạo quỹ đạo
    # trajectory_7point(point_A, point_C, tf, color)
    trajectory_2point(point_A, point_C, tf)