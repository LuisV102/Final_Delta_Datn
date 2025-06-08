import numpy as np
from scipy.optimize import brentq
import matplotlib.pyplot as plt


def generate_delay_profile(q0, qf, n_pulse, max_delay_us=10000, min_delay_us=10):
    """
    Tạo profile delay chính xác theo công thức S-curve bậc 5 với xét đến thời gian tích lũy.

    Parameters:
        q0: Vị trí bắt đầu (độ)
        qf: Vị trí kết thúc (độ)
        n_pulse: Tổng số xung (số nguyên dương)
        max_delay_us: Delay tối đa (microseconds)
        min_delay_us: Delay tối thiểu (microseconds)

    Returns:
        Mảng delay cho từng xung (microseconds)
        Mảng thời gian tích lũy tương ứng (s)
        Mảng vị trí tương ứng (độ)
    """
    if n_pulse <= 0:
        raise ValueError("n_pulse phải là số nguyên dương")

    delta_q = qf - q0
    delays = np.zeros(n_pulse)
    time_accumulated = np.zeros(n_pulse)
    positions = np.zeros(n_pulse)

    for i in range(n_pulse):
        # Tính tỉ lệ thời gian tích lũy (0→1)
        if i == 0:
            t_ratio = 0
        else:
            t_ratio = time_accumulated[i - 1] / np.sum(delays[:i] * 1e-6) if np.sum(delays[:i]) > 0 else 0

        # Hàm S-curve bậc 5 dựa trên thời gian tích lũy
        s = 10 * t_ratio ** 3 - 15 * t_ratio ** 4 + 6 * t_ratio ** 5
        pos = q0 + delta_q * s
        positions[i] = pos

        # Phương trình chính xác theo công thức gốc
        def equation(delay_us):
            d = delay_us * 1e-6  # μs → s
            term1 = q0 / n_pulse
            term2 = 10 * delta_q * (d ** 3) * (n_pulse ** 2)
            term3 = -15 * delta_q * (d ** 4) * (n_pulse ** 3)
            term4 = 6 * delta_q * (d ** 5) * (n_pulse ** 4)  # Đã sửa thành 6*30=180
            return term1 + term2 + term3 + term4 - 0.225

        try:
            # Giải phương trình với khoảng delay hợp lý
            delays[i] = brentq(equation, min_delay_us, max_delay_us, rtol=1e-6)
        except ValueError:
            # Fallback nếu không giải được
            delays[i] = max_delay_us if i == 0 else delays[i - 1]

        # Cập nhật thời gian tích lũy
        if i == 0:
            time_accumulated[i] = delays[i] * 1e-6
        else:
            time_accumulated[i] = time_accumulated[i - 1] + delays[i] * 1e-6

    return delays, time_accumulated, positions


if __name__ == "__main__":
    q_start = 0.0
    q_end = 30.0
    n_steps = 120  # Ví dụ: 120 xung

    delays, time_accumulated, positions = generate_delay_profile(q_start, q_end, n_steps)

    # In thông tin kiểm tra
    print(f"Tổng thời gian: {time_accumulated[-1]:.3f}s")
    print(f"Delay trung bình: {np.mean(delays):.1f}μs")
    print(f"Vị trí cuối: {positions[-1]:.2f}°")

    # Vẽ đồ thị kiểm tra
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(delays, 'b-o', markersize=3)
    plt.title("Delay Profile (µs)")
    plt.xlabel("Step")
    plt.ylabel("Delay (µs)")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(time_accumulated, positions, 'r-o', markersize=3)
    plt.title("Position vs Time")
    plt.xlabel("Thời gian tích lũy (s)")
    plt.ylabel("Vị trí (độ)")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(time_accumulated, delays, 'g-o', markersize=3)
    plt.title("Delay vs Time")
    plt.xlabel("Thời gian tích lũy (s)")
    plt.ylabel("Delay (µs)")
    plt.grid(True)

    plt.tight_layout()
    plt.show()