# ObjectDetection.py
import cv2
import numpy as np
import time

# import serial # Không import ở đây nữa, ser sẽ được truyền vào

# --- Các hằng số và biến toàn cục của module ---
# camera_matrix = np.array([
#     [1558.0, 0, 661.8934],
#     [0, 1559.5, 408.2412],
#     [0, 0, 1]
# ], dtype=np.float32)
CAMERA_MATRIX = np.array([
    [1501.8, 0, 659.018],
    [0, 1504.6, 391.2912],
    [0, 0, 1]
], dtype=np.float32)
# CAMERA_MATRIX = np.array([
# [1, 0, 1],
# [0, 1, 1],
# [0, 0, 1]
# ], dtype=np.float32)

# dist_coeffs = np.array([0.0430, 0.4256, 0.0, 0.0], dtype=np.float32)
DIST_COEFFS = np.array([0.0379, 0.4248, 0.0, 0.0], dtype=np.float32)
# DIST_COEFFS = np.array([0, 0, 0.0, 0.0], dtype=np.float32)

# CẤU HÌNH ROI (REGION OF INTEREST)
HEIGHT_ROI = [0, 480]
WIDTH_ROI = [100, 389]
ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = WIDTH_ROI[0], HEIGHT_ROI[0], WIDTH_ROI[1], HEIGHT_ROI[1]

Y_TOP = 80
Y_TRIGGER = 200
Y_BOTTOM = 400

# VÙNG MÀU CẦN NHẬN DIỆN
COLOR_RANGES = {
    'red': ([0, 70, 50], [10, 255, 255], [160, 70, 50], [180, 255, 255]),
    'green': ([40, 70, 50], [80, 255, 255]),
    'yold': ([30, 0, 60], [110, 50, 255], [140, 4, 50], [160, 6, 50]),  # 'gold' thành 'yold' để tránh trùng
}

# Biến trạng thái cho việc nhận diện
_timer_started = False
_start_time_detection = 0  # Đổi tên để tránh nhầm lẫn với module time
_end_time_detection = 0
_current_object_info = None  # Sẽ lưu (cx, cy)
_object_color_detected = None
_velocity_calculated = 0
_predicted_time_to_top = 0
_calibration_data_list = []


def reset_detection_state():
    """Reset all state variables for object detection."""
    global _timer_started, _start_time_detection, _end_time_detection, _current_object_info
    global _object_color_detected, _velocity_calculated, _predicted_time_to_top, _calibration_data_list
    _timer_started = False
    _start_time_detection = 0
    _end_time_detection = 0
    _current_object_info = None
    _object_color_detected = None
    _velocity_calculated = 0
    _predicted_time_to_top = 0
    _calibration_data_list = []
    print("Object detection state reset.")


def process_frame_for_detection(input_frame, ser_instance):
    """
    Processes a single frame for object detection, drawing, and communication.
    Args:
        input_frame: The raw frame from the camera.
        ser_instance: The serial port instance for communication with Arduino.
    Returns:
        The processed frame with detections and information drawn on it.
    """
    global _timer_started, _start_time_detection, _end_time_detection, _current_object_info
    global _object_color_detected, _velocity_calculated, _predicted_time_to_top, _calibration_data_list

    if input_frame is None:
        return None

    frame = input_frame.copy()  # Làm việc trên bản sao để không ảnh hưởng frame gốc từ camera

    # Vẽ các đường ROI và text lên frame
    cv2.rectangle(frame, (ROI_X1, ROI_Y1), (ROI_X2, ROI_Y2), (0, 0, 255), 2)
    cv2.line(frame, (ROI_X1, Y_BOTTOM), (ROI_X2, Y_BOTTOM), (255, 0, 255), 2)
    cv2.putText(frame, "Bottom line", (ROI_X1 + 10, Y_BOTTOM - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.line(frame, (ROI_X1, Y_TRIGGER), (ROI_X2, Y_TRIGGER), (0, 255, 255), 2)
    cv2.putText(frame, "Trigger line", (ROI_X1 + 10, Y_TRIGGER - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.line(frame, (ROI_X1, Y_TOP), (ROI_X2, Y_TOP), (255, 0, 0), 2)
    cv2.putText(frame, "Top line", (ROI_X1 + 10, Y_TOP - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    roi = frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
    if roi.size == 0:  # Kiểm tra ROI có hợp lệ không
        print("Warning: ROI is empty.")
        return frame  # Trả về frame gốc nếu ROI không hợp lệ

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # Đổi tên từ hsb sang hsv cho đúng chuẩn
    # combined_mask = np.zeros(roi.shape[:2], dtype=np.uint8) # Không cần thiết nếu chỉ xử lý từng màu

    for color_name, ranges in COLOR_RANGES.items():
        if color_name == 'red' or color_name == 'yold':
            lower1, upper1, lower2, upper2 = ranges
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:  # green
            lower, upper = ranges
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.medianBlur(mask, 5)

        # combined_mask = cv2.bitwise_or(combined_mask, mask) # Không cần thiết nữa
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < 150:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # --- Phần tính toán màu sắc trung bình và vẽ ---
            # (Giữ nguyên phần này, nhưng vẽ lên 'frame' thay vì 'roi' nếu tọa độ là toàn cục)
            mask_cnt = np.zeros(roi.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)
            mean_hsv_val = cv2.mean(hsv, mask=mask_cnt)  # Sử dụng hsv đã được chuyển đổi
            hue_val = int(mean_hsv_val[0])
            sat_val = int(mean_hsv_val[1])
            brightness_val = int(mean_hsv_val[2])

            # Chuyển màu trung bình HSV sang BGR để vẽ
            bgr_color_obj = cv2.cvtColor(np.uint8([[[hue_val, sat_val, brightness_val]]]), cv2.COLOR_HSV2BGR)[0][0]
            bgr_color_tuple = tuple(int(c) for c in bgr_color_obj)

            # Vẽ hình chữ nhật và tâm lên frame (tọa độ đã được điều chỉnh với ROI_X1, ROI_Y1)
            cv2.rectangle(frame, (ROI_X1 + x, ROI_Y1 + y), (ROI_X1 + x + w, ROI_Y1 + y + h), bgr_color_tuple, 2)

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx_roi = int(M["m10"] / M["m00"])  # Tọa độ tâm trong ROI
                cy_roi = int(M["m01"] / M["m00"])  # Tọa độ tâm trong ROI

                # Tọa độ tâm trên frame gốc
                cx_frame = ROI_X1 + cx_roi
                cy_frame = ROI_Y1 + cy_roi

                # Bỏ qua nếu tâm lệch (phần này có thể giữ nguyên logic)
                bbox_cx_roi = x + w // 2
                bbox_cy_roi = y + h // 2
                distance = np.sqrt((cx_roi - bbox_cx_roi) ** 2 + (cy_roi - bbox_cy_roi) ** 2)
                max_allowed_distance = 0.4 * min(w, h)
                if distance > max_allowed_distance:
                    continue

                cv2.circle(frame, (cx_frame, cy_frame), 6, (0, 0, 0), -1)

                # --- Hiệu chỉnh méo và chuyển đổi tọa độ ---
                distorted_point = np.array([[[cx_frame, cy_frame]]], dtype=np.float32)
                undistorted = cv2.undistortPoints(distorted_point, CAMERA_MATRIX, DIST_COEFFS, P=CAMERA_MATRIX)
                undistorted_coords = undistorted[0][0]

                Z_CONST = 210  # Chiều cao giả định của vật thể so với camera
                fx, fy = CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1]
                cx_intr, cy_intr = CAMERA_MATRIX[0, 2], CAMERA_MATRIX[1, 2]
                real_x = (undistorted_coords[0] - cx_intr) * Z_CONST / fx
                real_y = (undistorted_coords[1] - cy_intr) * Z_CONST / fy

                P_CA = np.array([[real_x], [real_y], [Z_CONST], [1]])
                T_0C = np.array([
                    [0, -1, 0, -160],
                    [-1, 0, 0, -30],
                    [0, 0, -1, -185],
                    [0, 0, 0, 1]
                ])
                P_OA = T_0C @ P_CA
                calib_x, calib_y, calib_z = P_OA[0, 0], P_OA[1, 0], P_OA[2, 0]
                _calibration_data_list.append(calib_x)  # Lưu trữ nếu cần

                # --- Xử lý logic timer và gửi lệnh ---
                # (cy_roi + ROI_Y1) là tọa độ y của tâm đối tượng trên frame gốc
                current_y_on_frame = cy_roi + ROI_Y1

                if current_y_on_frame >= Y_BOTTOM and not _timer_started:
                    _timer_started = True
                    _start_time_detection = time.time()
                    _current_object_info = (cx_frame, cy_frame)  # Lưu tọa độ trên frame
                    if color_name == 'yold':
                        _object_color_detected = 'Y'
                    else:
                        _object_color_detected = color_name[0].upper()

                if _timer_started and _current_object_info and current_y_on_frame <= Y_TRIGGER:
                    # Kiểm tra xem đối tượng hiện tại có phải là đối tượng đã bắt đầu timer không
                    # (có thể cần kiểm tra thêm cx_frame gần với _current_object_info[0] nếu có nhiều vật)
                    # Hiện tại, giả sử chỉ có 1 vật đang được theo dõi
                    _end_time_detection = time.time()
                    elapsed_time = _end_time_detection - _start_time_detection

                    if elapsed_time > 0.01:  # Tránh chia cho 0
                        distance_pixels = Y_BOTTOM - Y_TRIGGER
                        # pixel_to_mm = 100 / (WIDTH_ROI[1] - WIDTH_ROI[0]) # 100 là độ rộng băng tải
                        # distance_mm = distance_pixels * pixel_to_mm
                        # _velocity_calculated = distance_mm / elapsed_time

                        # Ước lượng vận tốc dựa trên pixel/s trước, sau đó có thể hiệu chỉnh
                        # Hoặc dùng một giá trị vận tốc cố định nếu băng tải chạy đều
                        # Ví dụ: giả sử vận tốc băng tải là 50 pixel/giây (cần đo thực tế)
                        # Vận tốc pixel thực tế
                        velocity_pixels_per_sec = distance_pixels / elapsed_time

                        distance_to_top_pixels = Y_TRIGGER - Y_TOP
                        _predicted_time_to_top = distance_to_top_pixels / velocity_pixels_per_sec if velocity_pixels_per_sec > 0 else 0

                        if ser_instance and ser_instance.is_open and _object_color_detected:
                            command = f"Next:{calib_x:.1f},{calib_y:.1f},{calib_z:.1f};T:{_predicted_time_to_top:.2f};C:{_object_color_detected}\n"
                            try:
                                ser_instance.write(command.encode())
                                print(f"Sent to Arduino: {command.strip()}")
                            except Exception as e:
                                print(f"Error sending to Arduino: {e}")

                    # Reset cho đối tượng tiếp theo
                    _timer_started = False
                    _current_object_info = None
                    _object_color_detected = None
                    _velocity_calculated = 0  # Reset vận tốc đã tính
                    # _predicted_time_to_top = 0 # Reset thời gian dự đoán

                # --- Vẽ thông tin lên frame ---
                # Tọa độ để vẽ text, liên quan đến bounding box (ROI_X1 + x, ROI_Y1 + y)
                text_x_base = ROI_X1 + x
                text_y_base = ROI_Y1 + y + h

                robot_coords_text = f"Robot: ({calib_x:.1f}, {calib_y:.1f}, {calib_z:.1f})"
                cv2.putText(frame, robot_coords_text, (text_x_base, text_y_base + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.putText(frame, f"Center(F): ({cx_frame}, {cy_frame})", (text_x_base, text_y_base + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

                if _velocity_calculated > 0:  # Chỉ hiển thị nếu đã tính
                    cv2.putText(frame, f"Velocity: {_velocity_calculated:.1f} mm/s", (text_x_base, text_y_base + 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                if _predicted_time_to_top > 0:  # Chỉ hiển thị nếu đã tính
                    cv2.putText(frame, f"Pred. Time: {_predicted_time_to_top:.2f} s",
                                (text_x_base, text_y_base + 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                # --- Nhận diện hình dạng ---
                epsilon = 0.04 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                shape = "Unknown"
                if len(approx) == 3:
                    shape = "Triangle"
                elif len(approx) == 4:
                    aspect_ratio = float(w) / h
                    shape = "Square" if 0.9 <= aspect_ratio <= 1.1 else "Rectangle"
                elif len(approx) > 7:
                    shape = "Circle"

                cv2.putText(frame, f"{color_name} {shape}", (text_x_base, ROI_Y1 + y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color_tuple, 2)

                mean_bgr_roi = cv2.mean(roi, mask=mask_cnt)[:3]  # Lấy màu từ ROI gốc
                color_text = f"RGB: {int(mean_bgr_roi[2])}, {int(mean_bgr_roi[1])}, {int(mean_bgr_roi[0])}"
                cv2.putText(frame, color_text, (text_x_base, text_y_base + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Không còn cv2.imshow hay cv2.waitKey ở đây nữa
    # debug_img = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR) # Nếu bạn muốn debug mask
    # cv2.imshow("Debug: Combined Mask", debug_img) # Chỉ để debug, không dùng trong GUI

    return frame