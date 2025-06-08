import cv2
import numpy as np
import time
import uuid  # Thêm thư viện để tạo ID duy nhất

# Thêm biến toàn cục để theo dõi vật thể đã được đếm
_counted_objects = set()
# --- Các hằng số và biến toàn cục của module ---
# MA TRẬN NỘI TẠI
CAMERA_MATRIX = np.array([
    [815.98760425, 0, 337.32019432],
    [0, 816.18913792, 241.58831339],
    [0, 0, 1]
], dtype=np.float32)
# HỆ SỐ MÉO
DIST_COEFFS = np.array([0.13648701, -0.15912073, -0.01095812, 0.00887588, -3.32255901], dtype=np.float32)

# CẤU HÌNH ROI (REGION OF INTEREST)
HEIGHT_ROI = [0, 480] # ĐÂY LÀ TỌA ĐỘ GÓC TRÁI TRÊN
WIDTH_ROI = [100, 389] # PHẢI DƯỚI
ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = WIDTH_ROI[0], HEIGHT_ROI[0], WIDTH_ROI[1], HEIGHT_ROI[1]

Y_TOP = 80
Y_TRIGGER = 200
Y_BOTTOM = 400
real_distance_trig_to_top = 39.5  # mm
real_distance_bottom_to_trig = 68  # mm

# VÙNG MÀU CẦN NHẬN DIỆN
COLOR_RANGES = {
    'red': ([0, 70, 50], [10, 255, 255], [160, 70, 50], [180, 255, 255]),
    'green': ([40, 70, 50], [80, 255, 255]),
    'yellow': ([25, 90, 95], [35, 250, 255]),
}
# KHI ĐẾN TRIGGER LINE THÌ NHẬN DIỆN VUÔNG XANH THÀNH TRÒN XANH
# Biến trạng thái cho việc nhận diện
_tracking_active = False
_start_time = 0 # THỜI GIAN XUẤT HIỆN LẦN ĐẦU
_start_y = 0
_max_velocity = 0
_current_object_info = None
_object_color_detected = None
_last_command_time = 0
_command_sent = False  # ĐÃ GỬI LỆNH ĐIỀU KHIỂN CHƯA
_predicted_time_to_top = 0
_calibration_data_list = []
_command_cooldown_duration = 3.0  # Thời gian cooldown sau khi gửi lệnh (giây)
_current_object_id = None
_current_shape_detected = None

# Ngưỡng diện tích tối thiểu cho contour được coi là hợp lệ
MIN_CONTOUR_AREA = 150

# LƯU BIẾN Ở DẠNG DICTIONARY (đây là dạng mảng liên kết, key - value)
_objects_memory = {
    'red_star': {'count': 0},
    'green_star': {'count': 0},
    'yellow_star': {'count': 0},
    'red_square': {'count': 0},
    'green_square': {'count': 0},
    'yellow_square': {'count': 0},
    'red_triangle': {'count': 0},
    'green_triangle': {'count': 0},
    'yellow_triangle': {'count': 0}
}
def set_operation_mode(is_auto):
    """
    Thiết lập chế độ hoạt động của hệ thống.
    Args:
        is_auto (bool): True nếu ở chế độ AUTO, False nếu ở chế độ MANUAL.
    """
    global _is_auto_mode
    _is_auto_mode = bool(is_auto)
    if _is_auto_mode:
        print("ObjectDetection: Chế độ AUTO - Gửi dữ liệu được phép.")
    else:
        print("ObjectDetection: Chế độ MANUAL - Không gửi dữ liệu.")
def get_object_memory():
    """Trả về bản sao của bộ nhớ vật thể để hiển thị trên GUI"""
    return _objects_memory.copy()
def reset_object_memory():
    """Reset tất cả bộ nhớ ĐẾM vật thể"""
    global _objects_memory
    for key in _objects_memory.keys():
        _objects_memory[key] = {'count': 0}

def reset_detection_state():
    """Reset all state variables for object detection."""
    global _tracking_active, _start_time, _start_y, _max_velocity
    global _current_object_info, _object_color_detected, _last_command_time
    global _command_sent, _predicted_time_to_top, _calibration_data_list
    global _counted_objects_id


    _current_object_id = None
    _tracking_active = False
    _start_time = 0
    _start_y = 0
    _max_velocity = 0
    _current_object_info = None
    _object_color_detected = None
    _last_command_time = 0
    _command_sent = False
    _predicted_time_to_top = 0
    _calibration_data_list = []

def process_frame_for_detection(input_frame, ser_instance):
    """
    Processes a single frame for object detection, drawing, and communication.
    Args:
        input_frame: The raw frame from the camera.
        ser_instance: The serial port instance for communication with Arduino.
    Returns:
        The processed frame with detections and information drawn on it.
    """
    global _tracking_active, _start_time, _start_y, _max_velocity
    global _current_object_info, _object_color_detected, _last_command_time
    global _command_sent, _predicted_time_to_top, _calibration_data_list
    global _predicted_time_robot_reach
    global _current_object_id
    global _current_shape_detected
    _predicted_time_robot_reach = 0

    if input_frame is None:
        return None

    frame = input_frame.copy() # TẠO BẢN SAO ĐỂ VẼ CÁC ĐƯỜNG ROI

    # Always draw ROI lines and text, regardless of detection state
    cv2.rectangle(frame, (ROI_X1, ROI_Y1), (ROI_X2, ROI_Y2), (0, 0, 255), 2)
    cv2.line(frame, (ROI_X1, Y_BOTTOM), (ROI_X2, Y_BOTTOM), (255, 0, 255), 2)
    cv2.putText(frame, "Bottom line", (ROI_X1 + 10, Y_BOTTOM - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.line(frame, (ROI_X1, Y_TRIGGER), (ROI_X2, Y_TRIGGER), (0, 255, 255), 2)
    cv2.putText(frame, "Trigger line", (ROI_X1 + 10, Y_TRIGGER - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.line(frame, (ROI_X1, Y_TOP), (ROI_X2, Y_TOP), (255, 0, 0), 2)
    cv2.putText(frame, "Top line", (ROI_X1 + 10, Y_TOP - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # --- Logic để tránh đơ camera sau khi gửi lệnh ---
    # Nếu đã gửi lệnh VÀ chưa đủ thời gian cooldown, thì chỉ vẽ khung và bỏ qua phần xử lý nhận diện vật thể
    if _command_sent and (time.time() - _last_command_time) < _command_cooldown_duration:
        return frame  # Trả về frame đã vẽ đường mà không thực hiện detect object

    # Nếu đã đủ thời gian cooldown, reset trạng thái để sẵn sàng nhận diện vật thể tiếp theo
    if _command_sent and (time.time() - _last_command_time) >= _command_cooldown_duration:
        reset_detection_state()  # Reset trạng thái để bắt đầu nhận diện vật thể mới

    roi = frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
    # CĂT PHẦN ẢNH TRONG VÙNG ROI ĐỂ GIẢM DIỆN TÍCH XỬ LÝ
    if roi.size == 0:
        print("Warning: ROI is empty.")
        return frame
##########################
    # Trước phần tìm contour, thêm các bước xử lý ảnh
    # gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # edges = cv2.Canny(blurred, 50, 150)
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    # dilated = cv2.dilate(edges, kernel, iterations=1)
##################################
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # CHUYỂN ROI SANG HSV ĐỂ XỬ LÝ NHẬN DIỆN MÀU TỐT HƠN SO VỚI BGR

    # Biến để lưu trữ contour lớn nhất được tìm thấy trong khung hình hiện tại
    # và màu sắc của nó, để đảm bảo chỉ xử lý một vật thể chính
    best_contour = None # GIẢM NHIỄU CHO HÌNH BAO VẬT THỂ LỚN NHẤT TÌM ĐƯỢC
    best_color_name = None

    # Tìm contour lớn nhất cho mỗi màu và chọn contour lớn nhất trong số đó
    for color_name, hsv_limits_tuple in COLOR_RANGES.items(): # Đổi tên biến 'ranges' cho rõ ràng
        mask = None # Khởi tạo mask

        if len(hsv_limits_tuple) == 4:
            # Trường hợp có 2 dải màu (ví dụ: red)
            # hsv_limits_tuple là (lower1, upper1, lower2, upper2)
            lower1, upper1, lower2, upper2 = hsv_limits_tuple
            # mask1 tạo mặt nạ nhị phân cho dải màu thứ nhất
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
            # Bất kỳ pixel nào thuộc về dải màu thứ nhất HOẶC dải màu thứ hai
            # đều sẽ được bao gồm trong mặt nạ cuối cùng.
            # Các màu như 'red' có thể nằm ở hai đầu của thang đo Hue (0/180)
        elif len(hsv_limits_tuple) == 2:
            # Trường hợp có 1 dải màu (ví dụ: green, yellow)
            # hsv_limits_tuple là (lower, upper)
            lower, upper = hsv_limits_tuple
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        else:
            print(f"Cảnh báo: Số lượng khoảng HSV không hợp lệ cho màu {color_name}. Bỏ qua màu này.")
            continue # Bỏ qua màu này nếu cấu trúc không mong đợi

        if mask is None: # Kiểm tra lại nếu mask chưa được tạo
            continue

        # LOẠI BỎ NHIỄU
        kernel = np.ones((5, 5), np.uint8) # kernel này là hình vuông
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.medianBlur(mask, 5)

        # contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Lọc các contour có diện tích nhỏ hơn ngưỡng và tìm contour lớn nhất cho màu hiện tại
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= MIN_CONTOUR_AREA]

        if valid_contours:
            current_largest_contour = max(valid_contours, key=cv2.contourArea)
            # So sánh với best_contour tổng thể để tìm ra vật thể lớn nhất trong tất cả các màu
            if best_contour is None or cv2.contourArea(current_largest_contour) > cv2.contourArea(best_contour):
                best_contour = current_largest_contour
                best_color_name = color_name
                _object_color_detected = 'Y' if best_color_name == 'yellow' else best_color_name[0].upper()

    # --- Chỉ xử lý contour lớn nhất được tìm thấy trong toàn bộ khung hình ---
    if best_contour is not None:
        cnt = best_contour
        color_name = best_color_name  # Gán lại color_name cho contour được chọn

        x, y, w, h = cv2.boundingRect(cnt)
        text_x_base = ROI_X1 + x
        # --- Phần tính toán màu sắc trung bình và vẽ ---
        # VẼ BOUNDING BOX VÀ TẠO MASK CHO CONTOUR
        mask_cnt = np.zeros(roi.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)
        mean_hsv_val = cv2.mean(hsv, mask=mask_cnt)
        # DÙNG HSV TRUNG BÌNH ĐỂ TÍNH LẠI MÀU SẮC
        hue_val = int(mean_hsv_val[0])
        sat_val = int(mean_hsv_val[1])
        brightness_val = int(mean_hsv_val[2])
        # if sat_val < 50 or brightness_val < 50:  # Ngưỡng có thể điều chỉnh
        #     return frame  # Bỏ qua frame này nếu màu không đủ rõ

        bgr_color_obj = cv2.cvtColor(np.uint8([[[hue_val, sat_val, brightness_val]]]), cv2.COLOR_HSV2BGR)[0][0]
        bgr_color_tuple = tuple(int(c) for c in bgr_color_obj)

        cv2.rectangle(frame, (ROI_X1 + x, ROI_Y1 + y), (ROI_X1 + x + w, ROI_Y1 + y + h), bgr_color_tuple, 2)

        M = cv2.moments(cnt)
        # hàm này tính moment hình học, trả về dictionary: m00,m10,m01
        # MOMENT CẤP 0: DIỆN TÍCH, CẤP 1: TRỌNG TÂM, CẤP 2: XÁC ĐỊNH VỊ TRÍ VÀ HƯỚNG ĐỐI TƯỢNG
        if M["m00"] != 0:
            cx_roi = int(M["m10"] / M["m00"]) # m00:diện tích đối tượng
            cy_roi = int(M["m01"] / M["m00"])
            # m10 và m01 là các giá trị tính theo x hoặc y để khi thực hiện phép toán
            # ta rút gọn được cường độ các điểm ảnh do đều bằng nhau
            # TÍNH TÂM CONTOUR THỰC TẾ
            cx_frame = ROI_X1 + cx_roi
            cy_frame = ROI_Y1 + cy_roi

            # Bỏ qua nếu tâm lệch (vẫn giữ logic này cho contour lớn nhất)
            bbox_cx_roi = x + w // 2 # tọa độ tâm hình chữ nhật bao quanh contour
            bbox_cy_roi = y + h // 2
            distance = np.sqrt((cx_roi - bbox_cx_roi) ** 2 + (cy_roi - bbox_cy_roi) ** 2)
            # distance: khoảng cách giữa tâm contour thực sự và tâm bounding box
            max_allowed_distance = 0.4 * min(w, h)
            if distance > max_allowed_distance:
                # Nếu contour lớn nhất vẫn bị lệch tâm, không xử lý tiếp
                return frame

            cv2.circle(frame, (cx_frame, cy_frame), 6, (0, 0, 0), -1)
            # Vẽ tâm vật thể lên GUI

            # --- Hiệu chỉnh méo và chuyển đổi tọa độ ---
            distorted_point = np.array([[[cx_frame, cy_frame]]], dtype=np.float32)
            undistorted = cv2.undistortPoints(distorted_point, CAMERA_MATRIX, DIST_COEFFS, P=CAMERA_MATRIX)
            undistorted_coords = undistorted[0][0]

            Z_CONST = 263  # Chiều cao giả định của vật thể so với camera
            fx, fy = CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1]
            cx_intr, cy_intr = CAMERA_MATRIX[0, 2], CAMERA_MATRIX[1, 2]
            real_x = (undistorted_coords[0] - cx_intr) * Z_CONST / fx
            real_y = (undistorted_coords[1] - cy_intr) * Z_CONST / fy
            # tọa độ thật của vật thể trong hệ tọa độ camera
            # DÙNG CÔNG THỨC CAMERA PINHOLE ĐỂ CHUYỂN PIXEL QUA mm
            real_y_top_trig = (Y_TRIGGER - Y_TOP) * Z_CONST / fy
            P_CA = np.array([[real_x], [real_y], [Z_CONST], [1]])
            T_0C = np.array([
                [0, -1, 0, -160],
                [-1, 0, 0, -27],
                [0, 0, -1, -132],
                [0, 0, 0, 1]
            ]) # MA TRẬN CHUYỂN ĐỔI CAMERA QUA ROBOT
            P_OA = T_0C @ P_CA
            calib_x, calib_y, calib_z = P_OA[0, 0], P_OA[1, 0], P_OA[2, 0]
            calib_x_top = P_OA[0, 0] + real_y_top_trig

            if len(_calibration_data_list) < 20:
                _calibration_data_list.append(calib_x)

            # --- Xử lý logic tracking và tính toán vận tốc ---
            current_y_on_frame = cy_roi + ROI_Y1
##############################
            # NHẬN DIỆN HÌNH DẠNG TRƯỚC KHI XỬ LÝ TRIGGER LINE
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # Đây là thuật toán xấp xỉ đường đa giác Douglas-Peucker.
            # Nó làm giảm số lượng đỉnh của một đường cong hoặc đa giác trong khi vẫn giữ được hình dạng tổng thể
            # shape = "Unknown"

            vertices = len(approx) # số đỉnh của đa giác
            hull = cv2.convexHull(cnt)
            area = cv2.contourArea(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(area) / hull_area if hull_area > 0 else 0

            (x, y, w, h) = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h if h != 0 else 1.0
            # tỉ lệ giữa chiều rộng và cao của bounding box

            # Tính độ lệch cạnh nếu là 4 đỉnh
            side_ratio = 0 # hình vuông = 1, hcn < 1
            if vertices == 4:
                sides = []
                for i in range(4):
                    pt1 = approx[i][0]
                    pt2 = approx[(i + 1) % 4][0]
                    length = np.linalg.norm(pt1 - pt2)
                    sides.append(length)
                max_len = max(sides)
                min_len = min(sides)
                side_ratio = min_len / max_len if max_len != 0 else 0

            # ------------------------------
            # DEBUG: In thông số để kiểm tra
            # print(
            #     f"Vertices: {vertices}, Solidity: {solidity:.2f}, Aspect Ratio: {aspect_ratio:.2f}, Side Ratio: {side_ratio:.2f}, Area: {area:.1f}")
            # # ------------------------------

            # đơn vị area là pixel vuông
            # ƯU TIÊN HÌNH CÓ 4 CẠNH TRƯỚC
            if vertices == 4 and 0.85 <= aspect_ratio <= 1.15 and side_ratio > 0.85 and solidity > 0.9:
                shape = "Square"
            elif vertices == 4 and solidity > 0.9:
                shape = "Rectangle"
            elif 6 <= vertices <= 9 and 0.9 <= aspect_ratio <= 1.1 and solidity > 0.93 and area > 1500:
                shape = "Square"
            elif 10 <= vertices <= 16 and solidity < 0.85:
                shape = "Star"
            elif vertices > 16 and solidity > 0.90:
                shape = "Circle"
            elif 3 <= vertices <= 6 and solidity > 0.85 and area > 300:
                shape = "Triangle"
            else:
                shape = "Unknown"

            # Hiển thị thông tin hình dạng và màu sắc
            cv2.putText(frame, f"{color_name} {shape}", (text_x_base, ROI_Y1 + y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color_tuple, 2)

            # Bắt đầu tracking khi vật đi qua bottom line
            if current_y_on_frame >= Y_BOTTOM and not _tracking_active and not _command_sent:
                # Ghi nhận thời gian, vị trí bắt đầu, màu vật thể, gán ID bằng uuid
                _tracking_active = True
                _start_time = time.time()
                _start_y = current_y_on_frame
                _current_object_info = (cx_frame, cy_frame)
                # if color_name == 'yellow':
                #     _object_color_detected = 'Y'
                # else:
                #     _object_color_detected = color_name[0].upper()
                _current_object_id = str(uuid.uuid4())

            # Tính toán vận tốc khi vật đang di chuyển
            if _tracking_active and not _command_sent:
                current_time = time.time()
                elapsed_time = current_time - _start_time

                if elapsed_time > 0:
                    distance_pixels = abs(current_y_on_frame - _start_y)
                    distance_mm = distance_pixels * Z_CONST / fy
                    current_velocity = distance_mm / elapsed_time
                    # TÍNH VẬN TỐC HIỆN TẠI DỰA TRÊN QUÃNG ĐƯỜNG ĐI TỪ Y_BOTTOM

                    if current_velocity > _max_velocity:
                        _max_velocity = current_velocity
                        distance_to_top_mm = (Y_TRIGGER - Y_TOP) * Z_CONST / fy
                        _predicted_time_to_top = distance_to_top_mm / _max_velocity if _max_velocity > 0 else 0
                        _predicted_time_robot_reach = _predicted_time_to_top - 0.4

            # XỬ LÝ TRIGGER LINE
            calib_x_top = calib_x_top + 25
            if _tracking_active and current_y_on_frame <= Y_TRIGGER and not _command_sent:
                object_key = f"{color_name}_{shape.lower()}"

                # Kiểm tra vật thể chưa được đếm
                if _current_object_id is not None and object_key in _objects_memory:
                    # Tạo key duy nhất cho vật thể này
                    object_unique_key = f"{object_key}_{_current_object_id}"

                    if object_unique_key not in _counted_objects:
                        _objects_memory[object_key]['count'] += 1
                        _counted_objects.add(object_unique_key)
                        print(f"Đã phát hiện: {object_key}, Tổng số: {_objects_memory[object_key]['count']}")

                # Gửi lệnh đến Arduino CHỈ KHI Ở CHẾ ĐỘ AUTO
                if _is_auto_mode: # <<< THÊM ĐIỀU KIỆN KIỂM TRA CHẾ ĐỘ AUTO
                    if _max_velocity > 0 and ser_instance and ser_instance.is_open and _object_color_detected:
                        if _predicted_time_robot_reach == 0:
                            _predicted_time_robot_reach = 0.5 # Hoặc một giá trị default hợp lý khác
                        command = f"Next:{calib_x_top:.1f},{calib_y:.1f},{calib_z:.1f};T:{_predicted_time_robot_reach:.2f};C:{_object_color_detected}\n"
                        try:
                            ser_instance.write(command.encode())
                            print(f"Sent to Arduino (AUTO): {command.strip()}")
                            _command_sent = True
                            reset_detection_state()
                            _last_command_time = time.time()
                            # reset_detection_state() # Xem xét lại vị trí của reset_detection_state()
                                                      # Nó đã được gọi ở đầu hàm nếu cooldown đã qua.
                                                      # Nếu bạn muốn reset ngay sau khi gửi, giữ nó ở đây.
                                                      # Hoặc bạn có thể để logic cooldown ở đầu hàm xử lý việc reset.
                        except Exception as e:
                            print(f"Error sending to Arduino (AUTO): {e}")
                    # else: # Có thể thêm log nếu các điều kiện khác không thỏa mãn khi ở AUTO
                    #     if not (_max_velocity > 0): print("AUTO Mode: Velocity not positive.")
                    #     if not (ser_instance and ser_instance.is_open): print("AUTO Mode: Serial not available.")
                    #     if not _object_color_detected: print("AUTO Mode: No object color detected.")
                else:
                    # Ở chế độ MANUAL, không gửi lệnh, nhưng có thể log thông tin
                    if _object_color_detected: # Chỉ log nếu có vật thể để tránh spam
                        print(f"MANUAL Mode: Object {_object_color_detected} at trigger. Data not sent. Coords: ({calib_x_top:.1f}, {calib_y:.1f})")

                # Các dòng này nên được thực thi dù có gửi lệnh hay không,
                # để chuẩn bị cho vật thể tiếp theo hoặc reset trạng thái logic
                _tracking_active = False # Kết thúc tracking cho vật thể này
                _max_velocity = 0        # Reset vận tốc max cho vật thể tiếp theo
                _current_object_id = None # Reset ID sau khi xử lý xong

                # Nếu _command_sent là True (tức là đã gửi lệnh ở AUTO mode),
                # thì logic cooldown ở đầu hàm sẽ xử lý việc reset_detection_state() sau đó.
                # Nếu ở MANUAL mode, _command_sent sẽ không bao giờ là True từ khối này,
                # nên reset_detection_state() có thể cần được gọi ở đây nếu bạn muốn
                # reset ngay lập tức sau khi một vật đi qua trigger line ở MANUAL mode.
                # Tuy nhiên, với logic cooldown hiện tại, có thể không cần thiết.
                if not _is_auto_mode:
                    # Cân nhắc xem có cần reset_detection_state() ngay ở đây cho MANUAL mode không
                    # Hoặc để nó tự nhiên reset khi không có vật thể nào được track nữa.
                    # Hiện tại, logic reset_detection_state() ở đầu hàm dựa vào _command_sent
                    # và cooldown, nên có thể không cần reset thêm ở đây.
                    pass

                _tracking_active = False
                _max_velocity = 0
                _current_object_id = None  # Reset ID sau khi xử lý xong

            # Hiển thị thông tin đếm lên frame
            # memory_text_y = 20
            # for obj_type, data in _objects_memory.items():
            #     display_text = f"{obj_type}: {data['count']}"
            #     cv2.putText(frame, display_text, (10, memory_text_y),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            #     memory_text_y += 20

            # --- Vẽ thông tin lên frame ---
            text_x_base = ROI_X1 + x
            text_y_base = ROI_Y1 + y + h

            robot_coords_text = f"Robot: ({calib_x:.1f}, {calib_y:.1f}, {calib_z:.1f})"
            cv2.putText(frame, robot_coords_text, (text_x_base, text_y_base + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)


    return frame