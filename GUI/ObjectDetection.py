import cv2
import numpy as np
import time
import uuid  # Thêm thư viện để tạo ID duy nhất
import serial

from queue import Queue
import threading

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

_predicted_time_robot_reach = 0.0  # Đảm bảo là float
_calibration_data_list = []
_command_cooldown_duration = 3.0  # Thời gian cooldown sau khi gửi lệnh (giây)
_current_object_id = None
_current_shape_detected = None
_is_auto_mode = False  # Khởi tạo chế độ (có thể thiết lập lại từ GUI)

# Ngưỡng diện tích tối thiểu cho contour được coi là hợp lệ
MIN_CONTOUR_AREA = 150

# Thêm biến toàn cục để theo dõi vật thể đã được đếm
_counted_objects = set()

# CÁC BIẾN MỚI CHO LOGIC VẬN TỐC
_initial_max_velocity_found = False
_velocity_samples = []
_AVERAGE_VELOCITY_SAMPLE_THRESHOLD = 5  # Số lượng mẫu vận tốc tối thiểu để bắt đầu tính trung bình
_VELOCITY_STABILIZATION_ZONE = 0.8  # Tỷ lệ quãng đường để xác định vận tốc ban đầu ổn định (80% quãng đường từ Y_BOTTOM đến Y_TRIGGER)
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
#########################
class SerialManager:
    def __init__(self, port, baudrate):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.command_queue = Queue()
        self.response_queue = Queue()
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial, daemon=True)
        # luôn lắng nghe dữ liệu từ Arduino. Khi có dữ liệu, nó sẽ đọc
        self.read_thread.start()
        self.write_thread = threading.Thread(target=self._write_serial, daemon=True)
        self.write_thread.start()

    def stop(self):
        self.running = False
        self.read_thread.join()
        self.write_thread.join()
        self.serial_port.close()

    def _read_serial(self):
        while self.running:
            if self.serial_port.in_waiting:
                with self.lock:
                    data = self.serial_port.readline().decode().strip()
                    self.response_queue.put(data)
            time.sleep(0.01)

    def _write_serial(self):
        while self.running:
            if not self.command_queue.empty():
                command = self.command_queue.get()
                with self.lock:
                    self.serial_port.write(command.encode())
            time.sleep(0.01)

    def send_command(self, command):
        self.command_queue.put(command)

    def get_response(self):
        if not self.response_queue.empty():
            return self.response_queue.get()
        return None
###########################
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
    global _objects_memory, _counted_objects
    for key in _objects_memory.keys():
        _objects_memory[key] = {'count': 0}
    _counted_objects.clear()  # Đảm bảo reset cả set các ID đã đếm

def reset_detection_state():
    """Reset all state variables for object detection."""
    global _tracking_active, _start_time, _start_y, _max_velocity
    global _current_object_info, _object_color_detected, _last_command_time
    global _command_sent, _predicted_time_to_top, _calibration_data_list
    global _counted_objects_id, _current_shape_detected
    global _initial_max_velocity_found, _velocity_samples


    _current_object_id = None
    _current_shape_detected = None
    _tracking_active = False
    _start_time = 0
    _start_y = 0
    _max_velocity = 0.0
    _current_object_info = None
    _object_color_detected = None
    _last_command_time = 0
    _command_sent = False
    _predicted_time_to_top = 0.0
    _calibration_data_list = []
    _predicted_time_robot_reach = 0.0

    _initial_max_velocity_found = False
    _velocity_samples = []  # Xóa các mẫu vận tốc

# def process_frame_for_detection(input_frame, ser_instance):
def process_frame_for_detection(input_frame, serial_manager):
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
    global _initial_max_velocity_found, _velocity_samples

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
        # tọa độ x,y góc trên bên trái, chiều rộng cao w,h hcn
        text_x_base = ROI_X1 + x
        # --- Phần tính toán màu sắc trung bình và vẽ ---
        # VẼ BOUNDING BOX VÀ TẠO MASK CHO CONTOUR
        mask_cnt = np.zeros(roi.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)
        # -1: Vẽ tất cả các đường viền trong contours
        mean_hsv_val = cv2.mean(hsv, mask=mask_cnt)
        # DÙNG HSV TRUNG BÌNH ĐỂ TÍNH LẠI MÀU SẮC
        hue_val = int(mean_hsv_val[0])
        sat_val = int(mean_hsv_val[1])
        brightness_val = int(mean_hsv_val[2])
        # if sat_val < 50 or brightness_val < 50:  # Ngưỡng có thể điều chỉnh
        #     return frame  # Bỏ qua frame này nếu màu không đủ rõ

        bgr_color_obj = cv2.cvtColor(np.uint8([[[hue_val, sat_val, brightness_val]]]), cv2.COLOR_HSV2BGR)[0][0]
        # bgr_color_obj sẽ chứa một mảng NumPy 1D với 3 giá trị BGR của màu đã được chuyển đổi (ví dụ: [B, G, R])
        bgr_color_tuple = tuple(int(c) for c in bgr_color_obj)
        # tuple dùng để lưu trữ các giá trị hằng số

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
                [0, -1, 0, -170],
                [-1, 0, 0, -30],
                [0, 0, -1, -132],
                [0, 0, 0, 1]
            ]) # MA TRẬN CHUYỂN ĐỔI CAMERA QUA ROBOT
            P_OA = T_0C @ P_CA
            calib_x, calib_y, calib_z = P_OA[0, 0], P_OA[1, 0], P_OA[2, 0]
            calib_x_top = P_OA[0, 0] + real_y_top_trig

            if len(_calibration_data_list) < 20:
                _calibration_data_list.append(calib_x)
                # append: thêm calib_x vào cuối list _calibration_data_list

            # --- Xử lý logic tracking và tính toán vận tốc ---
            current_y_on_frame = cy_roi + ROI_Y1
##############################
            # NHẬN DIỆN HÌNH DẠNG TRƯỚC KHI XỬ LÝ TRIGGER LINE
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # approx là mảng chưa các điểm đỉnh của contour
            # Đây là thuật toán xấp xỉ đường đa giác Douglas-Peucker.
            # Nó làm giảm số lượng đỉnh của một đường cong hoặc đa giác
            # trong khi vẫn giữ được hình dạng tổng thể
            # shape = "Unknown"

            vertices = len(approx) # số đỉnh của đa giác
            hull = cv2.convexHull(cnt) # tìm đường bao lồi
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
                for i in range(4): # vòng lặp chạy qua 4 cạnh của tứ giác
                    pt1 = approx[i][0] # lấy điểm đỉnh đầu tiên của cạnh hiện tại
                    pt2 = approx[(i + 1) % 4][0]
                    length = np.linalg.norm(pt1 - pt2)
                    sides.append(length) # append: thêm phần tử vào cuối chuỗi
                max_len = max(sides)
                min_len = min(sides)
                side_ratio = min_len / max_len if max_len != 0 else 0

            # ------------------------------
            # DEBUG: In thông số để kiểm tra
            # print(
            #     f"Vertices: {vertices}, Solidity: {solidity:.2f}, Aspect Ratio: {aspect_ratio:.2f}, Side Ratio: {side_ratio:.2f}, Area: {area:.1f}")
            # ------------------------------

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
            _current_shape_detected = shape
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
                _current_object_id = str(uuid.uuid4()) # gán id để nhận riêng vật này
                print(
                    f"DEBUG: Bắt đầu tracking vật thể {color_name} {shape} tại Y_BOTTOM={Y_BOTTOM}, thời gian: {_start_time}")

            # Tính toán vận tốc khi vật đang di chuyển và thời gian robot đến gắp vật
            if _tracking_active and not _command_sent:
                current_time = time.time()
                elapsed_time = current_time - _start_time

                if elapsed_time > 0:
                    distance_pixels = abs(current_y_on_frame - _start_y) # k/c theo trục y
                    distance_mm = distance_pixels * Z_CONST / fy
                    current_velocity = distance_mm / elapsed_time
                    # TÍNH VẬN TỐC HIỆN TẠI DỰA TRÊN QUÃNG ĐƯỜNG ĐI TỪ Y_BOTTOM
                    ######################
                    # print(f"DEBUG: current_y_on_frame: {current_y_on_frame}, current_velocity: {current_velocity:.2f} mm/s") # Bỏ comment để debug

                    # GIAI ĐOẠN 1: Xác định vận tốc tối đa ban đầu để loại bỏ vận tốc nhỏ
                    if not _initial_max_velocity_found:
                        if current_velocity > _max_velocity:
                            _max_velocity = current_velocity
                        # Xác định khi nào đã tìm thấy vận tốc tối đa ban đầu
                        # Vật đã đi được một phần quãng đường từ Y_BOTTOM và _max_velocity đã khác 0
                        stabilization_point_y = Y_BOTTOM - (Y_BOTTOM - Y_TRIGGER) * (1 - _VELOCITY_STABILIZATION_ZONE)
                        if current_y_on_frame <= stabilization_point_y and _max_velocity > 0:
                            _initial_max_velocity_found = True
                            print(
                                f"DEBUG: Đã xác định vận tốc tối đa ban đầu: {_max_velocity:.2f} mm/s tại Y={current_y_on_frame}")
                            # Reset danh sách mẫu vận tốc để chỉ thu thập sau khi vận tốc ban đầu ổn định
                            _velocity_samples = []

                    # GIAI ĐOẠN 2: Tính toán vận tốc trung bình sau khi vận tốc ban đầu đã ổn định
                    if _initial_max_velocity_found:
                        _velocity_samples.append(current_velocity)
                        # Giới hạn số lượng mẫu để tránh bộ nhớ tăng quá lớn và đảm bảo tính tức thời
                        if len(_velocity_samples) > 20:  # Giới hạn 20 mẫu gần nhất
                            _velocity_samples.pop(0)

                        # Tính vận tốc trung bình từ các mẫu
                        if len(_velocity_samples) >= _AVERAGE_VELOCITY_SAMPLE_THRESHOLD:
                            average_velocity = sum(_velocity_samples) / len(_velocity_samples)
                            # Sử dụng vận tốc trung bình này để dự đoán
                            # Đảm bảo vận tốc trung bình không quá nhỏ để tránh chia cho 0
                            effective_velocity = max(average_velocity, 0.1)  # Đặt một ngưỡng tối thiểu 0.1 mm/s

                            distance_to_top_mm = (Y_TRIGGER - Y_TOP) * Z_CONST / fy
                            _predicted_time_to_top = distance_to_top_mm / effective_velocity
                            _predicted_time_robot_reach = _predicted_time_to_top - 0.4  # Điều chỉnh thêm 0.4s
                            # print(f"DEBUG: Avg vel: {average_velocity:.2f}, Pred time to top: {_predicted_time_to_top:.2f}, Pred robot reach: {_predicted_time_robot_reach:.2f}") # Bỏ comment để debug

                    # XỬ LÝ TRIGGER LINE
                    # Cần đảm bảo calib_x_top được tính toán chính xác dựa trên tọa độ thực của vật thể
                    # Việc cộng 25 trực tiếp vào calib_x_top ở đây có thể cần xem xét lại tùy theo mục đích
                    # của calib_x_top (tọa độ đích của robot hay chỉ là offset).
                    # Nếu nó là tọa độ đích, nên tính toán dựa trên các phép biến đổi không gian.
                    # Nếu chỉ là offset, thì không cần thay đổi ở đây.
                    calib_x_top = calib_x_top + 10

                if _tracking_active and current_y_on_frame <= Y_TRIGGER and not _command_sent:
                    object_key = f"{color_name}_{shape.lower()}"

                    # Kiểm tra và đếm vật thể (logic này đã đúng)
                    if _current_object_id is not None and object_key in _objects_memory:
                        object_unique_key = f"{object_key}_{_current_object_id}"
                        if object_unique_key not in _counted_objects:
                            _objects_memory[object_key]['count'] += 1
                            _counted_objects.add(object_unique_key)
                            print(f"Đã phát hiện: {object_key}, Tổng số: {_objects_memory[object_key]['count']}")

                    # Gửi lệnh đến Arduino CHỈ KHI Ở CHẾ ĐỘ AUTO
                    if _is_auto_mode:
                        # ===================== SỬA LỖI Ở ĐÂY =====================
                        # 1. Sử dụng biến 'serial_manager' thay vì 'ser_instance'.
                        # 2. Truy cập 'serial_manager.serial_port.is_open' để kiểm tra.
                        if serial_manager and serial_manager.serial_port.is_open and _object_color_detected and _predicted_time_robot_reach > -0.3:
                            # if _predicted_time_robot_reach <= 0:
                            #     print(
                            #         "WARNING: _predicted_time_robot_reach không hợp lệ, gán lại giá trị mặc định 0.5s")
                            #     _predicted_time_robot_reach = 0.5

                            command = f"Next:{calib_x_top:.1f},{calib_y:.1f},{calib_z:.1f};T:{_predicted_time_robot_reach:.2f};C:{_object_color_detected}\n"

                            try:
                                # Logic gửi lệnh đã đúng vì bạn đang dùng serial_manager ở đây
                                serial_manager.send_command(command)
                                print(f"Sent to Arduino (AUTO): {command.strip()}")
                                _command_sent = True
                                _last_command_time = time.time()
                            except Exception as e:
                                print(f"Error sending to Arduino: {e}")
                        # ===================== KẾT THÚC SỬA LỖI =====================
                        else:
                            # In ra lý do không gửi được lệnh để dễ gỡ lỗi
                            if not (serial_manager and serial_manager.serial_port.is_open):
                                print("MANUAL Mode or Serial Port not ready. Data not sent.")
                            else:
                                print(
                                    f"MANUAL Mode: Did not send command for {_object_color_detected}. Predicted time was {_predicted_time_robot_reach:.2f}s")
                    else:
                        # Ở chế độ MANUAL, không gửi lệnh
                        if _object_color_detected:
                            print(f"MANUAL Mode: Object {_object_color_detected} at trigger. Data not sent.")

                    # Reset trạng thái sau khi xử lý trigger line
                    print(f"Vật thể {object_key} đã qua trigger, reset trạng thái cho vật thể tiếp theo.")
                    _tracking_active = False
                    _max_velocity = 0.0
                    _current_object_id = None
                    # Thêm reset cho các biến vận tốc để đảm bảo tính toán chính xác cho vật thể sau
                    _velocity_samples = []
                    _initial_max_velocity_found = False

            # --- Vẽ thông tin lên frame ---
            text_x_base = ROI_X1 + x
            text_y_base = ROI_Y1 + y + h

            robot_coords_text = f"Robot: ({calib_x:.1f}, {calib_y:.1f}, {calib_z:.1f})"
            cv2.putText(frame, robot_coords_text, (text_x_base, text_y_base + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            # Hiển thị thời gian dự đoán
            predicted_time_text = f"Pred. Time: {_predicted_time_robot_reach:.2f}s"
            cv2.putText(frame, predicted_time_text, (text_x_base, text_y_base + 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)


    return frame