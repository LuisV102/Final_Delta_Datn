import cv2
import numpy as np
import serial
import time


def detect_objects_from_camera():
    ################################################## HIỆU CHỈNH MÉO ẢNH ##################################################
    ## camera_matrix là ma trận thông số nội chứa tiêu cự fx,fy hay nói cách khác là ma trận K là ma trận chuyển đổi
    ## điểm từ tọa độ tọa độ camera sang hệ tọa độ pixel, ma trận thông số ngoại là ma trận chuyển đổi tọa độ thực tế
    ## sang hệ tọa độ camera
    # camera_matrix = np.array([
    #     [1501.8, 0, 659.018],
    #     [0, 1504.6, 391.2912],
    #     [0, 0, 1]
    # ], dtype=np.float32)
    camera_matrix = np.array([
        [815.98760425, 0, 337.32019432],
        [0, 816.18913792, 241.58831339],
        [0, 0, 1]
    ], dtype=np.float32)

    ## Mảng chứa các hệ số méo dist_coeffs = [k1 ,k2, p1, p2] -> k1 hệ số méo xuyên tâm bậc 1, k2 là bậc 2, p1 hệ số méo tiếp
    ## tuyến, p2 hệ số méo tếp tuyến
    # dist_coeffs = np.array([0.0430, 0.4256, 0.0, 0.0], dtype=np.float32)
    # dist_coeffs = np.array([0.0379, 0.0248, 0.0, 0.0], dtype=np.float32) ##[0.0379, 0.4248, 0.0, 0.0]
    dist_coeffs = np.array([0.13648701,-0.15912073,-0.01095812,0.00887588,-3.32255901], dtype=np.float32) #1

    ############################################# THIẾT LẬP VIDEO VÀ VÙNG XỬ LÝ#############################################
    ############################################# TỐI ƯU KHỞI TẠO CAMERA ##################################################
    cap = None
    for i in range(3):  # Thử tối đa 3 lần
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Dùng DirectShow giúp mở nhanh trên Windows
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(0.3)  # Cho camera ổn định

        ret, _ = cap.read()
        if ret:
            print("[INFO] Camera ready.")
            break
        else:
            cap.release()
            print("[WARN] Retry opening camera...")

    if cap is None or not cap.isOpened():
        print("[ERROR] Cannot open camera.")
        return

    ############################################ KHAI BÁO CỔNG ĐANG CẮM ARDUINO ############################################
    # ser = serial.Serial('COM5', 9600)  # Đổi COM nếu cần

    start_time = None
    end_time = None
    object_passed_bottom = False  # Đánh dấu trạng thái đã qua bottom line

    ############################################ CẤU HÌNH ROI (REGION OF INTEREST) #########################################
    height_roi = [0, 480]
    width_roi = [100, 389]
    roi_x1, roi_y1, roi_x2, roi_y2 = width_roi[0], height_roi[0], width_roi[1], height_roi[1]

    y_top = 80
    y_trigger = 200
    y_bottom = 400
    real_distance_trig_to_top = 39.5  # mm
    real_distance_bottom_to_trig = 68  # mm
    calibration_data = []
    _last_command_time = 0  # Thời điểm gửi lệnh cuối cùng
    _command_sent = False  # Cờ báo hiệu đã gửi lệnh

    # Variables for time calculation
    timer_started = False
    start_time = 0
    end_time = 0
    velocity = 0
    distance_trigger_to_top = y_trigger - y_top  # in pixels
    predicted_time = 0

    pixel_to_mm = 100 / (width_roi[1] - width_roi[0])  # 100 Là độ rộng của băng tải

    # Object tracking variables
    current_object = None
    object_color = None
    ################################################# VÙNG MÀU CẦN NHẬN DIỆN ###############################################
    color_ranges = {
        'red': ([0, 70, 50], [10, 255, 255], [160, 70, 50], [180, 255, 255]),
        'green': ([40, 70, 50], [80, 255, 255]),
        'yold': ([30, 0, 60], [110, 50, 255], [140, 4, 50], [160, 6, 50]),
    }

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        ############ Cắt vùng ROI -> Chuyển sang HSV -> Tạo mask cho từng màu -> Tìm cạnh, xử lý hình dạng, màu sắc ############
        ############################################# CẮT VÙNG ROI CỦA TỪNG FRAME ##############################################
        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 0, 255), 2)
        cv2.line(frame, (roi_x1, y_bottom), (roi_x2, y_bottom), (255, 0, 255), 2)
        cv2.putText(frame, "Bottom line", (roi_x1 + 10, y_bottom - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        cv2.line(frame, (roi_x1, y_trigger), (roi_x2, y_trigger), (0, 255, 255), 2)
        cv2.putText(frame, "Trigger line", (roi_x1 + 10, y_trigger - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255),
                    2)
        cv2.line(frame, (roi_x1, y_top), (roi_x2, y_top), (255, 0, 0), 2)
        cv2.putText(frame, "Top line", (roi_x1 + 10, y_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        hsb = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        combined_mask = np.zeros(roi.shape[:2], dtype=np.uint8)

        for color_name, ranges in color_ranges.items():
            if color_name == 'red' or color_name == 'yold':
                lower1, upper1, lower2, upper2 = ranges
                mask1 = cv2.inRange(hsb, np.array(lower1), np.array(upper1))
                mask2 = cv2.inRange(hsb, np.array(lower2), np.array(upper2))
                mask = mask1 | mask2
            else:
                lower, upper = ranges
                mask = cv2.inRange(hsb, np.array(lower), np.array(upper))

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.medianBlur(mask, 5)

            combined_mask = cv2.bitwise_or(combined_mask, mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 150:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)

                mask_cnt = np.zeros(roi.shape[:2], dtype=np.uint8)
                cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)
                mean_hsb = cv2.mean(hsb, mask=mask_cnt)
                hue_val = int(mean_hsb[0])
                sat_val = int(mean_hsb[1])
                brightness_val = int(mean_hsb[2])

                bgr_color = cv2.cvtColor(np.uint8([[[hue_val, sat_val, brightness_val]]]), cv2.COLOR_HSV2BGR)[0][0]
                bgr_color = tuple(int(c) for c in bgr_color)

                cv2.rectangle(frame, (roi_x1 + x, roi_y1 + y), (roi_x1 + x + w, roi_y1 + y + h), bgr_color, 2)

                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    bbox_cx = x + w // 2
                    bbox_cy = y + h // 2

                    distance = np.sqrt((cx - bbox_cx) ** 2 + (cy - bbox_cy) ** 2)
                    max_allowed_distance = 0.4 * min(w, h)

                    if distance > max_allowed_distance:
                        continue  # Bỏ qua nếu trọng tâm lệch quá xa

                    # -- Tiếp tục xử lý nếu hợp lệ --
                    mask_cnt = np.zeros(roi.shape[:2], dtype=np.uint8)
                    cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)
                    mean_hsb = cv2.mean(hsb, mask=mask_cnt)
                    hue_val = int(mean_hsb[0])
                    sat_val = int(mean_hsb[1])
                    brightness_val = int(mean_hsb[2])

                    bgr_color = cv2.cvtColor(np.uint8([[[hue_val, sat_val, brightness_val]]]), cv2.COLOR_HSV2BGR)[0][0]
                    bgr_color = tuple(int(c) for c in bgr_color)

                    cv2.rectangle(frame, (roi_x1 + x, roi_y1 + y), (roi_x1 + x + w, roi_y1 + y + h), bgr_color, 2)
                    cv2.circle(frame, (roi_x1 + cx, roi_y1 + cy), 6, (0, 0, 0), -1)

                    # --- Hiệu chỉnh và chuyển đổi ---
                    distorted_point = np.array([[[cx + roi_x1, cy + roi_y1]]], dtype=np.float32)

                    ## cv2.undistortPoints dùng để hiệu chỉnh ảnh méo
                    undistorted = cv2.undistortPoints(distorted_point, camera_matrix, dist_coeffs, P=camera_matrix)
                    undistorted = undistorted[0][0]

                    # Z = 210
                    # ## Độ dài tiêu cự theo chiều x,y
                    # fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
                    # ## Độ lệch pixel cx,cy
                    # cx_intr, cy_intr = camera_matrix[0, 2], camera_matrix[1, 2]
                    # real_x = (undistorted[0] - cx_intr) * Z / fx
                    # real_y = (undistorted[1] - cy_intr) * Z / fy
                    #
                    # P_CA = np.array([[real_x], [real_y], [Z], [1]])
                    # T_0C = np.array([
                    #     [0, -1, 0, -160],
                    #     [-1, 0, 0, -30],
                    #     [0, 0, -1, -185],
                    #     [0, 0, 0, 1]
                    # ])
                    Z_CONST = 263  # Chiều cao giả định của vật thể so với camera
                    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
                    cx_intr, cy_intr = camera_matrix[0, 2], camera_matrix[1, 2]
                    real_x = (undistorted[0] - cx_intr) * Z_CONST / fx
                    real_y = (undistorted[1] - cy_intr) * Z_CONST / fy

                    # real_x = real_x - 100
                    # real_y = real_y - 50
                    P_CA = np.array([[real_x], [real_y], [Z_CONST], [1]])
                    T_0C = np.array([
                        [0, -1, 0, -160],
                        [-1, 0, 0, -30],
                        [0, 0, -1, -125],
                        [0, 0, 0, 1]
                    ])
                    P_OA = T_0C @ P_CA

                    calib_x = P_OA[0, 0]
                    calib_y = P_OA[1, 0]
                    calib_z = P_OA[2, 0]
                    # if calib_y < 0:
                    #     calib_y = calib_y - 20

                    # --- Thêm vào mảng dữ liệu ---
                    calibration_data.append(calib_x)
                    calibration_data.append(calib_y)

                    # Check if object passes bottom line (start timer)
                    if roi_y1 + cy >= y_bottom and not timer_started:
                        timer_started = True
                        start_time = time.time()
                        current_object = (cx, cy)
                        # object_color = color_name[0].upper()  # Get first letter (R, G, Y)
                        # Xác định mã màu gửi đến Arduino
                        if color_name == 'yold':
                            object_color = 'Y'  # Gold → 'Y'
                        else:
                            object_color = color_name[0].upper()  # Red → 'R', Green → 'G'

                    # Check if object passes trigger line (stop timer and calculate)
                    if timer_started and roi_y1 + cy <= y_trigger:
                        end_time = time.time()
                        elapsed_time = end_time - start_time

                        # Calculate distance traveled (from bottom to trigger line)
                        distance_pixels = y_bottom - y_trigger
                        distance_mm = distance_pixels * (
                                100 / (width_roi[1] - width_roi[0]))

                        # Calculate velocity (mm/s)
                        velocity = distance_mm / elapsed_time

                        # Calculate predicted time to reach top line
                        distance_to_top = y_trigger - y_top
                        distance_to_top_mm = distance_to_top * (100 / (width_roi[1] - width_roi[0]))
                        predicted_time = distance_to_top_mm / velocity

                        # Send data to Arduino
                        if current_object:
                            command = f"Next:{calib_x:.1f},{calib_y:.1f},{calib_z:.1f};T:{predicted_time:.2f};C:{object_color}\n"
                            # ser.write(command.encode())
                            print(f"Sent to Arduino: {command.strip()}")

                        # Reset for next object
                        timer_started = False
                        current_object = None
                        object_color = None

                    robot_coords = f"Robot: ({calib_x:.1f}, {calib_y:.1f}, {calib_z:.1f})"
                    cv2.putText(frame, robot_coords, (roi_x1 + x, roi_y1 + y + h + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    cv2.putText(frame, f"Center: ({cx}, {cy})", (roi_x1 + x, roi_y1 + y + h + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

                    # Display velocity and predicted time
                    if velocity > 0:
                        cv2.putText(frame, f"Velocity: {velocity:.1f} mm/s", (roi_x1 + x, roi_y1 + y + h + 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                        cv2.putText(frame, f"Predicted time: {predicted_time:.2f} s",
                                    (roi_x1 + x, roi_y1 + y + h + 100),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

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

                cv2.putText(frame, f"{color_name} {shape}", (roi_x1 + x, roi_y1 + y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

                mean_bgr = cv2.mean(roi, mask=mask_cnt)[:3]
                color_text = f"RGB: {int(mean_bgr[2])}, {int(mean_bgr[1])}, {int(mean_bgr[0])}"
                cv2.putText(frame, color_text, (roi_x1 + x, roi_y1 + y + h + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            cv2.imshow("Multi-Color Object Detection", frame)
            debug_img = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            cv2.imshow("Debug: Combined Mask", debug_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    # ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_objects_from_camera()
