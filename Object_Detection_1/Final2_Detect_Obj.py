import cv2
import numpy as np
import serial


def detect_objects_from_camera():
    camera_matrix = np.array([
        [1558.0, 0, 661.8934],
        [0, 1559.5, 408.2412],
        [0, 0, 1]
    ], dtype=np.float32)

    dist_coeffs = np.array([0.0430, 0.4256, 0.0, 0.0], dtype=np.float32)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # ser = serial.Serial('COM3', 9600)  # Đổi COM nếu cần

    roi_x1, roi_y1, roi_x2, roi_y2 = 255, 0, 760, 645
    y_trigger = 300
    calibration_data = []

    color_ranges = {
        'red': ([0, 70, 50], [10, 255, 255], [160, 70, 50], [180, 255, 255]),
        'green': ([40, 70, 50], [80, 255, 255]),
        'gold': ([30, 0, 60], [110, 50, 255])
    }

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 0, 255), 2)
        cv2.line(frame, (roi_x1, 630), (roi_x2, 630), (255, 0, 255), 2)
        cv2.line(frame, (roi_x1, y_trigger), (roi_x2, y_trigger), (0, 255, 255), 2)

        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        hsb = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        combined_mask = np.zeros(roi.shape[:2], dtype=np.uint8)

        for color_name, ranges in color_ranges.items():
            if color_name == 'red':
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
                    undistorted = cv2.undistortPoints(distorted_point, camera_matrix, dist_coeffs, P=camera_matrix)
                    undistorted = undistorted[0][0]

                    Z = 210
                    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
                    cx_intr, cy_intr = camera_matrix[0, 2], camera_matrix[1, 2]
                    real_x = (undistorted[0] - cx_intr) * Z / fx
                    real_y = (undistorted[1] - cy_intr) * Z / fy

                    P_CA = np.array([[real_x], [real_y], [Z], [1]])
                    T_0C = np.array([
                        [0, -1, 0, -149],
                        [-1, 0, 0, -30],
                        [0, 0, -1, -185],
                        [0, 0, 0, 1]
                    ])
                    P_OA = T_0C @ P_CA

                    calib_x = P_OA[0, 0]
                    calib_y = P_OA[1, 0]

                    # --- Thêm vào mảng dữ liệu ---
                    calibration_data.append(calib_x)

                    if roi_y1 + cy <= y_trigger:
                        if calibration_data:
                            min_x = min(calibration_data)
                            x_to_send = min_x - 80.0  # Dịch lùi 8 cm
                            # command = f"{x_to_send:.1f},{calib_y:.1f}\n"
                            # ser.write(command.encode())
                            # print(f"Sent: {command.strip()}")
                            # calibration_data.clear()

                    robot_coords = f"Robot: ({P_OA[0, 0]:.1f}, {P_OA[1, 0]:.1f}, {P_OA[2, 0]:.1f})"
                    cv2.putText(frame, robot_coords, (roi_x1 + x, roi_y1 + y + h + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    cv2.putText(frame, f"Center: ({cx}, {cy})", (roi_x1 + x, roi_y1 + y + h + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

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
    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_objects_from_camera()
