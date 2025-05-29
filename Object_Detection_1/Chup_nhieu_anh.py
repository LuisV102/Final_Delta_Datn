import cv2
import os
import datetime


save_dir = r"D:\TAI LIEU MON HOC\Do_an_tot_nghiep\Object_Detection_1\Anh_ban_co_3/"
# Tạo thư mục lưu ảnh calibration-
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Khởi tạo camera thường (0 là ID camera, đổi nếu có nhiều camera)
cap = cv2.VideoCapture(0)

# Kiểm tra camera có mở được không
if not cap.isOpened():
    print("Không thể mở camera.")
    exit()

# Biến đếm số ảnh đã chụp
image_count = 0

try:
    while image_count < 40:
        # Đọc frame từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không đọc được frame từ camera.")
            break

        # Hiển thị frame
        cv2.imshow("Camera Live - Adjust Chessboard", frame)

        # Chờ phím nhấn
        key = cv2.waitKey(1) & 0xFF

        # Nhấn phím 'c' để chụp ảnh
        if key == ord('c'):
            # Tạo tên tệp duy nhất với thời gian
            # timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            # image_name = os.path.join(save_dir, f"image_{timestamp}_{image_count + 1}.jpg")
            image_name = os.path.join(save_dir, f"{image_count + 1}.jpg")



            # Lưu ảnh
            success = cv2.imwrite(image_name, frame)
            if success:
                print(f"Đã lưu ảnh: {image_name}")
                print(f"Kích thước ảnh: {frame.shape}")
                image_count += 1
            else:
                print(f"Lỗi khi lưu ảnh: {image_name}")

            # Hiển thị thông báo trên frame
            cv2.putText(frame, f"Captured: {image_count}/40", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Camera Live - Adjust Chessboard", frame)
            cv2.waitKey(500)

        # Nhấn phím 'q' để thoát
        elif key == ord('q'):
            break
finally:
    # Giải phóng camera và đóng cửa sổ
    cap.release()
    cv2.destroyAllWindows()
    print("Hoàn thành chụp ảnh calibration.")
