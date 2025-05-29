import cv2
import numpy as np
import glob

# Kích thước bảng cờ vua (số ô - 1)
chessboard_size = (9, 6)
frame_size = (640, 480)  # Kích thước ảnh

# Chuẩn bị các điểm ảnh trên bảng cờ vua
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Mảng để lưu các điểm ảnh và điểm thực tế
objpoints = []  # Điểm thực tế trong không gian 3D
imgpoints = []  # Điểm ảnh trong không gian 2D

# Đọc các ảnh calibration
images = glob.glob("calibration_images/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Tìm các góc trên bảng cờ vua
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Vẽ và hiển thị các góc
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow("Chessboard", img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibration camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, frame_size, None, None
)

# Lưu kết quả calibration
np.savez("calibration_data_deep.npz", camera_matrix_deep=camera_matrix, dist_coeffs_deep=dist_coeffs)

print("Calibration hoàn thành!")
print("Ma trận camera:\n", camera_matrix)
print("Hệ số biến dạng:\n", dist_coeffs)
