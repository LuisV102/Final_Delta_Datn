import cv2
import numpy as np
import glob
import os

# --- Thông số checkerboard ---
CHECKERBOARD = (9, 6)   # số góc trong (9 cột, 6 hàng)
SQUARE_SIZE = 2.35      # cm

# --- Tạo object points (tọa độ thực tế) ---
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # Danh sách điểm 3D
imgpoints = []  # Danh sách điểm 2D

# --- Đọc ảnh từ thư mục ---
image_dir = "calib_images"  # Đổi tên nếu khác
images = glob.glob(os.path.join(image_dir, "*.jpg"))

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)

        # Vẽ và hiển thị các góc
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow("Checkerboard", img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# --- Thực hiện calibration ---
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# --- Hiển thị kết quả ---
print("\n>>> KẾT QUẢ CALIBRATION <<<")
print("Ma trận nội (Camera Matrix):\n", camera_matrix)
print("\nHệ số méo (Distortion Coefficients):\n", dist_coeffs)

# --- Ma trận ngoại cho từng ảnh ---
for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
    R, _ = cv2.Rodrigues(rvec)
    print(f"\n--- Ảnh {i+1} ---")
    print("Ma trận xoay (R):\n", R)
    print("Vector tịnh tiến (T):\n", tvec)
