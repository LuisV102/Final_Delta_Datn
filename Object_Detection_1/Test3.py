import cv2
import numpy as np
import matplotlib.pyplot as plt

image_path = r"D:\TAI LIEU MON HOC\Do_an_tot_nghiep\Camera_Roll\vuong_do\img004-00051.jpg"
image = cv2.imread(image_path)

# Chuyển đổi ảnh sang HSV và tạo mask cho màu đỏ
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 70, 50])
upper_red2 = np.array([180, 255, 255])
mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

# Kết hợp với xử lý grayscale
img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
_, bw_gray = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

# Kết hợp mask màu đỏ và kết quả nhị phân
combined = cv2.bitwise_and(mask_red, bw_gray)

# Làm sạch kết quả
kernel = np.ones((3,3), np.uint8)
cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

# Hiển thị các bước trung gian
plt.figure(figsize=(15,10))
plt.subplot(231), plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), plt.title('Original')
plt.subplot(232), plt.imshow(mask_red, cmap='gray'), plt.title('Red Mask')
plt.subplot(233), plt.imshow(bw_gray, cmap='gray'), plt.title('Grayscale Binary')
plt.subplot(234), plt.imshow(combined, cmap='gray'), plt.title('Combined')
plt.subplot(235), plt.imshow(cleaned, cmap='gray'), plt.title('Cleaned')
plt.show()

# Phần còn lại giữ nguyên như code của bạn
num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
filtered_bw = np.zeros_like(cleaned)
for i in range(1, num_labels):
    if stats[i, cv2.CC_STAT_AREA] > 110:
        filtered_bw[labels == i] = 255

contours, _ = cv2.findContours(filtered_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
result_image = image.copy()

for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    bbox_cx, bbox_cy = x + w // 2, y + h // 2
    cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Xác định trọng tâm
    M = cv2.moments(cnt)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = bbox_cx, bbox_cy
    cv2.circle(result_image, (cx, cy), 6, (255, 0, 0), -1)
    cv2.putText(result_image, f"({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Nhận diện hình dạng
    approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
    shape = "None"
    if len(approx) == 3:
        shape = "Tam giac"
    elif len(approx) == 4:
        shape = "HCN hoac vuong"
    elif len(approx) > 4:
        shape = "Tron"
    cv2.putText(result_image, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Lấy màu trung bình của vật thể
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)
    mean_color = cv2.mean(hsv, mask=mask)[:3]
    color_text = f"HSV: ({int(mean_color[0])}, {int(mean_color[1])}, {int(mean_color[2])})"
    cv2.putText(result_image, color_text, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    print(f"Trong tam: ({cx}, {cy}), Hinh dang: {shape}, Mau (HSV): {mean_color}")

# Hiển thị ảnh kết quả
plt.figure()
plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
plt.title("Phát hiện Hình dạng, Bounding Box, Màu sắc và Trọng tâm")
plt.show()