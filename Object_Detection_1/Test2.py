import cv2
import numpy as np
import matplotlib.pyplot as plt

image_path = r"D:\TAI LIEU MON HOC\Do_an_tot_nghiep\Camera_Roll\vuong_do\img004-00051.jpg"
image = cv2.imread(image_path)

# Chuyển ảnh sang ảnh xám nếu là ảnh màu
img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
_, img_gray = cv2.threshold(img_gray, 90, 255, cv2.THRESH_BINARY)
# img_gray = cv2.equalizeHist(img_gray)  # Cân bằng histogram

# img_gray = rgb2gray(image)
plt.figure()
plt.imshow(img_gray)
plt.title("img_gray")
plt.show()

# # Áp dụng phương pháp đóng mở để giảm nhiễu
# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
# bw = cv2.morphologyEx(img_gray, cv2.MORPH_CLOSE, kernel)
# bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)

# # Áp dụng bộ lọc trung vị để giảm nhiễu
# img_gray = cv2.medianBlur(img_gray, 5)
kernel = np.ones((5,5),np.uint8)

blur = cv2.GaussianBlur(img_gray,(5,5),0)
erosion = cv2.erode(blur,kernel,iterations = 1)
opening = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, kernel)
bw = opening

# # Nhị phân hóa ảnh bằng phương pháp Otsu
# _, bw = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)
#
# bw = cv2.adaptiveThreshold(bw, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
#                            cv2.THRESH_BINARY, 11, 0.4)


for x in range(5):
    bw = cv2.medianBlur(bw, 5)

plt.figure()
plt.imshow(bw)
plt.title("bw")
plt.show()

# Loại bỏ nhiễu nhỏ hơn 100 pixel
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(bw, connectivity=8)
filtered_bw = np.zeros_like(bw)
for i in range(1, num_labels):
    if stats[i, cv2.CC_STAT_AREA] > 110:
        filtered_bw[labels == i] = 255

# Tìm contours
contours, _ = cv2.findContours(filtered_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
result_image = image.copy()

for cnt in contours:
    # Xác định bounding box
    x, y, w, h = cv2.boundingRect(cnt)
    bbox_cx, bbox_cy = x + w // 2, y + h // 2  # Trọng tâm của bounding box
    cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # # Xác định trọng tâm
    # M = cv2.moments(cnt)
    # if M["m00"] != 0:
    #     cx = int(round(M["m10"] / M["m00"]))
    #     cy = int(round(M["m01"] / M["m00"]))
    #     distance = np.sqrt((cx - bbox_cx) ** 2 + (cy - bbox_cy) ** 2)
    #     if distance < w // 6:  # Kiểm tra xem trọng tâm có gần trọng tâm bounding box không
    #         cv2.circle(result_image, (cx, cy), 6, (255, 0, 0), -1)
    # else:
    #     cx, cy = bbox_cx, bbox_cy
    #     cv2.circle(result_image, (cx, cy), 6, (255, 0, 0), -1)
    #
    # cv2.putText(result_image, f"({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Xác định vùng có cường độ sáng cao nhất trong bounding box
    roi = img_gray[y:y + h, x:x + w]
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(roi)
    cx, cy = x + max_loc[0], y + max_loc[1]
    distance = np.sqrt((cx - bbox_cx) ** 2 + (cy - bbox_cy) ** 2)
    if distance < w // 8:
        cv2.circle(result_image, (cx, cy), 6, (255, 0, 0), -1)
    else:
        cx, cy = bbox_cx, bbox_cy
        cv2.circle(result_image, (cx, cy), 6, (255, 0, 0), -1)

    cv2.putText(result_image, f"({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)

    # Nhận diện hình dạng
    approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
    shape = "None"
    if len(approx) == 3:
        shape = "Tam giac"
    elif len(approx) == 4:
        shape = "HCN hoac vuong"
    elif len(approx) > 4 :
        shape = "Tron"
    cv2.putText(result_image, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Lấy màu trung bình của vật thể
    mask = np.zeros_like(filtered_bw)
    cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)
    mean_color = cv2.mean(image, mask=mask)[:3]
    color_text = f"Mau: ({int(mean_color[0])}, {int(mean_color[1])}, {int(mean_color[2])})"
    cv2.putText(result_image, color_text, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    print(f"Trong tam: ({cx}, {cy}), Hinh dang: {shape}, Mau (BGR): {mean_color}")

# Hiển thị ảnh kết quả
plt.figure()
plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
plt.title("Phát hiện Hình dạng, Bounding Box, Màu sắc và Trọng tâm")
plt.show()
