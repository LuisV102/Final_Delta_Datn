import cv2
import numpy as np

def pick_color():
    def callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hsv_value = hsv[y, x]
            print(f"HSV tại ({x}, {y}): {hsv_value}")

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow("Frame", frame)
        cv2.setMouseCallback("Frame", callback)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

pick_color()
