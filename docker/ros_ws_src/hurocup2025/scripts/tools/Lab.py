import cv2
import numpy as np


current_frame = None
lab_frame = None


def show_lab_value(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        if lab_frame is not None:
            # Lab 格式：L（亮度），a（绿色到红色），b（蓝色到黄色）
            L, a, b = lab_frame[y, x]
            print(f"Position: ({x}, {y})  Lab: ({L:.2f}, {a:.2f}, {b:.2f})")



cap = cv2.VideoCapture(0)

cv2.namedWindow('Video')
cv2.setMouseCallback('Video', show_lab_value)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    current_frame = frame

    lab_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2Lab)

    cv2.imshow('Video', current_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
