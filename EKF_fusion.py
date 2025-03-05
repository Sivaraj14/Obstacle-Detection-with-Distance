import numpy as np
import cv2
import serial
from filterpy.kalman import ExtendedKalmanFilter as EKF
from scipy.linalg import block_diag

ser = serial.Serial('COM3', 115200)

ekf = EKF(dim_x=4, dim_z=2)

ekf.x = np.array([0, 0, 0, 0])

dt = 0.1
ekf.F = np.array([
    [1, dt, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

ekf.H = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 1]
])

ekf.Q = block_diag(np.eye(2) * 0.1, np.eye(2) * 0.1)

ekf.R = np.eye(2) * 0.2

ekf.P = np.eye(4) * 1

def estimate_camera_distance(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100,
                               param1=50, param2=30, minRadius=5, maxRadius=50)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - r, y - r), (x + r, y + r), (0, 128, 255), 2)
            distance = 1000 / r
            return distance
    return None

def HJacobian(x):
    return np.array([[1, 0, 0, 0], [0, 0, 1, 1]])

def Hx(x):
    return np.array([x[0], x[2]])

cap = cv2.VideoCapture("http://192.168.7.22:81/stream")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    cam_dist = estimate_camera_distance(frame)

    ir_dist = None
    if ser.in_waiting > 0:
        try:
            ir_dist = float(ser.readline().decode('utf-8').strip())
        except:
            pass

    if cam_dist and ir_dist:
        z = np.array([ir_dist, cam_dist])
        ekf.predict()
        ekf.update(z, HJacobian, Hx)
        fused_distance = ekf.x[0]

        cv2.putText(frame, f"IR Distance: {ir_dist:.2f} cm", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Camera Distance: {cam_dist:.2f} cm", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Fused Distance: {fused_distance:.2f} cm", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        print(f"IR Distance: {ir_dist:.2f} cm, Camera Distance: {cam_dist:.2f} cm, Fused Distance: {fused_distance:.2f} cm")

    cv2.imshow('ESP32-CAM Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
