Smart Vision Assistant for Visually Impaired

This project combines YOLOv3 object detection, ESP32-CAM live video streaming, and sensor fusion (ultrasonic + IR) using an Extended Kalman Filter (EKF). It is designed to assist visually impaired individuals by detecting objects and estimating their distance in real-time.

Features:

Real-time object detection using YOLOv3
Live video stream from ESP32-CAM to PC
Distance estimation using ultrasonic and IR sensors
Sensor fusion with Extended Kalman Filter (EKF)
Can be extended with audio or visual feedback
Optimized for assistive applications

Objectives:

Help visually impaired users perceive nearby obstacles or objects
Fuse multiple sensor inputs for reliable and accurate range detection
Provide real-time AI insights through a lightweight embedded setup

Technologies Used:

YOLOv3 – for object recognition (OpenCV + Darknet or ONNX)
ESP32-CAM – wireless camera module for video input
Ultrasonic sensor (HC-SR04) – for distance measurement
Digital IR sensor – for obstacle detection
Python – for processing and fusion
OpenCV – for video handling and object detection
Extended Kalman Filter (EKF) – for sensor data fusion

How It Works:

ESP32-CAM streams real-time video over Wi-Fi to a PC.
The PC processes the stream using OpenCV.
YOLOv3 runs on each frame to identify objects.
Ultrasonic and IR sensors send distance data to the PC via serial.
An Extended Kalman Filter fuses the camera and sensor data for accurate distance estimation.
The output includes object type and estimated distance.

Installation:

ESP32-CAM Setup:

Flash the CameraWebServer sketch using Arduino IDE
Set the correct camera model (e.g., AI Thinker) and Wi-Fi credentials
Use the Serial Monitor to get the video stream URL (e.g., http://192.168.x.x)

PC-Side Setup:

Clone your GitHub repository
Install required libraries using pip install -r requirements.txt
Run the main script: python yolov3_ekf_fusion.py
