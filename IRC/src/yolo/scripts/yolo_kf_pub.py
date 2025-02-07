#!/home/lyz/.conda/envs/yolov8/bin/python  # 指定 Miniconda Python 路径
import rospy
import cv2
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
from kalman import KalmanTracker  # 导入卡尔曼滤波器类

# 初始化ROS节点
rospy.init_node("yolo_publisher", anonymous=True)
bbox_pub = rospy.Publisher("/yolo/bounding_boxes", String, queue_size=10)
bridge = CvBridge()
cap = cv2.VideoCapture("/home/lyz/yolov8/handler.mp4")  # 0 = 摄像头，或换成视频路径
model = YOLO("/home/lyz/yolov8/train/runs/detect/handler_train/weights/best.pt")

# 获取输入视频的帧率和分辨率
fps = int(cap.get(cv2.CAP_PROP_FPS))  # 获取帧率

# 创建视频写入对象，选择与实时显示一致的大小
output_path = "/home/lyz/yolov8/kf.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 设置编码格式

# 仅在第一次读取时获取图像的大小（即窗口大小）
ret, frame = cap.read()
if ret:
    frame_height, frame_width = frame.shape[:2]
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

# 初始化卡尔曼滤波器
tracker = KalmanTracker()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break

    # 运行YOLO模型进行目标检测
    results = model(frame)

    # 处理YOLO检测到的目标
    detection = None
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取边界框坐标
            x_center = (x1 + x2) / 2.0
            y_center = (y1 + y2) / 2.0
            label = model.names[int(box.cls[0])]
            
            detection = (x_center, y_center)  # YOLO的实际检测值

            # 使用卡尔曼滤波器来预测目标的中心位置
            predicted_x, predicted_y = tracker.update(detection)

            # 发送ROS话题，包含目标类别和预测位置
            bbox_msg = f"{label},{predicted_x},{predicted_y}"
            bbox_pub.publish(bbox_msg)

    # 如果YOLO丢失目标，使用预测位置
    if detection is None:
        predicted_x, predicted_y = tracker.update(None)


                
    # 可视化：在图像上绘制卡尔曼预测的目标位置
    frame = results[0].plot()
    cv2.circle(frame, (predicted_x, predicted_y), 5, (0, 0, 255), -1)  # 红色：预测值
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, f"Handler: ({predicted_x}, {predicted_y})", (predicted_x, predicted_y-10), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    
    # 将帧写入视频文件
    video_writer.write(frame)

    # 显示图片
    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(50) & 0xFF == ord("q"):
        break

# 释放资源
cap.release()
video_writer.release()  # 关闭视频写入对象
cv2.destroyAllWindows()

