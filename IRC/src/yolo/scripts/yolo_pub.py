#!/home/lyz/.conda/envs/yolov8/bin/python  # 指定 Miniconda Python 路径
import rospy
import cv2
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

rospy.init_node("yolo_publisher", anonymous=True)
bbox_pub = rospy.Publisher("/yolo/bounding_boxes", String, queue_size=10)
bridge = CvBridge()
cap = cv2.VideoCapture("/home/lyz/yolov8/output.mp4")  # 0 = 摄像头，或换成视频路径
model = YOLO("/home/lyz/yolov8/train/runs/detect/train/weights/best.pt")

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取边界框坐标
            x_center = (x1 + x2) / 2.0
            y_center = (y1 + y2) / 2.0
            label = model.names[int(box.cls[0])]

            # 发送 ROS 话题
            bbox_msg = f"{label},{x_center},{y_center}"
            bbox_pub.publish(bbox_msg)

    # 可视化
    frame = results[0].plot()
    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(50) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

