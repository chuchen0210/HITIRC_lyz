#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 回调函数，处理接收到的图像
def image_callback(msg):
    # 创建 CvBridge 对象
    bridge = CvBridge()

    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义红色物体的 HSV 范围
        lower_hsv = np.array([15, 0, 0])
        upper_hsv = np.array([255, 255, 255])

        # 根据 HSV 范围生成二值化图像
        imgThresh = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

        # 噪声去除
        kernel = np.ones((5, 5), np.uint8)
        smoothed_img = cv2.dilate(imgThresh, kernel, iterations=11)
        smoothed_img = cv2.erode(smoothed_img, kernel, iterations=7)

        # 查找轮廓
        contours, hierarchy = cv2.findContours(255 - smoothed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 初始化变量
        max_area = 0
        largest_rect = None
        largest_cnt = None

        # 遍历轮廓
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1200:
                x, y, w, h = cv2.boundingRect(cnt)
                if area > max_area:
                    max_area = area
                    largest_rect = (x, y, w, h)
                    largest_cnt = cnt

        # 绘制轮廓和矩形框
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1200:
                x, y, w, h = cv2.boundingRect(cnt)

                # 绘制矩形框
                if (x, y, w, h) == largest_rect:
                    color = (0, 0, 255)  # 红色
                    cv2.rectangle(frame, (x, y ), (x + w, y + h ), color, 3)
                    cv2.circle(frame, (x, y  + h), 5, (255, 0, 0), -1)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    lower_left_text = f"({x}, {y  + h})"
                    cv2.putText(frame, lower_left_text, (x - 60, y + h + 30), font, 0.6, (0, 0, 255), 2)
                else:
                    color = (0, 255, 0)  # 绿色

                # 绘制质心
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) 
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    center_text = f"({cx}, {cy})"
                    cv2.putText(frame, center_text, (cx + 40, cy + 10), font, 0.6, (0, 0, 255), 2)

        # 显示处理后的图像
        #cv2.imshow('Processed Image', frame)

        # 将处理后的图像发布到 /processed/image 话题
        image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

    except Exception as e:
        rospy.logerr("图像转换错误: %s", str(e))

def image_subscriber():
    # 初始化 ROS 节点
    rospy.init_node('image_subscriber', anonymous=True)

    # 创建图像消息的发布者，发布到 /processed/image 话题
    global image_pub
    image_pub = rospy.Publisher('/processed/image', Image, queue_size=10)

    # 创建订阅者，订阅 /camera/image_raw 话题
    rospy.Subscriber('/camera/image_raw', Image, image_callback)

    # 循环等待消息
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
