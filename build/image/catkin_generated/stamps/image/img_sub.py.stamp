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

        # 转换为 HSV 颜色空间
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义颜色范围（示例红色范围）
        lower_hsv = np.array([0, 100, 100])
        upper_hsv = np.array([10, 255, 255])

        # 生成二值化图像
        imgThresh = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

        # 噪声去除
        kernel = np.ones((5, 5), np.uint8)
        smoothed_img = cv2.dilate(imgThresh, kernel, iterations=5)
        smoothed_img = cv2.erode(smoothed_img, kernel, iterations=3)

        # 查找轮廓
        contours, hierarchy = cv2.findContours(smoothed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 遍历轮廓并框选目标
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1200:  # 忽略小区域
                x, y, w, h = cv2.boundingRect(cnt)

                # 绘制矩形框
                color = (0, 255, 0)  # 绿色
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

                # 在图像上显示矩形的坐标
                font = cv2.FONT_HERSHEY_SIMPLEX
                rect_text = f"Rect: ({x}, {y}, {w}, {h})"
                cv2.putText(frame, rect_text, (x, y - 10), font, 0.5, (0, 255, 0), 1)

                # 输出矩形信息到控制台
                rospy.loginfo(f"Detected rectangle at: x={x}, y={y}, w={w}, h={h}")

        # 显示处理后的图像
        #.imshow('Processed Image', frame)
        cv2.waitKey(1)

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

