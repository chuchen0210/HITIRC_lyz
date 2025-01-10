#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_publisher():
    # 初始化 ROS 节点
    rospy.init_node('image_publisher', anonymous=True)
    
    # 创建发布者，发布图像消息到 /camera/image_raw 话题
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    
    # 创建 CvBridge 对象，用于转换 OpenCV 图像与 ROS 图像消息之间的格式
    bridge = CvBridge()
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)  # 默认摄像头
    
    if not cap.isOpened():
        rospy.logerr("无法打开摄像头")
        return

    rate = rospy.Rate(10)  # 设置发布频率为 10Hz

    while not rospy.is_shutdown():
        # 读取一帧图像
        ret, frame = cap.read()
        if ret:
            # 将 OpenCV 图像转换为 ROS 图像消息
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 发布图像消息
            image_pub.publish(img_msg)
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
