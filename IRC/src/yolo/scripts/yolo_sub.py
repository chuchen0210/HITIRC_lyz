#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    label, x_center, y_center = msg.data.split(",")
    rospy.loginfo(f"检测到: {label}, 中心坐标: ({x_center}, {y_center})")

rospy.init_node("yolo_subscriber", anonymous=True)
rospy.Subscriber("/yolo/bounding_boxes", String, callback)
rospy.spin()

