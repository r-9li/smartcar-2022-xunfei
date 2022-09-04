#!/usr/bin/env python3
# encoding=utf8
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

distance = 100
pub_vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
pub_duration = 11
pub_duration_cof = 0.1
corner_distance_tolerance = 50


def aruco_parking():
    rate_temp = rospy.Rate(20)
    for i in range(0,pub_duration):
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        pub_vel_cmd.publish(vel_msg)
        rate_temp.sleep()


if __name__ == "__main__":
    rospy.init_node('aruco_parking_1', anonymous=True)
    aruco_parking()
    rospy.spin()
