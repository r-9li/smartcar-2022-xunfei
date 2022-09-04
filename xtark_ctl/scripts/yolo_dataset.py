#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys, select, termios, tty

save_index = 5500
save_flag = False
save_path = '/home/ucar/yolo_dataset'
settings = None


def callback(data):
    # 通过cvBridge将ROS中图片的格式转换为Opencv中的格式
    global save_flag
    global save_index
    global save_path
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    getKey()
    if save_flag:
        cv2.imwrite('{0}/{1}.jpg'.format(save_path, save_index), frame)
        save_flag = False
        print(save_index)
        save_index += 1


def getKey():
    global save_flag
    global settings
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    if key == 'p':
        save_flag = True


def begin():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    # 创立节点
    rospy.init_node('yolo_dataset', anonymous=True)

    # 订阅usb_cam发出的图像消息，接收到消息后进入回调函数callback()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)

    # 等待
    rospy.spin()


if __name__ == '__main__':
    begin()
