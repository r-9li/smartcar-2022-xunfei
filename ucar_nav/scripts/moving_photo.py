#!/usr/bin/env python3
# encoding=utf8
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf


class photo_position:
    def __init__(self, position_x, position_y, radius, photo_number, room_number):
        self.position_x = position_x
        self.position_y = position_y
        self.radius = radius
        self.photo_number = photo_number
        self.room_number = room_number


##############################################全局变量##############################

photo_list = []
photo_list.append(photo_position(-4.4597439, 2.61673235, 0.3, 2, 1))  # 一房间一号点位
photo_list.append(photo_position(-4.68511676788, 3.4327545166, 0.35, 2, 1))  # 一房间二号点位
photo_list.append(photo_position(-3.0136568, 2.92768287, 0.3, 2, 2))  # 二房间一号点位
photo_list.append(photo_position(-1.79651737, 2.51415872, 0.3, 2, 3))  # 三房间一号点位
photo_list.append(photo_position(-1.2300416, 3.75481104, 0.35, 2, 3))  # 三房间二号点位
image_temp_dir = '/home/ucar/ucar_ws/src/ucar_nav/image_temp'


##################################################################################
def photo_when_moving():
    bridge = CvBridge()
    while True:
        rospy.sleep(0.3)
        (trans, _) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time())
        for position in photo_list:
            if ((trans[0] - position.position_x) ** 2 + (
                    trans[1] - position.position_y) ** 2) ** 0.5 < position.radius and position.photo_number > 0:
                image_temp = rospy.wait_for_message('usb_cam/image_raw', Image)
                image_temp = bridge.imgmsg_to_cv2(image_temp, "bgr8")
                cv2.imwrite('{0}/{1}__{2}.jpg'.format(image_temp_dir, position.room_number, position.room_number),
                            image_temp)
                position.photo_number -= 1
                rospy.sleep(0.15)


if __name__ == "__main__":
    rospy.init_node('moving_photo', anonymous=True)
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(3.0))
    photo_when_moving()
    rospy.spin()
