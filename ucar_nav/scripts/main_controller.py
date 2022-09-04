#!/usr/bin/env python3
# encoding=utf8
import time
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import roslib
import rospy
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import os
import random

###############################全局变量#################################################
step_flag = 0  # 1. 初始化位姿 2. 语音唤醒 3. 运动到一号导航点
current_angle_q = None  # 小车目前朝向(四元数形式)
angle_speed = 3.0  # 旋转速度
image_temp_dir = '/home/ucar/ucar_ws/src/ucar_nav/image_temp'  # 图片临时保存的地址
image_degree_tolerance = 5  # 拍照片角度误差
current_position = None  # 小车里程计当前位置
position_tolerance = 0.03  # 小车移动误差允许范围，单位为米
position_move_speed = 0.3  # 小车移动速度
#######################################################################################

###############################坐标数据#################################################
origin_pose = [-0.0376743040979, 0.105794891715, -0.999978635214,
               0.00653675112863]  # 改这个数组可以调整初始点的位置和车头方向；简化定义，使用方法参加auto_back
goal_pose_1 = [-5.00160264969, 3.97838020325, 0.999737501986, 0.0229112881094]
goal_pose_2 = [-2.98661851883, 4.84302425385, 0.730525525812, 0.682885390192]
goal_pose_3 = [-0.957392990589, 4.70927047729, 0.702809065952, 0.711378532721]
goal_pose_4 = [-0.0281128, 0.8121339, -0.016900325, 0.999857179305]
goal_pose_5 = [0.094, 0.879, -0.016900325, 0.999857179305]  # 准但是慢
angle_list_1 = [0, 50, 150]
angle_list_2 = [0, 60, 140, 225]
angle_list_3 = [0, 65, 180, 250]

parking_model = 2
pub_duration = 10

room_type_list = []  # 记录房间类型的列表

can_ting = [7, 0, 6, 4]
wo_shi = [4, 1, 2, 7]
ke_ting = [5, 1, 7, 3]

special_can_ting = [0, 6]
special_wo_shi = [2]
special_ke_ting = [5, 3]
room_score = []  # 记录房间得分的列表
different_room = False  # 三个房间不一样

room_param_dict = {'/move_base/TebLocalPlannerROS/acc_lim_x': 0.5, '/move_base/TebLocalPlannerROS/acc_lim_y': 0.5,
                   '/move_base/TebLocalPlannerROS/acc_lim_theta': 2.5, '/move_base/TebLocalPlannerROS/max_vel_x': 2.0,
                   '/move_base/TebLocalPlannerROS/max_vel_y': 2.0, '/move_base/TebLocalPlannerROS/max_vel_theta': 3,
                   '/move_base/TebLocalPlannerROS/force_reinit_new_goal_dist': 3.0,
                   '/move_base/TebLocalPlannerROS/penalty_epsilon': 0.16,
                   '/move_base/TebLocalPlannerROS/weight_kinematics_nh': 500.0,
                   '/move_base/TebLocalPlannerROS/weight_kinematics_forward_drive': 500.0,
                   '/move_base/TebLocalPlannerROS/weight_kinematics_turning_radius': 1.0,
                   '/move_base/TebLocalPlannerROS/weight_optimaltime': 45,
                   '/move_base/TebLocalPlannerROS/weight_viapoint': 50.0}

parking_move = [0.15, -0.85]


#####################################################################################
def open_terminal(commands):
    """
    打开一个新的终端并执行命令
    未作参数检查，不要试图在传入的命令字符串中添加多余的引号，可能会引发错误
    """
    cmd_list = []
    for cmd in commands:
        cmd_list.append(""" gnome-terminal --tab -e "bash -c '%s;exec bash'" >/dev/null  2>&1 """ % cmd)

    os.system(';'.join(cmd_list))


def change_param(param_dict):
    for key, value in param_dict.items():
        rospy.set_param(key, value)
    rospy.sleep(0.15)
    temp_Int8 = Int8()
    temp_Int8.data = 1
    pub_teb_reconfigure.publish(temp_Int8)


def search_room_type(object_list):
    global room_type_list
    global room_score
    can_ting_number = 0
    wo_shi_number = 0
    ke_ting_number = 0
    object_list = list(set(object_list))
    for i in object_list:
        if i in can_ting:
            if i in special_can_ting:
                can_ting_number += 100
            else:
                can_ting_number += 1
        if i in wo_shi:
            if i in special_wo_shi:
                wo_shi_number += 100
            else:
                wo_shi_number += 1
        if i in ke_ting:
            if i in special_ke_ting:
                ke_ting_number += 100
            else:
                ke_ting_number += 1
    if can_ting_number == wo_shi_number == ke_ting_number:
        room_score.append(can_ting_number)
        random_int = random.randint(0, 2)
        if random_int == 0:
            room_type_list.append("餐厅")
        elif random_int == 1:
            room_type_list.append("卧室")
        elif random_int == 2:
            room_type_list.append("客厅")
    else:
        my_temp_list = [can_ting_number, wo_shi_number, ke_ting_number]
        my_temp_list.sort(reverse=True)
        room_score.append(my_temp_list[0])
        if my_temp_list.count(my_temp_list[0]) == 2:
            random_int = random.randint(0, 1)
            if can_ting_number == wo_shi_number:
                if random_int == 0:
                    room_type_list.append("餐厅")
                elif random_int == 1:
                    room_type_list.append("卧室")
            elif can_ting_number == ke_ting_number:
                if random_int == 0:
                    room_type_list.append("餐厅")
                elif random_int == 1:
                    room_type_list.append("客厅")
            elif wo_shi_number == ke_ting_number:
                if random_int == 0:
                    room_type_list.append("卧室")
                elif random_int == 1:
                    room_type_list.append("客厅")
        elif my_temp_list.count(my_temp_list[0]) == 1:
            if can_ting_number == my_temp_list[0]:
                room_type_list.append("餐厅")
            elif wo_shi_number == my_temp_list[0]:
                room_type_list.append("卧室")
            elif ke_ting_number == my_temp_list[0]:
                room_type_list.append("客厅")


def change_room():
    global room_type_list
    global room_score
    temp_list_change_room = ["餐厅", "卧室", "客厅"]
    if room_type_list[0] == room_type_list[1] == room_type_list[2]:
        room_number_list = [0, 1, 2]
        temp_socre = room_score
        temp_socre.sort(reverse=True)
        max_number = room_score.index(temp_socre[0])
        room_number_list.remove(max_number)
        temp_list_change_room.remove(room_type_list[max_number])
        room_type_list[room_number_list[0]] = temp_list_change_room[0]
        room_type_list[room_number_list[1]] = temp_list_change_room[1]
    else:
        if room_type_list[0] == room_type_list[1]:
            if room_score[1] > room_score[0]:
                temp_list_change_room.remove(room_type_list[1])
                temp_list_change_room.remove(room_type_list[2])
                room_type_list[0] = temp_list_change_room[0]
            else:
                temp_list_change_room.remove(room_type_list[0])
                temp_list_change_room.remove(room_type_list[2])
                room_type_list[1] = temp_list_change_room[0]
        elif room_type_list[0] == room_type_list[2]:
            if room_score[2] > room_score[0]:
                temp_list_change_room.remove(room_type_list[2])
                temp_list_change_room.remove(room_type_list[1])
                room_type_list[0] = temp_list_change_room[0]
            else:
                temp_list_change_room.remove(room_type_list[0])
                temp_list_change_room.remove(room_type_list[1])
                room_type_list[2] = temp_list_change_room[0]
        elif room_type_list[1] == room_type_list[2]:
            if room_score[2] > room_score[1]:
                temp_list_change_room.remove(room_type_list[2])
                temp_list_change_room.remove(room_type_list[0])
                room_type_list[1] = temp_list_change_room[0]
            else:
                temp_list_change_room.remove(room_type_list[1])
                temp_list_change_room.remove(room_type_list[0])
                room_type_list[2] = temp_list_change_room[0]


def callback_one(my_data):
    global step_flag
    if step_flag == 1 and my_data.data == 1:
        step_flag = 2


def callback_two(my_data):
    global step_flag
    if step_flag == 10:
        temp_list = []
        temp_string = my_data.data
        for ch in temp_string:
            if ch == ',':
                search_room_type(temp_list)
                temp_list = []
                continue
            temp_list.append(int(ch))
        if different_room:
            change_room()
        step_flag = 11


def callback_three(odom_data):
    global current_angle_q
    global current_position
    current_angle_q = odom_data.pose.pose.orientation
    current_position = odom_data.pose.pose.position


def get_angle_now():  # 节省计算量
    global current_angle_q
    temp_angle = euler_from_quaternion([current_angle_q.x, current_angle_q.y, current_angle_q.z, current_angle_q.w])[2]
    temp_angle = math.degrees(temp_angle)
    return temp_angle


def postion_move(target_x, target_y):
    global current_position
    global position_tolerance
    global position_move_speed
    target_x += current_position.x
    target_y += current_position.y
    while abs(target_x - current_position.x) > position_tolerance or abs(
            target_y - current_position.y) > position_tolerance:
        vel_msg = Twist()
        if abs(target_x - current_position.x) > position_tolerance:
            temp_speed = (target_x - current_position.x) * 10000
            if temp_speed < -position_move_speed:
                temp_speed = -position_move_speed
            elif temp_speed > position_move_speed:
                temp_speed = position_move_speed
            vel_msg.linear.x = temp_speed
        if abs(target_y - current_position.y) > position_tolerance:
            temp_speed1 = (target_y - current_position.y) * 10000
            if temp_speed1 < -position_move_speed:
                temp_speed1 = -position_move_speed
            elif temp_speed1 > position_move_speed:
                temp_speed1 = position_move_speed
            vel_msg.linear.y = temp_speed1
        pub_vel_cmd.publish(vel_msg)


def init():
    global step_flag
    for i in range(0, 9):
        init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = origin_pose[0]
        pose_msg.pose.pose.position.y = origin_pose[1]
        pose_msg.pose.covariance[0] = 0.25  # 协方差矩阵，不用管
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942
        pose_msg.pose.pose.orientation.z = origin_pose[2]
        pose_msg.pose.pose.orientation.w = origin_pose[3]
        init_pub.publish(pose_msg)
        rospy.sleep(1)
    rospy.loginfo("init ok")
    str_temp = [
        "rosservice call /xf_mic_tts_offline_node/play_txt_wav " + "\"" + "初始化成功" + '\"' + ' ' + '\"' + "xiaofeng" + '\"']
    open_terminal(str_temp)
    step_flag = 1


def photo_image(position_number, angle_list):  # odom的角度是在-180到180范围内的,单位为弧度
    global image_temp_dir
    global image_degree_tolerance
    global angle_speed
    n = len(angle_list)
    begin_angle = get_angle_now()
    bridge = CvBridge()
    for i in range(n):
        target_angle = begin_angle + angle_list[i]
        if target_angle > 180:
            target_angle -= 360
        elif target_angle < -180:
            target_angle += 360
        while abs(target_angle - get_angle_now()) > image_degree_tolerance:
            vel_msg = Twist()
            vel_msg.angular.z = angle_speed
            pub_vel_cmd.publish(vel_msg)
        if angle_list[i] != 0:
            rospy.sleep(0.45)
        image_temp = rospy.wait_for_message('usb_cam/image_raw', Image)
        image_temp = bridge.imgmsg_to_cv2(image_temp, "bgr8")
        cv2.imwrite('{0}/{1}_{2}.jpg'.format(image_temp_dir, position_number, i), image_temp)


def control_function():
    global step_flag
    global room_param_dict
    global parking_move
    while True:
        if step_flag == 2:  # 一号点
            client_one = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = "map"
            goal_pose.target_pose.header.stamp = rospy.Time.now()
            goal_pose.target_pose.pose.position.x = goal_pose_1[0]
            goal_pose.target_pose.pose.position.y = goal_pose_1[1]
            goal_pose.target_pose.pose.orientation.z = goal_pose_1[2]
            goal_pose.target_pose.pose.orientation.w = goal_pose_1[3]
            client_one.wait_for_server()
            client_one.send_goal(goal_pose)
            client_one.wait_for_result()
            if client_one.get_state() == GoalStatus.SUCCEEDED:
                step_flag = 3
        elif step_flag == 3:
            photo_image(1, angle_list_1)
            step_flag = 4
        elif step_flag == 4:  # 二号点
            client_one = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = "map"
            goal_pose.target_pose.header.stamp = rospy.Time.now()
            goal_pose.target_pose.pose.position.x = goal_pose_2[0]
            goal_pose.target_pose.pose.position.y = goal_pose_2[1]
            goal_pose.target_pose.pose.orientation.z = goal_pose_2[2]
            goal_pose.target_pose.pose.orientation.w = goal_pose_2[3]
            client_one.wait_for_server()
            client_one.send_goal(goal_pose)
            client_one.wait_for_result()
            if client_one.get_state() == GoalStatus.SUCCEEDED:
                step_flag = 5
        elif step_flag == 5:
            photo_image(2, angle_list_2)
            step_flag = 6
        elif step_flag == 6:  # 三号点
            client_one = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = "map"
            goal_pose.target_pose.header.stamp = rospy.Time.now()
            goal_pose.target_pose.pose.position.x = goal_pose_3[0]
            goal_pose.target_pose.pose.position.y = goal_pose_3[1]
            goal_pose.target_pose.pose.orientation.z = goal_pose_3[2]
            goal_pose.target_pose.pose.orientation.w = goal_pose_3[3]
            client_one.wait_for_server()
            client_one.send_goal(goal_pose)
            client_one.wait_for_result()
            if client_one.get_state() == GoalStatus.SUCCEEDED:
                step_flag = 7
        elif step_flag == 7:
            photo_image(3, angle_list_3)
            step_flag = 8
        elif step_flag == 8:  # 四号点
            client_one = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = "map"
            goal_pose.target_pose.header.stamp = rospy.Time.now()
            if parking_model == 1:
                goal_pose.target_pose.pose.position.x = goal_pose_5[0]
                goal_pose.target_pose.pose.position.y = goal_pose_5[1]
                goal_pose.target_pose.pose.orientation.z = goal_pose_5[2]
                goal_pose.target_pose.pose.orientation.w = goal_pose_5[3]
            elif parking_model == 2:
                goal_pose.target_pose.pose.position.x = goal_pose_4[0]
                goal_pose.target_pose.pose.position.y = goal_pose_4[1]
                goal_pose.target_pose.pose.orientation.z = goal_pose_4[2]
                goal_pose.target_pose.pose.orientation.w = goal_pose_4[3]
            client_one.wait_for_server()
            client_one.send_goal(goal_pose)
            client_one.wait_for_result()
            if client_one.get_state() == GoalStatus.SUCCEEDED:
                if parking_model == 2:
                    rate_temp = rospy.Rate(20)
                    for i in range(0, pub_duration):
                        vel_msg = Twist()
                        vel_msg.linear.x = 0.2
                        pub_vel_cmd.publish(vel_msg)
                        rate_temp.sleep()
                step_flag = 9
        elif step_flag == 9:  # 识别指令
            temp_Int8 = Int8()
            temp_Int8.data = 1
            pub_detect.publish(temp_Int8)
            step_flag = 10
        elif step_flag == 11:  # 播报指令
            str_temp = [
                "rosservice call /xf_mic_tts_offline_node/play_txt_wav " + "\"" + "任务完成,B房间为" + room_type_list[
                    0] + ",C房间为" + room_type_list[1] + ",D房间为" + room_type_list[
                    2] + '\"' + ' ' + '\"' + "xiaofeng" + '\"']
            open_terminal(str_temp)
            step_flag = 12


if __name__ == "__main__":
    rospy.init_node('main_controller', anonymous=True)
    rospy.Subscriber("/my_wakeup", Int8, callback_one)
    rospy.Subscriber("/detect", String, callback_two)
    rospy.Subscriber("/odom", Odometry, callback_three)
    pub_vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    pub_detect = rospy.Publisher("/yolo_detect", Int8, queue_size=10)
    pub_teb_reconfigure = rospy.Publisher("/move_base/TebLocalPlannerROS/teb_reconfig", Int8, queue_size=10)
    init()
    control_function()
    rospy.spin()
