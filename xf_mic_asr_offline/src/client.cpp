/*
 * client.cpp
 */
#include "user_interface.h"
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>
#include <xf_mic_asr_offline/Set_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Set_Led_On_srv.h>
#include <xf_mic_asr_offline/Get_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <xf_mic_asr_offline/Get_Awake_Angle_srv.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <iostream>

using namespace std;

int awake_angle = -1;
ros::Publisher my_pub;

void awake_angle_Callback(std_msgs::Int32 msg)
{
    awake_angle = msg.data;
    std_msgs::Int8 my_wakeup_result;
    my_wakeup_result.data = 1;
    my_pub.publish(my_wakeup_result);
    ROS_INFO("Wake up!");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    /*离线命令词识别*/
    ros::ServiceClient get_offline_recognise_result_client = 
    nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("xf_asr_offline_node/get_offline_recognise_result_srv");
    /*设置主麦克风*/
    ros::ServiceClient Set_Major_Mic_client =
    nh.serviceClient<xf_mic_asr_offline::Set_Major_Mic_srv>("xf_asr_offline_node/set_major_mic_srv");
    /*获取主麦克风*/
    ros::ServiceClient Get_Major_Mic_client =
    nh.serviceClient<xf_mic_asr_offline::Get_Major_Mic_srv>("xf_asr_offline_node/get_major_mic_srv");
    /*修改唤醒词*/
    ros::ServiceClient Set_Awake_Word_client =
    nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("xf_asr_offline_node/set_awake_word_srv");
    /*订阅唤醒角度*/
    ros::Subscriber awake_angle_sub = nh.subscribe("/mic/awake/angle", 1, awake_angle_Callback);
    /*发布已唤醒消息*/
    my_pub = nh.advertise<std_msgs::Int8>("/my_wakeup", 1);   //0为未唤醒，1为已唤醒
    
    xf_mic_asr_offline::Get_Offline_Result_srv GetOfflineResult_srv;
    xf_mic_asr_offline::Set_Major_Mic_srv SetMajorMic_srv;
    xf_mic_asr_offline::Get_Major_Mic_srv GetMajorMic_srv;
    xf_mic_asr_offline::Set_Awake_Word_srv SetAwakeWord_srv;

    std::string word = "现在出发";
    int recognize_fail_count;
    int recognize_fail_count_threshold;
    int confidence_threshold;
    int time_per_order;


    sleep(2);

    SetAwakeWord_srv.request.awake_word = word;
    if (Set_Awake_Word_client.call(SetAwakeWord_srv))
    {
        ROS_INFO("succeed to call service");
        std::cout << SetAwakeWord_srv.response << endl;
    }
    else
	{
        ROS_INFO("failed to call service");
    }

    ros::spin();

    return 0;
}
