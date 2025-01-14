/*
测试接口是否能正常工作。从电脑发送给6x6
*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "controller/VcuCommandC1.h"

controller::VcuCommandC1 m_command; 
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init (argc, argv, "test_pc2veh");
    ros::NodeHandle nh;
    ros::Publisher Vcutest_pub = nh.advertise<controller::VcuCommandC1>("/ow/VcuCommand_cmd",10);
    ros::Rate loop_rate(100); 
    long int i=0;
    while(ros::ok())
    {
        i++;
        if((i % 16)==0) i=0;
        
        // m_command.Vehicle_spd       = 0;    //平均车速需求 km/h
        // m_command.Crt_set           = 0;    //转向曲率需求
        // m_command.Load_gain         = 0.5;  //载重系数
        // m_command.Eng_strt_flg      = 0;    //发动机启停使能 0--停机 1--启动
        // m_command.Brake_flg         = 1;    //驻车制动使能 00--驻车松开 01--驻车
        // m_command.Skid_steer_flg    = 0;    //原位转向标志位 0--不转 1--转
        // m_command.Reset_flg         = 1;    //整车复位标志位 01--正常工作 10--整车复位
        // m_command.YK_En             = 0;    //遥控使能 00--遥控不使能 01--遥控使能
        // m_command.Skid_steer_degsec = 0;    //原位转向角速度 deg/s
        // m_command.Vehicle_model     = 1;    //车辆模式 00--纯电 01--混动 10--水上
        // m_command.Light_flg         = 0;    //大灯使能 0--OFF 1--ON
        // m_command.Horn_flg          = 0;    //喇叭使能 0--OFF 1--ON
        // m_command.Pump_flg          = 0;    //水泵使能 0--OFF 1--ON
        // m_command.Left_light_flg    = 1;    //左转灯使能 0--ON 1--OFF
        // m_command.Right_light_flg   = 1;    //右转灯使能 0--ON 1--OFF
        // m_command.Reserved          = 0;    //置零
        // m_command.Cyclic_check      = i;    //循环校验


        Vcutest_pub.publish(m_command);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("m_Test_order End");
    return 0;
}