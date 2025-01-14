/*
自主和VCU通讯的CAN节点
接收 自主发送的车辆控制指令，并将指令转化为CAN信号发送
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

controller::VcuCommandC1 m_command; //
void Callback(const controller::VcuCommandC1::ConstPtr& controllerCmd);
void EncodeVcuCanFrame_(__u8* frame, controller::VcuCommandC1* m_VcuControlFlag);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init (argc, argv, "vcu_can_interface");
    ros::NodeHandle nh;
    ros::Subscriber vcuControllercmd_sub = nh.subscribe<controller::VcuCommandC1>("/ow/VcuCommand_cmd", 10, Callback);   
    ros::Rate loop_rate(50);                         
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame Data_frame[4] = {{0}}; //? How to set here
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");//实际上电气的命名是CAN2
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    // IVCU CAN ID
    Data_frame[0].can_id = 0x410;
    Data_frame[0].can_dlc = 0x08;
    
    // YK CAN ID
    Data_frame[1].can_id = 0x411;
    Data_frame[1].can_dlc = 0x08;

    m_command.Vehicle_spd       = 0;    //平均车速需求 km/h
    m_command.Crt_set           = 0;    //转向曲率需求
    m_command.Load_gain         = 0.5;    //载重系数
    m_command.Eng_strt_flg      = 0;    //发动机启停使能 0--停机 1--启动
    m_command.Brake_flg         = 1;    //驻车制动使能 00--驻车松开 01--驻车
    m_command.Skid_steer_flg    = 0;    //原位转向标志位 0--不转 1--转
    m_command.Reset_flg         = 1;    //整车复位标志位 01--正常工作 10--整车复位
    m_command.YK_En             = 0;    //遥控使能 00--遥控不使能 01--遥控使能
    m_command.Skid_steer_degsec = 0;    //原位转向角速度 deg/s
    m_command.Vehicle_model     = 1;    //车辆模式 00--纯电 01--混动 10--水上
    m_command.Light_flg         = 0;    //大灯使能 0--OFF 1--ON
    m_command.Horn_flg          = 0;    //喇叭使能 0--OFF 1--ON
    m_command.Pump_flg          = 0;    //水泵使能 0--OFF 1--ON
    m_command.Left_light_flg    = 1;    //左转灯使能 0--ON 1--OFF
    m_command.Right_light_flg   = 1;    //右转灯使能 0--ON 1--OFF
    m_command.Reserved          = 0;    //置零
    m_command.Cyclic_check      = 0;    //循环校验
    int i = 0;
    while (ros::ok())
    {
        // //暂时在这里添加一个校验计数器
        // if(i % 16 == 0) i = 0;
        // else i++;
        // m_command.Cyclic_check = i;
        
        EncodeVcuCanFrame_(Data_frame[0].data, &m_command);
        nbytes = write(s, &Data_frame[0], sizeof(Data_frame[0]));
        // ROS_INFO("nbytes: %d, sizeof(data)= %ld\n",nbytes,sizeof(Data_frame[0]));
        if(nbytes != sizeof(Data_frame[0]))
        {
            ROS_INFO("发送错误，退出! \n");
            std::cerr << "Error writing to socket: " << strerror(errno) << std::endl;
            break;
        }
        // ROS_INFO("VCU CAN发送成功！ \n");

        ros::spinOnce();
        loop_rate.sleep();
    }
    close(s);
    return 0;
}

void Callback(const controller::VcuCommandC1::ConstPtr& controllerCmd)
{
    ROS_INFO("VCU CAN发送成功！ \n");
    m_command.Vehicle_spd       = controllerCmd->Vehicle_spd;
    m_command.Crt_set           = controllerCmd->Crt_set;
    m_command.Load_gain         = controllerCmd->Load_gain;
    m_command.Eng_strt_flg      = controllerCmd->Eng_strt_flg;
    m_command.Brake_flg         = controllerCmd->Brake_flg;
    m_command.Skid_steer_flg    = controllerCmd->Skid_steer_flg;
    m_command.Reset_flg         = controllerCmd->Reset_flg;
    m_command.YK_En             = controllerCmd->YK_En;
    m_command.Skid_steer_degsec = controllerCmd->Skid_steer_degsec;
    m_command.Vehicle_model     = controllerCmd->Vehicle_model;
    m_command.Light_flg         = controllerCmd->Light_flg;
    m_command.Horn_flg          = controllerCmd->Horn_flg;
    m_command.Pump_flg          = controllerCmd->Pump_flg;
    m_command.Left_light_flg    = controllerCmd->Left_light_flg;
    m_command.Right_light_flg   = controllerCmd->Right_light_flg;
    m_command.Reserved          = controllerCmd->Reserved;
    m_command.Cyclic_check      = controllerCmd->Cyclic_check;
}

void EncodeVcuCanFrame_(__u8* frame, controller::VcuCommandC1* m_VcuControlFlag)
{   
    //除以系数
    //平均目标车速 系数0.1
    unsigned short int Vehicle_spd = (m_VcuControlFlag->Vehicle_spd+60)*10;
    frame[0] = (Vehicle_spd&0B0000000011111111);
    frame[1] = frame[1]&0B00001111;
    frame[1] = (Vehicle_spd&0B0000111100000000)>>8;

    //转向目标曲率 系数0.001
    unsigned short int Crt_set = (m_VcuControlFlag->Crt_set+1)*1000;
    frame[1] = (frame[1]|(Crt_set&0B0000000000001111)<<4);
    frame[2] = (Crt_set&0B0000111111110000)>>4;

    //载重系数 系数0.01
    unsigned short int Load_gain = m_VcuControlFlag->Load_gain*100;
    frame[3] = (Load_gain&0B0000000011111111);

    //发动机启停使能
    if(m_VcuControlFlag->Eng_strt_flg)
    {
        frame[4] = 0B00000001;
    }
    else
    {
        frame[4] = 0B00000000;
    }

    //驻车制动使能
    frame[4] = (frame[4]|(m_VcuControlFlag->Brake_flg&0B00000011)<<1);

    //原位转向标志位
    frame[4] = (frame[4]|(m_VcuControlFlag->Skid_steer_flg&0B00000001)<<3);

    //整车复位标志位
    frame[4] = (frame[4]|(m_VcuControlFlag->Reset_flg&0B00000011)<<4);

    //遥控使能
    frame[4] = (frame[4]|(m_VcuControlFlag->YK_En&0B00000011)<<6);

    //原位转向角速度
    unsigned short int Skid_steer_degsec = (m_VcuControlFlag->Skid_steer_degsec+60);
    frame[5] = (Skid_steer_degsec&0B0000000011111111);
    
    //车辆模式
    frame[6] = (m_VcuControlFlag->Vehicle_model&0B00000011);

    //大灯使能
    frame[6] = (frame[6]|(m_VcuControlFlag->Light_flg&0B00000001)<<2);

    //喇叭使能
    frame[6] = (frame[6]|(m_VcuControlFlag->Horn_flg&0B00000001)<<3);

    //水泵使能
    frame[6] = (frame[6]|(m_VcuControlFlag->Pump_flg&0B00000001)<<4);

    //左转向灯使能
    frame[6] = (frame[6]|(m_VcuControlFlag->Left_light_flg&0B00000001)<<5);

    //右转向灯使能
    frame[6] = (frame[6]|(m_VcuControlFlag->Right_light_flg&0B00000001)<<6);

    //置零 ? 
    frame[6] = (frame[6]|(m_VcuControlFlag->Reserved&0B00000001)<<7);

    //循环校验
    frame[7] = (m_VcuControlFlag->Cyclic_check&0B00001111);
}