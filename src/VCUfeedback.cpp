/*
接受VCU信号，并解析后发布
*/

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
#include "controller/VcuFeedbackC1.h"
#include "controller/error_map.hpp"

#define VEHICLE_STATE_FEEDBACK ((uint32_t)0x400)
#define VEHICLE_POWER_FEEDBACK ((uint32_t)0x401)
controller::VcuFeedbackC1 m_VcuFeedbackC1;

typedef struct                          //VCU 向 IVCU汇报
{
    unsigned int HBMSOC;               //动力电池电量
    unsigned char Pbrk_act;             //驻车制动状态
    bool Vehicle_model;                 //车辆模式 0--有人 1--无人
    unsigned char Cyclic_check;         //0-15循环校验
    unsigned int HBM_voltage;          //动力电池电压
    short Water_temp;           //发动机水温
    float Vehicle_speed_Kmh;            //车速
    unsigned char Gear_position;        //挡位
    unsigned char Sysflt;               //车辆故障码 0x01-P;2-R;3-N;4-D;5-WP;6-WD
    unsigned char Vehicle_state;        //车辆状态码

    unsigned int Whl_spd_l_rpm;   //左电机转速
    unsigned int Whl_spd_r_rpm;   //右电机转速
    unsigned int Eng_spd_rpm;     //发动机转速
    unsigned char EHB_actual_pedal;     //当前制动踏板开度
    unsigned int Pump_spd_rpm;         //水泵电机转速
    uint8_t raw[8];
} VCU_STATE_FEEDBACK;

bool DecodeVcuMsgFromCAN_(const struct can_frame *rx_frame, VCU_STATE_FEEDBACK *data);
void Decode_Vehicle_State_(VCU_STATE_FEEDBACK *data);
void Decode_Vehicle_Power_(VCU_STATE_FEEDBACK *data);
void Display_state(VCU_STATE_FEEDBACK *data);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "vcu_C1_interface");
    ros::NodeHandle node_handle;
    ros::Publisher VcuFeedback_pubC1 = node_handle.advertise<controller::VcuFeedbackC1>("/ow/vcuFeedback",10);

    ros::Rate loop_rate(100);
    VCU_STATE_FEEDBACK m_data;

    int s_receive, nbytes_receive;
    int buffer_zone = 32768;
    struct sockaddr_can addr_receive;
    struct ifreq ifr_receive;
    struct can_frame frame_receive;
    struct can_filter rfilter_receive[2];//滤波器

    s_receive = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr_receive.ifr_name, "can0");
    ioctl(s_receive, SIOCGIFINDEX, &ifr_receive);//指定can设备
    addr_receive.can_family = AF_CAN;
    addr_receive.can_ifindex = ifr_receive.ifr_ifindex;
    bind(s_receive, (struct sockaddr *)&addr_receive, sizeof(addr_receive));// 绑定socket

    //指定接收的CAN ID，并使用标准帧过滤器掩码
    rfilter_receive[0].can_id = VEHICLE_STATE_FEEDBACK;
    rfilter_receive[0].can_mask = CAN_EFF_MASK;
    rfilter_receive[1].can_id = VEHICLE_POWER_FEEDBACK;
    rfilter_receive[1].can_mask = CAN_EFF_MASK;
    
    setsockopt(s_receive, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_receive, sizeof(rfilter_receive));
    ROS_INFO("m_VcuFeedback start");

    while(ros::ok())
    {   
        // ROS_INFO("Fine");

        nbytes_receive = read(s_receive, &frame_receive, sizeof(frame_receive));    //接收

        if(nbytes_receive > 0)
        {   
            
            DecodeVcuMsgFromCAN_(&frame_receive, &m_data);
            m_VcuFeedbackC1.HBMSOC              = m_data.HBMSOC;
            m_VcuFeedbackC1.Vehicle_model       = m_data.Vehicle_model;
            m_VcuFeedbackC1.Pbrk_act            = m_data.Pbrk_act;
            m_VcuFeedbackC1.Cyclic_check        = m_data.Cyclic_check;
            m_VcuFeedbackC1.HBM_voltage         = m_data.HBM_voltage;
            m_VcuFeedbackC1.Water_temp          = m_data.Water_temp;
            m_VcuFeedbackC1.Vehicle_speed_Kmh   = m_data.Vehicle_speed_Kmh;
            m_VcuFeedbackC1.Gear_position       = m_data.Gear_position;
            m_VcuFeedbackC1.Sysflt              = m_data.Sysflt;
            m_VcuFeedbackC1.Vehicle_state       = m_data.Vehicle_state;

            m_VcuFeedbackC1.Whl_spd_l_rpm       = m_data.Whl_spd_l_rpm;
            m_VcuFeedbackC1.Whl_spd_r_rpm       = m_data.Whl_spd_r_rpm;
            m_VcuFeedbackC1.Eng_spd_rpm         = m_data.Eng_spd_rpm;
            m_VcuFeedbackC1.EHB_actual_pedal    = m_data.EHB_actual_pedal;
            m_VcuFeedbackC1.Pump_spd_rpm        = m_data.Pump_spd_rpm;

            Display_state(&m_data);
        }


        VcuFeedback_pubC1.publish(m_VcuFeedbackC1);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("m_VcuFeedback End");
    close(s_receive);
    return 0;
}

bool DecodeVcuMsgFromCAN_(const struct can_frame *rx_frame, VCU_STATE_FEEDBACK *data)
{
    memcpy(data->raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
    ROS_INFO("CAN_ID: %x",rx_frame->can_id);
    switch (rx_frame->can_id)
    {
        case VEHICLE_STATE_FEEDBACK:    //车辆状态
        {
            ROS_INFO("车辆状态-----------------------------------------------------------------------");
            Decode_Vehicle_State_(data);
            break;
        }
        case VEHICLE_POWER_FEEDBACK:    //车辆动力
        {
            ROS_INFO("车辆动力-----------------------------------------------------------------------");
            Decode_Vehicle_Power_(data);
            break;
        }

        default:
            break;
    }
    return true;
}

void Decode_Vehicle_State_(VCU_STATE_FEEDBACK *data) //解析车辆状态
{
    unsigned char frame[8];
    memcpy(frame, &(data->raw[0]), 8*sizeof(uint8_t));

    data->HBMSOC = (frame[0]&0B11111111);               //动力电池电量

    data->Pbrk_act = (frame[1]&0B00000011);             //驻车制动状态
    data->Vehicle_model = (frame[1]&0B00000100)>>2;     //车辆模式
    data->Cyclic_check = (frame[1]&0B11110000)>>4;      //校验

    data->HBM_voltage = (frame[2]&0B11111111)*5;          //动力电池电压

    data->Water_temp = (frame[3]&0B11111111)-50;        //发动机水温

    data->Vehicle_speed_Kmh = ((frame[4]&0B11111111) + ((frame[5]&0B00001111)*256))*0.1-60; //车速
    data->Gear_position = (frame[5]&0B11110000)>>4;     //挡位

    data->Sysflt = (frame[6]&0B11111111);               //车辆故障码

    data->Vehicle_state = (frame[7]&0B11111111);        //车辆状态码
    return;
}

void Decode_Vehicle_Power_(VCU_STATE_FEEDBACK *data) //解析车辆发动机状态
{
    unsigned char frame[8];
    memcpy(frame, &(data->raw[0]), 8*sizeof(uint8_t));
    data->Whl_spd_l_rpm = ((frame[0]&0B11111111) + ((frame[1]&0B11111111)*256) - 10000);    //左电机转速
    data->Whl_spd_r_rpm = ((frame[2]&0B11111111) + ((frame[3]&0B11111111)*256) - 10000);    //右电机转速
    data->Eng_spd_rpm = ((frame[4]&0B11111111) + ((frame[5]&0B11111111)*256));      //发动机转速
    data->EHB_actual_pedal = (frame[6]&0B11111111);                                         //当前制动踏板开度
    data->Pump_spd_rpm = (frame[7]&0B11111111)*50;                                          //水泵电机转速

    return;
}

void Display_state(VCU_STATE_FEEDBACK *data)
{
    printf("车速: %4.1f km/h \n \n",data->Vehicle_speed_Kmh);
    printf("电池电量:    %3d %% ||", data->HBMSOC);
    printf("动力电池电压: %4d V ||", data->HBM_voltage);
    printf("发动机水温:   %4d °C || \n \n", data->Water_temp);
    
    int pbrk = data->Pbrk_act;
    if(pbrk == 1)
        printf("驻车制动状态: 未驻车 ||");
    else if(pbrk == 2)
        printf("驻车制动状态:   制动 ||");
    else
        printf("驻车制动状态:数据错误 ||");

    int vehicle_model = data->Vehicle_model;
    if(vehicle_model == 0)
        printf("车辆模式:   有人 ||");
    else if(vehicle_model == 1)
        printf("车辆模式:   无人 ||");
    else
        printf("车辆模式:数据错误 ||");

    int gear_position = data->Gear_position;
    if(gear_position == 1)
        printf("挡位:   P档 ||");
    else if(gear_position == 2)
        printf("挡位:   R档 ||");
    else if(gear_position == 3)
        printf("挡位:   N档 ||");
    else if(gear_position == 4)
        printf("挡位:   D档 ||");
    else if(gear_position == 5)
        printf("挡位:  WP档 ||");
    else if(gear_position == 6)
        printf("挡位:  WD档 ||");
    else
        printf("挡位:数据错误 ||");
    
    printf("\n \n");

//-------------车辆故障码------------------
    std::map<int, std::string>::iterator iter;
    iter = error_code_6x6.find(data->Sysflt);
    if(iter != error_code_6x6.end())
    {
        printf("车辆故障码: %4d & 故障类型: %s || ",data->Sysflt, iter->second.c_str());
    }

    printf("车辆状态码: %4d || ", data->Vehicle_state);
    printf("校验码: %4d ||", data->Cyclic_check);

    printf("\n \n");

    printf("左电机转速: %6d RPM ||", data->Whl_spd_l_rpm);
    printf("右电机转速: %6d RPM ||", data->Whl_spd_r_rpm);
    printf("发动机转速: %6d RPM ||", data->Eng_spd_rpm);

    printf("\n \n");

    printf("当前制动踏板开度: %3d %% ||", data->EHB_actual_pedal);
    printf("水泵电机转速: %6d RPM ||", data->Pump_spd_rpm);

    printf("\n \n");
}