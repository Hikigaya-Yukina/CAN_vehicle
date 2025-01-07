float32 Vehicle_spd           #平均车速需求 km/h
float32 Crt_set               #转向曲率需求
float32 Load_gain           #载重系数
uint8 Eng_strt_flg         #发动机启停使能 0--停机 1--启动
uint8 Brake_flg            #驻车制动使能 00--驻车松开 01--驻车
bool Skid_steer_flg        #原位转向标志位 0--不转 1--转
uint8 Reset_flg            #整车复位标志位 01--正常工作 10--整车复位
uint8 YK_En                #遥控使能 00--遥控不使能 01--遥控使能
float32 Skid_steer_degsec   #原位转向角速度 deg/s
uint8 Vehicle_model        #车辆模式 00--纯电 01--混动 10--水上
bool Light_flg             #大灯使能 0--OFF 1--ON
bool Horn_flg              #喇叭使能 0--OFF 1--ON
bool Pump_flg              #水泵使能 0--OFF 1--ON
bool Left_light_flg        #左转灯使能 0--ON 1--OFF
bool Right_light_flg       #右转灯使能 0--ON 1--OFF
uint8 Reserved             #置零
uint8 Cyclic_check         #循环校验
