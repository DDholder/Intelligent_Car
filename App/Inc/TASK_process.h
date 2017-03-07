#ifndef _TASK_PROCESS_H_
#define _TASK_PROCESS_H_

#define    ANGLE_CONTROL_OUT_MAX      ( 9800)
#define    ANGLE_CONTROL_OUT_MIN      (-9800)

#define    OPTICAL_ENCODE_CONSTANT    (256)                                                    //编码器的刻槽数量
#define    SPEED_CONTROL_PEIORD       (100)                                                    //速度控制周期，ms
#define    CAR_SPEED_CONSTANT         (8824/SPEED_CONTROL_PEIORD/OPTICAL_ENCODE_CONSTANT )     //  单位转化比例值
//轮子周长20cm  轮胎齿数68 编码器齿数30 电机齿数16 

#define    DIRECTION_CONTROL_PERIOD   (2)
/*--------------------------速度控制参数外部声明---------------------------*/
extern float  CarSpeed;                                                   //电机速度设定
extern float  Old_CarSpeed;
extern float  CarSpeed_Exert;      //期望速度
extern float  Begin_CarSpeed;   //起跑速度  
extern float  Curve_Carspeed;   //弯道速度  
extern float  SPEED_CONTROL_P ;    //100             //这里是速度控制P
extern float  SPEED_CONTROL_I ;     //2
extern float  SPEED_CONTROL_D ;

extern float DIRECTION_CONTROL_P ;
extern float DIRECTION_CONTROL_D ;
extern float LeftMotorOut ;          //电机左右输出变量
extern float RightMotorOut ;
extern float  String_Angle_P ;              // 串级角度
extern float  String_Angle_I ;
extern float  String_Angle_D ;
extern float  String_Gyro_P ;                //串级角速度      
extern float  String_Gyro_I ;            //0.001
extern float  String_Gyro_D ;                 //2

extern float  CarAngle_Set;           //角度设置
extern float  AngleControlOut;
extern float  SpeedControlOut;
extern void  Car_Control_Process(void);
extern void  AngleControl(void);
extern void  MotorOutput(void);
extern void DirectionControl(void);
extern void DirectionControlOutput(void);
extern void SpeedControl(void);
extern void SpeedControlOutput(void);

#endif