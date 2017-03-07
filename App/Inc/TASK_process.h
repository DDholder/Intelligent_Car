#ifndef _TASK_PROCESS_H_
#define _TASK_PROCESS_H_

#define    ANGLE_CONTROL_OUT_MAX      ( 9800)
#define    ANGLE_CONTROL_OUT_MIN      (-9800)

#define    OPTICAL_ENCODE_CONSTANT    (256)                                                    //�������Ŀ̲�����
#define    SPEED_CONTROL_PEIORD       (100)                                                    //�ٶȿ������ڣ�ms
#define    CAR_SPEED_CONSTANT         (8824/SPEED_CONTROL_PEIORD/OPTICAL_ENCODE_CONSTANT )     //  ��λת������ֵ
//�����ܳ�20cm  ��̥����68 ����������30 �������16 

#define    DIRECTION_CONTROL_PERIOD   (2)
/*--------------------------�ٶȿ��Ʋ����ⲿ����---------------------------*/
extern float  CarSpeed;                                                   //����ٶ��趨
extern float  Old_CarSpeed;
extern float  CarSpeed_Exert;      //�����ٶ�
extern float  Begin_CarSpeed;   //�����ٶ�  
extern float  Curve_Carspeed;   //����ٶ�  
extern float  SPEED_CONTROL_P ;    //100             //�������ٶȿ���P
extern float  SPEED_CONTROL_I ;     //2
extern float  SPEED_CONTROL_D ;

extern float DIRECTION_CONTROL_P ;
extern float DIRECTION_CONTROL_D ;
extern float LeftMotorOut ;          //��������������
extern float RightMotorOut ;
extern float  String_Angle_P ;              // �����Ƕ�
extern float  String_Angle_I ;
extern float  String_Angle_D ;
extern float  String_Gyro_P ;                //�������ٶ�      
extern float  String_Gyro_I ;            //0.001
extern float  String_Gyro_D ;                 //2

extern float  CarAngle_Set;           //�Ƕ�����
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