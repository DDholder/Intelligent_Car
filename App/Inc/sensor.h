#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "common.h"
#define  qinghua  0
#define  kaerman  1
#define  MPU3050_USED   //ʹ��3050  
//MPU6050_USED    //ʹ��6050

#ifdef MPU3050_USED
/*���Ͷ���*/
typedef struct
{
	float X_GYRO;   //X����ٶ�
	float Y_GYRO;   //Y����ٶ�
	float Z_ACC;    //Z��Ƕ�
}Sensor_Type;
extern float CarAngle;                           //��ģ���
extern float Angular_Velocity_Y;				 //Y����ٶ� 
extern float point_center;						 //ƫ��
extern float Kalman_Filter(float angle_m, float gyro_m);
extern Sensor_Type Sensor_Value;                     //������ֵ
extern Sensor_Type Sensor_Offset;                    //������ƫ��ֵ  
extern float Q_angle; 
extern float Q_gyro;
extern float R_angle; 
extern float dt ;
#endif
#ifdef L3G4200_USED
extern float CarAngle;            //��ģ���
extern float angle, angle_dot; 		//�ⲿ��Ҫ���õı���
extern float AngleSpeed_X;   //��ֱ���ٶ�
extern float GravityAngle;        //���ٶȼƽǶ�
extern void Balance_Sensor_Init(void);
extern void Balance_Sensor_Calculate(void);
float Kalman_Filter(float angle_m, float gyro_m);
void QingHua_AngelCalculate(float G_angle, float Gyro);
#endif
#ifdef  MPU6050_USED
extern float Gyro_Balance;   //ƽ����������
extern float Gyro_Turn;
extern float Angel_Balance;  //���ٶȼƽ���Ƕ�
extern float CarAngle;            //��ģ���
extern float angle, angle_dot; 		//�ⲿ��Ҫ���õı���
/*һάkalman�˲��ṹ�嶨��,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*�ϴ�Ԥ�����Э������� C(k|k-1)*/
	float X_last;				    /*ϵͳ״̬Ԥ������о���*/

	float Q;						/*��������Э����*/
	float R;						/*��������Э����*/

	float K;						/*���������棬�о���*/
	float X;						/*���Ź�����������о���*/
	float C;						/*���Ź���Э�������C(k|k)*/

	float input;				    /*����ֵ����Z(k)*/
}
kal_filter;
extern float angle, angle_dot;
void Kalman_Filter(float Accel, float Gyro);
void Yijielvbo(float angle_m, float gyro_m);
float kalman_filter(kal_filter* k_flt, float input);
#endif
#endif