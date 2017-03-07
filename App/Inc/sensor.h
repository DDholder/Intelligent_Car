#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "common.h"
#define  qinghua  0
#define  kaerman  1
#define  MPU3050_USED   //使用3050  
//MPU6050_USED    //使用6050

#ifdef MPU3050_USED
/*类型定义*/
typedef struct
{
	float X_GYRO;   //X轴角速度
	float Y_GYRO;   //Y轴角速度
	float Z_ACC;    //Z轴角度
}Sensor_Type;
extern float CarAngle;                           //车模倾角
extern float Angular_Velocity_Y;				 //Y轴角速度 
extern float point_center;						 //偏差
extern float Kalman_Filter(float angle_m, float gyro_m);
extern Sensor_Type Sensor_Value;                     //传感器值
extern Sensor_Type Sensor_Offset;                    //传感器偏移值  
extern float Q_angle; 
extern float Q_gyro;
extern float R_angle; 
extern float dt ;
#endif
#ifdef L3G4200_USED
extern float CarAngle;            //车模倾角
extern float angle, angle_dot; 		//外部需要引用的变量
extern float AngleSpeed_X;   //垂直角速度
extern float GravityAngle;        //加速度计角度
extern void Balance_Sensor_Init(void);
extern void Balance_Sensor_Calculate(void);
float Kalman_Filter(float angle_m, float gyro_m);
void QingHua_AngelCalculate(float G_angle, float Gyro);
#endif
#ifdef  MPU6050_USED
extern float Gyro_Balance;   //平衡用陀螺仪
extern float Gyro_Turn;
extern float Angel_Balance;  //加速度计解算角度
extern float CarAngle;            //车模倾角
extern float angle, angle_dot; 		//外部需要引用的变量
/*一维kalman滤波结构体定义,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X_last;				    /*系统状态预测矩阵，列矩阵*/

	float Q;						/*过程噪声协方差*/
	float R;						/*量测噪声协方差*/

	float K;						/*卡尔曼增益，列矩阵*/
	float X;						/*最优估计输出矩阵，列矩阵*/
	float C;						/*最优估计协方差矩阵C(k|k)*/

	float input;				    /*量测值，即Z(k)*/
}
kal_filter;
extern float angle, angle_dot;
void Kalman_Filter(float Accel, float Gyro);
void Yijielvbo(float angle_m, float gyro_m);
float kalman_filter(kal_filter* k_flt, float input);
#endif
#endif