/*
				 ██████╗ █████╗ ██╗   ██╗ ██████╗                   
				██╔════╝██╔══██╗██║   ██║██╔════╝                   
				██║     ███████║██║   ██║██║                        
				██║     ██╔══██║██║   ██║██║                        
				╚██████╗██║  ██║╚██████╔╝╚██████╗                   
				 ╚═════╝╚═╝  ╚═╝ ╚═════╝  ╚═════╝                   

*/ 

/*!
* @file       sensor.c
* @brief      传感器控制
* @author     JassyLiu
* @date       2016-12-05
* @revision
* @note
*/
#include "common.h"
#include "include.h"
#include "sensor.h"
								 
/*         变量定义         */
/*
			  Z轴     
				\ ↗
			↖	 \
	  X轴一一一一一@
			↗    |
				↘|↗
				  |
				 Y轴
*/

#ifdef MPU3050_USED
Sensor_Type Sensor_Value;                     //传感器值
Sensor_Type Sensor_Offset;                    //传感器偏移值  
float GravityAngle = 0;                       //加速度计角度
float Angular_Velocity_Y = 0;				  //Y轴角速度 
float CarAngle = 0;                           //车模倾角
float CarAngle_record[10] = { 0 };            //角度偏差保存
/*  理论计算值为:0.4028; 分辨率--2mv/度每秒  量程--正负500度每秒 3300/4096/2 */
float GYROSCOPE_RATIO = 0.84;				  //陀螺仪比例因子
float pre_Lastgyro = 0, Lastgyro = 0;

float     point_center = 0;

void Balance_Sensor_Calculate(void)
{
	float tempa, tempb, tempc, max, min;             //用于陀螺仪滤波
	u8  i = 0;
	//此处AD采样已经是得到的g值  后面补偿的数字是用来调整零偏
	Sensor_Value.Z_ACC = MMA8451_GetResult(MMA8451_STATUS_Z_READY, MMA8451_REG_OUTZ_MSB);
	GravityAngle = 57.3248*asinf(Sensor_Value.Z_ACC);  //转换成角度，单位为度   180/3.14 = 57.3248
	if (GravityAngle>90)
		GravityAngle = 90;
	else if (GravityAngle<-90)
		GravityAngle = -90;
	//此处GravityAngle 为输出
	Sensor_Value.Y_GYRO = MPU3050_GetResult(1, MPU3050_OUT_Y_H);
	/*********************陀螺仪滤波，保证得到陀螺仪值不跳变**********/
	tempa = pre_Lastgyro;
	tempb = Lastgyro;
	tempc = Sensor_Value.Y_GYRO;
	max = tempa > tempb ? tempa : tempb;
	max = max > tempc ? max : tempc;
	min = tempa < tempb ? tempa : tempb;
	min = min < tempc ? min : tempc;
	if (tempa > min && tempa < max)    Sensor_Value.Y_GYRO = tempa;
	if (tempb > min  && tempb < max)  Sensor_Value.Y_GYRO = tempb;
	if (tempc > min  &&  tempc < max)  Sensor_Value.Y_GYRO = tempc;
	pre_Lastgyro = Lastgyro;   
	Lastgyro = Sensor_Value.Y_GYRO;
	//此处  Angular_Velocity_Y 为输出
	Angular_Velocity_Y = GYROSCOPE_RATIO*(Sensor_Offset.Y_GYRO - Sensor_Value.Y_GYRO);//此处速度为正反馈，故零偏值减去AD值


#if   qinghua   
	/****************清华互补融合**************/
	//CarAngle = GyroscopeAngleIntegral;

	//fDeltaValue = (GravityAngle - CarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;

	//GyroscopeAngleIntegral += (fDeltaValue + AngleSpeed_Y) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;

#elif  kaerman 

	CarAngle = Kalman_Filter(GravityAngle, Angular_Velocity_Y);   //卡尔曼滤波 
	vcan_send_buff[0] = GravityAngle;
	vcan_send_buff[1] = Angular_Velocity_Y;
	vcan_send_buff[2] = CarAngle;
	//CarAngle=KalmanFilter(GravityAngle,AngleSpeed_Y);   //卡尔曼滤波 
#endif       

	//CarAngle += 9;//重心确定
	for (i = 0;i<9;i++)
	{
		CarAngle_record[i] = CarAngle_record[i + 1];

	}
	CarAngle_record[9] = CarAngle;//记录角度偏差
}

//*
//-------------------------------------------------------
//Kalman滤波，8MHz的处理时间约1.8ms；
//-------------------------------------------------------
/*
Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
*/
float angle, angle_dot; 		//外部需要引用的变量
//-------------------------------------------------------
float Q_angle = 0.36, Q_gyro = 0.22, R_angle = 0.18, dt = 0.001;  //Q是预测值的协方差，R是测量值的协方差 105 0.0018
//float Q_angle=0.00000001, Q_gyro=0.0000003, R_angle=2.5, dt=0.005;  //Q是预测值的协方差，R是测量值的协方差
//R_angle = 0.5
//注意：dt的取值为kalman滤波器采样时间;
float P[2][2] = {
	{ 1, 0 },
	{ 0, 1 }
};
float Pdot[4] = { 0,0,0,0 };
const char C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------

float Kalman_Filter(float angle_m, float gyro_m)			//gyro_m:gyro_measure,angle_m:angle_measure
{
	angle += (gyro_m - q_bias) * dt;//先验估计

	Pdot[0] = Q_angle - P[0][1] - P[1][0];// Pk-' 先验估计误差协方差的微分
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;


	angle_err = angle_m - angle;//zk-先验估计


	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//后验估计误差协方差
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	angle += K_0 * angle_err;//后验估计
	q_bias += K_1 * angle_err;//后验估计
	angle_dot = gyro_m - q_bias;//输出值（后验估计）的微分 = 角速度
	return angle;
}

#endif 

#ifdef  MPU6050_USED
float MPU6050_AX, MPU6050_AY, MPU6050_AZ, MPU6050_GX, MPU6050_GY, MPU6050_GZ;
int16_t AX,AY,AZ,GX,GY,GZ;
float Gyro_Balance;   //平衡用陀螺仪
float Gyro_Turn;
float MPU6050_AXAngel_Balance;  //加速度计解算角度
float K1 = 0.02;
float angle, angle_dot;
float Q_angle = 0.001;// 过程噪声的协方差
float Q_gyro = 0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle = 0.5;// 测量噪声的协方差 既测量偏差
float dt = 0.001;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = { 0,0,0,0 };
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float CarAngle = 0;            //车模倾角
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Kalman_Filter(float Accel, float Gyro)
{
	angle += (Gyro - Q_bias) * dt; //先验估计
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - angle;	//zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle += K_0 * Angle_err;	 //后验估计
	Q_bias += K_1 * Angle_err;	 //后验估计
	angle_dot = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}
/**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
	angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * 0.005);
}

/*******************************************************************************
* Function Name  : kalman_filter
* Description    : 卡尔曼滤波
* Input          :
* Output         :
* Return         :
*******************************************************************************/
float kalman_filter(kal_filter* k_flt, float input)
{
	/*量测更新，3组方程*/
	k_flt->input = input;
	k_flt->K = (k_flt->C_last) / (k_flt->C_last + k_flt->R);
	k_flt->X = k_flt->X_last + k_flt->K * (k_flt->input - k_flt->X_last);
	k_flt->C = (1 - k_flt->K)*(k_flt->C_last);

	/*时间更新，2组方程*/
	k_flt->X_last = k_flt->X;
	k_flt->C_last = k_flt->C + k_flt->Q;

	return k_flt->X;
}
/*!
*  @brief  陀传感器参数计算
*  @param
*  @date   2016-12-05
*  @revision
*  @note  全局变量
//Gyro_L3G4200_X
//ACC_MMA8451_X
加速度计使用Y轴
*/
int filter_buf[15];
int ii;
int filter_sum = 0;
void Balance_Sensor_Calculate(void)
{

	float tempa, tempb, tempc, max, min;             //用于陀螺仪滤波
	u8  i = 0;
	MPU6050_getMotion6(&AX, &AY, &AZ,
		&GX, &GY, &GZ);
	//if (GX > 32768)  GX -= 65536;     //数据类型转换  也可通过short强制类型转换
	//if (GY > 32768)  GY -= 65536;     //数据类型转换
	//if (GZ > 32768)  GZ -= 65536;     //数据类型转换
	//if (AX > 32768) AX -= 65536;    //数据类型转换
	//if (AY > 32768) AY -= 65536;    //数据类型转换
	//if (AZ > 32768) AZ -= 65536;    //数据类型转换
	MPU6050_AX = AX;
	MPU6050_AY = AY;
	MPU6050_AZ = AZ;
	MPU6050_GX = GX;
	MPU6050_GY = GY;
	MPU6050_GZ = GZ;
	Gyro_Balance = -MPU6050_GY;
	MPU6050_AX /= 256.0;
	MPU6050_AZ /= 256.0;
	//Angel_Balance = atan2(MPU6050_AX, MPU6050_AZ) * 180 / PI;
	MPU6050_GY = MPU6050_GY / 16.4;
	//Kalman_Filter(Angel_Balance, -MPU6050_GY);
	Gyro_Turn = MPU6050_GZ;
	//CarAngle = angle;
	CarAngle = MPU6050_AX;

}
#endif
#ifdef L3G4200_USED
 float  Sensor_ACC_Y= 0;     //加速度计Y轴
 float  Sensor_ACC_Z= 0;	 //加速度计Z轴
 float  Sensor_GYRO_X = 0;   //陀螺仪X轴
 float  Sensor_GYRO_X_OFFSET = 0; //偏移量
 float GravityAngle = 0;   //加速度计重力角度
 float AngleSpeed_X = 0;   //垂直角速度
 float pre_Lastgyro = 0, Lastgyro = 0;
/*  理论计算值为:0.4028; 分辨率--2mv/度每秒  量程--正负500度每秒 3300/4096/2 */
//陀螺仪比例因子(如果不使用加速度灵敏度电压转换) 卡尔曼 0.70  清华0.94
float GYROSCOPE_ANGLE_RATIO = 0.85;
//-----------------清华滤波方案------------------
float DT = 0.001;               //清华滤波时间常数
float g_fGyroscopeAngleIntegral;//清华 最终融合角度
float g_fCarAngle;              //清华 缓存角度变量 
float g_fGRAVITY_ADJUST_TIME_CONSTANT = 0.08;//重力加速角度补偿时间常数 
//----------------------------------------------
float CarAngle = 0;            //车模倾角
float CarAngle_record[10] = { 0 };               //赛道偏差保存
float CarAngle_offset = 70;
/*!
*  @brief  陀螺仪加速度计初始化
*  @param
*  @date   2016-12-05
*  @revision
*  @note
*/
void Balance_Sensor_Init(void)
{
	/* 初始化L3G4200数字陀螺仪 */
	L3G4200_Init();
	DELAY_MS(5);
	/* 初始化MMA8451加速度传感器 */
	MMA8451_Init();
	DELAY_MS(5);
}

/*!
*  @brief  清华滤波方案
*  @param
*  @date   2016-12-05
*  @revision
*  @note
*/
void QingHua_AngelCalculate(float G_angle, float Gyro)
{
	float fDeltaValue;
	g_fCarAngle = g_fGyroscopeAngleIntegral;
	fDeltaValue = (G_angle - g_fCarAngle) / g_fGRAVITY_ADJUST_TIME_CONSTANT;
	g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;
}
/*!
*  @brief  陀传感器参数计算
*  @param
*  @date   2016-12-05
*  @revision
*  @note  全局变量 
//Gyro_L3G4200_X
//ACC_MMA8451_X 
加速度计使用Y轴 
*/
int filter_buf[15];
int ii;
int filter_sum = 0;
void Balance_Sensor_Calculate(void)
{
	float tempa, tempb, tempc, max, min;             //用于陀螺仪滤波
	u8  i = 0;
		Sensor_ACC_Y = (float)ACC_MMA8451_Y/256.0;
		Sensor_ACC_Z = (float)ACC_MMA8451_Z/256.0;
	GravityAngle = 573.248*atan2(Sensor_ACC_Y,Sensor_ACC_Z);  //转换成角度，单位为度   180/3.14 = 57.3248
	if (GravityAngle>900)
		GravityAngle = 900;
	else if (GravityAngle<-900)
		GravityAngle = -900;

	Sensor_GYRO_X = (float)Gyro_L3G4200_X;

	/*********************陀螺仪滤波，保证得到陀螺仪值不跳变**********/

	filter_buf[14] = Gyro_L3G4200_X;
	filter_sum = 0;
	for (ii = 0; ii < 14; ii++)
	{
		filter_buf[ii] = filter_buf[ii + 1]; // 所有数据左移，低位仍掉
		filter_sum += filter_buf[ii];
	}

	Sensor_GYRO_X =(float) (filter_sum / 14);
	//tempa = pre_Lastgyro;
	//tempb = Lastgyro;
	//tempc =Sensor_GYRO_X;
	//max = tempa > tempb ? tempa : tempb;
	//max = max > tempc ? max : tempc;
	//min = tempa < tempb ? tempa : tempb;
	//min = min < tempc ? min : tempc;
	//if (tempa > min && tempa < max)    Sensor_GYRO_X = tempa;
	//if (tempb > min  && tempb < max)   Sensor_GYRO_X = tempb;
	//if (tempc > min  &&  tempc < max)  Sensor_GYRO_X = tempc;
	//pre_Lastgyro = Lastgyro;          //角速度递推赋值
	//Lastgyro = Sensor_GYRO_X;

	AngleSpeed_X = GYROSCOPE_ANGLE_RATIO*(Sensor_GYRO_X_OFFSET-Sensor_GYRO_X);
	//此处速度为正反馈，故零偏值减去AD值
#if   qinghua   
	
	/****************清华互补融合**************/
	QingHua_AngelCalculate(GravityAngle,AngleSpeed_X);
	CarAngle = g_fCarAngle;

#elif  kaerman 
	CarAngle = Kalman_Filter(GravityAngle, AngleSpeed_X);   //卡尔曼滤波 															
#endif       

	CarAngle -= CarAngle_offset;//重心确定
	for (i = 0; i<9; i++)
	{
		CarAngle_record[i] = CarAngle_record[i + 1];

	}
	CarAngle_record[9] = CarAngle;//记录角度偏差
}
//*
//-------------------------------------------------------
//Kalman滤波，8MHz的处理时间约1.8ms；
//-------------------------------------------------------
/*
Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
*/
float angle, angle_dot; 		//外部需要引用的变量
//-------------------------------------------------------
//float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 10, dt = 0.0018;  
float Q_angle = 20, Q_gyro = 20, R_angle = 0.4, dt = 0.0018;
//Q是预测值的协方差，R是测量值的协方差 105 0.0018
//float Q_angle=0.00000001, Q_gyro=0.0000003, R_angle=2.5, dt=0.005; 
//Q是预测值的协方差，R是测量值的协方差
//R_angle = 0.5
 //注意：dt的取值为kalman滤波器采样时间;
float P[2][2] = {
	{ 1, 0 },
	{ 0, 1 }
};
float Pdot[4] = { 0,0,0,0 };
const char C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
float Kalman_Filter(float angle_m, float gyro_m)			
//gyro_m:gyro_measure,angle_m:angle_measure
{
	angle += (gyro_m - q_bias) * dt;//先验估计

	Pdot[0] = Q_angle - P[0][1] - P[1][0];// Pk-' 先验估计误差协方差的微分
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	angle_err = angle_m - angle;//zk-先验估计

	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//后验估计误差协方差
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;


	angle += K_0 * angle_err;//后验估计
	q_bias += K_1 * angle_err;//后验估计
	angle_dot = gyro_m - q_bias;//输出值（后验估计）的微分 = 角速度
	return angle;
}

#endif



