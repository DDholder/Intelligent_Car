/*
				 ██████╗ █████╗ ██╗   ██╗ ██████╗                   
				██╔════╝██╔══██╗██║   ██║██╔════╝                   
				██║     ███████║██║   ██║██║                        
				██║     ██╔══██║██║   ██║██║                        
				╚██████╗██║  ██║╚██████╔╝╚██████╗                   
				 ╚═════╝╚═╝  ╚═╝ ╚═════╝  ╚═════╝                   

* @file       TASK_process.c
* @brief      任务处理
* @author     JassyLiu
* @date       2016-12-06
* @revision
* @note
*/ 
#include "common.h"
#include "include.h"
#include "TASK_process.h"
/********************任务标志量定义*****************************/
int Task_flag = 0;                             //时序标志位
uint16  Timercount = 0;						   //时间片选区变量

/*************************速度控制*******************************/

uint16 SpeedControlCount = 0;                  //速度控制计数标志
uint16 SpeedControlPeriod = 0;                 //速度控制周期计数变量
float  CarSpeed = 0;             //当前小车速度
float  Old_CarSpeed = 0;
float  Speed_err = 0;
float  Speed_errlast = 0;	
float Speed_rate = 0.5;

float  CarSpeed_Exert = 0;       //期望速度
float  Begin_CarSpeed = 0;       //起跑速度  
float  Curve_Carspeed = 0;       //弯道速度      

float  SpeedControlIntegral = 0;       //速度输出积分累计变量
float  SpeedControlOutOld = 0, SpeedControlOutNew = 0;
float  SpeedControlOut = 0;
float  SPEED_CONTROL_P = -1;    //100             //这里是速度控制P
float  SPEED_CONTROL_I = -0.008;     //2
float  SPEED_CONTROL_D = -0.02;
float  Dir_P = 15;
float  Dir_D = 0;
/*************************方向控制*******************************/

uint16 DirectionControlCount = 0;              //方向控制计数标志位
uint16 DirectionControlPeriod = 0;
float  DirectionControlOut = 0;
float  DirectionControlOutOld = 0, DirectionControlOutNew = 0; //方向控制输出
float X_pre_Lastgyro = 0, X_Lastgyro = 0;
float I_pre_Lastgyro = 0, I_Lastgyro = 0;
float Angular_Velocity_X = 0;
extern float  turn_power;
float DIRECTION_CONTROL_P = 0;
float DIRECTION_CONTROL_D = 0;

/***********************角度控制*********************************/
float Angle_fDelta = 0;        //角度误差
float Angle_fDelta_Old = 0;
float Gyro_fDelta = 0;		   //角速度误差
float Gyro_fDelta_Old = 0;
float Speed2Angel = 0;		   //速度转换角度
float Speed2Angel_RATIO = 1000; //速度转换角度比例因子


float  String_Angle_P = -96;              // 串级角度P
float  String_Angle_I = 0;
float  String_Angle_D = 25;
float  String_Gyro_P =  3.6;                //串级角速度      
float  String_Gyro_I = 0;            
float  String_Gyro_D = 0.66;                 //2
float  Outside_AngleControlOut = 0;      //外环控制输出
float  AngleControlOut = 0;
float  CarAngle_Set = 46;

float LeftMotorOut;          //电机左右输出变量
float RightMotorOut;
/*!
*  @brief  控制任务处理函数 
*  @param  
*  @date   2016-12-06
*  @revision
*  @note
*/
void Car_Control_Process(void)
{
	SpeedControlPeriod++;			//速度分时间片计数变量
	SpeedControlOutput();			//均匀分配在 100个角度控制函数周期内，这样可以保证车模的稳定性。
	DirectionControlPeriod++;		//2ms /*  方向分时输出变量 +1 */
	DirectionControlOutput();        //==========未定义

	Balance_Sensor_Calculate();	    //得到直立相关姿态数据 
	AngleControl();                 //角度控制  角度环5ms  角速度环1ms
	Timercount++;				    //时间片计数变量  
	switch (Timercount)
	{
	case 300:      //300ms后初始化变量 开始起跑

		Timercount++;
		break;
	case 400:      //获取拨码开关挡位 

		Timercount++;
		break;
	default:
		if (Timercount < 6000)
		{
			if (Timercount >= 500 && Timercount < 600)//等待起跑
			{
				if ( 1)
				{
					Timercount = 601;
					CarSpeed_Exert = Begin_CarSpeed;  //设置起跑速度
				}

			}
			else Timercount++;
			if (Timercount == 1000)
				//Motor_enable();				      //使能电机
			if (Timercount > 1000)
			{
				//设定速度起跑
				if (Timercount > 1100)
					//R_angle = 50;                  //起跑后 卡尔曼收敛性增强
				if (Timercount > 1200)
				{

				}

			}

		}
		else
		{
			Timercount = 6000;
			//起跑6S后开始检测停车线
		}
		break;
	}

	DirectionControlCount++;
	if (DirectionControlCount >= 50)
	{
		DirectionControl();         //方向控制函数
		DirectionControlCount = 0;
		DirectionControlPeriod = 0;
	}
	SpeedControlCount += 1;
	if (SpeedControlCount >= SPEED_CONTROL_PEIORD)//速度100ms 控制一次
	{
		GetMotorPulse();
		SpeedControl();
		SpeedControlCount = 0;
		SpeedControlPeriod = 0;                               
	}
	MotorOutput();
}
/*!
*  @brief  平衡角度控制函数
*  @param
*  @date   2017-02-20
*  @revision
*  @note  角度PD控制通过获得AngleControlOut角度控制量
*/      
void  AngleControl(void)
{
	float Angle_fP = 0, Angle_fI = 0, Angle_fD = 0;        //角度控制PID参数
	float Gyro_fP = 0, Gyro_fI = 0, Gyro_fD = 0;           //角速度控制PID参数  
	u8 i = 0;
	static float String_Angle_Integral = 0;
	static float String_Gyro_Integral = 0;
	Task_flag++;										   //时序加1 控5ms周期
	if (Timercount >= 1000)
	{	/*=============角度环5ms==================*/
		if (Task_flag >= 5 && Timercount >= 1000)  
		{
			Task_flag = 0;                         //5ms时序置零
			Angle_fDelta_Old = Angle_fDelta;       //保存上一次的值
			//Speed2Angel = SpeedControlOut/Speed2Angel_RATIO; //速度控制误差转换成角度误差
			Angle_fDelta = CarAngle -	CarAngle_Set;
			//for (i = 0; i<9; i++)
			//{
			//	speedAngle_record[i] = speedAngle_record[i + 1];
			//}
			//speedAngle_record[9] = Speed2Angel;//记录角度偏差 
			Angle_fP = String_Angle_P* Angle_fDelta;
			Angle_fI = String_Angle_I * Angle_fDelta;                     //获得I值
			String_Angle_Integral += Angle_fI;                            //获得积分值
			Angle_fD = String_Angle_D *Angular_Velocity_Y;				  //获得D值 Angular_Velocity_Y在sensor.c

			if (String_Angle_Integral > 5)
			{String_Angle_Integral = 5;}
			if (String_Angle_Integral < -5)
			{String_Angle_Integral = -5;}              //限幅
			Outside_AngleControlOut = Angle_fP + String_Angle_Integral + Angle_fD; //获得角度控制输出角速度
		}
		//===============角速度环1ms=================
		Gyro_fDelta_Old = Gyro_fDelta;								  //保存上一次的值
		Gyro_fDelta = 0.4*Gyro_fDelta_Old + 0.6*(-Angular_Velocity_Y + Outside_AngleControlOut);
		//低通滤波   ##########注意符号###########

		Gyro_fP = String_Gyro_P *Gyro_fDelta;                         //获得P值
		Gyro_fI = String_Gyro_I *Gyro_fDelta;                         //获得I值
		String_Gyro_Integral += Gyro_fI;                              //获得积分值
		if (String_Gyro_Integral > 2000)   //满偏的5%  限幅
		{String_Gyro_Integral = 2000;}
		if (String_Gyro_Integral < -2000)
		{String_Gyro_Integral = -2000;}
		Gyro_fD = String_Gyro_D*(Gyro_fDelta - Gyro_fDelta_Old);      //获得D值  1ms系数是1
		AngleControlOut = Gyro_fP + String_Gyro_Integral + Gyro_fD;    //获得最终的输出
		/*--角度控制输出限幅处理--*/
		if (AngleControlOut > ANGLE_CONTROL_OUT_MAX)
			AngleControlOut = ANGLE_CONTROL_OUT_MAX;
		else  if (AngleControlOut < ANGLE_CONTROL_OUT_MIN)
			AngleControlOut = ANGLE_CONTROL_OUT_MIN;
	}
}

	//重大方案
	//****************************5ms获取一次速度****************************/
	//float ControlSpeed=0;
	//
	//void Get_Speed(void)
	//{
	//      u8 i = 0;
	//      for(i = 0 ;i <= 3;i++)
	//      {
	//         Old_CarSpeed_Save[i] =Old_CarSpeed_Save[i + 1] ;//速度保存
	//      }
	//      Old_CarSpeed_Save[3] = 0.4* Old_CarSpeed_Save[0] + 0.3 *Old_CarSpeed_Save[1] + 0.2* Old_CarSpeed_Save[2] + 0.1*(LeftMotorPulseSigma + RightMotorPulseSigma)/2.0*0.04566/0.1;
	//      CarSpeed = Old_CarSpeed_Save[3];   
	//      ControlSpeed=ControlSpeed*0.3+CarSpeed*0.7;
	//
	//}
	/*************************************************************************
	*  函数名称   SpeedControl
	*  功能说明： 速度控制函数
	*  参数说明：
	*  函数返回： 无
	*  修改时间：
	*  备    注： 这里通过获得编码器的脉冲后使用PI控制获得SpeedControlOutNew输出值

	*************************************************************************/
	float Old_CarSpeed_Save[4] = { 0 };

	void SpeedControl(void)
	{
		float fDelta;			     //速度偏差
		float fP = 0, fI = 0,fd = 0;        //这里是比例和积分的计算值
		Old_CarSpeed = CarSpeed;
		//Speed_rate = CarSpeed / Begin_CarSpeed;
		//计算两个电极脉冲数的平均值，再进行单位转换。单位：cm/秒
		CarSpeed = 0.5 * Old_CarSpeed + (LeftMotorPulseSigma + RightMotorPulseSigma)*0.862;//0.5*(LeftMotorPulseSigma + RightMotorPulseSigma) / 2.0*CAR_SPEED_CONSTANT;

		if (Timercount>1000)
		{

			fDelta = Begin_CarSpeed - CarSpeed; 
			vcan_send_buff[3] = Begin_CarSpeed;
			vcan_send_buff[4] = CarSpeed;
			SpeedControlOutOld = SpeedControlOutNew;
			/*P控制*/
			if (ABS(fDelta) <= 800)
			{
				if (ABS(CarSpeed - Begin_CarSpeed) <= 500)//变速积分
				{
					fI = fDelta* SPEED_CONTROL_I;
					SpeedControlIntegral += fI;
				}

				else
				{
					fI = fDelta*(800 - ABS(fDelta)) / 800 * SPEED_CONTROL_I;  //(A-ABS(fDelta)+B)/A
					SpeedControlIntegral += fI;
				}

				if (SpeedControlIntegral > 2000)
					SpeedControlIntegral = 2000;
				if (SpeedControlIntegral < -2000)
					SpeedControlIntegral = -2000; //积分限幅
			}
			fP = fDelta * SPEED_CONTROL_P;
			fd = SPEED_CONTROL_D *(CarSpeed - Old_CarSpeed);
			SpeedControlOutNew = fP + SpeedControlIntegral+fd;
			SpeedControlOutNew = 0.3*SpeedControlOutNew + 0.7*SpeedControlOutOld;

			if (SpeedControlOutNew >= 10000)       //6000   如果是角度方向速度三者叠加  此处限幅应小于10000
				SpeedControlOutNew = 10000;

			if (SpeedControlOutNew <= -10000)     //-6000
				SpeedControlOutNew = -10000;
		}
	}

	/*************************************************************************
	*  函数名称   SpeedControlOutput
	*  功能说明： 速度通过分多个时间片输出
	*  参数说明：
	*  函数返回： 无
	*  修改时间：
	*  备    注：这里是速度控制分时间片输出速度量
	*************************************************************************/
	void SpeedControlOutput(void)
	{
		float fValue;
		fValue = SpeedControlOutNew - SpeedControlOutOld;             //通过获得将要达到目标值的偏差
		SpeedControlOut = fValue * (SpeedControlPeriod + 1) / 100 + SpeedControlOutOld;  //分时间片输出 100ms/5ms 
	}
	void DirectionControl(void)
	{
		float tempa, tempb, tempc, max, min;             //用于陀螺仪滤波
		float tempaa, tempba, tempca, maxa, mina;             //用于陀螺仪滤波
		Sensor_Value.X_GYRO = MPU3050_GetResult(1, MPU3050_OUT_X_H);
		/*********************陀螺仪滤波，保证得到陀螺仪值不跳变**********/
		tempa = X_pre_Lastgyro;
		tempb = X_Lastgyro;
		tempc = Sensor_Value.X_GYRO;
		max = tempa > tempb ? tempa : tempb;
		max = max > tempc ? max : tempc;
		min = tempa < tempb ? tempa : tempb;
		min = min < tempc ? min : tempc;
		if (tempa > min && tempa < max)    Sensor_Value.X_GYRO = tempa;
		if (tempb > min  && tempb < max)  Sensor_Value.X_GYRO = tempb;
		if (tempc > min  &&  tempc < max)  Sensor_Value.X_GYRO = tempc;
		X_pre_Lastgyro = X_Lastgyro;
		X_Lastgyro = Sensor_Value.X_GYRO;

		Angular_Velocity_X = 0.8*Sensor_Value.X_GYRO;
		Angular_Velocity_X = Angular_Velocity_X > 300 ? 300 : Angular_Velocity_X;//限幅
		Angular_Velocity_X = Angular_Velocity_X < -300 ? -300 : Angular_Velocity_X;
		DirectionControlOutOld = DirectionControlOutNew;

		tempaa = I_pre_Lastgyro;
		tempba = I_Lastgyro;
		tempca = turn_power;
		maxa = tempaa > tempba ? tempaa : tempba;
		maxa = maxa > tempca ? maxa : tempca;
		mina = tempaa < tempba ? tempaa : tempba;
		mina = mina < tempca ? mina : tempca;
		if (tempaa > mina && tempaa < maxa)    turn_power = tempaa;
		if (tempba > mina  && tempba < maxa)  turn_power = tempba;
		if (tempca > mina  &&  tempca < maxa)  turn_power = tempca;
		I_pre_Lastgyro = I_Lastgyro;
		I_Lastgyro = turn_power;

		DirectionControlOutNew = DIRECTION_CONTROL_P * turn_power + DIRECTION_CONTROL_D *Angular_Velocity_X;
		if (DirectionControlOutNew > 7000)

			DirectionControlOutNew = 7000;

		if (DirectionControlOutNew < -7000)
			DirectionControlOutNew = -7000;
	}

	void DirectionControlOutput(void)
	{
		float fValue;
		fValue = DirectionControlOutNew - DirectionControlOutOld;
		DirectionControlOut = fValue * (DirectionControlPeriod + 1) /50 + DirectionControlOutOld;

	}
	/*************************************************************************
	*  函数名称   MotorOutput
	*  功能说明： 电机输出函数
	*  参数说明：
	*  函数返回： 无
	*  修改时间：
	*  备    注：
	*************************************************************************/
	void MotorOutput(void)
	{
		//DirectionControlOut = derec_p*derec_delta+derec_d*Gyro_L3G4200_Z;
		if (point_center <= 0)
		{

			LeftMotorOut = AngleControlOut - DirectionControlOut-SpeedControlOut;
				RightMotorOut = AngleControlOut + DirectionControlOut-SpeedControlOut;
			vcan_send_buff[5] = AngleControlOut;
			vcan_send_buff[6] = SpeedControlOut;
		}
		else if (point_center>0)
		{
			LeftMotorOut = AngleControlOut - DirectionControlOut-SpeedControlOut;
			RightMotorOut = AngleControlOut + DirectionControlOut-SpeedControlOut;
		}
		//ProtectStopCar();  //车体跌倒保护和停车 车一旦低于或高于某个角度停车  防止失控
		Right_Motor_Control((int32_t)(RightMotorOut));
		Left_Motor_Control((int32_t)(LeftMotorOut));
	}