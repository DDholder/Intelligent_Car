/*
				 �������������[ �����������[ �����[   �����[ �������������[                   
				�����X�T�T�T�T�a�����X�T�T�����[�����U   �����U�����X�T�T�T�T�a                   
				�����U     ���������������U�����U   �����U�����U                        
				�����U     �����X�T�T�����U�����U   �����U�����U                        
				�^�������������[�����U  �����U�^�������������X�a�^�������������[                   
				 �^�T�T�T�T�T�a�^�T�a  �^�T�a �^�T�T�T�T�T�a  �^�T�T�T�T�T�a                   

* @file       TASK_process.c
* @brief      ������
* @author     JassyLiu
* @date       2016-12-06
* @revision
* @note
*/ 
#include "common.h"
#include "include.h"
#include "TASK_process.h"
/********************�����־������*****************************/
int Task_flag = 0;                             //ʱ���־λ
uint16  Timercount = 0;						   //ʱ��Ƭѡ������

/*************************�ٶȿ���*******************************/

uint16 SpeedControlCount = 0;                  //�ٶȿ��Ƽ�����־
uint16 SpeedControlPeriod = 0;                 //�ٶȿ������ڼ�������
float  CarSpeed = 0;             //��ǰС���ٶ�
float  Old_CarSpeed = 0;
float  Speed_err = 0;
float  Speed_errlast = 0;	
float Speed_rate = 0.5;

float  CarSpeed_Exert = 0;       //�����ٶ�
float  Begin_CarSpeed = 0;       //�����ٶ�  
float  Curve_Carspeed = 0;       //����ٶ�      

float  SpeedControlIntegral = 0;       //�ٶ���������ۼƱ���
float  SpeedControlOutOld = 0, SpeedControlOutNew = 0;
float  SpeedControlOut = 0;
float  SPEED_CONTROL_P = -1;    //100             //�������ٶȿ���P
float  SPEED_CONTROL_I = -0.008;     //2
float  SPEED_CONTROL_D = -0.02;
float  Dir_P = 15;
float  Dir_D = 0;
/*************************�������*******************************/

uint16 DirectionControlCount = 0;              //������Ƽ�����־λ
uint16 DirectionControlPeriod = 0;
float  DirectionControlOut = 0;
float  DirectionControlOutOld = 0, DirectionControlOutNew = 0; //����������
float X_pre_Lastgyro = 0, X_Lastgyro = 0;
float I_pre_Lastgyro = 0, I_Lastgyro = 0;
float Angular_Velocity_X = 0;
extern float  turn_power;
float DIRECTION_CONTROL_P = 0;
float DIRECTION_CONTROL_D = 0;

/***********************�Ƕȿ���*********************************/
float Angle_fDelta = 0;        //�Ƕ����
float Angle_fDelta_Old = 0;
float Gyro_fDelta = 0;		   //���ٶ����
float Gyro_fDelta_Old = 0;
float Speed2Angel = 0;		   //�ٶ�ת���Ƕ�
float Speed2Angel_RATIO = 1000; //�ٶ�ת���Ƕȱ�������


float  String_Angle_P = -96;              // �����Ƕ�P
float  String_Angle_I = 0;
float  String_Angle_D = 25;
float  String_Gyro_P =  3.6;                //�������ٶ�      
float  String_Gyro_I = 0;            
float  String_Gyro_D = 0.66;                 //2
float  Outside_AngleControlOut = 0;      //�⻷�������
float  AngleControlOut = 0;
float  CarAngle_Set = 46;

float LeftMotorOut;          //��������������
float RightMotorOut;
/*!
*  @brief  ������������ 
*  @param  
*  @date   2016-12-06
*  @revision
*  @note
*/
void Car_Control_Process(void)
{
	SpeedControlPeriod++;			//�ٶȷ�ʱ��Ƭ��������
	SpeedControlOutput();			//���ȷ����� 100���Ƕȿ��ƺ��������ڣ��������Ա�֤��ģ���ȶ��ԡ�
	DirectionControlPeriod++;		//2ms /*  �����ʱ������� +1 */
	DirectionControlOutput();        //==========δ����

	Balance_Sensor_Calculate();	    //�õ�ֱ�������̬���� 
	AngleControl();                 //�Ƕȿ���  �ǶȻ�5ms  ���ٶȻ�1ms
	Timercount++;				    //ʱ��Ƭ��������  
	switch (Timercount)
	{
	case 300:      //300ms���ʼ������ ��ʼ����

		Timercount++;
		break;
	case 400:      //��ȡ���뿪�ص�λ 

		Timercount++;
		break;
	default:
		if (Timercount < 6000)
		{
			if (Timercount >= 500 && Timercount < 600)//�ȴ�����
			{
				if ( 1)
				{
					Timercount = 601;
					CarSpeed_Exert = Begin_CarSpeed;  //���������ٶ�
				}

			}
			else Timercount++;
			if (Timercount == 1000)
				//Motor_enable();				      //ʹ�ܵ��
			if (Timercount > 1000)
			{
				//�趨�ٶ�����
				if (Timercount > 1100)
					//R_angle = 50;                  //���ܺ� ��������������ǿ
				if (Timercount > 1200)
				{

				}

			}

		}
		else
		{
			Timercount = 6000;
			//����6S��ʼ���ͣ����
		}
		break;
	}

	DirectionControlCount++;
	if (DirectionControlCount >= 50)
	{
		DirectionControl();         //������ƺ���
		DirectionControlCount = 0;
		DirectionControlPeriod = 0;
	}
	SpeedControlCount += 1;
	if (SpeedControlCount >= SPEED_CONTROL_PEIORD)//�ٶ�100ms ����һ��
	{
		GetMotorPulse();
		SpeedControl();
		SpeedControlCount = 0;
		SpeedControlPeriod = 0;                               
	}
	MotorOutput();
}
/*!
*  @brief  ƽ��Ƕȿ��ƺ���
*  @param
*  @date   2017-02-20
*  @revision
*  @note  �Ƕ�PD����ͨ�����AngleControlOut�Ƕȿ�����
*/      
void  AngleControl(void)
{
	float Angle_fP = 0, Angle_fI = 0, Angle_fD = 0;        //�Ƕȿ���PID����
	float Gyro_fP = 0, Gyro_fI = 0, Gyro_fD = 0;           //���ٶȿ���PID����  
	u8 i = 0;
	static float String_Angle_Integral = 0;
	static float String_Gyro_Integral = 0;
	Task_flag++;										   //ʱ���1 ��5ms����
	if (Timercount >= 1000)
	{	/*=============�ǶȻ�5ms==================*/
		if (Task_flag >= 5 && Timercount >= 1000)  
		{
			Task_flag = 0;                         //5msʱ������
			Angle_fDelta_Old = Angle_fDelta;       //������һ�ε�ֵ
			//Speed2Angel = SpeedControlOut/Speed2Angel_RATIO; //�ٶȿ������ת���ɽǶ����
			Angle_fDelta = CarAngle -	CarAngle_Set;
			//for (i = 0; i<9; i++)
			//{
			//	speedAngle_record[i] = speedAngle_record[i + 1];
			//}
			//speedAngle_record[9] = Speed2Angel;//��¼�Ƕ�ƫ�� 
			Angle_fP = String_Angle_P* Angle_fDelta;
			Angle_fI = String_Angle_I * Angle_fDelta;                     //���Iֵ
			String_Angle_Integral += Angle_fI;                            //��û���ֵ
			Angle_fD = String_Angle_D *Angular_Velocity_Y;				  //���Dֵ Angular_Velocity_Y��sensor.c

			if (String_Angle_Integral > 5)
			{String_Angle_Integral = 5;}
			if (String_Angle_Integral < -5)
			{String_Angle_Integral = -5;}              //�޷�
			Outside_AngleControlOut = Angle_fP + String_Angle_Integral + Angle_fD; //��ýǶȿ���������ٶ�
		}
		//===============���ٶȻ�1ms=================
		Gyro_fDelta_Old = Gyro_fDelta;								  //������һ�ε�ֵ
		Gyro_fDelta = 0.4*Gyro_fDelta_Old + 0.6*(-Angular_Velocity_Y + Outside_AngleControlOut);
		//��ͨ�˲�   ##########ע�����###########

		Gyro_fP = String_Gyro_P *Gyro_fDelta;                         //���Pֵ
		Gyro_fI = String_Gyro_I *Gyro_fDelta;                         //���Iֵ
		String_Gyro_Integral += Gyro_fI;                              //��û���ֵ
		if (String_Gyro_Integral > 2000)   //��ƫ��5%  �޷�
		{String_Gyro_Integral = 2000;}
		if (String_Gyro_Integral < -2000)
		{String_Gyro_Integral = -2000;}
		Gyro_fD = String_Gyro_D*(Gyro_fDelta - Gyro_fDelta_Old);      //���Dֵ  1msϵ����1
		AngleControlOut = Gyro_fP + String_Gyro_Integral + Gyro_fD;    //������յ����
		/*--�Ƕȿ�������޷�����--*/
		if (AngleControlOut > ANGLE_CONTROL_OUT_MAX)
			AngleControlOut = ANGLE_CONTROL_OUT_MAX;
		else  if (AngleControlOut < ANGLE_CONTROL_OUT_MIN)
			AngleControlOut = ANGLE_CONTROL_OUT_MIN;
	}
}

	//�ش󷽰�
	//****************************5ms��ȡһ���ٶ�****************************/
	//float ControlSpeed=0;
	//
	//void Get_Speed(void)
	//{
	//      u8 i = 0;
	//      for(i = 0 ;i <= 3;i++)
	//      {
	//         Old_CarSpeed_Save[i] =Old_CarSpeed_Save[i + 1] ;//�ٶȱ���
	//      }
	//      Old_CarSpeed_Save[3] = 0.4* Old_CarSpeed_Save[0] + 0.3 *Old_CarSpeed_Save[1] + 0.2* Old_CarSpeed_Save[2] + 0.1*(LeftMotorPulseSigma + RightMotorPulseSigma)/2.0*0.04566/0.1;
	//      CarSpeed = Old_CarSpeed_Save[3];   
	//      ControlSpeed=ControlSpeed*0.3+CarSpeed*0.7;
	//
	//}
	/*************************************************************************
	*  ��������   SpeedControl
	*  ����˵���� �ٶȿ��ƺ���
	*  ����˵����
	*  �������أ� ��
	*  �޸�ʱ�䣺
	*  ��    ע�� ����ͨ����ñ������������ʹ��PI���ƻ��SpeedControlOutNew���ֵ

	*************************************************************************/
	float Old_CarSpeed_Save[4] = { 0 };

	void SpeedControl(void)
	{
		float fDelta;			     //�ٶ�ƫ��
		float fP = 0, fI = 0,fd = 0;        //�����Ǳ����ͻ��ֵļ���ֵ
		Old_CarSpeed = CarSpeed;
		//Speed_rate = CarSpeed / Begin_CarSpeed;
		//���������缫��������ƽ��ֵ���ٽ��е�λת������λ��cm/��
		CarSpeed = 0.5 * Old_CarSpeed + (LeftMotorPulseSigma + RightMotorPulseSigma)*0.862;//0.5*(LeftMotorPulseSigma + RightMotorPulseSigma) / 2.0*CAR_SPEED_CONSTANT;

		if (Timercount>1000)
		{

			fDelta = Begin_CarSpeed - CarSpeed; 
			vcan_send_buff[3] = Begin_CarSpeed;
			vcan_send_buff[4] = CarSpeed;
			SpeedControlOutOld = SpeedControlOutNew;
			/*P����*/
			if (ABS(fDelta) <= 800)
			{
				if (ABS(CarSpeed - Begin_CarSpeed) <= 500)//���ٻ���
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
					SpeedControlIntegral = -2000; //�����޷�
			}
			fP = fDelta * SPEED_CONTROL_P;
			fd = SPEED_CONTROL_D *(CarSpeed - Old_CarSpeed);
			SpeedControlOutNew = fP + SpeedControlIntegral+fd;
			SpeedControlOutNew = 0.3*SpeedControlOutNew + 0.7*SpeedControlOutOld;

			if (SpeedControlOutNew >= 10000)       //6000   ����ǽǶȷ����ٶ����ߵ���  �˴��޷�ӦС��10000
				SpeedControlOutNew = 10000;

			if (SpeedControlOutNew <= -10000)     //-6000
				SpeedControlOutNew = -10000;
		}
	}

	/*************************************************************************
	*  ��������   SpeedControlOutput
	*  ����˵���� �ٶ�ͨ���ֶ��ʱ��Ƭ���
	*  ����˵����
	*  �������أ� ��
	*  �޸�ʱ�䣺
	*  ��    ע���������ٶȿ��Ʒ�ʱ��Ƭ����ٶ���
	*************************************************************************/
	void SpeedControlOutput(void)
	{
		float fValue;
		fValue = SpeedControlOutNew - SpeedControlOutOld;             //ͨ����ý�Ҫ�ﵽĿ��ֵ��ƫ��
		SpeedControlOut = fValue * (SpeedControlPeriod + 1) / 100 + SpeedControlOutOld;  //��ʱ��Ƭ��� 100ms/5ms 
	}
	void DirectionControl(void)
	{
		float tempa, tempb, tempc, max, min;             //�����������˲�
		float tempaa, tempba, tempca, maxa, mina;             //�����������˲�
		Sensor_Value.X_GYRO = MPU3050_GetResult(1, MPU3050_OUT_X_H);
		/*********************�������˲�����֤�õ�������ֵ������**********/
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
		Angular_Velocity_X = Angular_Velocity_X > 300 ? 300 : Angular_Velocity_X;//�޷�
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
	*  ��������   MotorOutput
	*  ����˵���� ����������
	*  ����˵����
	*  �������أ� ��
	*  �޸�ʱ�䣺
	*  ��    ע��
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
		//ProtectStopCar();  //�������������ͣ�� ��һ�����ڻ����ĳ���Ƕ�ͣ��  ��ֹʧ��
		Right_Motor_Control((int32_t)(RightMotorOut));
		Left_Motor_Control((int32_t)(LeftMotorOut));
	}