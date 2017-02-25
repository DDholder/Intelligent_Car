/*
				 �������������[ �����������[ �����[   �����[ �������������[                   
				�����X�T�T�T�T�a�����X�T�T�����[�����U   �����U�����X�T�T�T�T�a                   
				�����U     ���������������U�����U   �����U�����U                        
				�����U     �����X�T�T�����U�����U   �����U�����U                        
				�^�������������[�����U  �����U�^�������������X�a�^�������������[                   
				 �^�T�T�T�T�T�a�^�T�a  �^�T�a �^�T�T�T�T�T�a  �^�T�T�T�T�T�a                   

*/ 

/*!
* @file       sensor.c
* @brief      ����������
* @author     JassyLiu
* @date       2016-12-05
* @revision
* @note
*/
#include "common.h"
#include "include.h"
#include "sensor.h"
								 
/*         ��������         */
/*
			  Z��     
				\ �J
			�I	 \
	  X��һһһһһ@
			�J    |
				�K|�J
				  |
				 Y��
*/

#ifdef MPU3050_USED
Sensor_Type Sensor_Value;                     //������ֵ
Sensor_Type Sensor_Offset;                    //������ƫ��ֵ  
float GravityAngle = 0;                       //���ٶȼƽǶ�
float Angular_Velocity_Y = 0;				  //Y����ٶ� 
float CarAngle = 0;                           //��ģ���
float CarAngle_record[10] = { 0 };            //�Ƕ�ƫ���
/*  ���ۼ���ֵΪ:0.4028; �ֱ���--2mv/��ÿ��  ����--����500��ÿ�� 3300/4096/2 */
float GYROSCOPE_RATIO = 0.84;				  //�����Ǳ�������
float pre_Lastgyro = 0, Lastgyro = 0;

float     point_center = 0;

void Balance_Sensor_Calculate(void)
{
	float tempa, tempb, tempc, max, min;             //�����������˲�
	u8  i = 0;
	//�˴�AD�����Ѿ��ǵõ���gֵ  ���油��������������������ƫ
	Sensor_Value.Z_ACC = MMA8451_GetResult(MMA8451_STATUS_Z_READY, MMA8451_REG_OUTZ_MSB);
	GravityAngle = 57.3248*asinf(Sensor_Value.Z_ACC);  //ת���ɽǶȣ���λΪ��   180/3.14 = 57.3248
	if (GravityAngle>90)
		GravityAngle = 90;
	else if (GravityAngle<-90)
		GravityAngle = -90;
	//�˴�GravityAngle Ϊ���
	Sensor_Value.Y_GYRO = MPU3050_GetResult(1, MPU3050_OUT_Y_H);
	/*********************�������˲�����֤�õ�������ֵ������**********/
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
	//�˴�  Angular_Velocity_Y Ϊ���
	Angular_Velocity_Y = GYROSCOPE_RATIO*(Sensor_Offset.Y_GYRO - Sensor_Value.Y_GYRO);//�˴��ٶ�Ϊ������������ƫֵ��ȥADֵ


#if   qinghua   
	/****************�廪�����ں�**************/
	//CarAngle = GyroscopeAngleIntegral;

	//fDeltaValue = (GravityAngle - CarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;

	//GyroscopeAngleIntegral += (fDeltaValue + AngleSpeed_Y) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;

#elif  kaerman 

	CarAngle = Kalman_Filter(GravityAngle, Angular_Velocity_Y);   //�������˲� 
	vcan_send_buff[0] = GravityAngle;
	vcan_send_buff[1] = Angular_Velocity_Y;
	vcan_send_buff[2] = CarAngle;
	//CarAngle=KalmanFilter(GravityAngle,AngleSpeed_Y);   //�������˲� 
#endif       

	//CarAngle += 9;//����ȷ��
	for (i = 0;i<9;i++)
	{
		CarAngle_record[i] = CarAngle_record[i + 1];

	}
	CarAngle_record[9] = CarAngle;//��¼�Ƕ�ƫ��
}

//*
//-------------------------------------------------------
//Kalman�˲���8MHz�Ĵ���ʱ��Լ1.8ms��
//-------------------------------------------------------
/*
Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
*/
float angle, angle_dot; 		//�ⲿ��Ҫ���õı���
//-------------------------------------------------------
float Q_angle = 0.36, Q_gyro = 0.22, R_angle = 0.18, dt = 0.001;  //Q��Ԥ��ֵ��Э���R�ǲ���ֵ��Э���� 105 0.0018
//float Q_angle=0.00000001, Q_gyro=0.0000003, R_angle=2.5, dt=0.005;  //Q��Ԥ��ֵ��Э���R�ǲ���ֵ��Э����
//R_angle = 0.5
//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;
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
	angle += (gyro_m - q_bias) * dt;//�������

	Pdot[0] = Q_angle - P[0][1] - P[1][0];// Pk-' ����������Э�����΢��
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;// Pk- ����������Э����΢�ֵĻ��� = ����������Э����
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;


	angle_err = angle_m - angle;//zk-�������


	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//����������Э����
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	angle += K_0 * angle_err;//�������
	q_bias += K_1 * angle_err;//�������
	angle_dot = gyro_m - q_bias;//���ֵ��������ƣ���΢�� = ���ٶ�
	return angle;
}

#endif 

#ifdef  MPU6050_USED
float MPU6050_AX, MPU6050_AY, MPU6050_AZ, MPU6050_GX, MPU6050_GY, MPU6050_GZ;
int16_t AX,AY,AZ,GX,GY,GZ;
float Gyro_Balance;   //ƽ����������
float Gyro_Turn;
float MPU6050_AXAngel_Balance;  //���ٶȼƽ���Ƕ�
float K1 = 0.02;
float angle, angle_dot;
float Q_angle = 0.001;// ����������Э����
float Q_gyro = 0.003;//0.03 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle = 0.5;// ����������Э���� �Ȳ���ƫ��
float dt = 0.001;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = { 0,0,0,0 };
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float CarAngle = 0;            //��ģ���
/**************************************************************************
�������ܣ����׿������˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
**************************************************************************/
void Kalman_Filter(float Accel, float Gyro)
{
	angle += (Gyro - Q_bias) * dt; //�������
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - angle;	//zk-�������

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle += K_0 * Angle_err;	 //�������
	Q_bias += K_1 * Angle_err;	 //�������
	angle_dot = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}
/**************************************************************************
�������ܣ�һ�׻����˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
	angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * 0.005);
}

/*******************************************************************************
* Function Name  : kalman_filter
* Description    : �������˲�
* Input          :
* Output         :
* Return         :
*******************************************************************************/
float kalman_filter(kal_filter* k_flt, float input)
{
	/*������£�3�鷽��*/
	k_flt->input = input;
	k_flt->K = (k_flt->C_last) / (k_flt->C_last + k_flt->R);
	k_flt->X = k_flt->X_last + k_flt->K * (k_flt->input - k_flt->X_last);
	k_flt->C = (1 - k_flt->K)*(k_flt->C_last);

	/*ʱ����£�2�鷽��*/
	k_flt->X_last = k_flt->X;
	k_flt->C_last = k_flt->C + k_flt->Q;

	return k_flt->X;
}
/*!
*  @brief  �Ӵ�������������
*  @param
*  @date   2016-12-05
*  @revision
*  @note  ȫ�ֱ���
//Gyro_L3G4200_X
//ACC_MMA8451_X
���ٶȼ�ʹ��Y��
*/
int filter_buf[15];
int ii;
int filter_sum = 0;
void Balance_Sensor_Calculate(void)
{

	float tempa, tempb, tempc, max, min;             //�����������˲�
	u8  i = 0;
	MPU6050_getMotion6(&AX, &AY, &AZ,
		&GX, &GY, &GZ);
	//if (GX > 32768)  GX -= 65536;     //��������ת��  Ҳ��ͨ��shortǿ������ת��
	//if (GY > 32768)  GY -= 65536;     //��������ת��
	//if (GZ > 32768)  GZ -= 65536;     //��������ת��
	//if (AX > 32768) AX -= 65536;    //��������ת��
	//if (AY > 32768) AY -= 65536;    //��������ת��
	//if (AZ > 32768) AZ -= 65536;    //��������ת��
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
 float  Sensor_ACC_Y= 0;     //���ٶȼ�Y��
 float  Sensor_ACC_Z= 0;	 //���ٶȼ�Z��
 float  Sensor_GYRO_X = 0;   //������X��
 float  Sensor_GYRO_X_OFFSET = 0; //ƫ����
 float GravityAngle = 0;   //���ٶȼ������Ƕ�
 float AngleSpeed_X = 0;   //��ֱ���ٶ�
 float pre_Lastgyro = 0, Lastgyro = 0;
/*  ���ۼ���ֵΪ:0.4028; �ֱ���--2mv/��ÿ��  ����--����500��ÿ�� 3300/4096/2 */
//�����Ǳ�������(�����ʹ�ü��ٶ������ȵ�ѹת��) ������ 0.70  �廪0.94
float GYROSCOPE_ANGLE_RATIO = 0.85;
//-----------------�廪�˲�����------------------
float DT = 0.001;               //�廪�˲�ʱ�䳣��
float g_fGyroscopeAngleIntegral;//�廪 �����ںϽǶ�
float g_fCarAngle;              //�廪 ����Ƕȱ��� 
float g_fGRAVITY_ADJUST_TIME_CONSTANT = 0.08;//�������ٽǶȲ���ʱ�䳣�� 
//----------------------------------------------
float CarAngle = 0;            //��ģ���
float CarAngle_record[10] = { 0 };               //����ƫ���
float CarAngle_offset = 70;
/*!
*  @brief  �����Ǽ��ٶȼƳ�ʼ��
*  @param
*  @date   2016-12-05
*  @revision
*  @note
*/
void Balance_Sensor_Init(void)
{
	/* ��ʼ��L3G4200���������� */
	L3G4200_Init();
	DELAY_MS(5);
	/* ��ʼ��MMA8451���ٶȴ����� */
	MMA8451_Init();
	DELAY_MS(5);
}

/*!
*  @brief  �廪�˲�����
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
*  @brief  �Ӵ�������������
*  @param
*  @date   2016-12-05
*  @revision
*  @note  ȫ�ֱ��� 
//Gyro_L3G4200_X
//ACC_MMA8451_X 
���ٶȼ�ʹ��Y�� 
*/
int filter_buf[15];
int ii;
int filter_sum = 0;
void Balance_Sensor_Calculate(void)
{
	float tempa, tempb, tempc, max, min;             //�����������˲�
	u8  i = 0;
		Sensor_ACC_Y = (float)ACC_MMA8451_Y/256.0;
		Sensor_ACC_Z = (float)ACC_MMA8451_Z/256.0;
	GravityAngle = 573.248*atan2(Sensor_ACC_Y,Sensor_ACC_Z);  //ת���ɽǶȣ���λΪ��   180/3.14 = 57.3248
	if (GravityAngle>900)
		GravityAngle = 900;
	else if (GravityAngle<-900)
		GravityAngle = -900;

	Sensor_GYRO_X = (float)Gyro_L3G4200_X;

	/*********************�������˲�����֤�õ�������ֵ������**********/

	filter_buf[14] = Gyro_L3G4200_X;
	filter_sum = 0;
	for (ii = 0; ii < 14; ii++)
	{
		filter_buf[ii] = filter_buf[ii + 1]; // �����������ƣ���λ�Ե�
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
	//pre_Lastgyro = Lastgyro;          //���ٶȵ��Ƹ�ֵ
	//Lastgyro = Sensor_GYRO_X;

	AngleSpeed_X = GYROSCOPE_ANGLE_RATIO*(Sensor_GYRO_X_OFFSET-Sensor_GYRO_X);
	//�˴��ٶ�Ϊ������������ƫֵ��ȥADֵ
#if   qinghua   
	
	/****************�廪�����ں�**************/
	QingHua_AngelCalculate(GravityAngle,AngleSpeed_X);
	CarAngle = g_fCarAngle;

#elif  kaerman 
	CarAngle = Kalman_Filter(GravityAngle, AngleSpeed_X);   //�������˲� 															
#endif       

	CarAngle -= CarAngle_offset;//����ȷ��
	for (i = 0; i<9; i++)
	{
		CarAngle_record[i] = CarAngle_record[i + 1];

	}
	CarAngle_record[9] = CarAngle;//��¼�Ƕ�ƫ��
}
//*
//-------------------------------------------------------
//Kalman�˲���8MHz�Ĵ���ʱ��Լ1.8ms��
//-------------------------------------------------------
/*
Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
*/
float angle, angle_dot; 		//�ⲿ��Ҫ���õı���
//-------------------------------------------------------
//float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 10, dt = 0.0018;  
float Q_angle = 20, Q_gyro = 20, R_angle = 0.4, dt = 0.0018;
//Q��Ԥ��ֵ��Э���R�ǲ���ֵ��Э���� 105 0.0018
//float Q_angle=0.00000001, Q_gyro=0.0000003, R_angle=2.5, dt=0.005; 
//Q��Ԥ��ֵ��Э���R�ǲ���ֵ��Э����
//R_angle = 0.5
 //ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;
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
	angle += (gyro_m - q_bias) * dt;//�������

	Pdot[0] = Q_angle - P[0][1] - P[1][0];// Pk-' ����������Э�����΢��
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;// Pk- ����������Э����΢�ֵĻ��� = ����������Э����
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	angle_err = angle_m - angle;//zk-�������

	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//����������Э����
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;


	angle += K_0 * angle_err;//�������
	q_bias += K_1 * angle_err;//�������
	angle_dot = gyro_m - q_bias;//���ֵ��������ƣ���΢�� = ���ٶ�
	return angle;
}

#endif



