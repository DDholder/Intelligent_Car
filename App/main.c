/*
				 �������������[ �����������[ �����[   �����[ �������������[
				�����X�T�T�T�T�a�����X�T�T�����[�����U   �����U�����X�T�T�T�T�a
				�����U     ���������������U�����U   �����U�����U
				�����U     �����X�T�T�����U�����U   �����U�����U
				�^�������������[�����U  �����U�^�������������X�a�^�������������[
				 �^�T�T�T�T�T�a�^�T�a  �^�T�a �^�T�T�T�T�T�a  �^�T�T�T�T�T�a
*/ 
/*!
 * @file       main.c
 * @brief      2017�����ֱ����ܳ�������
 * @author     Jassy
 * @date       2016-12-01
 */

#include "common.h"
#include "include.h"
#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)         //������������������ȷ����ȫ 


//====================��������========================
//�ڲ������ǵü�static 
//ʹ�ö��Խ��а�ȫ��� 
uint8 key_scan(void);		//����ɨ�� 
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);
void image_handler();
void changemap();
void weight_init();
void LineFitting(int upedge);
//====================��������========================
//�ж����ñ����ǵü�volatile
//ֻ�������ǵü�constant 
//����ȫ�ֱ������� ��static���� ������ʿɹ��캯��
uint8 key_value = 0; 
uint8 imgbuff[CAMERA_SIZE];      //����洢����ͼ�������
uint8 img[CAMERA_W*CAMERA_H];
float vcan_send_buff[8];         //ɽ����λ������ʾ����
KEY_MSG_t keymsg;                   

//��ʱ
float power = 10, index = 0, turn_power = 0;//0:λ�ã�1�����ʣ�
int midx[60];
int map[80][60];
float weight[60];
int midtempx = 0, n = 1, midn = 0;
float turnangleindex = 0;
float turn_offset = 0;
float xl[60];
int size = 60;
float A, B;
float yl[60];
int midx_aftercalc[64];
float edge_offsetl = 0, edge_offsetr = 0;  //�߽粹��   
int lnum = 0, rnum = 0, mnum = 0, x = 0;
int edgel[60];  //��߽�										      //
int edger[60];  //�ұ߽�                                                                               //��
float A_out, B_out;
float k1 = 0, k2 = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++
 /*!
  *  @brief      main����
  *  @note       
  */
void main()
{
#if   MK60F15
	/*      ����Ӳ������       */
    SCB->CPACR |=((3UL << 10*2)|(3UL << 11*2));     /* set CP10 and CP11 Full Access */
#endif
	DisableInterrupts;  //�����ж�
	printf("Init......begin\n");
	printf("**************************\n");
	/*		���԰�����ʼ��
	*KEYӲ��������VCAN_key.c�ļ��޸� 
	*ö������KEY_e 
	*/
	printf("Key Init......");
    key_init(KEY_MAX);      //��ʼ��ȫ������
	gpio_init(PTE9, GPI, 1);
	DELAY_MS(5);
	printf("OK\n");
	/*        ��������ʼ��         */
	printf("Beep Init......");
	gpio_init(PTA16, GPO, 0);
	printf("OK\n");
	/*        OLED��ʼ��          */ 
	printf("OLED Init......");
	OLED_Init();        //��ʼ��OLED
	OLED_Fill(0x00);    //����
	Draw_LibLogo();		//�ɻ�LOGO
	DELAY_MS(500);		//��ʱ0.5��
    gpio_set(PTA16, 1); //�ط�����
    DELAY_MS(10);
	printf("OK\n");
	/*		UI�����ʼ��		*/ 
	printf("UI Init......");
	SOLGUI_Init(&UI_List);     //UI��ʼ������
	DELAY_MS(5);
	SOLGUI_Refresh();          //UIˢ�� 
	printf("OK\n");
	/*		FTMģ���ʼ�� 
	*Ƶ�� 8K ���� 100 
	*�����޸�  MK60_FTM.H -> FTM0_PRECISON
	*/
	printf("PWM Init......");
	Motor_PWM_init();
	DELAY_MS(5);
	printf("OK\n");
	/*		���I2C��ʼ��      */
	printf("I2C Init......");
    IMU_IIC_Init();
	DELAY_MS(5);
	printf("OK\n");
	/*		��̬��������ʼ��      */
	printf("8451&3050 Init......");
    MPU3050_Init();                 //�����ǳ�ʼ��     
    MMA8451_Init();	           // ���ٶȼƳ�ʼ�� 
	printf("OK\n");
	/*       ���������ʼ��      */
	printf("Encoder Init......");
	Encoder_Init();
	port_init_NoALT (PTA12, PF | PULLUP);//�ڲ���������
	port_init_NoALT (PTA13, PF | PULLUP);
	port_init_NoALT (PTA10, PF | PULLUP);
	port_init_NoALT (PTA11, PF | PULLUP);
	DELAY_MS(5);
	printf("OK\n");
	/*       FLASH��ʼ��         */
	printf("Flash Init......");
	flash_init();             //��ʼ��flash
	parameter_init();         //��ʼ������  �˴�װ�����е��粻��ʧ����  
	printf("OK\n");
	DELAY_MS(5);
    /*       SD����ʼ��        */
    printf("SD card Init......");
    SD_OK= SD_Initialize();
    if(SD_OK)
    {printf("OK\n");}
    else
    {printf("NOT OK\n");}
    DELAY_MS(5);
	/*       ����ͷ��ʼ��        */
	printf("Camera Init......");
	camera_init(imgbuff);
	DELAY_MS(5);
	printf("OK\n");
	/*        �����ж�����       */
	printf("Interrupt Init......");
	NVIC_SetPriorityGrouping(2);				//�����жϷ���
	NVIC_SetPriority(PIT0_VECTORn, 1);          //����1ms�жϸ����ȼ�
	NVIC_SetPriority(PIT1_VECTORn, 2);          //����5ms�жϵ����ȼ�
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler

	/*    ��ʼ��PIT��ʱ���ж�    */
	pit_init_ms(PIT0, 1);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 1ms
	set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);   //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
	enable_irq(PIT0_IRQn);                               //ʹ��PIT0�ж�  
	pit_init_ms(PIT1, 80);
	set_vector_handler(PIT1_VECTORn, PIT1_IRQHandler);
	enable_irq(PIT1_IRQn);
	printf("OK\n");
	printf("**************************\n");
	printf("Init...Over\n");
	DELAY_MS(5);   
	EnableInterrupts;         //�����ж�
	while (1)
	{	
        camera_get_img(); 
		image_handler();
		if (A_out == 0)
		{
			turn_power = 0;
		}
		else
		{
			turn_power = A_out*k1+B_out*k2;
		}
		//turn_power = 1000.0 / A_out;
		SOLGUI_Menu_PageStage();   
		SOLGUI_Refresh();          //OLEDǰ̨ˢ��
		if(gpio_get(PTE9)==0){
		OLED_Init();        //��ʼ��OLED
		OLED_Fill(0x00);    //����
		}
		vcan_sendware((uint8_t *)vcan_send_buff, sizeof(vcan_send_buff));

	}
}
/*!
*  @brief      ����ɨ�躯�� ��ֵ����GUI
*  @author	   Jassy
*  @date       20161128
*/
uint8 key_scan(void)
{
	//key_check����ʱ����
	if (key_check(KEY_U) == KEY_DOWN) { return 0x50; }//�ϼ���ֵ     
	if (key_check(KEY_D) == KEY_DOWN) { return 0x20; }//�¼���ֵ
	if (key_check(KEY_L) == KEY_DOWN) { return 0x30; }//�����ֵ
	if (key_check(KEY_R) == KEY_DOWN) { return 0x10; }//�Ҽ���ֵ
	if (key_check(KEY_A) == KEY_DOWN) { return 0x40; }//ȷ�ϼ�ֵ
	if (key_check(KEY_B) == KEY_DOWN) { return 0x60; }//���ؼ�ֵ  
	return 0x00;
}
/*!
*  @brief      PIT0�жϷ�����
*  @sauthor	   Jassy
*  @date	   20161201
*  @note       
*/
void PIT0_IRQHandler(void)
{

//	//int16 val;
//	//val = ftm_quad_get(FTM1);          //��ȡFTM �������� ��������(������ʾ������)
//	//ftm_quad_clean(FTM1);

	//Balance_Sensor_Calculate();
	Car_Control_Process();
//gzz = (int32_t)MPU3050_GetResult(1, MPU3050_OUT_Y_H);
//	vcan_send_buff[0] = (float)ax;
//	vcan_send_buff[1] = (float)gx;
	//vcan_send_buff[2] = CarAngle;
	//vcan_send_buff[3] = AngleControlOut;
	PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}
void PIT1_IRQHandler(void)
{
    key_value = key_scan(); 
    SOLGUI_InputKey(key_value);//����GUI 
	PIT_Flag_Clear(PIT1);
}

/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
	uint8  n;    //���ź�
	uint32 flag;

	while(!PORTA_ISFR);
	flag = PORTA_ISFR;
	PORTA_ISFR  = ~0;                                   //���жϱ�־λ

	n = 29;                                             //���ж�
	if(flag & (1 << n))                                 //PTA29�����ж�
	{
		camera_vsync();
	}
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
	n = 28;
	if(flag & (1 << n))                                 //PTA28�����ж�
	{
		camera_href();
	}
#endif


}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
	camera_dma();
}


//======================================
void changemap()
{
	for (int i = 0; i < 60; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				if ((imgbuff[i * 10 + j] & (1 << k)) != 0)
				{
					map[j * 8 + 7 - k][i] = 1;
				}
				else
				{
					map[j * 8 + 7 - k][i] = 0;
				}
			}
		}
	}
}
void image_handler()
{
	///////////////////////////////////////////////////////////////////////////
	int offset = 0;                  //���߲���                         ��
	int upedge = 20;                 //ɨ���ϱ߽�                       ��
	int downedge = 60;
	int zoom = 20;                   //���ű���                         ��
	int microScanArea = 5;           //Ѱ�߷�Χ                         ��
									 //////////////////////////////////////////////////////////////////////////
	int tempcolor = map[40][59];                                                                                  //��  
																												  //��
	int midx[60];   //����											      //
					//��
	int confirmright = 0, confirmleft = 0;  //�߽�ȷ��                                                    //
	lnum = 0, rnum = 0, mnum = 0, x = 0;                                                                                 //��
																														 ////////////////////////////////////////////////////////////////////////////
	changemap();          //��ѹͼ��
	power = 0;
	for (int i = 0;i < 60;i++)
	{
		edgel[i] = 0;
		edger[i] = 79;
	}
	for (int i = downedge - 1; i > upedge && (!confirmright) || (!confirmleft); i--)
	{
		/////////////////////////////////////////////////////////////////////////
		//***********************��������Ѱ��*********************************//
		////////////////////////////////////////////////////////////////////////
		for (int j = 40; j < 80 && ((!confirmright) || (!confirmleft)); j++)    //��������Ѱ��
		{
			if ((tempcolor + map[j][i] == 1) || map[79][i] == 0)
			{
				///////////////////////////////////////////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				/////////////////        �ұ߽�        ////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				if (((tempcolor + map[j][i] == 1 && tempcolor == 0) || map[79][i] == 0) && (!confirmright)) //�ұ߽�
				{
					if (map[79][i] == 0)
					{
						x = 79;
					}
					else
					{
						x = j;
					}
					edger[i] = x;
					for (int b = i + 1; b >= upedge; b--)  //����Ѱ��
					{
						if (b > downedge - 1) b = downedge - 1;
						if (x > microScanArea - 1)             //��ֹ���
							tempcolor = map[x - microScanArea + 1][b];
						else
							tempcolor = map[0][b];
						for (int a = -microScanArea; a < microScanArea + 1; a++)
						{
							if (x + a < 80 && x + a > 0)
							{
								if (tempcolor + map[x + a][b] == 1 || (x + a == 79 && map[79][b] == 0))
								{
									edger[b] = x + a;
									x += a;
									rnum++;
									break;
								}
								tempcolor = map[x + a][b];
							}
							else if (x + a <= 0)
								confirmright = 1;
						}

					}
					confirmright = 1;
				}
				///////////////////////////////////////////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				/////////////////        ��߽�        ////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				if ((tempcolor + map[j][i] == 1 && tempcolor == 1) && (!confirmleft)) //��߽�
				{
					if (map[0][i] == 0)
					{
						x = 0;
					}
					else
					{
						x = j;
					}
					edgel[i] = x;
					for (int b = i + 1; b >= upedge; b--)  //����Ѱ��
					{
						if (b > downedge - 1) b = downedge - 1;
						if (x > microScanArea - 1)                    //��ֹ���
							tempcolor = map[x - microScanArea + 1][b];
						else
							tempcolor = map[0][b];
						for (int a = -microScanArea; a < microScanArea + 1; a++)
						{
							if (x + a < 80 && x + a > 0)
							{
								if (tempcolor + map[x + a][b] == 1 || (x + a == 0 && map[0][b] == 0))
								{
									edgel[b] = x + a;
									x += a;
									lnum++;
									break;
								}
								tempcolor = map[x + a][b];
							}
							else if (x + a >= 80)
								confirmleft = 1;
						}

					}
					confirmleft = 1;
				}
			}
			tempcolor = map[j][i];
		}
		/////////////////////////////////////////////////////////////////////////
		//***********************��������Ѱ��*********************************//
		////////////////////////////////////////////////////////////////////////
		for (int j = 40; j >= 0 && ((!confirmright) || (!confirmleft)); j--)    //��������Ѱ��
		{
			if ((tempcolor + map[j][i] == 1) || map[0][i] == 0)
			{
				///////////////////////////////////////////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				/////////////////        �ұ߽�        ////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				if ((tempcolor + map[j][i] == 1 && tempcolor == 1) && (!confirmright)) //�ұ߽�
				{
					if (map[79][i] == 0)
					{
						x = 79;
					}
					else
					{
						x = j;
					}
					edger[i] = x;
					for (int b = i + 1; b >= upedge; b--)  //����Ѱ��
					{
						if (b > downedge - 1) b = downedge - 1;                          //��ֹ���
						if (x > microScanArea - 1)
							tempcolor = map[x - microScanArea + 1][b];
						else
							tempcolor = map[0][b];
						for (int a = -microScanArea; a < microScanArea + 1; a++)
						{
							if (x + a < 80 && x + a > 0)
							{
								if (tempcolor + map[x + a][b] == 1 || (x + a == 79 && map[79][b] == 0))
								{
									edger[b] = x + a;
									x += a;
									rnum++;
									break;
								}
								tempcolor = map[x + a][b];
							}
							else if (x + a <= 0)
								confirmright = 1;
						}

					}
					confirmright = 1;
				}
				///////////////////////////////////////////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				/////////////////        ��߽�        ////////////////////////////////////////
				/////////////////                      ////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				if (((tempcolor + map[j][i] == 1 && tempcolor == 0) || map[0][i] == 0) && (!confirmleft)) //��߽�
				{
					if (map[0][i] == 0)
					{
						x = 0;
					}
					else
					{
						x = j;
					}
					edgel[i] = x;
					for (int b = i + 1; b >= upedge; b--)  //����Ѱ��
					{
						if (b > downedge - 1) b = downedge - 1;                               //��ֹ���
						if (x > microScanArea - 1)
							tempcolor = map[x - microScanArea + 1][b];
						else
							tempcolor = map[0][b];
						for (int a = -microScanArea; a < microScanArea + 1; a++)
						{
							if (x + a < 80 && x + a >= 0)
							{
								if (tempcolor + map[x + a][b] == 1 || (x + a == 0 && map[0][b] == 0))
								{
									edgel[b] = x + a;
									x += a;
									lnum++;
									break;
								}
								tempcolor = map[x + a][b];
							}
							else if (x + a >= 80)
								confirmleft = 1;
						}

					}
					confirmleft = 1;
				}
			}
			tempcolor = map[j][i];
		}
	}
	power = 0;
	for (int i = upedge; i < downedge; i++)
	{
		midx[i] = (edgel[i] + edger[i]) / 2;
		power += (midx[i] - 40)*weight[i];               //Ȩ�����
		yl[i] = i;
		xl[i] = midx[i];
	}
	//�ұ߽粹��
	edge_offsetr = (downedge - upedge - rnum) * edger[downedge - rnum] + 0.5*(downedge - upedge - rnum) * (downedge - upedge - rnum - 1) * (edger[downedge - rnum] - edger[downedge - rnum + 1]);
	//��߽粹��
	edge_offsetl = (downedge - upedge - lnum) * edgel[downedge - lnum] + 0.5*(downedge - upedge - lnum) * (downedge - upedge - lnum - 1) * (edgel[downedge - lnum] - edgel[downedge - lnum + 1]);
	//���ź󲹳����
	edge_offsetr = 0;
	edge_offsetl = 0;
	if ((power + offset + (edge_offsetr + edge_offsetl)*0.5) / zoom <= 100 && (power + offset + (edge_offsetr + edge_offsetl)*0.5) / zoom > -100)
		turn_power = (power + offset + (edge_offsetr + edge_offsetl)*0.5) / zoom;
	if (lnum < rnum) mnum = lnum;
	else mnum = rnum;
	size = upedge + mnum;
	LineFitting(upedge);
	if (A == 0) {
		;
	}
	else {
		A_out = 1000 / A;
		B_out = B / -A;
	}
}
//////////////////////////////////////////////////////////////////////
//////                  /////////////////////////
//////*****Ȩ������*****////////////////////
//////                 ///////////////////////
//////////////////////////////////////////////////////////////////////
void weight_init()
{
	for (int i = 0; i < 60; i++)
	{
		switch ((int)(i / 10))
		{
		case 0: weight[i] = 1; break;
		case 1: weight[i] = 1.1; break;
		case 2: weight[i] = 1.2; break;
		case 3: weight[i] = 1.25; break;
		case 4: weight[i] = 1.4; break;
		case 5: weight[i] = 1.5; break;
		default:
			break;
		}
	}
}
void LineFitting(int upedge)
{
	float xmean = 0.0f;
	float ymean = 0.0f;
	for (int i = upedge; i < size; i++)
	{
		xmean += xl[60 - i + upedge - 1];
		ymean += yl[60 - i + upedge - 1];
	}
	xmean /= (size - upedge);
	ymean /= (size - upedge);

	float sumx2 = 0.0f;
	float sumxy = 0.0f;
	for (int i = upedge; i < size; i++)
	{
		sumx2 += (xl[60 - i + upedge - 1] - xmean) * (xl[60 - i + upedge - 1] - xmean);
		sumxy += (yl[60 - i + upedge - 1] - ymean) * (xl[60 - i + upedge - 1] - xmean);
	}

	if (sumx2 == 0)
	{
		;
	}
	else {
		A = sumxy / sumx2;
		B = ymean - A * xmean;
	}

}