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
int midtempx = 0, n = 1, midn = 0;
float turnangleindex = 0;
float turn_offset = 0;
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
//	IIC_Init();
//  load_config();
//  gpio_init(PTC18, GPI, 0); //IMU INT
	DELAY_MS(5);
	printf("OK\n");
	/*		��̬��������ʼ��      */
	printf("8451&3050 Init......");
    MPU3050_Init();                 //�����ǳ�ʼ��     
    MMA8451_Init();	           // ���ٶȼƳ�ʼ�� 
//	MPU6050_initialize();
//  MPU6050_InitGyro_Offset();
	//Balance_Sensor_Init();
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
	printf("OK\n");
	//flash_erase_sector(SECTOR_NUM);  //�������� д��ǰ�������
	//flash_write(SECTOR_NUM, 0, 0x12345678)//д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
	//data32 = flash_read(SECTOR_NUM, 0, uint32);    //��ȡ4�ֽ� 
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
	//my_cnf[0].f = -222.22;
	//Write_config();
	//load_config();
	//printf("%d\n", ((int)(my_cnf[2].f * 100)));
	while (1)
	{	
        camera_get_img(); 
		image_handler();
		SOLGUI_Menu_PageStage();   
		SOLGUI_Refresh();          //OLEDǰ̨ˢ��
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
void image_handler()
{
	int tempcolor = map[0][0];
	int leftn = 0, rightn = 0;
	int left = 0, left_last = 0, right = 0, right_last = 0;
	int addedgel = 1, addedger = 1;
	int x = 0;
	int edgel[60];
	int edger[60];
	power = 0;
	for (int i = 0; i < 60; i++)
	{
		edgel[i] = 0;
		edger[i] = 79;
		midx[i] = 40;
	}
	for (int i = 20; i < 60; i++)
	{
		tempcolor = map[0][i];
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				if ((imgbuff[i * 10 + j] & (1 << k)) == (1 << k))
				{
					map[j * 8 + k][i] = 1;
				}
				else
				{
					map[j * 8 + k][i] = 0;
				}
				if (tempcolor + map[j * 8 + k][i] == 1)
				{
					x = j * 8 + k;
					if (tempcolor == 0)
					{
						edger[i] = x;
						rightn++;
						right_last = right;
						if (right_last == 0) right_last = x;
						right = x;
					}
					if (tempcolor == 1)
					{
						edgel[i] = x;
						leftn++;
						left_last = left;
						if (left_last == 79) left_last = x;
						left = x;
					}
				}
				if (j * 8 + k < 80)
					tempcolor = map[j * 8 + k][i];
			}

		}
		//        if ((edgel[i - 2] != 1) && (map[0][ i] == 0))
		//        {
		//            edgel[i - 1] += (left - left_last) * addedgel;
		//            addedgel++;
		//            if (i == 59)
		//                edgel[59] += (left - left_last) * addedgel;
		//        }
		//        if ((edger[i - 2] != 79) && (map[79][ i] == 0))
		//        {
		//            edger[i - 1] += (right - right_last) * addedger;
		//            addedger++;
		//            if (i == 59)
		//            {
		//                edger[59] += (right - right_last) * addedger;
		//            }
		//        }
	}
	for (int i = 20; i < 60; i++)
	{
		midx[i] = (edgel[i] + edger[i]) / 2;
		power += midx[i] - 40;
	}
	turn_power = power / 40 - turn_offset;
}