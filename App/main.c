/*
				 ██████╗ █████╗ ██╗   ██╗ ██████╗
				██╔════╝██╔══██╗██║   ██║██╔════╝
				██║     ███████║██║   ██║██║
				██║     ██╔══██║██║   ██║██║
				╚██████╗██║  ██║╚██████╔╝╚██████╗
				 ╚═════╝╚═╝  ╚═╝ ╚═════╝  ╚═════╝
*/ 
/*!
 * @file       main.c
 * @brief      2017恩智浦杯智能车主程序
 * @author     Jassy
 * @date       2016-12-01
 */

#include "common.h"
#include "include.h"
#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)         //尽量用最后面的扇区，确保安全 


//====================函数声明========================
//内部函数记得加static 
//使用断言进行安全检查 
uint8 key_scan(void);		//按键扫描 
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);
void image_handler();
//====================参数定义========================
//中断引用变量记得加volatile
//只读变量记得加constant 
//减少全局变量定义 加static修饰 如需访问可构造函数
uint8 key_value = 0; 
uint8 imgbuff[CAMERA_SIZE];      //定义存储接收图像的数组
uint8 img[CAMERA_W*CAMERA_H];
float vcan_send_buff[8];         //山外上位机虚拟示波器
KEY_MSG_t keymsg;                   

//临时
float power = 10, index = 0, turn_power = 0;//0:位置，1：曲率；
int midx[60];
int map[80][60];
int midtempx = 0, n = 1, midn = 0;
float turnangleindex = 0;
float turn_offset = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++
 /*!
  *  @brief      main函数
  *  @note       
  */
void main()
{
#if   MK60F15
	/*      开启硬件浮点       */
    SCB->CPACR |=((3UL << 10*2)|(3UL << 11*2));     /* set CP10 and CP11 Full Access */
#endif
	DisableInterrupts;  //关总中断
	printf("Init......begin\n");
	printf("**************************\n");
	/*		调试按键初始化
	*KEY硬件定义在VCAN_key.c文件修改 
	*枚举类型KEY_e 
	*/
	printf("Key Init......");
        key_init(KEY_MAX);      //初始化全部按键
	DELAY_MS(5);
	printf("OK\n");
	/*        蜂鸣器初始化         */
	printf("Beep Init......");
	gpio_init(PTA16, GPO, 0);


	printf("OK\n");
	/*        OLED初始化          */ 
	printf("OLED Init......");
	OLED_Init();        //初始化OLED
	OLED_Fill(0x00);    //黑屏
	Draw_LibLogo();		//飞机LOGO
	DELAY_MS(500);		//延时0.5秒
    gpio_set(PTA16, 1); //关蜂鸣器
    DELAY_MS(10);
	printf("OK\n");
	/*		UI界面初始化		*/ 
	printf("UI Init......");
	SOLGUI_Init(&UI_List);     //UI初始化界面
	DELAY_MS(5);
	SOLGUI_Refresh();          //UI刷新 
	printf("OK\n");
	/*		FTM模块初始化 
	*频率 8K 精度 100 
	*精度修改  MK60_FTM.H -> FTM0_PRECISON
	*/
	printf("PWM Init......");
	Motor_PWM_init();
	DELAY_MS(5);
	printf("OK\n");
	/*		软件I2C初始化      */
	printf("I2C Init......");
    IMU_IIC_Init();
//	IIC_Init();
//  load_config();
//  gpio_init(PTC18, GPI, 0); //IMU INT
	DELAY_MS(5);
	printf("OK\n");
	/*		姿态传感器初始化      */
	printf("8451&3050 Init......");
    MPU3050_Init();                 //陀螺仪初始化     
    MMA8451_Init();	           // 加速度计初始化 
//	MPU6050_initialize();
//  MPU6050_InitGyro_Offset();
	//Balance_Sensor_Init();
	printf("OK\n");
	/*       正交解码初始化      */
	printf("Encoder Init......");
	Encoder_Init();
	port_init_NoALT (PTA12, PF | PULLUP);//内部配置上拉
	port_init_NoALT (PTA13, PF | PULLUP);
	port_init_NoALT (PTA10, PF | PULLUP);
	port_init_NoALT (PTA11, PF | PULLUP);
	DELAY_MS(5);
	printf("OK\n");
	/*       FLASH初始化         */
	printf("Flash Init......");
	flash_init();             //初始化flash
	printf("OK\n");
	//flash_erase_sector(SECTOR_NUM);  //擦除扇区 写入前必须擦除
	//flash_write(SECTOR_NUM, 0, 0x12345678)//写入数据到扇区，偏移地址为0，必须一次写入4字节
	//data32 = flash_read(SECTOR_NUM, 0, uint32);    //读取4字节 
	DELAY_MS(5);
    /*       SD卡初始化        */
    printf("SD card Init......");
    SD_OK= SD_Initialize();
    if(SD_OK)
    {printf("OK\n");}
    else
    {printf("NOT OK\n");}
    DELAY_MS(5);
	/*       摄像头初始化        */
	printf("Camera Init......");
	camera_init(imgbuff);
	DELAY_MS(5);
	printf("OK\n");
	/*        设置中断向量       */
	printf("Interrupt Init......");
	NVIC_SetPriorityGrouping(2);				//设置中断分组
	NVIC_SetPriority(PIT0_VECTORn, 1);          //配置1ms中断高优先级
	NVIC_SetPriority(PIT1_VECTORn, 2);          //配置5ms中断低优先级
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置LPTMR的中断服务函数为 PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置LPTMR的中断服务函数为 PORTA_IRQHandler

	/*    初始化PIT定时器中断    */
	pit_init_ms(PIT0, 1);                                //初始化PIT0，定时时间为： 1ms
	set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);   //设置PIT0的中断服务函数为 PIT0_IRQHandler
	enable_irq(PIT0_IRQn);                               //使能PIT0中断  
	pit_init_ms(PIT1, 80);
	set_vector_handler(PIT1_VECTORn, PIT1_IRQHandler);
	enable_irq(PIT1_IRQn);
	printf("OK\n");
	printf("**************************\n");
	printf("Init...Over\n");
	DELAY_MS(5);
    
	EnableInterrupts;         //开总中断
	//my_cnf[0].f = -222.22;
	//Write_config();
	//load_config();
	//printf("%d\n", ((int)(my_cnf[2].f * 100)));
	while (1)
	{	
        camera_get_img(); 
		image_handler();
		SOLGUI_Menu_PageStage();   
		SOLGUI_Refresh();          //OLED前台刷新
		vcan_sendware((uint8_t *)vcan_send_buff, sizeof(vcan_send_buff));

	}
}
/*!
*  @brief      按键扫描函数 键值传入GUI
*  @author	   Jassy
*  @date       20161128
*/
uint8 key_scan(void)
{
	//key_check带延时消抖
	if (key_check(KEY_U) == KEY_DOWN) { return 0x50; }//上键键值     
	if (key_check(KEY_D) == KEY_DOWN) { return 0x20; }//下键键值
	if (key_check(KEY_L) == KEY_DOWN) { return 0x30; }//左键键值
	if (key_check(KEY_R) == KEY_DOWN) { return 0x10; }//右键键值
	if (key_check(KEY_A) == KEY_DOWN) { return 0x40; }//确认键值
	if (key_check(KEY_B) == KEY_DOWN) { return 0x60; }//返回键值  
	return 0x00;
}
/*!
*  @brief      PIT0中断服务函数
*  @sauthor	   Jassy
*  @date	   20161201
*  @note       
*/
void PIT0_IRQHandler(void)
{

//	//int16 val;
//	//val = ftm_quad_get(FTM1);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
//	//ftm_quad_clean(FTM1);

	//Balance_Sensor_Calculate();
	Car_Control_Process();
//gzz = (int32_t)MPU3050_GetResult(1, MPU3050_OUT_Y_H);
//	vcan_send_buff[0] = (float)ax;
//	vcan_send_buff[1] = (float)gx;
	//vcan_send_buff[2] = CarAngle;
	//vcan_send_buff[3] = AngleControlOut;
	PIT_Flag_Clear(PIT0);       //清中断标志位
}
void PIT1_IRQHandler(void)
{
    key_value = key_scan(); 
    SOLGUI_InputKey(key_value);//传入GUI 
	PIT_Flag_Clear(PIT1);
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
	uint8  n;    //引脚号
	uint32 flag;

	while(!PORTA_ISFR);
	flag = PORTA_ISFR;
	PORTA_ISFR  = ~0;                                   //清中断标志位

	n = 29;                                             //场中断
	if(flag & (1 << n))                                 //PTA29触发中断
	{
		camera_vsync();
	}
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
	n = 28;
	if(flag & (1 << n))                                 //PTA28触发中断
	{
		camera_href();
	}
#endif


}

/*!
 *  @brief      DMA0中断服务函数
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