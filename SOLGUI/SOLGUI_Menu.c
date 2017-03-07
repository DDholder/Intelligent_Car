#include"SOLGUI_Include.h"
#include<string.h>


extern uint8 imgbuff[CAMERA_SIZE];
extern float CarAngle;            //��ģ���
extern float GravityAngle;   //���ٶȼƽǶ�
extern float AngleSpeed_X;   //��ֱ���ٶ�
extern float  Sensor_ACC_Y;
/*     ���Բ���      */
extern float Q_angle ; 
extern float Q_gyro ;
extern float R_angle;
extern float dt;

extern float  String_Angle_P ;              // �����Ƕ�P
extern float  String_Angle_I ;
extern float  String_Angle_D ;
extern float  String_Gyro_P ;                //�������ٶ�      
extern float  String_Gyro_I ;
extern float  String_Gyro_D ;                 //2

extern float  turn_power;
extern float  Begin_CarSpeed; 
extern float turn_offset;
extern float A_out;
extern float B_out;
extern	int edge_offsetl, edge_offsetr;  //�߽粹��    
extern int lnum, rnum, mnum, x;                                                                                 //��
extern	int edgel[60];  //��߽�										      //
extern	int edger[60];  //�ұ߽�
extern int map[80][60];
char pic_choose = 1;

extern float k1 ;
extern float k2 ;
#if MENU_FRAME_EN==1
void drawedge(int left[], int right[], int up, int down)
{
	int l = 0, r = 0;
	for (int i = up;i < down;i++)
	{
		SOLGUI_DrawPoint(left[i], 60 - i, 1);
		SOLGUI_DrawPoint(right[i], 60 - i, 1);
	}
}
void lookmap(int map[][60])
{
	for (int i = 0;i < 80;i++)
	{
		for (int j = 0;j < 60;j++)
		{
			if (!map[i][60 - j])
				SOLGUI_DrawPoint(i, j, 1);
		}
	}
}

void CarStand(void)
{
  DELAY_MS(1000);
//  CarStandStart_flag = 1;
}
void SaveParameter(void)
{
	my_cnf[0].f = Q_angle;
	my_cnf[1].f =  Q_gyro;
	my_cnf[2].f =  R_angle;
	my_cnf[3].f =  dt;           //�˲�����
	my_cnf[4].f = CarAngle_Set;
	my_cnf[5].f = String_Angle_P;
	my_cnf[6].f = String_Angle_I;
	my_cnf[7].f = String_Angle_D;
	my_cnf[8].f = String_Gyro_P;
    my_cnf[9].f = String_Gyro_I;
	my_cnf[10].f = String_Gyro_D;
	my_cnf[11].f = SPEED_CONTROL_P;
	my_cnf[12].f = SPEED_CONTROL_I;
	my_cnf[13].f = SPEED_CONTROL_D;
	my_cnf[14].f = DIRECTION_CONTROL_P;
	my_cnf[15].f = DIRECTION_CONTROL_D;
	my_cnf[16].f = Begin_CarSpeed;
	Write_config();
	SOLGUI_Widget_OptionText(3, "SAVE OK!");
	DELAY_MS(500);

}

//##############################���Զ���ҳ�桿##############################
MENU_PAGE UI_List;MENU_PAGE UI_image_show;MENU_PAGE UI_Dataview;MENU_PAGE UI_Debug;                    
__M_PAGE(UI_List,"CAUC",PAGE_NULL,
{
  	SOLGUI_Cursor(6,0,14);
	SOLGUI_Widget_GotoPage(0,&UI_image_show);
    SOLGUI_Widget_GotoPage(1,&UI_Dataview);	
    SOLGUI_Widget_GotoPage(2,&UI_Debug);	
    SOLGUI_Widget_Button(3,"SavePara", SaveParameter);    //�������
	SOLGUI_Widget_OptionText(4, "angle:  %f", CarAngle);
	SOLGUI_Widget_OptionText(5, "speed:  %f", CarSpeed);
	SOLGUI_Widget_Spin(6, "SP_SET", FLT16, -8000, 8000, &Begin_CarSpeed);
	SOLGUI_Widget_Spin(7, "OFFSET", FLT16, -8000, 8000, &turn_offset);
	SOLGUI_Widget_Spin(8, "PIC", INT8, 0, 1, &pic_choose);
//    SOLGUI_Widget_OptionText(4,"ANGLE:  %f", CarAngle);
//	SOLGUI_Widget_OptionText(5, "SP:  %f", CarSpeed);
//	SOLGUI_Widget_OptionText(6, "YY:  %d", Gyro_L3G4200_Y);
//	SOLGUI_Widget_OptionText(7, "R:  %d", LeftMotorPulseSigma);
	
//        SOLGUI_Widget_OptionText(5,"angel:%lf",g_fAngleControlOutput);   
//        SOLGUI_Widget_OptionText(6,"speed:%f",g_fSpeedControlOutput);   
//        SOLGUI_Widget_OptionText(7,"direc:%f",g_fDirectionControlOutput);    
//        SOLGUI_Widget_OptionText(8,"int:%d",g_nInterrupt_Count);        
//        SOLGUI_Widget_OptionText(9,"spe:%d",g_nSpeedControlPeriod); 
//        SOLGUI_Widget_OptionText(10,"direc:%d",g_nDirectionControlPeriod);
//        SOLGUI_Widget_OptionText(11,"distan:%f",distance_real); 
//        SOLGUI_Widget_OptionText(12,"time:%f",time_real); 
//        SOLGUI_Widget_OptionText(13,"speed:%f",speed_ave); 
});

//-----------------------

__M_PAGE(UI_image_show,"Image",&UI_List,
{
	//SOLGUI_Cursor(6,0,5);

	//SOLGUI_Widget_Picture(47,0,80,60,imgbuff,80,60,NML);
	//SOLGUI_Widget_OptionText(2, "%f", CarAngle);
	SOLGUI_Widget_OptionText(1, "             %f", B_out);
//SOLGUI_Widget_OptionText(2, "             %f", 1000.0 / A_out);
if (A_out == 0)
{
	SOLGUI_Widget_OptionText(3, "             0");
}
else {
	SOLGUI_Widget_OptionText(3, "             %f", A_out ); }
	SOLGUI_Widget_OptionText(4, "             %f", turn_power);
	SOLGUI_Widget_OptionText(5, "             %d", lnum);
	SOLGUI_Widget_OptionText(6, "             %d", rnum);
	if (pic_choose == 1)
	{
		drawedge(edgel, edger, 20, 60);
	}
	else
	{
		lookmap(map);
	}
        //SOLGUI_Widget_Bar(100,0,6,56,80,0, 39+derec_delta,DIREC_Y|PROGBAR);
//        SOLGUI_Widget_Spin(1,"Angle_pid_ki",FLT16,0,20,&Angle_pid_ki);
//        SOLGUI_Widget_Spin(2,"Angle_pid_kd",FLT16,0,5,&Angle_pid_kd);	
});

//-----------------------

__M_PAGE(UI_Dataview,"Dataview",&UI_List,
{
  	SOLGUI_Cursor(6,0,14); 
    SOLGUI_Widget_Spin(0, "SP", FLT16, -2000, 2000, &Q_angle);
	SOLGUI_Widget_Spin(0, "SP", FLT16, -2000, 2000, &Q_gyro);
	SOLGUI_Widget_Spin(0, "SP", FLT16, -2000, 2000, &R_angle);
	SOLGUI_Widget_Spin(0, "SP", FLT16, -2000, 2000, &dt);
//	    SOLGUI_Widget_Spin(0,"SP",FLT16,-2000,2000,&String_Angle_P);	
//        SOLGUI_Widget_Spin(1,"DP",FLT16,-2000,2000,&D_String_Angle_P);
//        SOLGUI_Widget_Spin(2,"SI",FLT16,-2000,2000,&String_Angle_I);	
//        SOLGUI_Widget_Spin(3,"SD",FLT16,-2000,2000,&String_Angle_D);
//        SOLGUI_Widget_Spin(4,"GP",FLT16,-2000,2000,&String_Gyro_P);
//        SOLGUI_Widget_Spin(5,"GI",FLT16,-2000,2000,&String_Gyro_I);	
//        SOLGUI_Widget_Spin(6,"GD",FLT16,-2000,2000,&String_Gyro_D);
//        //SOLGUI_Widget_Spin(7,"GA",FLT16,-2000,2000,&CarAngle_offset);
//        SOLGUI_Widget_Spin(8,"SP",FLT16,-2000,2000,&Begin_CarSpeed);
//        SOLGUI_Widget_Spin(9,"S_P",FLT16,-2000,2000,&SPEED_CONTROL_P);
//		SOLGUI_Widget_Spin(10, "D_P", FLT16, -2000, 2000, &derec_p);
//        SOLGUI_Widget_OptionText(11, "sout:  %f",SpeedControlOutNew);
//        SOLGUI_Widget_OptionText(12, "ADE:  %f",Angle_fDelta);
//        SOLGUI_Widget_Spin(13,"D_D",FLT16,-2000,2000,&derec_d);
//	SOLGUI_Widget_OptionText (0,"+----[ACC Value]----+");	 
//        SOLGUI_Widget_OptionText (1,"angel:%f",g_fAngle); 
//        SOLGUI_Widget_OptionText (2,"speed:%f",g_fCarSpeed_Stand); 
//        SOLGUI_Widget_OptionText (3,"pwm:%d",g_nRightMotorOut);  
//        SOLGUI_Widget_OptionText (4,"AD1:%d",Direction_AD[0]); 
//        SOLGUI_Widget_OptionText (5,"AD2:%d",Direction_AD[1]); 
//        SOLGUI_Widget_OptionText (6,"AD3:%d",Direction_AD[2]); 
//        SOLGUI_Widget_OptionText (7,"AD4:%d",Direction_AD[3]); 
//        SOLGUI_Widget_OptionText (8,"GYRO:%d",gyro_Y_ADresult); 
//        SOLGUI_Widget_OptionText (9,"direc:%f",g_fDirectionControlOutput);
});
//-----------------------
__M_PAGE(UI_Debug,"Debug",&UI_List,
{
  	SOLGUI_Cursor(6,0,15);
SOLGUI_Widget_Spin(11, "Q_angle", FLT16, -2000, 2000, &Q_angle);
SOLGUI_Widget_Spin(12, "Q_gyro", FLT16, -2000, 2000, &Q_gyro);
SOLGUI_Widget_Spin(13, "R_angle", FLT16, -2000, 2000, &R_angle);
SOLGUI_Widget_Spin(14, "ag", FLT16, -20000, 20000, &CarAngle_Set);
		SOLGUI_Widget_Spin(4, "SAP", FLT16,-2000, 2000, &String_Angle_P);
		SOLGUI_Widget_Spin(5, "SAD", FLT16, -2000, 2000, &String_Angle_D);
		SOLGUI_Widget_Spin(6, "SGP", FLT16, -2000, 20000, &String_Gyro_P);
		SOLGUI_Widget_Spin(7, "SGD", FLT16, -2000, 2000, &String_Gyro_D);
		SOLGUI_Widget_Spin(8, "SPP", FLT16, -2000, 2000, &SPEED_CONTROL_P);
		SOLGUI_Widget_Spin(9, "SPI", FLT16, -2000, 2000, &SPEED_CONTROL_I);
		SOLGUI_Widget_Spin(10, "SPD", FLT16, -2000, 2000, &SPEED_CONTROL_D);
		SOLGUI_Widget_Spin(0, "DIRP", FLT16, -2000, 2000, &DIRECTION_CONTROL_P);
		SOLGUI_Widget_Spin(1, "DIRD", FLT16, -2000, 2000, &DIRECTION_CONTROL_D);
		SOLGUI_Widget_Spin(2, "k1", FLT16, -2000, 2000, &k1);
		SOLGUI_Widget_Spin(3, "k2", FLT16, -2000, 2000, &k2);
		//SOLGUI_Widget_Spin(8, "GRA", FLT16, 0, 200, &g_fGRAVITY_ADJUST_TIME_CONSTANT);
		//SOLGUI_Widget_Spin(9, "Gy_ratio", FLT16, 0, 200, &Gypo_ratio);
//        SOLGUI_Widget_Spin(7,"Speed_kp",FLT16,-300,20,&Speed_pid_kp);	
//        SOLGUI_Widget_Spin(8,"Speed_ki",FLT16,-300,20,&Speed_pid_ki);
//        SOLGUI_Widget_Spin(9,"Speed_kd",FLT16,-300,20,&Speed_pid_kd);
//        SOLGUI_Widget_Spin(10,"Direc_kp",FLT16,-300,300,&Direction_pid_kp);
//        SOLGUI_Widget_Spin(11,"Direc_ki",FLT16,-300-300,200,&Direction_pid_ki);
//        SOLGUI_Widget_Spin(12,"Direc_kd",FLT16,-300,200,&Direction_pid_kd);
//        SOLGUI_Widget_Spin(13,"speed",FLT16,-3000,2000,&g_fSpeed_Set);
//        SOLGUI_Widget_Spin(14,"Direc_k",FLT16,0,2,&Direction_k);
});

//##############################��ȫ�ֱ������塿##############################
MENU_PAGE *current_page;//��ǰҳ��
GUI_FIFO _key_cache;	//��ֵFIFO
u8 cur_key=0;			//ȫ�ּ�ֵ

u8 SOLGUI_CSR=0;		//ռ�ñ�־�Ĵ����������㼴��ʾռ�ã������˳���ǰҳ�棩
/*----------------��ռ�ñ�־�Ĵ�����------------------	
	SOLGUI_CSR[0]��	OK[���1]ռ��1������0
	SOLGUI_CSR[1]��	ȫ��ռ��1������0��Ҫռ��ȫ����
	SOLGUI_CSR[2]�� OK[���2]ռ��1���ر�0
-----------------------------------------------------*/

extern CURSOR *cursor;	//������

//##############################���ڲ�ʹ�á�##############################

void FIFO_Init(void)
{
//------------��FIFO�������㡿
	memset(_key_cache.FIFOBuffer,0,sizeof(_key_cache.FIFOBuffer));
//------------��FIFO��дָ�����㡿
	_key_cache.Read=0;
	_key_cache.Write=0;
}

void FIFO_EnQueue(u8 KeyCode)
{
	_key_cache.FIFOBuffer[_key_cache.Write]=KeyCode;
	if(++_key_cache.Write>=FIFOBUF_SIZE) _key_cache.Write=0;
}

u8 FIFO_DeQueue(void)
{
	u8 ret;
	if(_key_cache.Read==_key_cache.Write) return(FIFO_NONE); 	//FIFO��
	else
	{
		ret=_key_cache.FIFOBuffer[_key_cache.Read];
		if (++_key_cache.Read>=FIFOBUF_SIZE) _key_cache.Read=0;
		return(ret);
	}
} 

void SOLGUI_Menu_Title(MENU_PAGE *page)
{
	u8 left_len=0,title_len=0;

	title_len=6*(strlen((const char*)page->pageTitle)+4);					//����������ؿ�� 
	left_len=(SCREEN_X_WIDTH-title_len)>>1;									//�������ƫ������

//---------�����⡿
	SOLGUI_printf(left_len+2,56,F6X8,"[ %s ]",page->pageTitle);				//page.pageTitle  ҳ��������
	SOLGUI_GBasic_Line(6,57,left_len+2,57,ACTUAL);							//��������
	SOLGUI_GBasic_Line(left_len+title_len-2,57,SCREEN_X_WIDTH-7,57,ACTUAL);	//��������
//---------������ͼ�꡿
	if((page->parentPage!=PAGE_NULL)&&(SOLGUI_CSR==0)) SOLGUI_printf(0,56,F6X8,"%c",ICON_BACK);	//�и�ҳ�����ڷ�ռ��ģʽ�����������ͼ��

}

//##############################��API��##############################
void SOLGUI_Menu_SetHomePage(MENU_PAGE *home_page)	//��ҳ����
{
	current_page=home_page;
	FIFO_Init();
}

void SOLGUI_InputKey(u8 key_value) 					//������������ϵͳ�л�ȡ��ǰ������ֵ�������
{
	FIFO_EnQueue(key_value);
}

u8 SOLGUI_GetCurrentKey(void) 						//������������ȡGUI��ǰ��ȫ�ּ�ֵ
{
  	return(cur_key); 	
}

void SOLGUI_Menu_PageStage(void)					//����������SOLGUIǰ̨ҳ���л���
{
//------------------����ֵ��ȡ��
	cur_key=FIFO_DeQueue();							//�ӳ���ȡһ����ֵ��Ϊ��ǰGUI��ȫ�ּ�ֵ
//------------------��������ơ�
	SOLGUI_Menu_Title(current_page);				//����
//------------------��ִ��ҳ�溯����
	current_page->pageFunc();						//ִ��ҳ�溯��
	if(cur_key==SOLGUI_KEY_BACK&&SOLGUI_CSR==0){	//����Ƿ�Ϊ���ؼ�ֵ��CSR��ռ��
		if(current_page->parentPage!=PAGE_NULL)		//�и�ҳ��ſ�ʹ�÷��ؼ�
		{
			current_page=current_page->parentPage;	//�´�ִ�и�ҳ�溯��
			cursor->cursor_rel_offset=0;			//��չ��ƫ�Ƽ�����
			cursor->viewport_offset=0;
		}
	}
}

#endif
