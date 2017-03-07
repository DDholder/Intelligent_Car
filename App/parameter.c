/*
                 ██████╗ █████╗ ██╗   ██╗ ██████╗                   
                ██╔════╝██╔══██╗██║   ██║██╔════╝                   
                ██║     ███████║██║   ██║██║                        
                ██║     ██╔══██║██║   ██║██║                        
                ╚██████╗██║  ██║╚██████╔╝╚██████╗                   
                 ╚═════╝╚═╝  ╚═╝ ╚═════╝  ╚═════╝                   
                                                                    
███████╗███╗   ███╗ █████╗ ██████╗ ████████╗ ██████╗ █████╗ ██████╗ 
██╔════╝████╗ ████║██╔══██╗██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
███████╗██╔████╔██║███████║██████╔╝   ██║   ██║     ███████║██████╔╝
╚════██║██║╚██╔╝██║██╔══██║██╔══██╗   ██║   ██║     ██╔══██║██╔══██╗
███████║██║ ╚═╝ ██║██║  ██║██║  ██║   ██║   ╚██████╗██║  ██║██║  ██║
╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝    
*/ 
/*!
* @file       parameter.c
* @brief      全局参数
* @author     JassyLiu
* @date       2016-12-06
* @revision
* @note
*/
#include "common.h"
#include "include.h"
#include "parameter.h"

void parameter_init(void)
{
	load_config();      //更新联合体
	Q_angle = my_cnf[0].f;
	Q_gyro = my_cnf[1].f;
	R_angle = my_cnf[2].f;
	dt = my_cnf[3].f;
	CarAngle_Set = my_cnf[4].f;
	String_Angle_P = my_cnf[5].f;
	String_Angle_I = my_cnf[6].f;
	String_Angle_D = my_cnf[7].f;
	String_Gyro_P = my_cnf[8].f;
	String_Gyro_I = my_cnf[9].f;
	String_Gyro_D = my_cnf[10].f;
	SPEED_CONTROL_P = my_cnf[11].f;
	SPEED_CONTROL_I = my_cnf[12].f;
	SPEED_CONTROL_D = my_cnf[13].f;
	DIRECTION_CONTROL_P = my_cnf[14].f;
	DIRECTION_CONTROL_D = my_cnf[15].f;
	Begin_CarSpeed = my_cnf[16].f;
}

