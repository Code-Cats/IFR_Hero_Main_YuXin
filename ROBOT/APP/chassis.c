#include "chassis.h"

CHASSIS_DATA chassis_Data={0};

PID_GENERAL PID_Chassis_Speed[4]={PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT};
PID_GENERAL PID_Chassis_Follow=PID_CHASSIS_FOLLOW_DEFAULT;

FIRST_ORDER_FILTER Yaw_Follow_Filter=YAW_FOLLOW_FILTER_DEFAULT;

s16 Chassis_Vx=0;
s16 Chassis_Vy=0;
s16 Chassis_Vw=0;

extern RC_Ctl_t RC_Ctl;
extern GYRO_DATA Gyro_Data;
extern YUN_MOTOR_DATA 	yunMotorData;


#define K_SPEED 8
s32 t_Vw_PID=0;
void Remote_Task(void)
{
	if(GetWorkState()==NORMAL_STATE)
	{
		Chassis_Vx=RC_Ctl.rc.ch1-1024;
	}
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)	//仅在正常情况下遥控器可驱动电机，(自动)登岛模式下交由程序自动控制
	{
//////		Chassis_Vx=RC_Ctl.rc.ch1-1024;
		
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
		if(YAW_INIT-yunMotorData.yaw_fdbP>8192/2)
		{
			Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP+8192,&PID_Chassis_Follow);	//正常状况下
		}
		else if(YAW_INIT-yunMotorData.yaw_fdbP<-8192/2)
		{
			Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP-8192,&PID_Chassis_Follow);	//正常状况下
		}
		else
		{
			Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP,&PID_Chassis_Follow);	//正常状况下
		}
		
		Chassis_Vw=chassis_Vw_filter(Chassis_Vw);	//对速度一阶滤波
////		Chassis_Vw=PID_Robust(YAW_INIT,yunMotorData.yaw_fdbP,-yunMotorData.yaw_fdbV,&PID_Chassis_Follow);	//云台速度无/错误反馈
//		Chassis_Vw=(s16)(FirstOrder_General((YAW_INIT-yunMotorData.yaw_fdbP),&Yaw_Follow_Filter)*0.43f);
//		Chassis_Vw=(s16)((YAW_INIT-yunMotorData.yaw_fdbP)*0.6f);	//YUN_INIT为目标位置，故为YAW_INIT-
	}
	Chassis_Vy=RC_Ctl.rc.ch0-1024;

	chassis_Data.lf_wheel_tarV=(Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rf_wheel_tarV=(-Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.lb_wheel_tarV=(Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rb_wheel_tarV=(-Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	

	
///////////////////////////////////////////////////////////////// 
//	chassis_Data.lf_wheel_tarV=remote_tem;
//	chassis_Data.rf_wheel_tarV=remote_tem;
//	chassis_Data.lb_wheel_tarV=remote_tem;
//	chassis_Data.rb_wheel_tarV=remote_tem;
	
	chassis_Data.lf_wheel_output=PID_General(chassis_Data.lf_wheel_tarV,chassis_Data.lf_wheel_fdbV,&PID_Chassis_Speed[LF]);
	chassis_Data.rf_wheel_output=PID_General(chassis_Data.rf_wheel_tarV,chassis_Data.rf_wheel_fdbV,&PID_Chassis_Speed[RF]);
	chassis_Data.lb_wheel_output=PID_General(chassis_Data.lb_wheel_tarV,chassis_Data.lb_wheel_fdbV,&PID_Chassis_Speed[LB]);
	chassis_Data.rb_wheel_output=PID_General(chassis_Data.rb_wheel_tarV,chassis_Data.rb_wheel_fdbV,&PID_Chassis_Speed[RB]);
	
//	CAN_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
}


s16 chassis_Vw_filter(s16 now_V)
{
	static s32 Last_V=0;
	static s32 now_acc_V=0;
	static float a=0.25f;
	now_acc_V=(s32)(Last_V*(1-a)+now_V*a);
	Last_V=now_acc_V;
	return now_acc_V;
}


