#include "chassis.h"
#include "math.h"

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
extern tPowerHeatData 	testPowerHeatData;      //实时功率热量数据

#define K_SPEED 10
s32 t_Vw_PID=0;
s32 yaw_follow_tarP=YAW_INIT;
s32 yaw_follow_error=0;
void Remote_Task(void)
{
//	yaw_follow_tarP=yunMotorData.yaw_fdbP;
	if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10>1800)	//过零点
	{
		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-(Gyro_Data.angle[2]*10+3600))*8192/3600;
	}
	else if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10<-1800)
	{
		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-(Gyro_Data.angle[2]*10-3600))*8192/3600;
	}
	else
	{
		yaw_follow_tarP=yunMotorData.yaw_fdbP+(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10)*8192/3600;
	}
	
	
	
	if(GetWorkState()==NORMAL_STATE)
	{
		Chassis_Vx=RC_Ctl.rc.ch1-1024;
	}
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)	//仅在正常情况下遥控器可驱动电机，(自动)登岛模式下交由程序自动控制
	{
//////		Chassis_Vx=RC_Ctl.rc.ch1-1024;
		
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
		if(abs(RC_Ctl.rc.ch2-1024)<40&&abs(YAW_INIT-yunMotorData.yaw_fdbP)<200)//此处陀螺仪在加速度过大时反馈会有较大误差，因此采用低转向普通跟随，高转向智能跟随模式
		{
			if(YAW_INIT-yunMotorData.yaw_fdbP>8192/2)	//智能跟随块	
			{
				Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP+8192,&PID_Chassis_Follow);	//负过界状况下
				yaw_follow_error=YAW_INIT-(yunMotorData.yaw_fdbP+8192);
			}
			else if(YAW_INIT-yunMotorData.yaw_fdbP<-8192/2)
			{
				Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP-8192,&PID_Chassis_Follow);	//正过界状况下
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP-8192);
			}
			else
			{
				Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP,&PID_Chassis_Follow);	//正常状况下
				yaw_follow_error=YAW_INIT-yaw_follow_tarP;
			}
		}
		else
		{
			if(YAW_INIT-yaw_follow_tarP>8192/2)	//智能跟随块	
			{
				Chassis_Vw=PID_General(YAW_INIT,yaw_follow_tarP+8192,&PID_Chassis_Follow);	//负过界状况下
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP+8192);
			}
			else if(YAW_INIT-yaw_follow_tarP<-8192/2)
			{
				Chassis_Vw=PID_General(YAW_INIT,yaw_follow_tarP-8192,&PID_Chassis_Follow);	//正过界状况下
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP-8192);
			}
			else
			{
				Chassis_Vw=PID_General(YAW_INIT,yaw_follow_tarP,&PID_Chassis_Follow);	//正常状况下
				yaw_follow_error=YAW_INIT-yaw_follow_tarP;
			}
		}
			


		Chassis_Vw=chassis_Vw_filter(Chassis_Vw);	//对速度一阶滤波
		
		
//		Chassis_Vw=(s16)(FirstOrder_General((YAW_INIT-yunMotorData.yaw_fdbP),&Yaw_Follow_Filter)*0.43f);
//		Chassis_Vw=(s16)((YAW_INIT-yunMotorData.yaw_fdbP)*0.6f);	//YUN_INIT为目标位置，故为YAW_INIT-
	}
	Chassis_Vy=RC_Ctl.rc.ch0-1024;

	if(GetWorkState()==NORMAL_STATE)
	{	//智能转向块
		s16 Vx_record=Chassis_Vx;
		Chassis_Vx=0;
		yaw_follow_error=yaw_follow_error/8192*2*PI;
		Chassis_Vx+=(s16)(Vx_record*(cos(yaw_follow_error)));
		Chassis_Vy+=(s16)(Vx_record*(sin(yaw_follow_error)));
	}
	
	
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

	{	//功率限制块
		float limit_k=Limit_Power(testPowerHeatData.chassisPower,testPowerHeatData.chassisPowerBuffer);	//testPowerHeatData.chassisPowerBuffer
		float output_limit_lf=chassis_Data.lf_wheel_output*limit_k;
		float output_limit_rf=chassis_Data.rf_wheel_output*limit_k;
		float output_limit_lb=chassis_Data.lb_wheel_output*limit_k;
		float output_limit_rb=chassis_Data.rb_wheel_output*limit_k;
		chassis_Data.lf_wheel_output=(s32)output_limit_lf;
		chassis_Data.rf_wheel_output=(s32)output_limit_rf;
		chassis_Data.lb_wheel_output=(s32)output_limit_lb;
		chassis_Data.rb_wheel_output=(s32)output_limit_rb;
	}
	
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

#define POWERLIMIT 120 	//120w功率限制
#define POWERBUFFER 60	//60J功率缓冲
float Limit_Power(float power,float powerbuffer)	//英雄120J热量限制，直接限制总输出
{
	float limit_k=1;
//	if(power>POWERLIMIT*0.6)
//	{
		limit_k=3.0f*powerbuffer/200.0f+0.1f;	//0.4
		limit_k=limit_k>1?1:limit_k;
		limit_k=limit_k<0.1f?0.1f:limit_k;
//	}
	return limit_k;
}

