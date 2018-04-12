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
extern u32 time_1ms_count;

#define K_SPEED 10
s32 t_Vw_PID=0;
s32 yaw_follow_tarP=YAW_INIT;
s32 yaw_follow_error=0;

u8 Chassis_Control_RCorPC=RC_CONTROL;
void Remote_Task(void)
{
	
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//模式切换
	{
		if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0)
		{
			Chassis_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch0-1024)>3||abs(RC_Ctl.rc.ch1-1024)>3)
		{
			Chassis_Control_RCorPC=RC_CONTROL;
		}
	}
	
	if(Chassis_Control_RCorPC==RC_CONTROL)
	{
		RC_Control_Chassis();
	}
	else if(Chassis_Control_RCorPC==PC_CONTROL)
	{
		PC_Control_Chassis(&Chassis_Vx,&Chassis_Vy);
	}
	
	
	
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
	
	
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE)	//仅在正常情况下遥控器可驱动电机，(自动)登岛模式下交由程序自动控制
	{
//////		Chassis_Vx=RC_Ctl.rc.ch1-1024;
		
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
		if(abs(RC_Ctl.rc.ch2-1024)<40&&abs(YAW_INIT-yaw_follow_tarP)<200)//此处陀螺仪在加速度过大时反馈会有较大误差，因此采用低转向普通跟随，高转向智能跟随模式	//此处应把fdb改为tarP
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

void RC_Control_Chassis(void)
{
	if(GetWorkState()==NORMAL_STATE)
	{
		static s16 Chassis_Vx_last=0;
		if(time_1ms_count%1==0)
		{
			if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last>1)
			{
				Chassis_Vx+=1;
			}
//			else if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last<-1)	//刹车按不缓冲
//			{
//				Chassis_Vx-=1;
//			}
			else
			{
				Chassis_Vx=RC_Ctl.rc.ch1-1024;
			}
		}
//		Chassis_Vx=RC_Ctl.rc.ch1-1024;	//代替为斜坡函数
		Chassis_Vx_last=Chassis_Vx;
	}
	
	Chassis_Vy=RC_Ctl.rc.ch0-1024;
}

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Chassis(s16 * chassis_vx,s16 * chassis_vy)	//1000Hz
{
	if(GetWorkState()==NORMAL_STATE)
	{
		if(KeyBoardData[KEY_W].value!=0)
		{
			if(*chassis_vx<660&&*chassis_vx>=0)
			{
				(*chassis_vx)++;
			}
			else if(*chassis_vx<0)
			{
				*chassis_vx=0;
			}
		}
		else if(KeyBoardData[KEY_S].value!=0)
		{
			if(*chassis_vx>-660&&*chassis_vx<=0)
			{
				(*chassis_vx)--;
			}
			else if(*chassis_vx>0)
			{
				*chassis_vx=0;
			}
		}
		else
		{
			*chassis_vx=0;
		}
		///////////////////////////////////////
		if(KeyBoardData[KEY_D].value!=0)
		{
			if(*chassis_vy<660&&*chassis_vy>=0)
			{
				(*chassis_vy)++;
			}
			else if(*chassis_vy<0)
			{
				*chassis_vy=0;
			}
		}
		else if(KeyBoardData[KEY_A].value!=0)
		{
			if(*chassis_vy>-660&&*chassis_vy<=0)
			{
				(*chassis_vy)--;
			}
			else if(*chassis_vy>0)
			{
				*chassis_vy=0;
			}
		}
		else
		{
			*chassis_vy=0;
		}
	}
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

