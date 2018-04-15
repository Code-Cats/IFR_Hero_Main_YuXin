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
extern tPowerHeatData 	testPowerHeatData;      //ʵʱ������������
extern u32 time_1ms_count;

#define K_SPEED 10
s32 t_Vw_PID=0;
s32 yaw_follow_tarP=YAW_INIT_DEFINE;
s32 yaw_follow_error=0;

u8 Chassis_Control_RCorPC=RC_CONTROL;
void Remote_Task(void)
{
	
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//ģʽ�л�
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
	
	
	
	
	if(GetWorkState()==WAIST_STATE)
	{
		static u8 turn_flag=0;
		s32 yaw_init_def=YAW_INIT_DEFINE;
		switch(turn_flag)
		{
			case 0:
			{
				YAW_INIT=yaw_init_def-750;
				if(abs(YAW_INIT-yunMotorData.yaw_fdbP)<15)
				{
					turn_flag=1;
				}
				break;
			}
			case 1:
			{
				YAW_INIT=yaw_init_def+750;
				if(abs(YAW_INIT-yunMotorData.yaw_fdbP)<15)
				{
					turn_flag=0;
				}
				break;
			}
		}
		
		PID_Chassis_Follow.k_p=CHASSIS_FOLLOW_PID_P/2.0f;
	}
	else
	{
		YAW_INIT=YAW_INIT_DEFINE;
		PID_Chassis_Follow.k_p=CHASSIS_FOLLOW_PID_P;
	}
		
	
	
	
	
	
//	yaw_follow_tarP=yunMotorData.yaw_fdbP;
	if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10>1800)	//�����
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
	
	
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE||GetWorkState()==WAIST_STATE)	//�������������ң���������������(�Զ�)�ǵ�ģʽ�½��ɳ����Զ�����
	{
//////		Chassis_Vx=RC_Ctl.rc.ch1-1024;
		
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
		if(abs(RC_Ctl.rc.ch2-1024)<40&&abs(YAW_INIT-yunMotorData.yaw_fdbP)<180)//�˴��������ڼ��ٶȹ���ʱ�������нϴ�����˲��õ�ת����ͨ���棬��ת�����ܸ���ģʽ	//�˴�Ӧ��fdb��ΪtarP
		{
			if(YAW_INIT-yunMotorData.yaw_fdbP>8192/2)	//���ܸ����	
			{
				Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP+8192,&PID_Chassis_Follow);	//������״����
				yaw_follow_error=YAW_INIT-(yunMotorData.yaw_fdbP+8192);
			}
			else if(YAW_INIT-yunMotorData.yaw_fdbP<-8192/2)
			{
				Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP-8192,&PID_Chassis_Follow);	//������״����
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP-8192);
			}
			else
			{
				Chassis_Vw=PID_General(YAW_INIT,yunMotorData.yaw_fdbP,&PID_Chassis_Follow);	//����״����
				yaw_follow_error=YAW_INIT-yaw_follow_tarP;
			}
		}
		else
		{
			if(YAW_INIT-yaw_follow_tarP>8192/2)	//���ܸ����	
			{
				Chassis_Vw=PID_General(YAW_INIT,yaw_follow_tarP+8192,&PID_Chassis_Follow);	//������״����
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP+8192);
			}
			else if(YAW_INIT-yaw_follow_tarP<-8192/2)
			{
				Chassis_Vw=PID_General(YAW_INIT,yaw_follow_tarP-8192,&PID_Chassis_Follow);	//������״����
				yaw_follow_error=YAW_INIT-(yaw_follow_tarP-8192);
			}
			else
			{
				Chassis_Vw=PID_General(YAW_INIT,yaw_follow_tarP,&PID_Chassis_Follow);	//����״����
				yaw_follow_error=YAW_INIT-yaw_follow_tarP;
			}
		}
			


		Chassis_Vw=chassis_Vw_filter(Chassis_Vw);	//���ٶ�һ���˲�
		
		
//		Chassis_Vw=(s16)(FirstOrder_General((YAW_INIT-yunMotorData.yaw_fdbP),&Yaw_Follow_Filter)*0.43f);
//		Chassis_Vw=(s16)((YAW_INIT-yunMotorData.yaw_fdbP)*0.6f);	//YUN_INITΪĿ��λ�ã���ΪYAW_INIT-
	}
	

	if(GetWorkState()==NORMAL_STATE)
	{	//����ת���
		s16 Vx_record=Chassis_Vx;
		Chassis_Vx=0;
		yaw_follow_error=yaw_follow_error/8192*2*PI;
		Chassis_Vx+=(s16)(Vx_record*(cos(yaw_follow_error)));
		Chassis_Vy+=(s16)(Vx_record*(sin(yaw_follow_error)));
	}
	
	if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	//��̨����ת��
	{
		float chassis_vw_record=Chassis_Vw;
		Chassis_Vy-=(s16)(chassis_vw_record/1.7);	//2
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

	{	//�������ƿ�
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
}


void RC_Control_Chassis(void)
{
	static s16 Chassis_Vx_last=0;
	static s16 Chassis_Vy_last=0;
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)
	{
		
		if(time_1ms_count%1==0)
		{
			if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last>1&&RC_Ctl.rc.ch1-1024>10)	//ֻ��ǰ������ʱ��Ч��
			{
				Chassis_Vx+=1;
			}
			else if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last<-1&&RC_Ctl.rc.ch1-1024<-10)	//ֻ�ں��˼���ʱ��Ч
			{
				Chassis_Vx-=1;
			}
			else
			{
				Chassis_Vx=RC_Ctl.rc.ch1-1024;
			}
		}
//		Chassis_Vx=RC_Ctl.rc.ch1-1024;	//����Ϊб�º���
		Chassis_Vx_last=Chassis_Vx;
	}
	
	if(time_1ms_count%1==0)
	{
		if(RC_Ctl.rc.ch0-1024-Chassis_Vy_last>1&&RC_Ctl.rc.ch0-1024>10)
		{
			Chassis_Vy+=1;
		}
		else if(RC_Ctl.rc.ch0-1024-Chassis_Vy_last<-1&&RC_Ctl.rc.ch0-1024<-10)	//ɲ����������
		{
			Chassis_Vy-=1;
		}
		else
		{
			Chassis_Vy=RC_Ctl.rc.ch0-1024;
		}
	}
	Chassis_Vy_last=Chassis_Vy;
//	Chassis_Vy=RC_Ctl.rc.ch0-1024;
}

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Chassis(s16 * chassis_vx,s16 * chassis_vy)	//1000Hz
{
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)
	{
		if(time_1ms_count%2==0)
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

#define POWERLIMIT 120 	//120w��������
#define POWERBUFFER 60	//60J���ʻ���
float Limit_Power(float power,float powerbuffer)	//Ӣ��120J�������ƣ�ֱ�����������
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

