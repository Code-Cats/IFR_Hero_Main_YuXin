#include "auto_lift.h"
#include "common_definition.h"


AscendState_e AscendState=FULLRISE_GO1;	//�ǵ�״̬λ

extern PID_GENERAL PID_Chassis_Speed[4];
//rise fall 
//
void Ascend_Control_Center(void)	//ȫ�Զ��ǵ���������
{
	
	for(int i=0;i<4;i++)
	{
		PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I*3;
		PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX*1.5f;
	}
	
	switch(AscendState)
	{
		case FULLRISE_GO1:
		{
			if(Ascend_FullRise_GO1()==1)
				AscendState=BACKFALL_GO1;
			break;
		}
		case BACKFALL_GO1:
		{
			if(Ascend_BackFall_GO()==1)
				AscendState=FULLFALL_GO1;
			break;
		}
		case FULLFALL_GO1:
		{
			if(Ascend_FullFall_GO()==1)
				AscendState=FULLRISE_GO2;
			break;
		}
		case FULLRISE_GO2:
		{
			if(Ascend_FullRise_GO2()==1)
				AscendState=BACKFALL_GO2;
			break;
		}
		case BACKFALL_GO2:
		{
			if(Ascend_BackFall_GO()==1)
				AscendState=FULLFALL_GO2;
			break;
		}
		case FULLFALL_GO2:
		{
			if(Ascend_FullFall_GO()==1)
			{
			SetWorkState(NORMAL_STATE);
			AscendState=FULLRISE_GO1;	//���÷�ֹ��һ���쳣
			for(int i=0;i<4;i++)
			{
				PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
				PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
			}
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼״̬
			}
		//		AscendState=FULLFALL_GO1;
			break;
		}
	}
	
	
}


extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern s16 Chassis_Vw;

extern SensorDataTypeDef SensorData;

extern u32 time_1ms_count;

#define VW_REDRESS 30
#define VX_SPEED_UP 110
#define VX_SPEED_NEAR 10
#define VX_PRESS_V0 50

//���⿪����Ϊ�ͣ�����������0
//��λ
	//���¶����Գ���ǰ������Ϊǰ��
u8 Ascend_FullRise_GO1(void)	//ǰ�����������������Ⱥ���
{
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//����Ĭ�ϵ��̽�������������IF���ִ�н��Ḳ��
	
	SetCheck_FrontLift(1);
	SetCheck_BackLift(1);
	Chassis_Vw=0;
		if(SetCheck_FrontLift(1)!=1||SetCheck_BackLift(1)!=1)
		{
			Chassis_Vx=-70;	//û����ȫ������ʱ
			Chassis_Vw=0;
		}
		else
		{
			Chassis_Vx=-VX_SPEED_UP;
			Chassis_Vw=0;
		}
		
		
		if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==0)	//���뵽��
		{
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=0;
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==1&&SensorData.Infrare[3]==0)	//����С�����ұߣ�1����δ����
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=VW_REDRESS;	//����	˳ʱ��ת��
		}
		
		
		if(SensorData.Limit[2]==1&&SensorData.Limit[3]==1)
		{
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=0;
				///////////////////////���õ���һ��״̬
			return 1;
		}
		else if(SensorData.Limit[2]==0&&SensorData.Limit[3]==1)	//0����δ����
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
		}
		else if(SensorData.Limit[2]==1&&SensorData.Limit[3]==0)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=VW_REDRESS;	//���� ��ʱ��ת��
		}
		
		return 0;
}



u8 Ascend_BackFall_GO(void)	//3��4������̧��ǰ���Ľ���
{
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//����Ĭ�ϵ��̽�������������IF���ִ�н��Ḳ��
	SetCheck_FrontLift(1);
	SetCheck_BackLift(0);
	Chassis_Vw=0;
	if(SetCheck_FrontLift(1)==1&&SetCheck_BackLift(0)==1)	//���̧�ȶ�����
	{
		Chassis_Vx=-VX_SPEED_UP;	//���ֵ�ڼ�⵽��ᱻif�е�ֵ����	//����ӿ����ٶ�-����
		Chassis_Vw=0;
		if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==0)
		{
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=0;
		}
		else if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==0)	//����С�����ұߣ�1����δ����
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
		}
		else if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=VW_REDRESS;	//���� ˳ʱ��ת��
		}
		
		
		if(SensorData.Limit[0]==1&&SensorData.Limit[1]==1)
		{
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=0;
			////////////////////////////////�л�����һ��״̬
			return 1;
		}
		else if(SensorData.Limit[0]==0&&SensorData.Limit[1]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
		}
		else if(SensorData.Limit[0]==1&&SensorData.Limit[1]==0)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=VW_REDRESS;	//���� ˳ʱ��ת��
		}
			
	}
	return 0;
}



u8 Ascend_FullFall_GO(void)	//��̧�𵽶�̧���	//ͨ���۲���Ƶ��֪����ײһ˲�䵯������˲������û�л���Ч��������ʱ
{	//�ý׶����⣺�������ƫ�����أ�����Ӱ��
	static u32 time_record=0;	//���������λ�û�������
	static u32 time_record_all=0;
	
	if(time_record_all==0)
	{
		time_record_all=time_1ms_count;
	}
	if(time_1ms_count-time_record_all>450)	//��ʱ0.6s
	{
			Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//����Ĭ�ϵ��̽�������������IF���ִ�н��Ḳ��
			SetCheck_FrontLift(0);
			SetCheck_BackLift(0);
			Chassis_Vw=0;
			
			if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)	//��������
			{
				if(time_record==0)	//time_record=0��˼��Ϊ��һ��ִ��
				{
					time_record=time_1ms_count;
					Chassis_Vx=-160;
					Chassis_Vw=0;
				}
				
				if(time_1ms_count-time_record>650)	//0.6s
				{
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=0;
					/////////////////////////////���õ���һ��״̬
					time_record=0;
					time_record_all=0;
					return 1;
					
				}
			}
			else	//��δ��������У�ʵ�ʲ��Ի���ƫ�ƣ��ʼ��ϴ��н���
			{
				if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==0)
				{
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=0;
				}
				else if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==0)	//����С�����ұߣ�1����δ����
				{
		//			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
				}
				else if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==1)
				{
		//			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
					Chassis_Vx=-VX_PRESS_V0;
					Chassis_Vw=VW_REDRESS;	//���� ˳ʱ��ת��
				}
			}
	}
	return 0;
}

u8 Ascend_FullRise_GO2(void)	//ǰ�����������������Ⱥ���
{ 
	Set_Attitude_Correct_State(CORRECT_CHASSIS_STATE);	//����Ĭ�ϵ��̽�������������IF���ִ�н��Ḳ��
	SetCheck_FrontLift(1);
	SetCheck_BackLift(1);
	Chassis_Vw=0;
	if(SetCheck_FrontLift(1)!=1||SetCheck_BackLift(1)!=1)
		{
			Chassis_Vx=-60;	//û����ȫ������ʱ
			Chassis_Vw=0;
		}
		else
		{
			Chassis_Vx=-VX_SPEED_UP;
			Chassis_Vw=0;
		}
		
		
		if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==0)	//���뵽��
		{
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=0;
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==1&&SensorData.Infrare[3]==0)	//����С�����ұߣ�1����δ����
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
		}
		else if(SetCheck_FrontLift(1)==1&&SensorData.Infrare[2]==0&&SensorData.Infrare[3]==1)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_SPEED_NEAR;
			Chassis_Vw=VW_REDRESS;	//����	˳ʱ��ת��
		}
		
		
		if(SensorData.Limit[2]==1&&SensorData.Limit[3]==1)
		{
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=0;
				///////////////////////���õ���һ��״̬
			return 1;
		}
		else if(SensorData.Limit[2]==0&&SensorData.Limit[3]==1)	//0����δ����
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=-VW_REDRESS;	//���� ��ʱ��ת��
		}
		else if(SensorData.Limit[2]==1&&SensorData.Limit[3]==0)
		{
			Set_Attitude_Correct_State(CALI_SELF_STATE);	//��̬��������������У׼
			Chassis_Vx=-VX_PRESS_V0;
			Chassis_Vw=VW_REDRESS;	//���� ��ʱ��ת��
		}
		
	return 0;	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////��Ϊ�ǵ�����Ϊ�µ�///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////



DescendState_e DescendState=FULLFALL_DOWN1;	//�µ�״̬��¼λ

void Descend_Control_Center(void)
{
	
	for(int i=0;i<4;i++)
	{
		PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I*3;
		PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX*1.5f;
	}
	
	
		switch(DescendState)
		{
			case FULLFALL_DOWN1:
			{
				if(Descend_FullFall_Down()==1)
					DescendState=FRONTRISE_DOWM1;
				break;
			}
			case FRONTRISE_DOWM1:
			{
				if(Descend_FrontRise_Down()==1)
					DescendState=FULLRISE_DOWN1;
				break;
			}
			case FULLRISE_DOWN1:
			{
				if(Descend_FullRise_Down1()==1)
					DescendState=FULLFALL_DOWN2;
				break;
			}
			case FULLFALL_DOWN2:
			{
				if(Descend_FullFall_Down()==1)
					DescendState=FRONTRISE_DOWN2;
				break;
			}
			case FRONTRISE_DOWN2:
			{
				if(Descend_FrontRise_Down()==1)
					DescendState=FULLRISE_DOWN2;
				break;
			}
			case FULLRISE_DOWN2:
			{
				if(Descend_FullRise_Down1()==1)
				{
					DescendState=FULLFALL_DOWN1;	//���÷�ֹ��һ���쳣
					for(int i=0;i<4;i++)
					{
						PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
						PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
					}
					SetWorkState(NORMAL_STATE);
				}
				break;
			}
		}
}



#define VX_NEAR_DOWN 40
u8 Descend_FullFall_Down(void)
{
  Chassis_Vx=0;
	Chassis_Vw=0;
	
	SetCheck_FrontLift(0);
	SetCheck_BackLift(0);
	if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)
	{
		Chassis_Vx=VX_NEAR_DOWN;
		
		if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==1)
		{
			Chassis_Vx=0;
			Chassis_Vw=0;
				///////////////////////���õ���һ��״̬
			return 1;
		}
		else if(SensorData.Infrare[0]==0&&SensorData.Infrare[1]==1)	//0����δ����(���ڵ���)
		{
			Chassis_Vx=0;
			Chassis_Vw=30;	//���� ˳ʱ��ת��
		}
		else if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==0)
		{
			Chassis_Vx=0;
			Chassis_Vw=-30;	//���� ��ʱ��ת��
		}
		
	}
	
	return 0;
}
 

u8 Descend_FrontRise_Down(void)
{
	Chassis_Vx=0;
	Chassis_Vw=0;
	
	SetCheck_FrontLift(1);
	SetCheck_BackLift(0);
	
	if(SetCheck_FrontLift(1)==1&&SetCheck_BackLift(0)==1)	//���ǰ�����Ⱥ�
	{
			Chassis_Vx=VX_NEAR_DOWN-5;
			Chassis_Vw=0;
		if(SensorData.Infrare[2]==1&&SensorData.Infrare[3]==1)
		{
			Chassis_Vx=0;
			Chassis_Vw=0;
				///////////////////////���õ���һ��״̬
			return 1;
		}
		else if(SensorData.Infrare[2]==0&&SensorData.Infrare[3]==1)	//0����δ����(���ڵ���)
		{
			Chassis_Vx=0;
			Chassis_Vw=30;	//���� ˳ʱ��ת��
		}
		else if(SensorData.Infrare[2]==1&&SensorData.Infrare[3]==0)
		{
			Chassis_Vx=0;
			Chassis_Vw=-30;	//���� ��ʱ��ת��
		}
		
	}
	return 0;
}


u8 Descend_FullRise_Down1(void)
{
	static u32 time_record=0;
		
	Chassis_Vx=0;
	Chassis_Vw=0;
	
	SetCheck_FrontLift(1);
	SetCheck_BackLift(1);
	
	if(SetCheck_FrontLift(1)==1&&SetCheck_BackLift(1)==1)	//��ɺ������Ⱥ�
	{
		if(time_record==0)	//time_record=0��˼��Ϊ��һ��ִ��
		{
			time_record=time_1ms_count;
		}
		
		Chassis_Vx=VX_NEAR_DOWN+50;	//40
		Chassis_Vw=0;
		
		if(time_1ms_count-time_record>100)
		{
			SetCheck_FrontLift(0);	//��ȫ����
			SetCheck_BackLift(0);
		}
		
		if(time_1ms_count-time_record>1200)
		{
			Chassis_Vx=0;
			Chassis_Vw=0;
			time_record=0;
			return 1;
		}
		
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*******************************��̬��У������***************************************/
#define ATTITUDE_CORRECT_KP 10	//yaw�ĵ�λΪ�ȣ�-180��+180�����ֽ���
#define ATTITUDE_CORRECT_KD 0.008f

extern GYRO_DATA Gyro_Data;
extern float Chassis_GYRO[3];	//�ںϺ�ĵ�������
AttitudeCorrectState_e Attitude_Correct_State=CALI_SELF_STATE;
float tar_attitude=0;
float Attitude_error=0;
s16 t_Vw_correct=0;
//�����ǵķ�����ʱ��Ϊ������̬��ת˳ʱ��Ϊ��
s16 Chassis_Attitude_Correct(float fdbP,int16_t fdbV)	//��״̬��⼯�������ڲ�
{
	Attitude_error=fdbP-tar_attitude;  
	s16 Vw_Correct=0;
	
	Attitude_error=Attitude_error>180.0f?(Attitude_error-360.0f):(Attitude_error<-180.0f?(Attitude_error+360.0f):Attitude_error);
	
	switch (Attitude_Correct_State)
	{
		case CALI_SELF_STATE:	//У׼Ŀ��λ��(�Լ�)
		{
			tar_attitude=fdbP;
			Vw_Correct=0;
			break;
		}
		case CORRECT_CHASSIS_STATE:	//����������̬
		{
			Vw_Correct=ATTITUDE_CORRECT_KP*(s16)(Attitude_error);	//+(s16)(ATTITUDE_CORRECT_KD*fdbV)
			break;
		}
	}
	
	Vw_Correct=Vw_Correct>100?100:Vw_Correct;
	Vw_Correct=Vw_Correct<-100?-100:Vw_Correct;
	t_Vw_correct=Vw_Correct;
	return Vw_Correct;
}

void Set_Attitude_Correct_State(AttitudeCorrectState_e state)
{
	Attitude_Correct_State=state;
}






///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


extern LIFT_DATA lift_Data;

u8 SetCheck_FrontLift(u8 rise_state)	//ǰ����������/���²����	//0��ʾFALL��1��ʾISLAND
{
	lift_Data.lf_lift_tarP=FALL-(rise_state!=0)*(FALL-ISLAND);
	lift_Data.rf_lift_tarP=FALL-(rise_state!=0)*(FALL-ISLAND);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(FALL-(rise_state!=0)*(FALL-ISLAND)))<30);	//20
}

u8 SetCheck_BackLift(u8 rise_state)
{
	lift_Data.lb_lift_tarP=FALL-(rise_state!=0)*(FALL-ISLAND);
	lift_Data.rb_lift_tarP=FALL-(rise_state!=0)*(FALL-ISLAND);
	
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(FALL-(rise_state!=0)*(FALL-ISLAND)))<30);	//20
}

