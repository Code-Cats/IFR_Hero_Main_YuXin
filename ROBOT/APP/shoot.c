#include "shoot.h"
#include "math.h"

SHOOT_DATA shoot_Data_Down=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data_Down ={0};

SHOOT_DATA shoot_Data_Up=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data_Up ={0};

PID_GENERAL   PID_Shoot_Down_Position=PID_SHOOT_POSITION_DEFAULT;
PID_GENERAL   PID_Shoot_Down_Speed=PID_SHOOT_SPEED_DEFAULT;

extern u32 time_1ms_count;

s16 tem=0;
u16 fri_t=800;
u16 pwm_l=660;
u16 pwm_r=2400;
u16 pwm_lc=2300;
u16 pwm_rc=800;
float pwm_l_t=660;
float pwm_r_t=2400;
void Shoot_Task(void)	//��ʱƵ�ʣ�1ms
{ 
	
	Shoot_Instruction();
	shoot_Motor_Data_Down.tarP=(s32)shoot_Data_Down.motor_tarP;
	shoot_Motor_Data_Down.tarV=PID_General(shoot_Motor_Data_Down.tarP,shoot_Motor_Data_Down.fdbP,&PID_Shoot_Down_Position);
	
	static u8 swicth_Last_state=0;	//�Ҳ���

////////	if(RC_Ctl.rc.switch_right==RC_SWITCH_UP)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//������
////////	{
////////		if(pwm_l_t-pwm_l>0.01)
////////		{
////////			pwm_l_t-=0.8;
////////		}
////////		else
////////		{
////////			pwm_l_t=pwm_l;
////////		}
////////		
////////		if(pwm_r-pwm_r_t>0.01)
////////		{
////////			pwm_r_t+=0.8;
////////		}
////////		else
////////		{
////////			pwm_r_t=pwm_r;
////////		}
////////	}	
////////	else if(RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
////////	{
////////		if(pwm_lc-pwm_l_t>0.01)
////////		{
////////			pwm_l_t+=0.8;
////////		}
////////		else
////////		{
////////			pwm_l_t=pwm_lc;
////////		}
////////		
////////		if(pwm_r_t-pwm_rc>0.01)
////////		{
////////			pwm_r_t-=0.8;
////////		}
////////		else
////////		{
////////			pwm_r_t=pwm_rc;
////////		}
////////	}
////////	PWM3_3=(u16)pwm_l_t;
////////  PWM3_4=(u16)pwm_r_t;
	if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
	{
		static u8 frc_state=0;
		frc_state=!frc_state;
		fri_t=800-(800-2000)*frc_state;
	}
////////////////	if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
/////��ʱ///////	{
/////����///////		static u8 frc_state=0;
/////����///////		frc_state=!frc_state;
/////��̬///////		fri_t=800-(800-2000)*frc_state;
////////////////	}
////////////////	else if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)
////////////////	{
////////////////		shoot_Motor_Data_Down.tarV=0;
////////////////	}
////////////////	else if(RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
////////////////	{
////////////////		shoot_Motor_Data_Down.tarV=2000;
////////////////	}
////////////////swicth_Last_state=RC_Ctl.rc.switch_right;
	shoot_Motor_Data_Down.output=PID_General(shoot_Motor_Data_Down.tarV,shoot_Motor_Data_Down.fdbV,&PID_Shoot_Down_Speed);
	SetFrictionWheelSpeed(fri_t);
	swicth_Last_state=RC_Ctl.rc.switch_right;
//	CAN_Shoot_SendMsg(shoot_Motor_Data.output);
//	CAN_Shoot_SendMsg(tem);
}


u16 shoot_time_record=0;
u16 shoot_time_measure(const s16 tarP,const s16 fbdP,const u8 last_mouse_press_l)
{
	static u8 once_statu=0;
	static u16 time_count_start=0;
	static u16 time_count_end=0;
	static u16 time_count_tem=0;
	if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)
	{
		time_count_tem=time_1ms_count;
		once_statu=0;
	}
	if(abs(tarP-fbdP)<2&&once_statu!=1)
	{
		once_statu=1;
		time_count_end=time_1ms_count;
		time_count_start=time_count_tem;
	}
	
	return  time_count_end-time_count_start;
}




#define SINGLE_INCREMENT_OLD_2006 196.608f	//8192*96/4/1000	һȦ���ۼ�ֵ8192*96����һȦ7���ӵ����Ա�����ת������=����һ���ӵ���λ������
#define SINGLE_INCREMENT_NEW_2006 37.45f		//8192*32/7/1000
#define SINGLE_INCREMENT SINGLE_INCREMENT_OLD_2006
//���Ϊ����������λ��
//ע��Ӧ���ڱ�����������һָ����������������߼����л�״̬�����÷���ָ�����ͻ�����ʹ��������ͺ��ԣ�
//���߽������߼���Ϊ��������ʽ������Ƶ�ʿ���
void Shoot_Instruction(void)	//����ָ��ģ��
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	static u8 last_mouse_press_l=0;
	if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//������
	{
		shoot_Data_Down.count++;
	}
	
	if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
	{
		shoot_Data_Down.count+=3;
	}
	
	shoot_time_record=shoot_time_measure(shoot_Data_Down.count,shoot_Data_Down.count_fdb,last_mouse_press_l);////////////////////////////////
	last_mouse_press_l=RC_Ctl.mouse.press_l;
	swicth_Last_state=RC_Ctl.rc.switch_right;
	
	shoot_Data_Down.motor_tarP=shoot_Data_Down.count*SINGLE_INCREMENT;
	Prevent_Jam(&shoot_Data_Down,&shoot_Motor_Data_Down);
}

#define G 9.80151f
/**********************************
Visual_Pretreatment
deal the Visual data
output:disitance: dm
			 priority:0-10
**********************************/
void Visual_Pretreatment()
{
	shoot_Data_Down.Visual.distance=20;
	shoot_Data_Down.Visual.priority=10;
}

/*********************************
Shoot_Rate_Set
caculate the rate data based on the visual.distance data
�����Զ����
*********************************/
void Shoot_Rate_Set()
{
	
}

/*********************************
Shoot_Frequency_Set
*********************************/
void Shoot_Frequency_Set()
{
	
}



/*********************************
Shoot_Frequency_Limit
�����Զ����
*********************************/
#define UPPER_LIMIT_OF_HEAT 4500	//��������
#define COOLING_PER_SECOND 1500	//ÿ����ȴ
void Shoot_Frequency_Limit(int* ferquency,u16 rate,u16 heat)	//m/sΪ��λ
{
	u16 heating=rate*rate;
	s16 ferquency_safe=(s16)(COOLING_PER_SECOND/heating);
	if(*ferquency!=0)
	{
		if(heat<5*heating&&heat>=2*heating)	//4������ʱ��ʼ���壬�Է�����
		{
			*ferquency=(u16)ferquency_safe+1;
		}
		else if(heat<=heating)	//����������������
		{
			*ferquency=0;
		}
		else if(heat>=heating&&heat<2*heating)
		{
			*ferquency=(u16)((ferquency_safe-1)>0?(ferquency_safe-1):0);
		}
	}

}




/*********************************
Shoot_Rate_Adjust
*********************************/
void Shoot_Rate_Adjust()
{
	
}
u32 jam_fdbP_record;
#define JAM_FALLBACK 80	//100
//��tarP�Ĳ���
void Prevent_Jam(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
{
	static s32 deviation=0;	//ƫ��
	static u8 jam_deal_state=0;
//	static u16 ferquency_last=0;
	
	deviation=shoot_motor_Data->tarP-shoot_motor_Data->fdbP;
	
//	if(shoot_data->frequency!=ferquency_last)
//	{
//		shoot_data->Jam.count=0;	//����count
//	}
//	ferquency_last=shoot_data->frequency;	//����
	
	
	if(abs(deviation)>6&&abs(shoot_motor_Data->fdbV)<10)	//�����ٶȲ�Ϊ0ʱλ��δ�����仯	//bug:Ƶ��ˢ��ʱ��Ҫˢ��count	//�ֶ������Ƶ�ʼ��ɾ��
	{
		shoot_data->Jam.count++;
	}
	else
	{
		shoot_data->Jam.count=0;
	}
	
//	if(shoot_data->cycle!=0)
//	{
		if(shoot_data->Jam.count>100&&shoot_data->Jam.sign==0)	//����������ʱ��	//�ҽ�ִ��һ��
		{
			 shoot_data->Jam.sign=1;	//��ǿ���
			 jam_deal_state=1;	//��ǿ����������״̬
		}
//	}
	
	if(shoot_data->Jam.sign==1)	//������ģ��
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					jam_fdbP_record=shoot_motor_Data->fdbP-JAM_FALLBACK;
					shoot_data->motor_tarP=jam_fdbP_record;
					jam_deal_state=2;
					break;
				}
				case 2:
				{
					shoot_data->motor_tarP=jam_fdbP_record;
					if(abs(shoot_motor_Data->fdbP-jam_fdbP_record)<40)	//��Ϊ�Ѿ�ִ���˶���	//50
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_data->Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_data->count=shoot_data->count_fdb;	//�����ӵ����ݣ���ֹ����	//���Ƿ���Ҫ+-1��
					shoot_data->Jam.count=0;	//���ÿ���������ݣ���ֹ����
					break;
				}
		}
	}
	
}   




u32 shoot_fdb_count=0;
/*****************************************
�������ƣ�Shoot_Feedback_Deal
�������ܣ���������������ݽ���+����

*****************************************/
void Shoot_Feedback_Deal(SHOOT_DATA *shoot_data,SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg)
{
	shoot_fdb_count++;
	shoot_motor_data->fdbP_raw=(msg->Data[0]<<8)|msg->Data[1];//���յ�����ʵ����ֵ  ����Ƶ��1KHz
	shoot_motor_data->fdbV=(msg->Data[2]<<8)|msg->Data[3];
	
	shoot_motor_data->fdbP_diff=shoot_motor_data->fdbP_raw_last-shoot_motor_data->fdbP_raw;
	if(shoot_motor_data->fdbP_diff>5460)	//����6�����������㣬��е�Ƕȹ�8192����λ���������ֲ�ֵΪ6826
	{																			//ע���˺���δ�Ե�һ������ʱ�Ŀ��ܵ�Ȧ��ֱ��Ϊ1��ƫ���������������ڳ�ʼ���б궨��ʼ�Ƕ�ֵ��
		shoot_motor_data->fdbP_raw_sum+=8192;
	}
	else if(shoot_motor_data->fdbP_diff<-5460)
	{
		shoot_motor_data->fdbP_raw_sum-=8192;
	}
	
	shoot_motor_data->fdbP=(s32)((shoot_motor_data->fdbP_raw_sum+shoot_motor_data->fdbP_raw)/1000);	//��Ϊ2006���ٱȹ��� ���㾫ȷ
	
	shoot_motor_data->fdbP_raw_last=shoot_motor_data->fdbP_raw;	//���ݵ���
	
	shoot_data->count_fdb=(u16)(shoot_motor_data->fdbP/SINGLE_INCREMENT);	///////��ֹ������ˢ�·�����
}




//һȦ7��
//����15С

