#include "shoot.h"
#include "math.h"

SHOOT_DATA shoot_Data=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data ={0};

PID_GENERAL   PID_Shoot_Position=PID_SHOOT_POSITION_DEFAULT;
PID_GENERAL   PID_Shoot_Speed=PID_SHOOT_SPEED_DEFAULT;

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
	static u8 swicth_Last_state=0;
	Shoot_Instruction();
//	shoot_Motor_Data.tarP=(s32)shoot_Data.motor_tarP;
//	shoot_Motor_Data.tarV=PID_General(shoot_Motor_Data.tarP,shoot_Motor_Data.fdbP,&PID_Shoot_Position);


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
	else if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)
	{
		shoot_Motor_Data.tarV=0;
	}
	else if(RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
	{
		shoot_Motor_Data.tarV=2000;
	}
	shoot_Motor_Data.output=PID_General(shoot_Motor_Data.tarV,shoot_Motor_Data.fdbV,&PID_Shoot_Speed);
	SetFrictionWheelSpeed(fri_t);
	swicth_Last_state=RC_Ctl.rc.switch_right;
//	CAN_Shoot_SendMsg(shoot_Motor_Data.output);
//	CAN_Shoot_SendMsg(tem);
}
#define SINGLE_INCREMENT 196.608f	//8192*96/4/1000	һȦ���ۼ�ֵ8192*96����һȦ7���ӵ����Ա�����ת������=����һ���ӵ���λ������
//���Ϊ����������λ��
//ע��Ӧ���ڱ�����������һָ����������������߼����л�״̬�����÷���ָ�����ͻ�����ʹ��������ͺ��ԣ�
//���߽������߼���Ϊ��������ʽ������Ƶ�ʿ���
void Shoot_Instruction(void)	//����ָ��ģ��
{
	static u8 last_mouse_press_l=0;
	if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//������
	{
		shoot_Data.count++;
	}	
	last_mouse_press_l=RC_Ctl.mouse.press_l;
	
	shoot_Data.motor_tarP=shoot_Data.count*SINGLE_INCREMENT;
//	Prevent_Jam();
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
	shoot_Data.Visual.distance=20;
	shoot_Data.Visual.priority=10;
}

/*********************************
Shoot_Rate_Set
caculate the rate data based on the visual.distance data
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
#define JAM_FALLBACK 100
//��tarP�Ĳ���
void Prevent_Jam(void)	//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
{
	static s32 deviation=0;	//ƫ��
	static u8 jam_deal_state=0;
	static u16 ferquency_last=0;
	
	deviation=shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP;
	
	if(shoot_Data.frequency!=ferquency_last)
	{
		shoot_Data.Jam.count=0;	//����count
	}
	ferquency_last=shoot_Data.frequency;	//����
	
	
	if(abs(deviation)>6&&shoot_Data.frequency!=0&&shoot_Motor_Data.fdbV<10)	//�����ٶȲ�Ϊ0ʱλ��δ�����仯	//bug:Ƶ��ˢ��ʱ��Ҫˢ��count
	{
		shoot_Data.Jam.count++;
	}
	else
	{
		shoot_Data.Jam.count=0;
	}
	
	if(shoot_Data.cycle!=0)
	{
		if(shoot_Data.Jam.count>100&&shoot_Data.Jam.sign==0)	//����������ʱ��	//�ҽ�ִ��һ��
		{
			 shoot_Data.Jam.sign=1;	//��ǿ���
			 jam_deal_state=1;	//��ǿ����������״̬
		}
	}
	
	if(shoot_Data.Jam.sign==1)	//������ģ��
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					jam_fdbP_record=shoot_Motor_Data.fdbP-JAM_FALLBACK;
					shoot_Data.motor_tarP=jam_fdbP_record;
					jam_deal_state=2;
					break;
				}
				case 2:
				{
					shoot_Data.motor_tarP=jam_fdbP_record;
					if(abs(shoot_Motor_Data.fdbP-jam_fdbP_record)<50)	//��Ϊ�Ѿ�ִ���˶���
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_Data.Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_Data.count=shoot_Data.count_fdb;	//�����ӵ����ݣ���ֹ����	//���Ƿ���Ҫ+-1��
					shoot_Data.Jam.count=0;	//���ÿ���������ݣ���ֹ����
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
void Shoot_Feedback_Deal(SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg)
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
	
	shoot_Data.count_fdb=(u16)(shoot_motor_data->fdbP/SINGLE_INCREMENT);	///////��ֹ������ˢ�·�����
}




//һȦ7��
//����15С

