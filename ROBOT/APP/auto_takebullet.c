#include "auto_takebullet.h"
#include "main.h"

#define LIFT_DISTANCE_BULLET 700

TakeBulletState_e TakeBulletState=BULLET_ACQUIRE;	//���Զ���ȡ��״̬λ


extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;


const u16 pwm_l=660;
const u16 pwm_r=1400;
const u16 pwm_lc=1700;
const u16 pwm_rc=700;
float pwm_l_t=660;
float pwm_r_t=1400;


u8 valve_fdbstate[6]={0};	//��¼�Ƿ�����ķ�����־
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={0};	//�����룬��ʱ����
const u32 valve_POORdelay[6]={0};	//�����룬��ʱ����
const u32 servo_GOODdelay[2]={0};	//�����룬��ʱ����
const u32 servo_POORdelay[2]={0};	//�����룬��ʱ����

u8 auto_takebullet_statu=0;
void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	static u8 pwm_state=0;
	static u8 valve_state=0;
	static u8 valve_delay_statu=0;
	static u32 valve_time_record=0;
	
	static u8 valve_last[6]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	static u8 servo_last[2]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	
	static u32 valve_startGOOD_time[6]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startGOOD_time[2]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 valve_startPOOR_time[6]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startPOOR_time[2]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	

	if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
	{
		if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//������
		{
			auto_takebullet_statu=!auto_takebullet_statu;
			valve_state=!valve_state;
		}	
		else if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
		{
			pwm_state=!pwm_state;
		}
		
		
		{	//�Զ�ȡ����
			if(auto_takebullet_statu==1)	//�Զ�ȡ��
			{
				switch(TakeBulletState)
				{
					case BULLET_ACQUIRE:	//ǰ�졢�н���̧����	��֮Ϊ��ù���
					{
						
						break;
					}
					case BULLET_POUROUT:	//������б�������ת	��֮Ϊ��������
					{
						break;
					}
					case BULLET_THROWOUT:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
					{
						break;
					}
				}
			}
			else
			{
				
			}
		}
		
		
		if(valve_state==0)	//���Է�������
		{
			ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			valve_delay_statu=0;
		}
		else
		{
			if(valve_delay_statu==0)
			{
				valve_time_record=time_1ms_count;
				valve_delay_statu=1;
			}
			ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;
			if(valve_delay_statu==1&&time_1ms_count-valve_time_record>1500)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
			}
			
		}
		
		
		
		
/******************************************************************
��������forΪ���������ⷽ��
�ֱ�Ϊ��
1.�������½��صĴ���ʱ���¼
2.���ݴ���ʱ������ݷ���ֵ����
3.���ݵ���
******************************************************************/

		for(int i=0;i<6;i++)	//����ʱ���
		{
			if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//�������
			{
				valve_startGOOD_time[i]=time_1ms_count;
			}
			else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//�ջش���
			{
				valve_startPOOR_time[i]=time_1ms_count;
			}
			
			if(i<2)
			{
				if(servo_last[i]==0&&ViceControlData.servo[i]==1)
				{
					servo_startGOOD_time[i]=time_1ms_count;
				}
				else if(servo_last[i]==1&&ViceControlData.servo[i]==0)
				{
					servo_startPOOR_time[i]=time_1ms_count;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//��������λ
		{
			if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>1000)	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
			{
				valve_fdbstate[i]=1;
			}
			else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>1000)	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
			{
				valve_fdbstate[i]=0;
			}
			
			if(i<2)
			{
				if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>1000)	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
				{
					servo_fdbstate[i]=1;
				}
				else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>1000)	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
				{
					servo_fdbstate[i]=0;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//������
		{
			valve_last[i]=ViceControlData.valve[i];
			if(i<2)	servo_last[i]=ViceControlData.servo[i];
		}
////////////////////////////////////////////////////////////////////////////////////////////�������־
		
		
		//���ִ�п�
		if(ViceControlData.servo[0]==0)
		{
			if(pwm_l_t-pwm_l>0.01)
			{
				pwm_l_t-=1.5;
			}
			else
			{
				pwm_l_t=pwm_l;
			}
			
			if(pwm_r-pwm_r_t>0.01)
			{
				pwm_r_t+=1.5;
			}
			else
			{
				pwm_r_t=pwm_r;
			}
		}
		else
		{
			if(pwm_lc-pwm_l_t>0.01)
			{
				pwm_l_t+=1.5;
			}
			else
			{
				pwm_l_t=pwm_lc;
			}
			
			if(pwm_r_t-pwm_rc>0.01)
			{
				pwm_r_t-=1.5;
			}
			else
			{
				pwm_r_t=pwm_rc;
			}
		}
		
		
	}
	
	swicth_Last_state=RC_Ctl.rc.switch_right;
	
	PWM3_1=(u16)pwm_l_t;
  PWM3_2=(u16)pwm_r_t;
}
