#include "auto_takebullet.h"
#include "main.h"

#define LIFT_DISTANCE_BULLET 700

TakeBulletState_e TakeBulletState=BULLET_ACQUIRE;	//���Զ���ȡ��״̬λ


extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;


const u16 pwm_l=660;
const u16 pwm_r=1350;
const u16 pwm_lc=1650;
const u16 pwm_rc=650;
float pwm_l_t=660;
float pwm_r_t=1350;


u8 valve_fdbstate[6]={0};	//��¼�Ƿ�����ķ�����־
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={300,1200,300,1000,1000,1000};	//�����룬��ʱ����
const u32 valve_POORdelay[6]={300,1200,300,1000,1000,1000};	//�����룬��ʱ����
const u32 servo_GOODdelay[2]={3000,1000};	//�����룬��ʱ����	//��һ��Ϊ2000�ǽ��ӵ����µ���ʱҲ�ӽ�ȥ�ˣ���Ϊ�����ת���ӵ��������������һ���
const u32 servo_POORdelay[2]={1500,1000};	//�����룬��ʱ����


//#define VALVE_ISLAND 0		//��ŷ�����λ����
//#define VALVE_BULLET_PROTRACT 1	//ǰ��
//#define VALVE_BULLET_CLAMP 2	//�н�

u8 auto_takebullet_statu=0;
void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//�Ҳ���
//	static u8 pwm_state=0;
//	static u8 valve_state=0;
//	static u8 valve_delay_statu=0;
//	static u32 valve_time_record=0;
	
	static u8 valve_last[6]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	static u8 servo_last[2]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	
	static u32 valve_startGOOD_time[6]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startGOOD_time[2]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 valve_startPOOR_time[6]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startPOOR_time[2]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	

	if(GetWorkState()==TAKEBULLET_STATE&&RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
	{
		if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//������
		{
			auto_takebullet_statu=!auto_takebullet_statu;
			TakeBulletState=BULLET_ACQUIRE;
		}	
		else if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//��ʱ����
		{
		}
		
		
		{	//�Զ�ȡ����
			if(auto_takebullet_statu==1)	//�Զ�ȡ��
			{
				switch(TakeBulletState)
				{
					case BULLET_ACQUIRE:	//ǰ�졢�н���̧����	��֮Ϊ��ù���
					{
						ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;	//ǰ�캯��
						if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)//���ǰ��û��λ ����ִ�е�ȡ������λ�ú���	//�����������ö��ȡ�����һ��ȡ������
						{
							SetCheck_GripLift(1);	//�½���ץȡ�߶�
						}
						
						if(valve_fdbstate[VALVE_BULLET_PROTRACT]==1&&SetCheck_GripLift(1)==1)	//���ǰ�쵽λ��������λ
						{
							ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
						}
						if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)//���ǰ�쵽λ��������
						{
							if(SetCheck_GripLift(0)==1)	//����������ת�߶�
							TakeBulletState=BULLET_POUROUT;//ֱ���л�����һ״̬
						}
						break;
					}
					case BULLET_POUROUT:	//������б�������ת	��֮Ϊ��������
					{
						ViceControlData.servo[0]=1;
						if(SetCheck_SlopeLift(1)==1)//����б����
						{//������ʱ���л�����һ״̬	//�˴�ȱ����ʱ
							if(servo_fdbstate[0]==1)
							TakeBulletState=BULLET_THROWOUT;
						}
						break;
					}
					case BULLET_THROWOUT:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
					{
						ViceControlData.servo[0]=0;
						if(servo_fdbstate[0]==0)	//���ö����λ��ԭ�����Ա��õ�ҩ���ܹ�˳����λ
						{
							if(SetCheck_GripLift(0)==1)//����̧����	/
							{
								ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
							//	ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;	//ע���Ա�ƽ��ȡ��һ�ŵ�
							}//�������̧���Ҷ����λ�����ɿ��н�������һ������ȡ������
						}
						break;
					}
				}
			}
			else
			{
				ViceControlData.servo[0]=0;
				if(servo_fdbstate[0]==0)
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
				//	ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;	//ע���Ա�ƽ��ȡ��һ�ŵ�
				}
			}
		}
		
	
//		if(valve_state==0)	//���Է�������
//		{
//			ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
//			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
//			valve_delay_statu=0;
//		}
//		else
//		{
//			if(valve_delay_statu==0)
//			{
//				valve_time_record=time_1ms_count;
//				valve_delay_statu=1;
//			}
//			ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;
//			if(valve_delay_statu==1&&time_1ms_count-valve_time_record>1500)
//			{
//				ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
//			}
//			
//		}
		
		
		
		
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
			if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
			{
				valve_fdbstate[i]=1;
			}
			else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
			{
				valve_fdbstate[i]=0;
			}
			
			if(i<2)
			{
				if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
				{
					servo_fdbstate[i]=1;
				}
				else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
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
		
		
		//���ִ�п�	//��ŷ��ڸ���ִ��
		if(ViceControlData.servo[0]==0)
		{
			if(pwm_l_t-pwm_l>0.01f)
			{
				pwm_l_t-=1.5f;
			}
			else
			{
				pwm_l_t=pwm_l;
			}
			
			if(pwm_r-pwm_r_t>0.01f)
			{
				pwm_r_t+=1.5f;
			}
			else
			{
				pwm_r_t=pwm_r;
			}
		}
		else
		{
			if(pwm_lc-pwm_l_t>0.01f)
			{
				pwm_l_t+=1.5f;
			}
			else
			{
				pwm_l_t=pwm_lc;
			}
			
			if(pwm_r_t-pwm_rc>0.01f)
			{
				pwm_r_t-=1.5f;
			}
			else
			{
				pwm_r_t=pwm_rc;
			}
		}
		
		
	}
	else
	{
		ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
		ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
	}
	
	swicth_Last_state=RC_Ctl.rc.switch_right;
	
	PWM3_1=(u16)pwm_l_t;
  PWM3_2=(u16)pwm_r_t;
}


#define LIFT_DISTANCE_GRIPBULLET	630	//�е�ҩ��ʱ�߶�
#define LIFT_DISTANCE_DISGRIPBULLET	1060	//��������ҩ��߶�
#define LIFT_DISTANCE_SLOPEBACKBULLET	1170	//��бʱ���ȸ߶�
#define LIFT_DISTANCE_SLOPEFRONTBULLET	900	//��бʱǰ�ȸ߶�
extern LIFT_DATA lift_Data;

u8 SetCheck_GripLift(u8 grip_state)	//�Ƿ��뵯ҩ��ƽ��,gripץס����˼	//0��ʾ��ץס������Ҫ����ҩ������ҩ��߶ȣ�1��ʾץס������Ҫ�н���ҩ��ʱ�ĸ߶�
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET)))<30);	//�����ǽ���ǰ����Ϊ�������ص�
}

u8 SetCheck_SlopeLift(u8 slope_state)	//��ʱֻ������	slope��б����˼	//0��ʾ����б�����ָ�������ҩ��߶ȣ�1��ʾ��б������б���ӵ�״̬
{
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET)))<30);	//�����ǽ���ǰ����Ϊ�������ص�,��Ϊǰ����Ϊ�������г̸���ʱ���������ǰ���ѵ�λ�����һ����λ
}






