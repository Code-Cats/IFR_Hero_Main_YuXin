#include "yun.h"
#include "remote_analysis.h"
s32 YAW_INIT=YAW_INIT_DEFINE;
/*
����ṹ��yaw���ݶ������ٶȻ�//���ڼƻ��������������λ�û�����ѡ��
pitch��λ�û�

����������Ϣ�����������Ϊ
yaw�ٶȣ�����λ��
pitchλ��


*/
YUN_MOTOR_DATA 			yunMotorData=YUN_MOTOR_DATA_DEFAULT;
YUN_DATA          	yunData=YUN_DATA_DEFAULT;

PID_GENERAL          PID_PITCH_POSITION=PID_PITCH_POSITION_DEFAULT;
PID_GENERAL          PID_PITCH_SPEED=PID_PITCH_SPEED_DEFAULT;
PID_GENERAL          PID_YAW_POSITION=PID_YAW_POSITION_DEFAULT;
PID_GENERAL          PID_YAW_SPEED=PID_YAW_SPEED_DEFAULT;

extern  FIRST_ORDER_FILTER   FILTER_MOUSE_YAW;
extern  FIRST_ORDER_FILTER   FILTER_WAIST_YAW;
extern  MPU6050_REAL_DATA    MPU6050_Real_Data;
extern	RC_Ctl_t RC_Ctl;
extern GYRO_DATA Gyro_Data;
extern IslandAttitudeCorrectState_e IslandAttitude_Correct_State;	//�ǵ���̬��У��
extern u8 Chassis_Follow_Statu;	//���̸����־λ
extern volatile float yaw_follow_real_error;	//Ť��ʱ�ĵ��̸���ƫ��
extern float yaw_follow_error;	//��ͨʱ�ĵ��̸������

extern u32 time_1ms_count;
s32 t_pitch____=0;
s32 t_yaw___=0;
void Yun_Task(void)	//��̨�������� 
{
	Yun_Control_External_Solution();
}


u8 Yun_Control_RCorPC=RC_CONTROL;
void Yun_Control_External_Solution(void)	//���÷�������
{
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//ģʽ�л�
	{
		if(RC_Ctl.mouse.press_l==1||RC_Ctl.mouse.press_r==1||RC_Ctl.mouse.x>1||RC_Ctl.mouse.y>1)
		{
			Yun_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch2-1024)>3||abs(RC_Ctl.rc.ch3-1024)>3)
		{
			Yun_Control_RCorPC=RC_CONTROL;
		}
	}
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)	//��������ģʽ���ܿ�	//ȡ���ܿ�Ϊ��ʱ���룬֮���Դ������Զ�����	//ȡ���ܿ���ȡ������̨�������
	{
		if(Yun_Control_RCorPC==PC_CONTROL)
		{	//PC��������
			PC_Control_Yun(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
		}
		else if(Yun_Control_RCorPC==RC_CONTROL)
		{	//RC��������
			RC_Control_Yun(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
		}
	}
	
	
	if(GetWorkState()==ASCEND_STATE||GetWorkState()==DESCEND_STATE)	//�ǵ���̬���� ����У��״̬ʱ���õ��̸��棬������������״̬������̨��λ
	{
		switch(IslandAttitude_Correct_State)
		{
			case CALI_SELF_STATE:
			{
				yunMotorData.yaw_tarP=(s32)(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//�����Ŵ�10������Ŀ��λ����Ϊ�е�
				break;
			}
			case CORRECT_CHASSIS_STATE:
			{
				
				break;
			}
		}
	}
	else if(GetWorkState()==TAKEBULLET_STATE)	//ȡ��ģʽһֱУ׼
	{
		yunMotorData.yaw_tarP=(s32)(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//�����Ŵ�10������Ŀ��λ����Ϊ�е�
	}

	
	yunMotorData.pitch_tarV=-PID_General(yunMotorData.pitch_tarP,(Gyro_Data.angle[0]*8192/360.0f+PITCH_INIT),&PID_PITCH_POSITION);
		
	if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10>1800)	//�����
	{
		yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[2]*10+3600,&PID_YAW_POSITION);
	}
	else if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10<-1800)
	{
		yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[2]*10-3600,&PID_YAW_POSITION);
	}
	else
	{
		yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[2]*10,&PID_YAW_POSITION);
	}
	
	yunMotorData.pitch_output=PID_General(yunMotorData.pitch_tarV,(-Gyro_Data.angvel[1]/10.0),&PID_PITCH_SPEED);
	yunMotorData.yaw_output=PID_General(yunMotorData.yaw_tarV,(-Gyro_Data.angvel[2]/10.0),&PID_YAW_SPEED);	//�������������������
}


void Yun_Control_Inscribe_Solution(void)	//�ڽӷ�������
{
	//		yunMotorData.yaw_tarV=(int32_t)((RC_Ctl.rc.ch2-1024)*300.0/660.0);
//	yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[2]*10,&PID_YAW_POSITION);
//	yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,yunMotorData.yaw_fdbP,&PID_YAW_POSITION);
	{	//���ð��������ǵĴ���-�ٶ�
		//yunMotorData.yaw_output=PID_General(yunMotorData.yaw_tarV,MPU6050_Real_Data.Gyro_Z,&PID_YAW_SPEED);
		//yunMotorData.pitch_tarV=-PID_General(yunMotorData.pitch_tarP,yunMotorData.pitch_fdbP,&PID_PITCH_POSITION);
	}
	



//	yunMotorData.pitch_tarV=yun_pitch_tarV(yunMotorData.pitch_tarV);
}

void RC_Control_Yun(s32 * yaw_tarp,s32 * pitch_tarp)	//1000Hz
{
	if(time_1ms_count%15==0)	//66.67hz
	{
		yunMotorData.yaw_tarP-=(int32_t)((RC_Ctl.rc.ch2-1024)*20.0/660.0);
		yunMotorData.yaw_tarP=yunMotorData.yaw_tarP>1800?yunMotorData.yaw_tarP-3600:yunMotorData.yaw_tarP;	//�����
		yunMotorData.yaw_tarP=yunMotorData.yaw_tarP<-1800?yunMotorData.yaw_tarP+3600:yunMotorData.yaw_tarP;	//�����
	}
	
	yunMotorData.pitch_tarP=(int32_t)(-(RC_Ctl.rc.ch3-1024)*460.0/660.0)+PITCH_INIT;	//-50����Ϊ������ˮƽʱ��̨����
}


#define YUN_UPMAX 430
#define YUN_DOWNMAX 430	//ƫ����
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Yun(s32 * yaw_tarp,s32 * pitch_tarp)	//1000Hz	
{
	static float yaw_tarp_float=0;
	static float pitch_tarp_float=PITCH_INIT;
	static u8 start_state=0;	//��ʼλ��

	if(start_state==0)
	{
		yaw_tarp_float=(float)*yaw_tarp;
		start_state=1;
	}
	static u8 keyQ_last,keyE_last=0;	//��ʱ����
	if(keyQ_last==0&&KeyBoardData[KEY_Q].value==1&&abs(yaw_follow_error)<PI/10)
	{
		yaw_tarp_float+=900;
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//�����
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//�����
	}
	keyQ_last=KeyBoardData[KEY_Q].value;
	
	if(keyE_last==0&&KeyBoardData[KEY_E].value==1&&abs(yaw_follow_error)<PI/10)
	{
		yaw_tarp_float-=900;
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//�����
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//�����
	}
	keyE_last=KeyBoardData[KEY_E].value;
	
	if(time_1ms_count%10==0)
	{
		yaw_tarp_float-=RC_Ctl.mouse.x*15.0f/40.0f;
		pitch_tarp_float+=RC_Ctl.mouse.y*2.0f/3.0f;	//2/4
		
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//�����
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//�����
		
		pitch_tarp_float=pitch_tarp_float>(PITCH_INIT+500)?(PITCH_INIT+500):pitch_tarp_float;	//�����г�
		pitch_tarp_float=pitch_tarp_float<(PITCH_INIT-650)?(PITCH_INIT-650):pitch_tarp_float;	//�����г�
		
		*yaw_tarp=(s32)yaw_tarp_float;
		*pitch_tarp=(s32)pitch_tarp_float;
	}
}


float yaw_move_optimize_PC(s16 mouse_x)
{
	return 0;
}

//s16 __t_yaw_offset=0;
//	void __yun_yaw_offset(void)
//	{
//	static s32 Last_V_yaw=0;
//	static float a=0.05f;
//	__t_yaw_offset=(s32)(Last_V_yaw*(1-a)+yunMotorData.yaw_output*a);
//	Last_V_yaw=__t_yaw_offset;
//	}

s32 yun_pitch_tarV(s32 now_V)
{
	static s32 Last_V=0;
	static s32 now_acc_V=0;
	static float a=0.97f;
	now_acc_V=(s32)(Last_V*(1-a)+now_V*a);
	Last_V=now_acc_V;
	return now_acc_V;
}



/***************************************
�������ƣ�Yaw_Angle_Calculate
�������ܣ�ͨ����ǰ��е�Ƕ�����ֵ��е�ǶȱȽϵõ�ʵ�ʲ��
������������ǰ��е�Ƕȣ�src_angle
          ��ֵ��е�Ƕȣ�Init
��������ֵ��ʵ�ʲ�ǣ�output
��������
****************************************/
float Yaw_Angle_Calculate(int16_t src_angle,int16_t Init)
{
    float output=-(float)(src_angle-Init)/8192*2*PI;	
	  return output;
}
//��¼����ֵ����
#define YAW_OFFSET_COUNT 11
const s32 YAW_OFFSET_VALUE[YAW_OFFSET_COUNT][2]=\
{\
	{5310,-620},\
	{5200,-530},\
	{5100,-475},\
	{5000,-390},\
	{4900,-110},\
	{4800,70},\
	{4700,280},\
	{4600,400},\
	{4500,470},\
	{4400,530},\
	{4300,573},\
};	//3.6���

//const s32 YAW_OFFSET_VALUE[YAW_OFFSET_COUNT][2]=\
{\
	{5310,-1000},\
	{5180,-800},\
	{5000,-700},\
	{4910,-600},\
	{4800,-350},\
	{4710,0},\
	{4600,20},\
	{4500,240},\
	{4450,420},\
};	//��

#define PITCH_OFFSET_COUNT 12
const s32 PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT][2]=\
{\
	{6600,-3500},\
	{6500,-2800},\
	{6400,-2500},\
	{6300,-2300},\
	{6200,-1920},\
	{6100,-1465},\
	{6000,-1300},\
	{5900,-1186},\
	{5800,-1200},\
	{5700,-1000},\
	{5600,-900},\
	{5500,-800},\
};	//ԭ�棬����
//const s32 PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT][2]=\
{\
	{6600,-3000},\
	{6500,-2400},\
	{6400,-2100},\
	{6300,-1900},\
	{6200,-1750},\
	{6100,-1400},\
	{6000,-1200},\
	{5900,-1186},\
	{5800,-1100},\
	{5700,-1000},\
	{5600,-900},\
	{5500,-800},\
};

s32 Yaw_output_offset(s32 yaw_fbdP)	//�˷���̨yaw������������ǶԳ��ԵĲ��� //��Ȼyaw��̨����������������������Ϊ�����ﲢ��Ϊ��̨�������Ӧ���õ�ǰ����λ��������
{
	s32 offset=0;
	int i=0;
	
	yaw_fbdP=yaw_fbdP>YAW_OFFSET_VALUE[0][0]?YAW_OFFSET_VALUE[0][0]:yaw_fbdP;
	yaw_fbdP=yaw_fbdP<YAW_OFFSET_VALUE[YAW_OFFSET_COUNT-1][0]?YAW_OFFSET_VALUE[YAW_OFFSET_COUNT-1][0]:yaw_fbdP;
	
	for(i=0;i<YAW_OFFSET_COUNT;i++)	//��������Ѱ��λ��
	{
		if(yaw_fbdP>=YAW_OFFSET_VALUE[i][0]) break;
	}
	
	i=i>YAW_OFFSET_COUNT-2?YAW_OFFSET_COUNT-2:i;	//���Ƶ������ڶ���Ԫ�ص�λ�ã�������һ����������Խ��
	
	offset=YAW_OFFSET_VALUE[i][1]+(YAW_OFFSET_VALUE[i+1][1]-YAW_OFFSET_VALUE[i][1])*(YAW_OFFSET_VALUE[i][0]-yaw_fbdP)/(YAW_OFFSET_VALUE[i][0]-YAW_OFFSET_VALUE[i+1][0]);
	return offset;
}

s16 Pitch_output_offset(s32 pitch_tarP)	//�˷���̨pitch������������ǶԳ��ԵĲ���	//��Ϊ��̨pitch���������������������ⲿ���������ֻ����һ����Сֵ�����ʲ���tarP��Ϊ�������տ�����߼�ӷ�Ӧ�ٶ�
{
	s16 offset=0;
//	int i=0;
//	
//	pitch_tarP=pitch_tarP>PITCH_OFFSET_VALUE[0][0]?PITCH_OFFSET_VALUE[0][0]:pitch_tarP;
//	pitch_tarP=pitch_tarP<PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT-1][0]?PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT-1][0]:pitch_tarP;
//	
//	for(i=0;i<PITCH_OFFSET_COUNT;i++)	//��������Ѱ��λ��
//	{
//		if(pitch_tarP>=PITCH_OFFSET_VALUE[i][0]) break;
//	}
//	
//	i=i>PITCH_OFFSET_COUNT-2?PITCH_OFFSET_COUNT-2:i;	//���Ƶ������ڶ���Ԫ�ص�λ�ã�������һ����������Խ��
//	
//	offset=PITCH_OFFSET_VALUE[i][1]+(PITCH_OFFSET_VALUE[i+1][1]-PITCH_OFFSET_VALUE[i][1])*(PITCH_OFFSET_VALUE[i][0]-pitch_tarP)/(PITCH_OFFSET_VALUE[i][0]-PITCH_OFFSET_VALUE[i+1][0]);
	return offset;
}




