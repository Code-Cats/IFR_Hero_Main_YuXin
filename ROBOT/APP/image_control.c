#include "main.h"
#include "image_control.h"

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;

#define STEER_IMAGE_INIT	1750
#define STEER_IMAGE_REVERSAL	1000

#define IMAGE_START_DELAY	(1000*10)	//10s��ʼ
void Screen_Start(void)	//��Ļ�����л���AV�ŵ�
{
	if(time_1ms_count<IMAGE_START_DELAY)	//10s��ʼ
	{
		IMAGE_START=PWM_IO_ON;
	}
	else if(time_1ms_count>IMAGE_START_DELAY&&time_1ms_count<IMAGE_START_DELAY+1000)
	{
		IMAGE_START=PWM_IO_OFF;
	}
	else
	{
		IMAGE_START=PWM_IO_ON;
	}

}

u8 av_cut=0;
u8 Steer_Image_state=0;
u16 steer_image=STEER_IMAGE_REVERSAL;
void Image_Cut_Task(void)	//����ͷ�л������
{
	static u8 key_r_last=0;
	static WorkState_e State_Record=CHECK_STATE;
	t_AV_CUT=av_cut*20000;	//һֱ������
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		Steer_Image_state=1;
	}
	else if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)
	{
		Steer_Image_state=0;
	}
	else if(GetWorkState()==NORMAL_STATE)
	{
		Steer_Image_state=0;
	}
	
	if(key_r_last==0&&KeyBoardData[KEY_R].value==1)
	{
		Steer_Image_state=!Steer_Image_state;
	}
	STEER_IMAGE=STEER_IMAGE_INIT-Steer_Image_state*(STEER_IMAGE_INIT-STEER_IMAGE_REVERSAL);
	
	State_Record=GetWorkState();
	
	key_r_last=KeyBoardData[KEY_R].value;
}
