#include "yun.h"
#include "remote_analysis.h"
s32 YAW_INIT=YAW_INIT_DEFINE;
/*
整体结构：yaw轴暂定单独速度环//后期计划增加外接陀螺仪位置环进行选择
pitch轴位置环

经过控制信息处理，传入参数为
yaw速度？或者位置
pitch位置


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
extern IslandAttitudeCorrectState_e IslandAttitude_Correct_State;	//登岛姿态自校正
extern u8 Chassis_Follow_Statu;	//底盘跟随标志位
extern volatile float yaw_follow_real_error;	//扭腰时的底盘跟随偏差
extern float yaw_follow_error;	//普通时的底盘跟随误差

extern u32 time_1ms_count;
s32 t_pitch____=0;
s32 t_yaw___=0;
void Yun_Task(void)	//云台控制任务 
{
	Yun_Control_External_Solution();
}


u8 Yun_Control_RCorPC=RC_CONTROL;
void Yun_Control_External_Solution(void)	//外置反馈方案
{
	if(GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE)	//模式切换
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
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==WAIST_STATE)	//仅在正常模式下受控	//取弹受控为暂时加入，之后以传感器自动进行	//取弹受控已取消，云台跟随底盘
	{
		if(Yun_Control_RCorPC==PC_CONTROL)
		{	//PC控制数据
			PC_Control_Yun(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
		}
		else if(Yun_Control_RCorPC==RC_CONTROL)
		{	//RC控制数据
			RC_Control_Yun(&yunMotorData.yaw_tarP,&yunMotorData.pitch_tarP);
		}
	}
	
	
	if(GetWorkState()==ASCEND_STATE||GetWorkState()==DESCEND_STATE)	//登岛姿态调整 当自校正状态时利用底盘跟随，其他主动矫正状态进行云台归位
	{
		switch(IslandAttitude_Correct_State)
		{
			case CALI_SELF_STATE:
			{
				yunMotorData.yaw_tarP=(s32)(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//反馈放大10倍并将目标位置置为中点
				break;
			}
			case CORRECT_CHASSIS_STATE:
			{
				
				break;
			}
		}
	}
	else if(GetWorkState()==TAKEBULLET_STATE)	//取弹模式一直校准
	{
		yunMotorData.yaw_tarP=(s32)(Gyro_Data.angle[2]*10+(YAW_INIT-yunMotorData.yaw_fdbP)*3600/8192);	//反馈放大10倍并将目标位置置为中点
	}

	
	yunMotorData.pitch_tarV=-PID_General(yunMotorData.pitch_tarP,(Gyro_Data.angle[0]*8192/360.0f+PITCH_INIT),&PID_PITCH_POSITION);
		
	if(yunMotorData.yaw_tarP-Gyro_Data.angle[2]*10>1800)	//过零点
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
	yunMotorData.yaw_output=PID_General(yunMotorData.yaw_tarV,(-Gyro_Data.angvel[2]/10.0),&PID_YAW_SPEED);	//采用外界陀螺仪做反馈
}


void Yun_Control_Inscribe_Solution(void)	//内接反馈方案
{
	//		yunMotorData.yaw_tarV=(int32_t)((RC_Ctl.rc.ch2-1024)*300.0/660.0);
//	yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,Gyro_Data.angle[2]*10,&PID_YAW_POSITION);
//	yunMotorData.yaw_tarV=-PID_General(yunMotorData.yaw_tarP,yunMotorData.yaw_fdbP,&PID_YAW_POSITION);
	{	//采用板载陀螺仪的处理-速度
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
		yunMotorData.yaw_tarP=yunMotorData.yaw_tarP>1800?yunMotorData.yaw_tarP-3600:yunMotorData.yaw_tarP;	//过零点
		yunMotorData.yaw_tarP=yunMotorData.yaw_tarP<-1800?yunMotorData.yaw_tarP+3600:yunMotorData.yaw_tarP;	//过零点
	}
	
	yunMotorData.pitch_tarP=(int32_t)(-(RC_Ctl.rc.ch3-1024)*460.0/660.0)+PITCH_INIT;	//-50是因为陀螺仪水平时云台上扬
}


#define YUN_UPMAX 430
#define YUN_DOWNMAX 430	//偏差量
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Yun(s32 * yaw_tarp,s32 * pitch_tarp)	//1000Hz	
{
	static float yaw_tarp_float=0;
	static float pitch_tarp_float=PITCH_INIT;
	static u8 start_state=0;	//初始位置

	if(start_state==0)
	{
		yaw_tarp_float=(float)*yaw_tarp;
		start_state=1;
	}
	static u8 keyQ_last,keyE_last=0;	//暂时屏蔽
	if(keyQ_last==0&&KeyBoardData[KEY_Q].value==1&&abs(yaw_follow_error)<PI/10)
	{
		yaw_tarp_float+=900;
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//过零点
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//过零点
	}
	keyQ_last=KeyBoardData[KEY_Q].value;
	
	if(keyE_last==0&&KeyBoardData[KEY_E].value==1&&abs(yaw_follow_error)<PI/10)
	{
		yaw_tarp_float-=900;
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//过零点
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//过零点
	}
	keyE_last=KeyBoardData[KEY_E].value;
	
	if(time_1ms_count%10==0)
	{
		yaw_tarp_float-=RC_Ctl.mouse.x*15.0f/40.0f;
		pitch_tarp_float+=RC_Ctl.mouse.y*2.0f/3.0f;	//2/4
		
		yaw_tarp_float=yaw_tarp_float>1800?yaw_tarp_float-3600:yaw_tarp_float;	//过零点
		yaw_tarp_float=yaw_tarp_float<-1800?yaw_tarp_float+3600:yaw_tarp_float;	//过零点
		
		pitch_tarp_float=pitch_tarp_float>(PITCH_INIT+500)?(PITCH_INIT+500):pitch_tarp_float;	//限制行程
		pitch_tarp_float=pitch_tarp_float<(PITCH_INIT-650)?(PITCH_INIT-650):pitch_tarp_float;	//限制行程
		
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
函数名称：Yaw_Angle_Calculate
函数功能：通过当前机械角度与中值机械角度比较得到实际差角
函数参数：当前机械角度：src_angle
          中值机械角度：Init
函数返回值：实际差角：output
描述：无
****************************************/
float Yaw_Angle_Calculate(int16_t src_angle,int16_t Init)
{
    float output=-(float)(src_angle-Init)/8192*2*PI;	
	  return output;
}
//记录补偿值曲线
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
};	//3.6测得

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
};	//旧

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
};	//原版，过大
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

s32 Yaw_output_offset(s32 yaw_fbdP)	//克服云台yaw轴非线性力及非对称性的补偿 //虽然yaw云台阻尼曲线满足收敛，但因为参照物并非为云台电机，故应采用当前反馈位置做参照
{
	s32 offset=0;
	int i=0;
	
	yaw_fbdP=yaw_fbdP>YAW_OFFSET_VALUE[0][0]?YAW_OFFSET_VALUE[0][0]:yaw_fbdP;
	yaw_fbdP=yaw_fbdP<YAW_OFFSET_VALUE[YAW_OFFSET_COUNT-1][0]?YAW_OFFSET_VALUE[YAW_OFFSET_COUNT-1][0]:yaw_fbdP;
	
	for(i=0;i<YAW_OFFSET_COUNT;i++)	//遍历数组寻找位置
	{
		if(yaw_fbdP>=YAW_OFFSET_VALUE[i][0]) break;
	}
	
	i=i>YAW_OFFSET_COUNT-2?YAW_OFFSET_COUNT-2:i;	//限制到倒数第二个元素的位置，以免下一步运算数组越界
	
	offset=YAW_OFFSET_VALUE[i][1]+(YAW_OFFSET_VALUE[i+1][1]-YAW_OFFSET_VALUE[i][1])*(YAW_OFFSET_VALUE[i][0]-yaw_fbdP)/(YAW_OFFSET_VALUE[i][0]-YAW_OFFSET_VALUE[i+1][0]);
	return offset;
}

s16 Pitch_output_offset(s32 pitch_tarP)	//克服云台pitch轴非线性力及非对称性的补偿	//因为云台pitch阻尼曲线满足收敛（在外部激励情况下只存在一个最小值），故采用tarP作为补偿参照可以提高间接反应速度
{
	s16 offset=0;
//	int i=0;
//	
//	pitch_tarP=pitch_tarP>PITCH_OFFSET_VALUE[0][0]?PITCH_OFFSET_VALUE[0][0]:pitch_tarP;
//	pitch_tarP=pitch_tarP<PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT-1][0]?PITCH_OFFSET_VALUE[PITCH_OFFSET_COUNT-1][0]:pitch_tarP;
//	
//	for(i=0;i<PITCH_OFFSET_COUNT;i++)	//遍历数组寻找位置
//	{
//		if(pitch_tarP>=PITCH_OFFSET_VALUE[i][0]) break;
//	}
//	
//	i=i>PITCH_OFFSET_COUNT-2?PITCH_OFFSET_COUNT-2:i;	//限制到倒数第二个元素的位置，以免下一步运算数组越界
//	
//	offset=PITCH_OFFSET_VALUE[i][1]+(PITCH_OFFSET_VALUE[i+1][1]-PITCH_OFFSET_VALUE[i][1])*(PITCH_OFFSET_VALUE[i][0]-pitch_tarP)/(PITCH_OFFSET_VALUE[i][0]-PITCH_OFFSET_VALUE[i+1][0]);
	return offset;
}




