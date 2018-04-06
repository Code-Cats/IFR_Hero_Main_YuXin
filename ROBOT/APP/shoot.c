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
void Shoot_Task(void)	//定时频率：1ms
{ 
	
	Shoot_Instruction();
	shoot_Motor_Data_Down.tarP=(s32)shoot_Data_Down.motor_tarP;
	shoot_Motor_Data_Down.tarV=PID_General(shoot_Motor_Data_Down.tarP,shoot_Motor_Data_Down.fdbP,&PID_Shoot_Down_Position);
	
	static u8 swicth_Last_state=0;	//右拨杆

////////	if(RC_Ctl.rc.switch_right==RC_SWITCH_UP)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//待加入
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
/////临时///////	{
/////测试///////		static u8 frc_state=0;
/////完整///////		frc_state=!frc_state;
/////形态///////		fri_t=800-(800-2000)*frc_state;
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




#define SINGLE_INCREMENT_OLD_2006 196.608f	//8192*96/4/1000	一圈的累加值8192*96除上一圈7个子弹除以编码器转换倍数=发射一颗子弹的位置增量
#define SINGLE_INCREMENT_NEW_2006 37.45f		//8192*32/7/1000
#define SINGLE_INCREMENT SINGLE_INCREMENT_OLD_2006
//输出为发弹量，单位颗
//注：应当在本函数或者另一指令解析函数中设置逻辑：切换状态就重置发弹指令（以免突发情况使程序具有滞后性）
//或者将发弹逻辑改为基于增量式方法的频率控制
void Shoot_Instruction(void)	//发弹指令模块
{
	static u8 swicth_Last_state=0;	//右拨杆
	static u8 last_mouse_press_l=0;
	if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//待加入
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
用于自动射击
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
用于自动打击
*********************************/
#define UPPER_LIMIT_OF_HEAT 4500	//热量上限
#define COOLING_PER_SECOND 1500	//每秒冷却
void Shoot_Frequency_Limit(int* ferquency,u16 rate,u16 heat)	//m/s为单位
{
	u16 heating=rate*rate;
	s16 ferquency_safe=(s16)(COOLING_PER_SECOND/heating);
	if(*ferquency!=0)
	{
		if(heat<5*heating&&heat>=2*heating)	//4倍余量时开始缓冲，以防超出
		{
			*ferquency=(u16)ferquency_safe+1;
		}
		else if(heat<=heating)	//单发余量触发保护
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
//对tarP的操作
void Prevent_Jam(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//防卡弹程序	//同时包含防鸡蛋的功能	//放在tarP计算出之后
{
	static s32 deviation=0;	//偏差
	static u8 jam_deal_state=0;
//	static u16 ferquency_last=0;
	
	deviation=shoot_motor_Data->tarP-shoot_motor_Data->fdbP;
	
//	if(shoot_data->frequency!=ferquency_last)
//	{
//		shoot_data->Jam.count=0;	//重置count
//	}
//	ferquency_last=shoot_data->frequency;	//迭代
	
	
	if(abs(deviation)>6&&abs(shoot_motor_Data->fdbV)<10)	//期望速度不为0时位置未发生变化	//bug:频率刷新时需要刷新count	//手动射击将频率检测删除
	{
		shoot_data->Jam.count++;
	}
	else
	{
		shoot_data->Jam.count=0;
	}
	
//	if(shoot_data->cycle!=0)
//	{
		if(shoot_data->Jam.count>100&&shoot_data->Jam.sign==0)	//超出非正常时间	//且仅执行一次
		{
			 shoot_data->Jam.sign=1;	//标记卡弹
			 jam_deal_state=1;	//标记卡弹处理进程状态
		}
//	}
	
	if(shoot_data->Jam.sign==1)	//处理卡弹模块
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
					if(abs(shoot_motor_Data->fdbP-jam_fdbP_record)<40)	//认为已经执行了动作	//50
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_data->Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_data->count=shoot_data->count_fdb;	//重置子弹数据，防止鸡蛋	//？是否需要+-1？
					shoot_data->Jam.count=0;	//重置卡弹检测数据，防止误检测
					break;
				}
		}
	}
	
}   




u32 shoot_fdb_count=0;
/*****************************************
函数名称：Shoot_Feedback_Deal
函数功能：拨弹电机反馈数据解析+处理

*****************************************/
void Shoot_Feedback_Deal(SHOOT_DATA *shoot_data,SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg)
{
	shoot_fdb_count++;
	shoot_motor_data->fdbP_raw=(msg->Data[0]<<8)|msg->Data[1];//接收到的真实数据值  处理频率1KHz
	shoot_motor_data->fdbV=(msg->Data[2]<<8)|msg->Data[3];
	
	shoot_motor_data->fdbP_diff=shoot_motor_data->fdbP_raw_last-shoot_motor_data->fdbP_raw;
	if(shoot_motor_data->fdbP_diff>5460)	//按照6倍采样来计算，机械角度共8192个挡位，则过界表现差值为6826
	{																			//注：此函数未对第一次运行时的可能的圈数直接为1的偏差做处理（处理方法在初始化中标定初始角度值）
		shoot_motor_data->fdbP_raw_sum+=8192;
	}
	else if(shoot_motor_data->fdbP_diff<-5460)
	{
		shoot_motor_data->fdbP_raw_sum-=8192;
	}
	
	shoot_motor_data->fdbP=(s32)((shoot_motor_data->fdbP_raw_sum+shoot_motor_data->fdbP_raw)/1000);	//因为2006减速比过大 不便精确
	
	shoot_motor_data->fdbP_raw_last=shoot_motor_data->fdbP_raw;	//数据迭代
	
	shoot_data->count_fdb=(u16)(shoot_motor_data->fdbP/SINGLE_INCREMENT);	///////防止卡弹，刷新反馈的
}




//一圈7个
//极限15小

