#include "auto_takebullet.h"
#include "main.h"

#define LIFT_DISTANCE_BULLET 700

TakeBulletState_e TakeBulletState=BULLET_ACQUIRE;	//（自动）取弹状态位


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


u8 valve_fdbstate[6]={0};	//记录是否伸出的反馈标志
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={300,1200,300,1000,1000,1000};	//待加入，延时参数
const u32 valve_POORdelay[6]={300,1200,300,1000,1000,1000};	//待加入，延时参数
const u32 servo_GOODdelay[2]={3000,1000};	//待加入，延时参数	//第一段为2000是将子弹落下的延时也加进去了，因为舵机翻转和子弹下落必须是连在一体的
const u32 servo_POORdelay[2]={1500,1000};	//待加入，延时参数


//#define VALVE_ISLAND 0		//电磁阀控制位定义
//#define VALVE_BULLET_PROTRACT 1	//前伸
//#define VALVE_BULLET_CLAMP 2	//夹紧

u8 auto_takebullet_statu=0;
void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//右拨杆
//	static u8 pwm_state=0;
//	static u8 valve_state=0;
//	static u8 valve_delay_statu=0;
//	static u32 valve_time_record=0;
	
	static u8 valve_last[6]={0};	//记录上一次数值	//保持与工程车兼容性
	static u8 servo_last[2]={0};	//记录上一次数值	//保持与工程车兼容性
	
	static u32 valve_startGOOD_time[6]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 servo_startGOOD_time[2]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 valve_startPOOR_time[6]={0};	//记录逆向触发时间	//保持与工程车兼容性
	static u32 servo_startPOOR_time[2]={0};	//记录逆向触发时间	//保持与工程车兼容性
	

	if(GetWorkState()==TAKEBULLET_STATE&&RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
	{
		if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//待加入
		{
			auto_takebullet_statu=!auto_takebullet_statu;
			TakeBulletState=BULLET_ACQUIRE;
		}	
		else if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//暂时无用
		{
		}
		
		
		{	//自动取弹块
			if(auto_takebullet_statu==1)	//自动取弹
			{
				switch(TakeBulletState)
				{
					case BULLET_ACQUIRE:	//前伸、夹紧、抬起动作	称之为获得过程
					{
						ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;	//前伸函数
						if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)//如果前伸没到位 ，腿执行到取弹合适位置函数	//这条语句可以让多次取弹与第一次取弹兼容
						{
							SetCheck_GripLift(1);	//下降到抓取高度
						}
						
						if(valve_fdbstate[VALVE_BULLET_PROTRACT]==1&&SetCheck_GripLift(1)==1)	//如果前伸到位且升降到位
						{
							ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
						}
						if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)//如果前伸到位腿升起函数
						{
							if(SetCheck_GripLift(0)==1)	//上升至可旋转高度
							TakeBulletState=BULLET_POUROUT;//直接切换到下一状态
						}
						break;
					}
					case BULLET_POUROUT:	//车身倾斜、舵机旋转	称之为倒弹过程
					{
						ViceControlData.servo[0]=1;
						if(SetCheck_SlopeLift(1)==1)//腿倾斜函数
						{//略作延时，切换到下一状态	//此处缺少延时
							if(servo_fdbstate[0]==1)
							TakeBulletState=BULLET_THROWOUT;
						}
						break;
					}
					case BULLET_THROWOUT:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
					{
						ViceControlData.servo[0]=0;
						if(servo_fdbstate[0]==0)	//先让舵机归位的原因是以便让弹药箱能够顺利回位
						{
							if(SetCheck_GripLift(0)==1)//车身抬起函数	/
							{
								ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
							//	ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;	//注释以便平移取下一颗弹
							}//如果车身抬起且舵机到位，则松开夹紧，至此一个完整取弹结束
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
				//	ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;	//注释以便平移取下一颗弹
				}
			}
		}
		
	
//		if(valve_state==0)	//测试分立操作
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
以下三个for为反馈假设检测方案
分别为：
1.上升，下降沿的触发时间记录
2.根据触发时间的推演反馈值计算
3.数据迭代
******************************************************************/

		for(int i=0;i<6;i++)	//触发时间块
		{
			if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//伸出触发
			{
				valve_startGOOD_time[i]=time_1ms_count;
			}
			else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//收回触发
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
		
		for(int i=0;i<6;i++)	//反馈计算位
		{
			if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
			{
				valve_fdbstate[i]=1;
			}
			else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
			{
				valve_fdbstate[i]=0;
			}
			
			if(i<2)
			{
				if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
				{
					servo_fdbstate[i]=1;
				}
				else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
				{
					servo_fdbstate[i]=0;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//迭代块
		{
			valve_last[i]=ViceControlData.valve[i];
			if(i<2)	servo_last[i]=ViceControlData.servo[i];
		}
////////////////////////////////////////////////////////////////////////////////////////////块结束标志
		
		
		//舵机执行块	//电磁阀在副板执行
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


#define LIFT_DISTANCE_GRIPBULLET	630	//夹弹药箱时高度
#define LIFT_DISTANCE_DISGRIPBULLET	1060	//拔起来后弹药箱高度
#define LIFT_DISTANCE_SLOPEBACKBULLET	1170	//倾斜时后腿高度
#define LIFT_DISTANCE_SLOPEFRONTBULLET	900	//倾斜时前腿高度
extern LIFT_DATA lift_Data;

u8 SetCheck_GripLift(u8 grip_state)	//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_GRIPBULLET)))<30);	//这里是仅以前两腿为反馈传回的
}

u8 SetCheck_SlopeLift(u8 slope_state)	//暂时只升后腿	slope倾斜的意思	//0表示不倾斜，即恢复到拔起弹药箱高度，1表示倾斜，即倾斜倒子弹状态
{
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET);
	
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEFRONTBULLET);
	
	return (abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*(LIFT_DISTANCE_DISGRIPBULLET-(slope_state!=0)*(LIFT_DISTANCE_DISGRIPBULLET-LIFT_DISTANCE_SLOPEBACKBULLET)))<30);	//这里是仅以前两腿为反馈传回的,因为前两腿为升高且行程更大时间更长，若前腿已到位则后腿一定到位
}






