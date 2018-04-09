#include "lift.h"
#include "control.h"

extern LIFT_DATA lift_Data;
extern u32 time_1ms_count;

#define AUTOCHASSIS_LIFT 15
void AutoChassisAttitude_Lift(float chassis_pitch_raw)	//自动调整姿态	//pitch正方向为前上	//注意放在lift_task前面
{
	static float chassis_pitch=0;
	static float ka=0.1f;
	static u16 steady_flat_count=0;
	
	chassis_pitch=chassis_pitch*(1-ka)+chassis_pitch_raw*ka;
	
	if(GetWorkState()==NORMAL_STATE)	//正常状态
	{
		
		if(abs(chassis_pitch)>8)	//8度阈值
		{
			steady_flat_count=0;
			
			lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
			lift_Data.rf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.rf_lift_fdbP;
			lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
			lift_Data.rb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.rb_lift_fdbP;
			
			if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
			{
				s32 l_r_error=(lift_Data.lf_lift_fdbP-lift_Data.rf_lift_fdbP)/2;
				lift_Data.lf_lift_tarP-=l_r_error;
				lift_Data.rf_lift_tarP+=l_r_error;
			}
			
			if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
			{
				s32 l_r_error=(lift_Data.lb_lift_fdbP-lift_Data.rb_lift_fdbP)/2;
				lift_Data.lb_lift_tarP-=l_r_error;
				lift_Data.rb_lift_tarP+=l_r_error;
			}
		}
		else
		{
			if(abs(lift_Data.lf_lift_tarP-lift_Data.lb_lift_tarP)<300)
			{
				if(steady_flat_count<0xFFFE)
				{
					steady_flat_count++;
				}
			}
				
			if(abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*FALL)>18&&abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*FALL)>18)
			{
				if(time_1ms_count%40==0)	//1ms加的太快10MS
				{
					if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
					{
						lift_Data.lf_lift_tarP=lift_Data.lf_lift_fdbP<lift_Data.rf_lift_fdbP?lift_Data.rf_lift_fdbP:lift_Data.rf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;
					}
					
					if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
					{
						lift_Data.lb_lift_tarP=lift_Data.lb_lift_fdbP<lift_Data.rb_lift_fdbP?lift_Data.lb_lift_fdbP:lift_Data.rb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					lift_Data.lf_lift_tarP-=5;
					lift_Data.rf_lift_tarP-=5;
					lift_Data.lb_lift_tarP-=5;
					lift_Data.rb_lift_tarP-=5;
				}
			}
			
		}
		
		if(steady_flat_count>1000)
		{
			lift_Data.lf_lift_tarP=FALL;
			lift_Data.rf_lift_tarP=FALL;
			lift_Data.lb_lift_tarP=FALL;
			lift_Data.rb_lift_tarP=FALL;
		}
//		if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
//		{
//			lift_Data.lf_lift_tarP=lift_Data.lf_lift_fdbP<lift_Data.rf_lift_fdbP?lift_Data.rf_lift_fdbP:lift_Data.rf_lift_fdbP;
//			lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;
//		}
//		
//		if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
//		{
//			lift_Data.lb_lift_tarP=lift_Data.lb_lift_fdbP<lift_Data.rb_lift_fdbP?lift_Data.lb_lift_fdbP:lift_Data.rb_lift_fdbP;
//			lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
//		}
		
		
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<FALL?FALL:lift_Data.lf_lift_tarP;	//限制行程
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>ISLAND?ISLAND:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<FALL?FALL:lift_Data.rf_lift_tarP;	//限制行程
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>ISLAND?ISLAND:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<FALL?FALL:lift_Data.lb_lift_tarP;	//限制行程
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>ISLAND?ISLAND:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<FALL?FALL:lift_Data.rb_lift_tarP;	//限制行程
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>ISLAND?ISLAND:lift_Data.rb_lift_tarP;
	}
}

