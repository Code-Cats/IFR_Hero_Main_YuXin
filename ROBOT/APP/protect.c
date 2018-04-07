#include "protect.h"
#include "control.h"
/*
���ļ���;���ṩ�����������㷨���������л����Լ�����ʵʱ״̬��⣬����״̬�л�
Ԥ���幦�ܣ�
1.��̨��̬��������̬����Դ�����㷨
2.�����������⼰��Ч��ֹ
3.�����ģ�����߼����
4.�����������
5.����...
*/
#define LOST_THRESHOLD 5

Error_check_t Error_Check={LOST_CYCLE,{0},{0}};


void LostCountAdd(u16* lostcount)
{
	if(*lostcount<0xFFFE)
	(*lostcount)++;
}

void LostCountFeed(u16* lostcount)
{
	*lostcount=0;
}

u8 LostCountCheck(u16 lostcount,u8* statu,const u16 cycle)
{
	if(lostcount>LOST_THRESHOLD*cycle)
		*statu=1;
	else
		*statu=0;
	return *statu;
}


void Check_Task(void)
{
	for(int i=0;i<LOST_TYPE_NUM;i++)
	{
		LostCountAdd(&Error_Check.count[i]);
		LostCountCheck(Error_Check.count[i],&Error_Check.statu[i],Error_Check.cycle[i]);
	}
	
	if(Error_Check.statu[LOST_IMU]==1)
	{
		SetWorkState(ERROR_STATE);
	}
	
	if(Error_Check.statu[LOST_DBUS]==1)
	{
		SetWorkState(PROTECT_STATE);
	}
	
	for(int i=4;i<LOST_TYPE_NUM-1;i++)
	{
		if(Error_Check.statu[i]==1)
			SetWorkState(ERROR_STATE);
	}
}



