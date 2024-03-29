#ifndef __PROTECT_H
#define __PROTECT_H
#include "stm32f4xx.h"

enum LOST_TYPES
{
	LOST_IMU,
	LOST_DBUS,
	LOST_VICEBOARD,
	LOST_REFEREE,
	LOST_CM1,
	LOST_CM2,
	LOST_CM3,
	LOST_CM4,
	LOST_YAW,
	LOST_PITCH,
	LOST_LIFT1,
	LOST_LIFT2,
	LOST_LIFT3,
	LOST_LIFT4,
	LOST_SM_DOWN,
	LOST_SM_UP,
	LOST_TYPE_NUM,
};


typedef struct
{
	const u16 cycle[LOST_TYPE_NUM];
	u16 count[LOST_TYPE_NUM];
	u8 statu[LOST_TYPE_NUM];
}Error_check_t;

extern Error_check_t Error_Check;

#define LOST_CYCLE \
{\
	17,\
	17,\
	4,\
	10,\
	2,\
	2,\
	2,\
	2,\
	2,\
	2,\
	2,\
	2,\
	2,\
	2,\
	2,\
	3,\
}

void LostCountAdd(u16* lostcount);
void LostCountFeed(u16* lostcoun);
	u8 LostCountCheck(u16 lostcoun,u8* statu,const u16 cycle);

void Check_Task(void);	//control.c
void IMU_Check_Useless(void);	//�����Ǽ��ʧЧ

extern Error_check_t Error_Check;
extern u8 IMU_Check_Useless_State;

#endif
