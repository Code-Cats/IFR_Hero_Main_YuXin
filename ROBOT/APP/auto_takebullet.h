#ifndef __AUTO_TAKEBULLET_H
#define __AUTO_TAKEBULLET_H
#include "stm32f4xx.h"


void TakeBullet_Control_Center(void);

typedef enum
{
    BULLET_ACQUIRE,  		//前伸、夹紧、抬起动作	称之为获得过程
    BULLET_POUROUT,			//车身倾斜、舵机旋转	称之为倒弹过程
		BULLET_THROWOUT,			//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
}TakeBulletState_e;


#endif
