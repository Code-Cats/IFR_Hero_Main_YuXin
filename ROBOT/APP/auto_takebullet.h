#ifndef __AUTO_TAKEBULLET_H
#define __AUTO_TAKEBULLET_H
#include "stm32f4xx.h"


void TakeBullet_Control_Center(void);

typedef enum
{
    BULLET_ACQUIRE,  		//ǰ�졢�н���̧����	��֮Ϊ��ù���
    BULLET_POUROUT,			//������б�������ת	��֮Ϊ��������
		BULLET_THROWOUT,			//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
}TakeBulletState_e;


#endif
