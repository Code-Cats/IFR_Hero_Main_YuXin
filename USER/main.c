#include "main.h"
#include "math.h"
//���з�����¼��
//	��һ������������
//	�ڶ�������ǰ��������λ��
//	����������������λ��
//	״̬�ƣ��Զ�ȡ��ģʽ	����LOST	
extern int realBulletNum;	//������
extern u8 Guiding_Lights_Data;

extern LIFT_DATA lift_Data;

extern u8 auto_takebullet_statu;

float Judge_Data_Send_A=0.0;
float Judge_Data_Send_B=0.0;
float Judge_Data_Send_C=0.0;

u8 Judge_Send_Statu=0;	//ˢ�±�־

int main(void)
{
	delay_ms(1500);
	SetWorkState(CHECK_STATE);	//������Ĭ���Լ�״̬	//����״̬�л���������������
	delay_ms(500+0);	//�������ʱ�Ƶ���BSP
	BSP_Init();	//���������ʼ��	//��ʱ��ʱ����������ʼ��ʱ
	Data_Init();
	SetWorkState(CHECK_STATE);	//�����Լ�״̬
	delay_ms(100);
	while(1)
	 {
		 Screen_Start();	//��Ļ�����л���AV�ŵ�
//		 Image_Cut_Task();	//����ͷ�л������
		 if(Judge_Send_Statu==1)
		 {
			 Judge_Data_Send_A=realBulletNum;	//������
			 Judge_Data_Send_B=(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP)/2.0;	//ǰ����
			 Judge_Data_Send_C=(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP)/2.0;	//������
			 Guiding_Lights_Data=Judagement_Send_Guiding_lights(auto_takebullet_statu,0,!Error_Check.statu[LOST_REFEREE],0,0,1);
			 Judgement_DataSend(Judge_Data_Send_A,Judge_Data_Send_B,Judge_Data_Send_C,Guiding_Lights_Data);
		 }
		 
	 }
}





//////////////////////////////////////////////////////////////////
//??????,??printf??,??????use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//??????????                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//??_sys_exit()??????????    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//???fputc?? 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//????,??????   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
