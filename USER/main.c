#include "main.h"
#include "math.h"

extern int realBulletNum;	//������
extern u8 Guiding_Lights_Data;

float Judge_Data_Send_A=1.0;
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
			 Judge_Data_Send_B=realBulletNum;
			 Guiding_Lights_Data=Judagement_Send_Guiding_lights(1,0,1,0,1,0);
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
