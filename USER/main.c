#include "main.h"

int main(void)
{
	 SetWorkState(CHECK_STATE);	//������Ĭ���Լ�״̬	//����״̬�л���������������
   delay_ms(500);
	 BSP_Init();	//���������ʼ��	//��ʱ��ʱ����������ʼ��ʱ
	 SetWorkState(CHECK_STATE);	//�����Լ�״̬
	 delay_ms(100);

	while(1)
	 {
//		 if(GetWorkState()==CALI_STATE)	//�궨���������
//		 {
//			 Lift_Calibration();
//		 }
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
