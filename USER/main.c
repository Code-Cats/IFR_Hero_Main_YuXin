#include "main.h"
#include "math.h"

extern int realBulletNum;	//发弹量
extern u8 Guiding_Lights_Data;

float Judge_Data_Send_A=1.0;
float Judge_Data_Send_B=0.0;
float Judge_Data_Send_C=0.0;

u8 Judge_Send_Statu=0;	//刷新标志

int main(void)
{
	delay_ms(1500);
	SetWorkState(CHECK_STATE);	//启动后默认自检状态	//后续状态切换不在主函数进行
	delay_ms(500+0);	//将这个延时移到了BSP
	BSP_Init();	//板载外设初始化	//此时定时器启动，开始计时
	Data_Init();
	SetWorkState(CHECK_STATE);	//进入自检状态
	delay_ms(100);
	while(1)
	 {
		 Screen_Start();	//屏幕启动切换到AV信道
//		 Image_Cut_Task();	//摄像头切换、舵机
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
