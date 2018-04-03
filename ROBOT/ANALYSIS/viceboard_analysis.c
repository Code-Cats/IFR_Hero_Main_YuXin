#include "viceboard_analysis.h"

SensorDataTypeDef SensorData={0};
ViceBoardSendTypeDef SendData=MAINBOARD_SENDDATA_DEFAULT;

//��������崮�ڷ��ͺ���	//2msִ��һ�Σ�10ms����һ�ν����14400�Ĳ����ʣ�һ���ֽ���ഫ��11λ��11/14400=0.76ms
void MainBoard_SendData(void)
{
	if(USART_GetFlagStatus(USART3,USART_FLAG_TC)== SET)	//�����һ֡�������
	{
		if(SendData.statu==1)
		{
			SendData.data[0]=0x5A;	//��ֹ֡ͷ֡β���ƻ�
			SendData.data[4]=0xA5;	//��ֹ֡ͷ֡β���ƻ�
			USART_SendData(USART3,SendData.data[SendData.count]);
			SendData.count++;
			if(SendData.count>4)
			{
				SendData.statu=0;
				SendData.count=0;
			}
		}
	}
	
}


ReceiveDataTypeDef ReceiveData={0};
u16 t_vice_count=0;
void Data_Receive(u8 data)	//�����崫���������ݽ�����������ͨ�ã�
{
	t_vice_count++;
	if(data==0x5A&&ReceiveData.headOK_state==0)
	{
		ReceiveData.valid_state=0;	//���ݽ����ڼ䲻�������ݽ���
		ReceiveData.headOK_state=1;	
		ReceiveData.count=0;	//����count
	}
	
	if(ReceiveData.headOK_state==1)	//֡ͷ���ҵ�
	{
		ReceiveData.databuffer[ReceiveData.count]=data;
		ReceiveData.count++;
		if((data==0xA5&&ReceiveData.count!=5)||(ReceiveData.count>5))	//ʧЧ
		{
			ReceiveData.valid_state=0;
			ReceiveData.headOK_state=0;
			ReceiveData.count=0;	//����count
		}
		else if(data==0xA5&&ReceiveData.count==5)
		{
			ReceiveData.valid_state=1;
			ReceiveData.headOK_state=0;
			ReceiveData.count=0;	//����count
		}
	}
	
	//////////////////////////////��������ݽ�������-->����Ϊ��ʵ����
	SensorData_Deal(ReceiveData.databuffer);
}

void SensorData_Deal(u8 *pData)
{
	for(int i=0;i<4;i++)
	{
		SensorData.Limit[i]=*(pData+1)>>(7-i)&0x01;
	}
	
	for(int i=0;i<4;i++)
	{
		SensorData.Infrare[i]=*(pData+1)>>(3-i)&0x01;
	}
}

