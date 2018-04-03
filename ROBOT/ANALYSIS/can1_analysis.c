#include "can1_analysis.h"

extern LIFT_DATA lift_Data;
extern YUN_MOTOR_DATA	yunMotorData;	//��̨������CAN1�ϣ���ΪCAN2Ԥ����6pin�ӿڣ���̨����Ҫ�ýӿڣ�Ϊ���˷ѣ��ʽ�CAN1

LIFT_POSITION_ENCODER lift_position_encoder[4]={0};
/**************** **************************
��������CAN1_Feedback_Analysis
�������ܣ��Ե��̵�����ݽ��н���
          �õ���������
������������
��������ֵ����
������������
*******************************************/
void CAN1_Feedback_Analysis(CanRxMsg *rx_message)
{		
		CAN_Receive(CAN1, CAN_FIFO0, rx_message);//��ȡ����	
		switch(rx_message->StdId)
		{
			case 0x201:
			 {
				 Speed_Data_deal(&lift_Data.lf_lift_fdbV,rx_message);
				 Position_Data_deal(&lift_Data.lf_lift_fdbP,&lift_position_encoder[LF],rx_message);
				  break;
			 }
			 case 0x202:
			 {
				 Speed_Data_deal(&lift_Data.rf_lift_fdbV,rx_message);
				 Position_Data_deal(&lift_Data.rf_lift_fdbP,&lift_position_encoder[RF],rx_message);
				  break;
			 }
			 case 0x203:
			 {
				 Speed_Data_deal(&lift_Data.lb_lift_fdbV,rx_message);
				 Position_Data_deal(&lift_Data.lb_lift_fdbP,&lift_position_encoder[LB],rx_message);
				  break;
			 }
			 case 0x204:
			 {
				 Speed_Data_deal(&lift_Data.rb_lift_fdbV,rx_message);
				 Position_Data_deal(&lift_Data.rb_lift_fdbP,&lift_position_encoder[RB],rx_message);
				  break;
			 }
			 case 0x205:	//yaw
			 {
				 yunMotorData.yaw_fdbP=((rx_message->Data[0]<<8)|rx_message->Data[1])&0xffff;  //��е�Ƕ�
				  break;
			 }case 0x206:	//pitch
			 {
				 yunMotorData.pitch_fdbP=((rx_message->Data[0]<<8)|rx_message->Data[1])&0xffff;  //��е�Ƕ�
				  break;
			 }
			default:
			break;
		}
}


/****************************************************
�������ƣ�CAN_Lift_SendMsg
�������ܣ����������ݽ����󷢳�
����������motor_201*******������ǰ���ת��
          motor_202*******������ǰ���ת��
          motor_203*******���������ת��
          motor_204*******�����Һ���ת��
��������ֵ�� ��
�����������ݴ���TxMessage�ṹ������CAN_Transmit����
****************************************************/
void CAN1_Lift_SendMsg(int motor_201,int motor_202,int motor_203,int motor_204)
{	
		CanTxMsg TxMessage;
	  TxMessage.StdId = 0x200;      //֡IDΪ���������CAN_ID
    TxMessage.IDE = CAN_ID_STD;    //��׼֡
    TxMessage.RTR = CAN_RTR_DATA;  //����֡
    TxMessage.DLC = 0x08;          //֡����Ϊ8
    
    TxMessage.Data[0] =(unsigned char)((motor_201>>8)&0xff);
    TxMessage.Data[1] = (unsigned char)(motor_201&0xff);
    TxMessage.Data[2] =(unsigned char)((motor_202>>8)&0xff);
    TxMessage.Data[3] = (unsigned char)(motor_202&0xff);
    TxMessage.Data[4] =(unsigned char)((motor_203>>8)&0xff);
    TxMessage.Data[5] = (unsigned char)(motor_203&0xff);
    TxMessage.Data[6] =(unsigned char)((motor_204>>8)&0xff);
    TxMessage.Data[7] = (unsigned char)(motor_204&0xff);
		 
    CAN_Transmit(CAN1,&TxMessage);
}

/****************************************************
�������ƣ�CAN1_Yun_SendMsg
�������ܣ�����̨������ݽ����󷢳�
����������motor_205*******Yaw����ת��
          motor_206*******Pitch����ת��

��������ֵ�� ��
�����������ݴ���tx_message�ṹ������CAN_Transmit����
****************************************************/
void CAN1_Yun_SendMsg(int16_t motor_205,int16_t motor_206)	//yaw  pitch
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1ff;
    tx_message.IDE = CAN_Id_Standard;//��׼֡
    tx_message.RTR = CAN_RTR_Data;   //����֡
    tx_message.DLC = 0x08;           //֡����Ϊ8
    
    tx_message.Data[0] = (char)(motor_205 >> 8);
    tx_message.Data[1] = (char)motor_205;
    tx_message.Data[2] = (char)(motor_206 >> 8);
    tx_message.Data[3] = (char)motor_206;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}


void Speed_Data_deal(s32 * fdbV,CanRxMsg * msg)
{
	s16 v_tem=(msg->Data[2]<<8)|msg->Data[3];
	*fdbV=v_tem;//���յ�����ʵ����ֵ  ����Ƶ��1KHz
}



void Position_Data_deal(s32 * value,LIFT_POSITION_ENCODER *Receive,CanRxMsg * msg)
{
	Receive->calc=(msg->Data[0]<<8)|msg->Data[1];//���յ�����ʵ����ֵ  ����Ƶ��1KHz
	Position_To_Turns(Receive);
	*value=Receive->turns*8+(s32)(0.19f*Receive->turns+Receive->calc/1000.0);	//0.19Ϊ�������ۻ����
}




void Position_To_Turns(LIFT_POSITION_ENCODER *Receive)	//����6�����������㣬��е�Ƕȹ�8192����λ���������ֲ�ֵΪ6826
{																								//ע���˺���δ�Ե�һ������ʱ�Ŀ��ܵ�Ȧ��ֱ��Ϊ1��ƫ���������������ڳ�ʼ���б궨��ʼ�Ƕ�ֵ��
	Receive->calc_diff=Receive->calc_last-Receive->calc;
	if(Receive->calc_diff>5460)
	{
		Receive->turns=Receive->turns+1;
	}
	else if(Receive->calc_diff<-5460)
	{
		Receive->turns=Receive->turns-1;
	}
	else
	{
	}
	Receive->calc_last=Receive->calc;
}


