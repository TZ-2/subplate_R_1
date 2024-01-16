#include "can_feedback.h"
#include "can.h"

CHASSIS chassis_;

uint8_t             rx_data[8];
uint8_t             Data_can_tx[8];
uint32_t            box = 0;
static CanRxMsg     rx_message[2];
CAN_RxHeaderTypeDef rx_header;
union rc_data rc_ch_0;
union rc_data rc_ch_2;
union rc_data rc_ch_3;

void can_motor_send(uint16_t stid, int i1,int i2,int i3,int i4)
{
	
	CAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.DLC=0x08;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.StdId=stid;
	
	Data_can_tx[0]=i1 >> 8;
	Data_can_tx[1]=i1;
	Data_can_tx[2]=i2 >> 8;
	Data_can_tx[3]=i2;
	Data_can_tx[4]=i3 >> 8;
	Data_can_tx[5]=i3;
	Data_can_tx[6]=i4 >> 8;
	Data_can_tx[7]=i4; 
	
	HAL_CAN_AddTxMessage( &hcan ,&TxMessage,Data_can_tx,&box);
	
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		for(int i=0;i<8;i++)
			rx_message[0].Data[i] = rx_data[i];		
		
    switch (rx_header.StdId)
    {
			case ID_2006: 
				get_motor_measure(&chassis_.motor_2006.feedback,rx_message[0]);break;	
			case ID_3508: 
				get_motor_measure(&chassis_.motor_3508.feedback,rx_message[0]);break;    					
			case ID_hos1:
				get_host_measure_1(&chassis_.rc_contr,rx_message[0]);break;		
    }
}

void get_host_measure_1(RC_contr *ptr,CanRxMsg rx_message)                         
{         
	(ptr)->ch[0] = (rx_message).Data[0] << 8 | (rx_message).Data[1];
	(ptr)->ch[2] =(rx_message).Data[2] << 8 | (rx_message).Data[3];
	(ptr)->ch[3] = (rx_message).Data[4] << 8 | (rx_message).Data[5];
	(ptr)->s[0] = (rx_message).Data[6] >> 4;
	(ptr)->s[1] = (rx_message).Data[6] & 0x0F;
}

void get_host_measure_2(RC_contr *ptr,CanRxMsg rx_message)                         
{                
	(ptr)->ch[3] = ((rx_message).Data[3] << 8) | (rx_message).Data[1];   	
	(ptr)->s[0] =  (rx_message).Data[5];                              
	(ptr)->s[1] =  (rx_message).Data[7];                                  
}


//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//    switch (rx_header.StdId)
//    {
//        case 0x203:
//        {
//          
//				//			flag = 1;
//            break;
//        }

//        default:
//        {
//            break;
//        }
//    }
//}
