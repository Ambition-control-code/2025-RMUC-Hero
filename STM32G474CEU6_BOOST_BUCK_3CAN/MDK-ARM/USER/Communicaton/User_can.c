#include "User_com_all.h"

uint16_t CHASSIS_LIMIT_POWER=0;
uint16_t CHASSIS_ENG=0;
extern  Robot_Para robot; 

void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;                   //允许接收两个ID
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x0000;                               //32位ID
	fdcan_filter.FilterID2 = 0x0000;                               //接收ID1
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
}

void CAN_BSP_INIT(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
uint8_t FDCAN1_RX_ANALYSE(FDCAN_HandleTypeDef *hfdcan,uint8_t *buf);
void GET_UM_system_data(uint8_t *buf);

uint32_t RE_ID;
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
	{
	  return 0;//接收数据
	}
	RE_ID=fdcan_RxHeader.Identifier; 
	Solution_Message_IN_CAN3(buf,fdcan_RxHeader.Identifier);
  return fdcan_RxHeader.DataLength>>16;	
}

//uint8_t FDCAN3_RX_ANALYSE(FDCAN_HandleTypeDef *hfdcan,uint8_t *buf)
//{
//  FDCAN_RxHeaderTypeDef fdcan_RxHeader;
//  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
//	{
//	  return 0;//接收数据
//	}
//	
//	return 1;
//}

uint8_t rx_data1[8] = {0};
void fdcan1_rx_callback(void)
{
	FDCAN1_RX_ANALYSE(&hfdcan1, rx_data1);
}
uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{
//	fdcanx_receive(&hfdcan2, rx_data2);
}
uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
	fdcanx_receive(&hfdcan3, rx_data3);
}


//super_cap_V=(a[0]<<8)|a[1];
void CAP_DATA_TX_IN_CAN(void)
{  
   uint8_t CAN_TX_DATA[8];
	 uint16_t CAP_U_TX_DATA=robot.shooter_42mm_barrel_heat;
	 CAN_TX_DATA[0]=CAP_U_TX_DATA>>8;
	 CAN_TX_DATA[1]=CAP_U_TX_DATA;
	 CAP_U_TX_DATA=robot.shooter_barrel_heat_limit;
	 CAN_TX_DATA[2]=CAP_U_TX_DATA>>8;
	 CAN_TX_DATA[3]=CAP_U_TX_DATA;
	 CAN_TX_DATA[4]=robot.chassis_power_limit;
	 CAN_TX_DATA[5]=robot.power_management_shooter_output;
//	 CAN_TX_DATA[6]=;
//	 CAN_TX_DATA[7]=CAP_U_TX_DATA>>8;
	
	fdcan_transmit(&hfdcan3, &TxHeader3,0x211, FDCAN_DLC_BYTES_8, CAN_TX_DATA);	
}


uint8_t FDCAN1_RX_ANALYSE(FDCAN_HandleTypeDef *hfdcan,uint8_t *buf)
{
  FDCAN_RxHeaderTypeDef fdcan_RxHeader;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
	{
	  return 0;//接收数据
	}
	//RE_ID=fdcan_RxHeader.Identifier; 
	//解算3508 6020数据
	Solution_Message_IN_CAN(buf,fdcan_RxHeader.Identifier);
	
	return 1;
}


void GET_UM_system_data(uint8_t *buf)
{
	
	CHASSIS_LIMIT_POWER=buf[0];
	CHASSIS_ENG=(((uint16_t)buf[1])<<8)+(uint16_t)buf[2];
	
//	tx_message.Data[0] = 100;
//	tx_message.Data[1] = real_send>>8;
//	tx_message.Data[2] = real_send;
//	tx_message.Data[3] = 1;  
//	tx_message.Data[4] = robot.mains_power_chassis_output;

	
}





