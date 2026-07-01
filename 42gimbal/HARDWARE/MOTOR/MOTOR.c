#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "motor.h"
#include "MI_Cyber_Gear.h"
#include "rc.h"
#include "stdlib.h"   
#include "string.h"   
#include "gimbal_task.h"

extern CAN_TxHeaderTypeDef	TxHeader;      //发送
extern CAN_RxHeaderTypeDef	RxHeader;      //接收
extern uint32_t Motor_Can_ID;    //接收数据电机ID
extern uint8_t rx_data[8];       //接收数据
extern CAN_RxHeaderTypeDef rxMsg;//发送接收结构体    
extern  RC rc;
extern Gimbal_all guard_gimbal;

pack_recieve_gimbal abc;
mi_leg_motor MI_LEFT,MI_RIGHT;
old_mi OMI_1_4,OMI_1_1,OMI_2_4,OMI_2_1;
float MI_T,MI_V,MI_A,MI_T2,MI_V2,MI_A2;
 

dji_motor HERO_CLIP,SHOOT_L,SHOOT_R,hero_pitch,SHOOT_H,balence_pitch,balence_yaw,HERO_WATCH;
uint8_t usedata[8];
xiaomi_motor Pitch_mi;

uint16  crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t *buffer, size_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}

uint16_t CRC_INIT = 0XFFFF;
const uint16_t wCRC_Table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &0x00ff];
    }
    return wCRC;
}
/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
   uint16_t wExpected = 0;
   if ((pchMessage == NULL) || (dwLength <= 2))
   {
      return 0;
   }
   wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);

   return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
   uint16_t wCRC = 0;
   if ((pchMessage == NULL) || (dwLength <= 2))
   {
       return;
   }
   wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
   pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
   pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}


union{
uint8_t send[17];
MCU2MoterMessage crcwatch;
}unitee_crc_check;

MCU2MoterMessage *convertSetVal2Msg(MCU2MoterMessage *cmdmsg, const MotorSetValTypeDef setVal)
{	
    cmdmsg->s1 = 0xfe;
    cmdmsg->s2 = 0xee;
    cmdmsg->motorID = setVal.ID;
    cmdmsg->work_mode=setVal.mode;

    cmdmsg->t =  (setVal.tor_des * 256);
    cmdmsg->w =  (setVal.spd_des * 128);
    cmdmsg->pos =(setVal.pos_des * (32768 / (2 * 3.1415926)));
    cmdmsg->kp = (setVal.kp_des * 1280);
    cmdmsg->kw = (setVal.kd_des * 1280);
	
	 unitee_crc_check.crcwatch=*cmdmsg;
	
	 cmdmsg->CRC16=crc_ccitt(0,unitee_crc_check.send,15); 

   return cmdmsg;
}; // 17bytes


void Motor_Set_Current2_0_4(signed short int i1, signed short int i2, signed short int i3, signed short int i4) //发送
{
	uint8_t DATA_TX[8];
	
	DATA_TX[0] = i1 >> 8;
	DATA_TX[1] = i1;
	DATA_TX[2] = i2 >> 8;
	DATA_TX[3] = i2;
	DATA_TX[4] = i3 >> 8;
	DATA_TX[5] = i3;
	DATA_TX[6] = i4 >> 8;
	DATA_TX[7] = i4;
	
	CANx_SendStdData(&hcan2,0x1ff,DATA_TX,8);	
}

void PHC_GIMBAL(int16_t pitch,int16_t yaw)
{
	uint8_t DATA_TX[8];
	
	DATA_TX[0] = pitch >> 8;
	DATA_TX[1] = pitch;
	DATA_TX[2] = yaw >> 8;
	DATA_TX[3] = yaw;     

	CANx_SendStdData(&hcan1,0X1FF,DATA_TX,8);	
//	CANx_SendStdData(&hcan2,0X1FF,DATA_TX,8);
}

void Motor_Set_Current1_0_4(signed short int i1, signed short int i2, signed short int i3, signed short int i4) //发送
{

	uint8_t DATA_TX[8];
	
	DATA_TX[0] = i1 >> 8;
	DATA_TX[1] = i1;
	DATA_TX[2] = i2 >> 8;
	DATA_TX[3] = i2;
	DATA_TX[4] = i3 >> 8;
	DATA_TX[5] = i3;
	DATA_TX[6] = i4 >> 8;
	DATA_TX[7] = i4;
	
	CANx_SendStdData(&hcan1,0x200,DATA_TX,8);
	
}

void Set_gimbal_6020(int16_t pitch,int16_t yaw)
{
	uint8_t DATA_TX[8];
	
	DATA_TX[0] = pitch >> 8;
	DATA_TX[1] = pitch;
	DATA_TX[2] = yaw >> 8;
	DATA_TX[3] = yaw;
	
	CANx_SendStdData(&hcan1,0x1FF,DATA_TX,8);
}

void Solution_6020(volatile dji_motor *v)
{
int16_t  temp,temp_speed;
v->last_raw_value=v->raw_value;
v->raw_value=(usedata[0]<<8)|usedata[1];
if(v->tool_flag==0){v->tool_flag++;temp=v->raw_value;}
else{
v->diff=v->raw_value-v->last_raw_value;
if(v->diff>4000)
{
temp=v->raw_value-8192;
v->round_cnt--;
}
else if(v->diff<-4000)
{
temp=v->raw_value+8192;
v->round_cnt++;
}
else{temp=v->raw_value;} }
v->real_angle=temp*0.0439453125f;
temp_speed=(usedata[2]<<8)|usedata[3];
if(temp_speed>30000) {v->re_speed=(temp_speed-65535)/8192;}
else {v->re_speed=temp_speed/8192;}
v->ALL_angle=v->real_angle+v->round_cnt*360;
v->electricity=(usedata[4]<<8)|usedata[5];
}

char Solution_2006(volatile dji_motor *v)
{
	static int xyz;
	v->last_raw_value = v->raw_value;
	v->raw_value = (usedata[0]<<8)|usedata[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8191;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8191;
	}			
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 8191;
	v->ecd_angle = ((float)(v->raw_value - v->ecd_bias)*360/8191 + v->round_cnt * 360)/19;
//	if(xyz==0) {v->zero_angle=v->ecd_angle;xyz++;}
//	v->relative_zero_angle=v->ecd_angle-v->zero_angle;   //相对上电零度
	v->re_speed = (int16_t)(usedata[2]<<8|usedata[3]);
return 0;	
}

char Solution_3508(volatile dji_motor *v)
{
	static int xyz;
	v->last_raw_value = v->raw_value;
	v->raw_value = (usedata[0]<<8)|usedata[1];
	v->electricity = (usedata[4]<<4)|(usedata[5]);
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8191;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8191;
	}			
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 8191;
	v->ecd_angle = ((float)(v->raw_value - v->ecd_bias)*360/8191 + v->round_cnt * 360)/19;
//	if(xyz==0) {v->zero_angle=v->ecd_angle;xyz++;}
//	v->relative_zero_angle=v->ecd_angle-v->zero_angle;   //相对上电零度
	v->re_speed = (int16_t)(usedata[2]<<8|usedata[3]);
return 0;	
}

RxCAN_info_type_2_s MI_L,MI_R;
int16_t temp111;

void solution_xiaomi(uint8_t *usedata,mi_leg_motor *abc,RxCAN_info_type_2_s *mi)
{
	RxCAN_info_s RxCAN_info;//用于存储小米电机反馈的数据
	memcpy(&RxCAN_info,&RxHeader.ExtId,4);//将扩展标识符的内容解码成对应内容
	if(RxCAN_info.communication_type==2){
//	RxCAN_info_type_2_s RxCAN_info_type_2;
	memcpy(&mi,&RxHeader.ExtId,4);//将扩展标识符的内容解码成通信类型2的对应内容
	temp111= ((usedata[0]<<8)|usedata[1]);	
  if(temp111>0) 		
	{abc->angle=((float)temp111-32767.5)/32767.5*4*3.1415926f;}
	else
	{abc->angle=((float)temp111+32767.5)/32767.5*4*3.1415926f;}
	temp111=(usedata[2]<<8)|(usedata[3]);
	if(temp111>0) 
	{abc->rad_s=((float)temp111-32767.5)/32767.5*30.0f;} 
  else
  {abc->rad_s=((float)temp111+32767.5)/32767.5*30.0f;}
	temp111= ((usedata[4]<<8)|usedata[5]);
	if(temp111>0) 
	{abc->torque=((float)temp111-32767.5)/32767.5*12.0f;}
	else
	{abc->torque=((float)temp111+32767.5)/32767.5*12.0f;}	
		}
}


void Send_to_chassic(u8 u1,u8 u2,u8 u3,u8 u4,pack_send_gimbal*abc,pack_send_mouse *pdf,u8 u5,u8 u6) //发送
{
	uint8_t DATA_TX[8];
	
	DATA_TX[0] = u1;
	DATA_TX[1] = u2;
	DATA_TX[2] = u3;
	DATA_TX[3] = u4;	
	DATA_TX[4] = abc->rc_station;
	DATA_TX[5] = pdf->keyboard;
	DATA_TX[6] = u5;
	DATA_TX[7] = u6;


  CANx_SendStdData(&hcan1,0x00a,DATA_TX,8);	
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

float solution_old_mi(old_mi* a,uint8_t *data)
{
        int id = data[0];
        int p_int = data[1];
        p_int<<=8;
        p_int|=data[2];
        int v_int = data[3];
        v_int<<=4;
        v_int|=data[4]>>4;
        int t_int = data[4]&0xf;
        t_int<<=8;
        t_int|=data[5];

        a->pos = uint_to_float(p_int, P_MIN, P_MAX, 16);
        a->vel = uint_to_float(v_int, V_MIN, V_MAX, 12);
        a->current = uint_to_float(t_int, -27, 27, 12);
}

CAN_HandleTypeDef hcan;
int16_t ID_LIST_1[6];
int16_t ID_LIST_2[6];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{   
  uint16_t temp;
	static uint16_t last_angle;	
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, usedata) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  } 
 if(hcan->Instance==CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_FILTER_FIFO0,&RxHeader,usedata);
		if(RxHeader.IDE == CAN_ID_EXT) {
		if(Get_Motor_ID(RxHeader.ExtId)==0x7F) 
    {solution_xiaomi(usedata,&MI_LEFT,&MI_L);}}
		else if(RxHeader.IDE==CAN_ID_STD){		
    switch(RxHeader.StdId)                   
    {   
			case 0x205:Solution_3508(&SHOOT_R);   break;
			case 0x206:Solution_3508(&SHOOT_L);   break;	
			case 0x207:Solution_3508(&SHOOT_H);   break;	
//			case 0x1FF:break;
//			case 0x200:break;
			case 0x000:
			{
			 switch(usedata[0])
			 {
				 case 1:solution_old_mi(&OMI_2_1,usedata);break;
				 case 4:solution_old_mi(&OMI_2_4,usedata);break;
			 }
			}break;
      default : ID_LIST_1[1]=RxHeader.StdId; break;			
		}
	}}	
 if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&RxHeader,usedata);
		if(RxHeader.IDE == CAN_ID_EXT) {
		if(Get_Motor_ID(RxHeader.ExtId)==0x7F) 
    {solution_xiaomi(usedata,&MI_RIGHT,&MI_L);} }
		else if(RxHeader.IDE==CAN_ID_STD){
	   switch(RxHeader.StdId)                   
     {
			case 0x201:Solution_3508(&HERO_CLIP); 
			static int c;
			if(c==0) {HERO_CLIP.zero_angle=HERO_CLIP.ecd_angle;c++;}
	    HERO_CLIP.relative_zero_angle=HERO_CLIP.ecd_angle-HERO_CLIP.zero_angle;
			break;	
			case 0x202:Solution_3508(&HERO_WATCH); 
			static int d;
			if(d==0) {HERO_WATCH.zero_angle=HERO_WATCH.ecd_angle;c++;}
	    HERO_WATCH.relative_zero_angle=HERO_WATCH.ecd_angle-HERO_WATCH.zero_angle;
			break;	
			case 211:
			guard_gimbal.now_hot=(usedata[0]<<8)|usedata[1];
			guard_gimbal.max_hot=(usedata[2]<<8)|usedata[3];				
			break;
			case 0x000:
			{
			 switch(usedata[0])
			 {
				 case 1:solution_old_mi(&OMI_1_1,usedata);break;
				 case 4:solution_old_mi(&OMI_1_4,usedata);break;
			 }
			}break;
      default : ID_LIST_2[1]=RxHeader.StdId; break;			
		 }
	} }   
}

extern UART_HandleTypeDef huart1;
u8 recieve_unitee[40];


/// @brief 将接收到的定点类型原始数据转换为浮点参数类型
/// @param motor_r 要转换的电机反馈结构体  宇树
void extract_data(Unitee_MotorData_t *motor_r)
{
//	if(motor_r->rxlen != sizeof(motor_r->motor_recv_data))
//	{
//		motor_r->correct = 0;
//		return;
//	}

	if (motor_r->motor_recv_data.head[0] != 0xFD || motor_r->motor_recv_data.head[1] != 0xEE)
	{
		motor_r->correct = 0;
		return;
	}
	
	if(motor_r->motor_recv_data.CRC16 !=
		crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, sizeof(motor_r->motor_recv_data) - 2)){
		motor_r->correct = 0;
		return;
	}else
	{
		motor_r->motor_id = motor_r->motor_recv_data.mode.id;
		motor_r->mode = motor_r->motor_recv_data.mode.status;
		motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
		motor_r->MError = motor_r->motor_recv_data.fbk.MError;
		motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256.0f)*6.28318f ;
		motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256.0f;
		motor_r->Pos = 6.28318f*((float)motor_r->motor_recv_data.fbk.pos) / 32768.0f;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;
		return;
	}
}

#pragma pack(1)
typedef struct{
int16_t pos;
int16_t vel    :12;
int16_t KP     :12;
int16_t KD     :12;
int16_t Tor    :12;
}old_mi_send;
#pragma pack()

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

void Send_to_LMI(u8 ID,u8 CAN,float Tor) //发送
{
	uint8_t data[8];
		
    int p_int = float_to_uint(0, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(0, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(0, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(0.1f, KD_MIN, KD_MAX, 12);
    int tq_int = float_to_uint(Tor, -27, 27, 12);
    
    data[0] = p_int >> 8;
    data[1] = p_int & 0xff;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xf)<<4) | (kp_int>>8);
    data[4] = kp_int & 0xff;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xf)<<4) | (tq_int>>8);
    data[7] = tq_int & 0xff;
		
	if(CAN==1)
  CANx_SendStdData(&hcan1,ID,data,8);	
	else
  CANx_SendStdData(&hcan2,ID,data,8);			
}


