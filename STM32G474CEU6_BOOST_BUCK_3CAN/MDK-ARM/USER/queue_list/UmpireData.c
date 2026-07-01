#include "UmpireData.h"
#include "string.h"

CCMRAM uint16_t STATE_UM_data_length=0;
CCMRAM uint16_t cmd_id=0;
CCMRAM Robot_Para robot; 
//裁判系统读取状态机结构体
typedef struct
{
	
  int16_t  BUFF_LENTH;       //进入一个步骤前需要读取的数据长度
	uint16_t READ_QUEUE_ADD;  //实际读取队列的位置
	uint16_t READ_BUFF_ADD;   //实际读取缓冲区的位置
	uint16_t READ_STATE;      //当前进行的读取阶段
	uint8_t BUFF[FrameSize];  //用于缓存读取阶段所需数据的缓冲区   
	
}Um_state;
CCMRAM Um_state SUPER_Um_State;

//裁判系统分析状态结构体初始化
void SUPER_Um_State_INIT()
{
  SUPER_Um_State.READ_STATE=STATE_FIND_0XA5;
	SUPER_Um_State.BUFF_LENTH=STATE_FIND_0XA5_BUFF_LENGTH;
	
	SUPER_Um_State.READ_BUFF_ADD=0;
  SUPER_Um_State.READ_QUEUE_ADD=0;
	SUPER_Um_State.BUFF[0]=0xff;
	//初始化读取阶段，读取阶段0，从队列里取一个字节，寻找0XA5
	return ;
}

//裁判系统分析状态结构体复位到find_0xa5
CCMRAM void SUPER_Um_State_RESET()
{
  SUPER_Um_State.READ_STATE=STATE_FIND_0XA5;
	SUPER_Um_State.BUFF_LENTH=STATE_FIND_0XA5_BUFF_LENGTH;
	
	SUPER_Um_State.READ_BUFF_ADD=0;
	SUPER_Um_State.BUFF[0]=0xff;
	return ;
}

//裁判系统数据分析
CCMRAM void GET_Umpire_DATA(uint8_t *data,uint16_t front_add,uint16_t data_size ,uint16_t length)
{
	 //获取队列中可读取数据的长度
   uint16_t READ_length=length;
	 SUPER_Um_State.READ_QUEUE_ADD=front_add;
	 //循环读取分析数据开始
	 while((READ_length--)>0)
	 {

	   SUPER_Um_State.BUFF[SUPER_Um_State.READ_BUFF_ADD++]=data[SUPER_Um_State.READ_QUEUE_ADD++];
		 SUPER_Um_State.BUFF_LENTH--;
		 
		 //buff数据搬运完成进行阶段性数据解算
		 if(SUPER_Um_State.BUFF_LENTH<=0)
		 {
		   switch (SUPER_Um_State.READ_STATE)
		   {
			   case STATE_FIND_0XA5 :           {  GET_0XA5();          break;}
				 
				 case STATE_FIND_frame_header:    {  GET_frame_header();  break;}
				 
			   case STATE_FIND_cmdid_data:      {  GET_robot_info();    break;}
			 
		     default:SUPER_Um_State_RESET();break;
			 }
		 }
     if(SUPER_Um_State.READ_QUEUE_ADD>data_size){SUPER_Um_State.READ_QUEUE_ADD=0;} 
	 }
}


CCMRAM void GET_0XA5(void)
{
		if(SUPER_Um_State.BUFF[0]==0xA5)//数据帧头为0XA5
		{
		  SUPER_Um_State.READ_STATE=STATE_FIND_frame_header;
			SUPER_Um_State.BUFF_LENTH=STATE_FIND_frame_header_LENGTH;
		}
		else
    { 
	    SUPER_Um_State_RESET();
		}
		return;
}

CCMRAM void GET_frame_header(void)
{
	  if(!Verify_CRC8_Check_Sum(SUPER_Um_State.BUFF,frame_data_length)){ SUPER_Um_State_RESET();return;} 
		//从帧头获取后面数据的字符长度datalength+2byte_cmdid+2byte_crc16
		  STATE_UM_data_length=Uint8_to_Uint16(SUPER_Um_State.BUFF[2],SUPER_Um_State.BUFF[1])+4;
      SUPER_Um_State.BUFF_LENTH=STATE_UM_data_length;                                
      SUPER_Um_State.READ_STATE=STATE_FIND_cmdid_data;
		return;
}

CCMRAM void GET_robot_info(void)
{
    cmd_id=Uint8_to_Uint16(SUPER_Um_State.BUFF[6],SUPER_Um_State.BUFF[5]);
		if(!Verify_CRC16_Check_Sum(SUPER_Um_State.BUFF,frame_data_length+STATE_UM_data_length)){SUPER_Um_State_RESET();return;}
		
		switch(cmd_id)
		{
			case  game_status_t:        GET_game_status_info();      break;
			
		  case  game_result_t:        GET_game_result_info();      break;
			 
			case  robot_status_t:       GET_robot_status_info();     break;
			
			case shoot_data_t:          GET_shoot_data_info();       break;
			
			case  power_heat_data_t:    GET_power_heat_data_info();  break;
				
		  default:break;
		}
		SUPER_Um_State_RESET();
		return;
}

CCMRAM void GET_game_status_info(void)
{
	robot.game_type    =Uint8_to_Bit(SUPER_Um_State.BUFF[7],1,3);  
	robot.game_progress=SUPER_Um_State.BUFF[7]>>4;
	robot.remain_time  =Uint8_to_Uint16(SUPER_Um_State.BUFF[9],SUPER_Um_State.BUFF[8]);
  return;
}

CCMRAM void GET_game_result_info(void)
{
	robot.winner=SUPER_Um_State.BUFF[7];
  return;
}

CCMRAM void GET_robot_status_info(void)   //0x201
{
	robot.robot_id       =SUPER_Um_State.BUFF[7]; 
  robot.robot_level    =SUPER_Um_State.BUFF[8];  
  robot.current_HP  =Uint8_to_Uint16(SUPER_Um_State.BUFF[10],SUPER_Um_State.BUFF[9]);  
  robot.maximum_HP  =Uint8_to_Uint16(SUPER_Um_State.BUFF[12],SUPER_Um_State.BUFF[11]);  
  robot.shooter_barrel_cooling_value=Uint8_to_Uint16(SUPER_Um_State.BUFF[14],SUPER_Um_State.BUFF[13]); 
  robot.shooter_barrel_heat_limit   =Uint8_to_Uint16(SUPER_Um_State.BUFF[16],SUPER_Um_State.BUFF[15]); 
  robot.chassis_power_limit         =Uint8_to_Uint16(SUPER_Um_State.BUFF[18],SUPER_Um_State.BUFF[17]);//底盘限制功率
  robot.power_management_gimbal_output  =SUPER_Um_State.BUFF[19]>>2; 
  robot.power_management_chassis_output =SUPER_Um_State.BUFF[19]>>1;  //底盘功率
  robot.power_management_shooter_output =SUPER_Um_State.BUFF[19]>>0;
	return;
}
CCMRAM void GET_power_heat_data_info(void)     //0x202
{
  robot.buffer_energy               =Uint8_to_Uint16(SUPER_Um_State.BUFF[16],SUPER_Um_State.BUFF[15]);//缓冲能量默认60j飞坡250j
  robot.shooter_17mm_1_barrel_heat  =Uint8_to_Uint16(SUPER_Um_State.BUFF[18],SUPER_Um_State.BUFF[17]);
  robot.shooter_17mm_2_barrel_heat  =Uint8_to_Uint16(SUPER_Um_State.BUFF[20],SUPER_Um_State.BUFF[19]);
  robot.shooter_42mm_barrel_heat    =Uint8_to_Uint16(SUPER_Um_State.BUFF[22],SUPER_Um_State.BUFF[21]);
	return;
}
CCMRAM void GET_shoot_data_info(void)   //0x207
{
  robot.bullet_speed  =Uint8_to_float(SUPER_Um_State.BUFF[13],SUPER_Um_State.BUFF[12],SUPER_Um_State.BUFF[11],SUPER_Um_State.BUFF[10]);  
	return;
}

union {
float ccc;
uint8_t abc[4];
}turn_temp;

CCMRAM float Uint8_to_float (u8 a,u8 b,u8 c,u8 d)   
{
	turn_temp.abc[0]=a;
	turn_temp.abc[1]=b;
	turn_temp.abc[2]=c;
	turn_temp.abc[3]=d;

  return turn_temp.ccc;
}

CCMRAM u16 Uint8_to_Uint16 (u8 a,u8 b)   
{
	u16 c=0;
  c=((c+a)<<8)+b;
  return c;
}

CCMRAM u32 Uint8_to_Uint32(u8 a,u8 b,u8 c,u8 d)  
{
  u32 chu=0;
	u16 zhong1=0;
	u16 zhong2=0;
	zhong1=Uint8_to_Uint16( a, b);
	zhong2=Uint8_to_Uint16( c, d);
	chu=((0x00000000+zhong1)<<16)+zhong2;
	return chu;
}

CCMRAM u8 Uint8_to_Bit (u8 shuru,u8 wei,u8 shu)  
{
	u8 shuchu;
	u8 weiyi;
	weiyi=8-wei+1-shu;
	shuchu=shuru<<weiyi;
	shuchu=shuchu>>(8-shu);
  return shuchu;
}




