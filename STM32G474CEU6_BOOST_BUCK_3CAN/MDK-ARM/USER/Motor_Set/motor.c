#include "motor.h"

uint16_t can_rx_data_num;  
RC rc;


DJI_motor STEER_6020_I_MOTOR[4],WHEEL_3508_I_MOTOR[4],YAW_6020,HERO_CLIP; 

void Solution_6020(volatile DJI_motor *v,u8 *usedata)
{
	int16_t  temp;//temp_speed;
	v->last_raw_value=v->raw_value;
	v->raw_value=(usedata[0]<<8)|usedata[1];
	if(v->tool_flag==0){v->tool_flag++;temp=v->raw_value;}
	else
	{
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
		else{temp=v->raw_value;} 
	}
	v->real_angle=temp*0.0439453125f;
	v->re_speed=(usedata[2]<<8)|usedata[3];
	//if(temp_speed>30000) {v->re_speed=(float)(temp_speed-65535)/8192;}
	//else {v->re_speed=(float)temp_speed/8192;}
	v->ALL_angle=v->real_angle+v->round_cnt*360;
	if(v->zero_flag==0) {v->zero_angle=v->ALL_angle;v->zero_flag++;}
	v->relative_zero_angle=v->ALL_angle-v->zero_angle;   //相对上电零度
	v->electricity=(usedata[4]<<8)|usedata[5];
}


char Solution_3508(volatile DJI_motor *v,u8 *usedata)
{
	//static int xyz;
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
	v->electricity=(usedata[4]<<8)|usedata[5];
return 0;	
}


void Solution_Message_IN_CAN(u8 *a,int16_t ID)
{
	can_rx_data_num++;
	switch(ID)
	{
	  case 0x0201: Solution_3508(&WHEEL_3508_I_MOTOR[0],a); break; 
	  case 0x0202: Solution_3508(&WHEEL_3508_I_MOTOR[1],a); break; 
	  case 0x0203: Solution_3508(&WHEEL_3508_I_MOTOR[2],a); break; 
	  case 0x0204: Solution_3508(&WHEEL_3508_I_MOTOR[3],a); break; 		
		default :  break;	
	}
}

typedef union{
u8    keyboard;
struct{
u8    f      :1;
u8	  e      :1;
u8    shift  :1;
u8    q      :1;
u8    r      :1;
u8	  c      :1;
u8    mouse_r:1;
u8    mouse_l:1;
}rc;
}pack_send_mouse;
pack_recieve_gimbal save_gim;
pack_send_mouse save_board;
int16_t recieve_pitch;

void getControlData(uint8_t *recieve)
{
	 static u8 last_sr,last_sl,last_turn_q;
	 int16_t tempccc;
	  tempccc = (recieve[0]<<8|(recieve[1]));
	if(tempccc>30000)   rc.R_x=tempccc-65535;
	else   {rc.R_x=tempccc;}
   tempccc  = (recieve[2]<<8|(recieve[3]));
	if(tempccc>30000)   rc.R_y=tempccc-65535;
	else   {rc.R_y=tempccc;}
	
//  recieve_bdp_err=recieve[6];
	recieve_pitch=recieve[6]<<8|recieve[7];

	 save_gim.rc_station  = recieve[4];
	 save_board.keyboard=recieve[5];
	 rc.Shift=save_board.rc.shift;
	 rc.Q=save_board.rc.q;
	 rc.E=save_board.rc.e;
	 rc.sl=save_gim.rc.rc_sl;
	 rc.sr=save_gim.rc.rc_sr;	
	 rc.if_zimiao=save_gim.rc.zimiao_live;
//	count_shoot=save_gim.rc.direction;
	rc.live_over=100;
	
//	 rc.sl_change_flag=if_change(rc.sl,last_sl);
//	 rc.sr_change_flag=if_change(rc.sr,last_sr);

	 last_sr=rc.sr;
	 last_sl=rc.sl;   	
}

void Solution_Message_IN_CAN3(u8 *a,int16_t ID)
{
	can_rx_data_num++;
	switch(ID)
	{
			case 0x201:Solution_3508(&HERO_CLIP,a); 
			static int c;
			if(c==0) {HERO_CLIP.zero_angle=HERO_CLIP.ecd_angle;c++;}
	    HERO_CLIP.relative_zero_angle=HERO_CLIP.ecd_angle-HERO_CLIP.zero_angle;
			break;	
      case 0x206:Solution_6020(&YAW_6020,a);break;		//1M	
			case 0x00a:{getControlData(a);}break;        //1M
      default: break;		
  }
}

