#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
#include "rc.h"        
#include "kalman.h"
#include "ulitility_tools.h"
#include "vision_task.h"
#include "gimbal_task.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "refree_task.h"
#include "motor.h"
#include "SolveTrajectory.h"

extern long int refree[9];
extern struct SolveTrajectoryParams st;
extern bucket_imu imu;      
extern Gimbal_all guard_gimbal;
extern UART_HandleTypeDef huart5;

unsigned char recieve_usb[12];
char usb_send[5],usb_send_data;

RV_PacketVision_send send_vision;
extKalman_t yaw_RV;

u8 send_vision_watch[30];
u8 save_vision_watch[50];
int z;

union 
{
ProtocolSentry send_vision;
u8 send_watch[16];
}FYT_VISION;

union
{
FYT_recieve recieve_vision;
u8 send_watch[16];
u8 aaa[16];
}vision_recieve_union;  

void send_without_crc16(void)
{
	memcpy(send_vision_watch,FYT_VISION.send_watch,16);//分配储存空间
	usb_cdc_send(send_vision_watch,16);
}

float recieve_pitch,recieve_yaw,x,y,zvi,AFFLITER;

float zimiao_watch[4],watch_pitch,watch_yaw;
void recieve_vision_cdc(u8 *temp_data)
{	
	memcpy(save_vision_watch,temp_data,16);//分配内存
	
	if(save_vision_watch[15]==0x0d)
	{
		memcpy(vision_recieve_union.aaa,save_vision_watch,16);
		
		watch_pitch=zimiao_watch[0]= -vision_recieve_union.recieve_vision.pitch;
		watch_yaw=zimiao_watch[1]= vision_recieve_union.recieve_vision.yaw;	
		zimiao_watch[2]= vision_recieve_union.recieve_vision.distance;	
		zimiao_watch[3]= vision_recieve_union.recieve_vision.fire;
	}

}

float aim_yawww;
void vision_task(void)   
{	
	FYT_VISION.send_vision.head=0xff;
	FYT_VISION.send_vision.enemy_color=1;	
	for(;;)
	{
		FYT_VISION.send_vision.pitch=imu.pitch;
		FYT_VISION.send_vision.yaw=imu.yaw[2];	
		FYT_VISION.send_vision.roll=imu.roll;	
//		FYT_VISION.send_vision.pitch=0;
//		FYT_VISION.send_vision.yaw=0;	
//		FYT_VISION.send_vision.roll=0;	
	    FYT_VISION.send_vision.shoot_speed=155;
	    FYT_VISION.send_watch[15]=0x0d;
		
		refree[VISION_TASK_SUM]++;    
				
		send_without_crc16();		
			
		osDelay(10);
	}
}












