#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "main.h"

#pragma pack(1)
typedef struct
{
  uint8_t header ;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
} RV_PacketVision_send;
#pragma pack()

#pragma pack(1)
typedef struct 
{
  uint8_t header   ;
  uint8_t tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum ;
} RV_PacketVision_recieve;
#pragma pack()

#pragma pack(1)
typedef  struct
{    
//	  u8 head;                           //0
//    u8 enemy_color;               //1
//    float pitch;                       //2
//    float yaw;                       //6
//	  float chassis_angle;               //10
//    u8 id;            //16
//    uint16_t blood;                       //14
//    int16_t remaining_time;            //16
//    float outpost_hp;                  //20                            
//    int16_t is_outpost_attacking;      //22
//    u8 is_retreating;                  //23
//    u8 is_drone_avoiding;              //24
//    u8 game_status;                    //25

//    u8 rear;                           //26
	
	  u8 head;                           //0
    u8 enemy_color;               //1
    float roll;                       //2
    float pitch;                       //6
	  float yaw;               //10
    u8 shoot_speed;                       //14
    u8 rear;                           //26
} ProtocolSentry;
#pragma pack()

#pragma pack(1)
typedef  struct
{    
	  u8 head;   
    u8 fire;
	  float pitch;
  	float yaw;
  	float distance;
    u8 rear;                           
} FYT_recieve;
#pragma pack()

void vision_task();
void recieve_vision_cdc(u8 *temp_data);


#endif
