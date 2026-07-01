#ifndef _UMPIREDATA_H_
#define _UMPIREDATA_H_
/*-------------------UMpireSystemDefineState-----------------*/
#include  "main.h"

typedef  uint8_t u8;
typedef  uint16_t u16;
typedef  uint32_t u32;

#define  FrameSize          128  

#define  frame_data_length                5
#define  cmdid_crc16_length               4

#define  STATE_FIND_0XA5                  0
#define  STATE_FIND_0XA5_BUFF_LENGTH      1

#define  STATE_FIND_frame_header          1  
#define  STATE_FIND_frame_header_LENGTH   4

#define  STATE_FIND_cmdid_data            2  

//cmdid_state
#define  game_status_t          0x0001
#define  game_result_t          0x0002
#define  game_robot_HP_t        0x0003

#define  event_data_t           0x0101
#define  referee_warning_t      0x0104
#define  dart_info_t            0x0105

#define  robot_status_t         0x0201
#define  power_heat_data_t      0x0202
#define  robot_pos_t            0x0203
#define  buff_t                 0x0204
#define  hurt_data_t            0x0206
#define  shoot_data_t             0x0207
#define  projectile_allowance_t   0x0208
#define  rfid_status_t            0x0209
#define  dart_client_cmd_t        0x020A
#define  ground_robot_position_t  0x020B
#define  radar_mark_data_t        0x020C
#define  sentry_info_t            0x020D
#define  radar_info_t             0x020E

#define  robot_interaction_data_t 0x0301



/*-------------------UMpireSystemDatastruct-----------------*/
typedef  struct
{ //0001
	u8   game_type:4;       //比赛类型
	u8   game_progress:4;   //当前比赛阶段         
	u16  remain_time;       //当前阶段剩余时间     *
	//0002
	u8   winner:4;          //比赛结果
	//0003
	u16  red_1_HP;          //红1英雄血量
	u16  red_2_HP;          //红2工程血量
	u16  red_3_HP;          //红3步兵血量
	u16  red_4_HP;          //红4步兵血量
	u16  red_5_HP;          //红5步兵血量
	u16  red_7_HP;          //红7哨兵血量
	u16  red_outpost_HP;    //红方前哨站血量
	u16  red_base_HP;       //红方基地血量
	u16  blue_1_HP;         //蓝1英雄血量
	u16  blue_2_HP;         //蓝2工程血量
  u16  blue_3_HP; 	      //蓝3步兵血量
	u16  blue_4_HP;         //。。。。
	u16  blue_5_HP;         //。。。。
	u16  blue_7_HP;         //。。。。
	u16  blue_outpost_HP;   //蓝方前哨站血量
	u16  blue_base_HP;
	//0004
	u8   dart_belong;             //发射飞镖任务
	u16  stage_remaining_time;    //发射剩余时间
	//0101 
	u32  event_type;              //场地事件数据
	//0102 
	u8   supply_projectile_id;   //补给站口ID
	u8   supply_robot_id;        //补单机器人
	u8   supply_projectile_step; //出弹口状态
	u8   supply_projectile_num;  //补单数量
	//0103
	
	//0104
	u8   level;                  //警告等级
	u8   foul_id;                //犯规机器人ID
	//0105
	u8 dart_remaining_time;       //飞镖发射口倒计时
	//0201 
	uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output :  1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
	//0202
	uint16_t reserved_202_1;
  uint16_t reserved_202_2; 
  float    reserved_202_3; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat;
	//0x0203 
  float x;                                      //位置x m
  float y; 
  float z;
	float yaw;                                    //位置枪口 度
	//0x0204
	u8  power_rune_buff;                           //机器人增益        *
	//0x0205
	u16  energy_point;                              //空中机器人能量点
	u8   attack_time;                               //可攻击时间
	//0x0206
	u8 armar_id :4;                                 //被打装甲id       *
	u8 hurt_type:4;                                 //伤害类型         *
	//0x0207
	u8 bullet_type;                                 //子弹类型
	u8 bullet_freq;                                 //子弹射频         *
	float bullet_speed;                             //子弹射速         *
	//0x0208 
	u16 bullet_remaining_num;                       //弹丸剩余         *
	//0x0209
	u32 rfid_status;                                //RFID状态
	//0x020A
	u8  dart_launch_opening_status;                 //飞镖口发射状态 
	u8  dart_attack_target;                         //飞镖打击目标
	u16	target_change_time;                         //切换目标时剩余时间
	u8  first_dart_speed;                           //检测第一枚飞镖速度
	u8  second_dart_speed;                          //检测第二枚飞镖速度
	u8  third_dart_speed;                           //3
	u8  fourth_dart_speed;                          //4
	u16 last_dart_launch_time;                      //最近一次发射比赛剩余时间
	u16 operate_launch_cmd_time;                    //最近一次操作手确定发射时比赛时间     
	//0x0301
	
}Robot_Para;//存放机器人参数结构体

extern Robot_Para robot; 

void GET_Umpire_DATA(uint8_t *data,uint16_t front_add,uint16_t data_size ,uint16_t length);
void SUPER_Um_State_INIT(void);

u16 Uint8_to_Uint16 (u8 a,u8 b);
u32 Uint8_to_Uint32(u8 a,u8 b,u8 c,u8 d);
u8  Uint8_to_Bit (u8 shuru,u8 wei,u8 shu);

void GET_0XA5(void);
void GET_frame_header(void);
void GET_robot_info(void);

void GET_game_status_info(void);
void GET_game_result_info(void);
void GET_robot_status_info(void);
void GET_power_heat_data_info(void);
void GET_shoot_data_info(void);
float Uint8_to_float (u8 a,u8 b,u8 c,u8 d);

#endif


