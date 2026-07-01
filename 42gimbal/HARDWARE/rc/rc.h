#ifndef RC_H
#define RC_H
#include "main.h"

typedef struct
{
	int16_t R_x;  //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //右手y    上1684 下364 中1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//拨动左  上1 中3 下2
	uint8_t sr;
	
	uint8_t sl_change_flag;			//拨动左  上1 中3 下2
	uint8_t sr_change_flag;
	
	int16_t mouse_x;	//鼠标
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//鼠标按键
	uint8_t mouse_r;
	uint16_t key;		//键盘
	u8 W;
	u8 S;
	u8 A;
	u8 D;
	u8 Shift;
	u8 Ctrl;
	u8 Q;
	u8 E;
	u8 R;
	u8 F;
	u8 G;
	u8 Z;
	u8 X;
	u8 C;
	u8 V;
	u8 B;
	
	uint8_t last_sr;
	uint8_t last_sl;
	uint8_t last_mouse_l;
	uint8_t last_R;
	uint64_t cnts;
	uint16_t trigger;
	
	float time;
}RC;

//PS2手柄键值数据结构体	 
typedef struct			 				
{
  uint8_t mode;		    /* 手柄的工作模式 */

  uint8_t btn1;         /* B0:SLCT B3:STRT B4:UP B5:R B6:DOWN B7:L   */

  uint8_t btn2;         /* B0:L2 B1:R2 B2:L1 B3:R1 B4:/\ B5:O B6:X B7:口 */

  uint8_t RJoy_LR;      /* 右边摇杆  0x00 = 左    0xff = 右   */

  uint8_t RJoy_UD;      /* 右边摇杆  0x00 = 上    0xff = 下   */

  uint8_t LJoy_LR;      /* 左边摇杆  0x00 = 左    0xff = 右   */

  uint8_t LJoy_UD;      /* 左边摇杆  0x00 = 上    0xff = 下   */
	
}JOYSTICK_TypeDef;


typedef union{
u8 rc_station;
struct{
int16_t rc_sr : 4;
int16_t rc_sl : 4;}rc;
}pack_recieve_gimbal;

typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint8_t without:2;
	  uint16_t W:1;
	  uint16_t S:1;
	  uint16_t A:1;
	  uint16_t D:1;
	  uint16_t SHIFT:1;
	  uint16_t CTRL:1;
	  uint16_t Q:1;
	  uint16_t E:1;
	  uint16_t R:1;
	  uint16_t F:1;
	  uint16_t G:1;
	  uint16_t Z:1;
	  uint16_t X:1;
	  uint16_t C:1;
	  uint16_t V:1;
	  uint16_t B:1;
    uint16_t crc16;
}VT13_data;

void rc_init(void);
void deal_dbus(u8 *RXBuff);
void RC_restart(uint16_t dma_buf_num);
static uint8_t PS2_ReadWriteData(uint8_t data);
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct);
void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);

#endif 
