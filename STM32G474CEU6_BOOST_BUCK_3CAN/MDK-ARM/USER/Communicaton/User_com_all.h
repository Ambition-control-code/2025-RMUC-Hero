#ifndef __USER_COM_H
#define __USER_COM_H
#include "main.h"
#include "fdcan.h"
#include "usart.h"

#define KEY_UP 3
#define KEY_DOWN 2
#define KEY_PRESS 1

#define BEEP_ON   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4)
#define BEEP_OFF  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4)

//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
//DBUS接收结构体
typedef  struct
{

	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	union
	{
		uint16_t v;
		struct
		{
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
		} set;
	} key;

	uint8_t rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

typedef struct
{
	int16_t R_x;  //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //右手y    上1684 下364 中1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//拨动左  上1 中3 下2
	uint8_t sr;
	
	int16_t mouse_x;	//鼠标
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//鼠标按键
	uint8_t mouse_r;
	uint16_t key;		//键盘
	uint8_t W;
	uint8_t S;
	uint8_t A;
	uint8_t D;
	uint8_t Shift;
	uint8_t Ctrl;
	uint8_t Q;
	uint8_t E;
	uint8_t R;
	uint8_t F;
	uint8_t G;
	uint8_t Z;
	uint8_t X;
	uint8_t C;
	uint8_t V;
	uint8_t B;
	
	uint8_t last_sr;
	uint8_t last_sl;
	uint8_t last_mouse_l;
	uint8_t last_R;
	float cnts;	
		
	uint8_t sl_change_flag;			//拨动左  上1 中3 下2
	uint8_t sr_change_flag;
	
	uint8_t if_zimiao;
	
	uint8_t live_over,direction,jump;
}RC;


#define LED1_ON   HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
#define LED2_ON	  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
#define LED1_OFF  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
#define LED2_OFF	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);

#define RC_FRAME_LENGTH		18u

#define USART1_RX_BUFF_LENGTH   36  //RC_FRAME_LENGTH*2
#define USART1_TX_BUFF_LENGTH   256

#define USART3_RX_BUFF_LENGTH   256
#define USART3_TX_BUFF_LENGTH   256
  	
#define CAP_DATA_TX_CAN_ID  0x211
#define ENG_DATA_RX_CAN_ID  0x02f

//串口二级缓冲区

extern uint8_t   USART1_RX_BUFF[USART1_RX_BUFF_LENGTH];
extern uint8_t   USART1_TX_BUFF[USART1_TX_BUFF_LENGTH];

extern uint8_t   USART3_RX_BUFF[USART3_RX_BUFF_LENGTH];
extern uint8_t   USART3_TX_BUFF[USART3_TX_BUFF_LENGTH];

extern uint16_t CHASSIS_LIMIT_POWER;
extern uint16_t CHASSIS_ENG;

extern uint8_t  key_input_state;

void CAN_BSP_INIT(void);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

void USART_SET_INIT(void);
void USART3_IRQ_IDLE_CALLBACK(void);
void USART1_IRQ_IDLE_CALLBACK(void);
void USART1_XferHalfCpltCallback2(DMA_HandleTypeDef *hdma);
void USART3_XferHalfCpltCallback2(DMA_HandleTypeDef *hdma);
void USART3_TX_INT(void);

void CAP_DATA_TX_IN_CAN(void);

extern Remote_Info_Typedef Remote_Ctrl;
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl);

void KEY_scan(void);


#endif

