#include "User_com_all.h"
#define READ_SA HAL_GPIO_ReadPin(SA_GPIO_Port,SA_Pin)
#define READ_SB HAL_GPIO_ReadPin(SB_GPIO_Port,SB_Pin)
#define READ_SKEY HAL_GPIO_ReadPin(SKEY_GPIO_Port,SKEY_Pin)

uint16_t key_scan_time[3];
uint8_t  key_input_state;

void KEY_scan(void)
{
   key_scan_time[0]=(READ_SA==0) ?     ++key_scan_time[0] : 0;
	 key_scan_time[1]=(READ_SB==0) ?     ++key_scan_time[1] : 0;
	 key_scan_time[2]=(READ_SKEY==0)   ? ++key_scan_time[2] : 0;

	 if(key_scan_time[0]==5)
	 { 
	    key_input_state=KEY_DOWN;//ÏÂ
	 }
	 if(key_scan_time[1]==5)
	 {
	    key_input_state=KEY_UP;//ÉÏ
	 }
	 if(key_scan_time[2]==5)
	 {
	    key_input_state=KEY_PRESS;//°´
	 }
}

