#include "STMflash.h"
#include "main.h"
uint32_t aa;
FLASH_D FLASH_READ_DATA;
FLASH_D FLASH_WRITE_DATA;

//工作模式设置 
//1外部超电模式，使用can通信获得当前功率限制，发出电容容量，不控制底盘
//0全控底盘模式，底盘控制，电容功率，UI，裁判系统接收全部由超电完成

void (*GET_CAP_CTRL)(void); //电容缓冲能量，控制功率，数据获取函数
void (*CHASSIS_CTRL)(void); //底盘控制函数
void null_void(void)
{

}

void INIT_FLASH_DATA()
{
	 //FLASH_RESTART();
	 GET_CAP_CTRL=null_void;
	 CHASSIS_CTRL=null_void;
	 FLASH_RW_INIT();
	 READ_FLASH_DATA(&FLASH_USER);
	 
	 switch(FLASH_USER.MODE_SET&WORK_MODE_STATE)
	 {
	    
			case 0://全部控制底盘模式
			{
//			  CHASSIS_CTRL=chassic_control_task;
			  CHASSIS_CTRL=chassic_control_task;
			  break;
			}
			case 1:
			{
			
			  break;
				
			}
			default :break;
	 
	 }
}


