#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "LCD.h"

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#define  u32  uint32_t
#define  u16  uint16_t
#define  u8   uint8_t

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif

#define  u32  uint32_t
#define  u16  uint16_t
#define  u8   uint8_t


//-----------------LCD端口定义---------------- 
//	    HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
//#define LCD_SCLK_Clr() //GPIO_ResetBits(GPIOA,GPIO_Pin_0)//SCL=SCLK
//#define LCD_SCLK_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_0)

//#define LCD_MOSI_Clr() //GPIO_ResetBits(GPIOA,GPIO_Pin_1)//SDA=MOSI
//#define LCD_MOSI_Set() //GPIO_SetBits(GPIOA,GPIO_Pin_1)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_RESET)//GPIO_ResetBits(GPIOA,GPIO_Pin_2)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(BLK_GPIO_Port,BLK_Pin,GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()   HAL_GPIO_WritePin(BLK_GPIO_Port,BLK_Pin,GPIO_PIN_SET)




void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
void LCD_INIT(void);
#endif




