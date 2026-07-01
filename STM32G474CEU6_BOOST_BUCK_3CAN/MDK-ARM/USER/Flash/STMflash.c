#include "STMflash.h"
//flash 数据循环写，写满在擦除

FLASH_D FLASH_USER;             //存放flash数据结构体
uint16_t MAX_PACK;              //存放数据的最大份数
uint32_t write_pack_addr;
uint32_t read_pack_addr;

static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + 0x0040000))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + 0x0040000)) / FLASH_PAGE_SIZE;
  }

  return page;
}
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PageError = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint32_t data32 = 0;
	  static FLASH_EraseInitTypeDef EraseInitStruct;
//
FLASH_D FALSH_INIT_DATA;

uint8_t test=0;
void FLASH_PACK_READ(uint32_t pack_addr,FLASH_D * DATA)
{
	
	  uint32_t FLASH_DATA[USER_FLASH_DATA_LENTH];//数据包缓冲区
	 
	  uint8_t i=0;//,j=0;
    for(i=0;i<USER_FLASH_DATA_LENTH;i++)
	  {
	     FLASH_DATA[i]=flash_read((pack_addr*USER_FLASH_DATA_LENTH+i));
	  }
		for(i=0;i<USER_FLASH_DATA_LENTH;i++)
	  {
	     *(((uint32_t*)DATA)+i)=FLASH_DATA[i]; 
	  }	
}
uint64_t FLASH_64DATA[USER_FLASH_DATA_LENTH/2];//数据包缓冲区
void FLASH_PACK_WRITE(uint32_t pack_addr,FLASH_D * DATA)
{
	
	
	  //uint64_t FLASH_DATA[USER_FLASH_DATA_LENTH/2];//数据包缓冲区
	  uint8_t i=0;//j=0;
		for(i=0;i<USER_FLASH_DATA_LENTH/2;i++)
	  {
	      FLASH_64DATA[i]=*(((uint64_t*)DATA)+i); 
	  }
	  HAL_FLASH_Unlock();	
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
    //                    FLASH_FLAG_PGAERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_PGSERR);//清除一些错误标志
		i=0;
		while(i<USER_FLASH_DATA_LENTH/2)
		{
			  Address=FLASH_USER_START_ADDR+pack_addr*USER_FLASH_DATA_LENTH*4+i*8;
		    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,Address,FLASH_64DATA[i])== HAL_OK)
        {
           i++;
        }
				else
				{
				  HAL_Delay(10);
					//HAL_FLASH_Lock();
					//HAL_FLASH_Unlock();
				}
		}
		HAL_FLASH_Lock(); 	
}
void FLASH_RESTART()
{
    FLASH_PACK_READ(write_pack_addr-1,&FALSH_INIT_DATA);
	
	  HAL_FLASH_Unlock();

	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	
	
    //  FLASH_Erase_Sector(FLASH_SECTOR_11,FLASH_VOLTAGE_RANGE_3);//擦除指定的闪存扇区(0~7)}; 				
	  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    /* Get the 1st page to erase */
    FirstPage = GetPage(FLASH_USER_START_ADDR);

    /* Get the number of pages to erase from 1st page */
    NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;

    /* Get the bank */
    BankNumber = GetBank(FLASH_USER_START_ADDR);

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = BankNumber;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = NbOfPages;

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
    while (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
       //Error_Handler();
    }
   
		HAL_FLASH_Lock(); 
	
	  FLASH_PACK_WRITE(0,&FALSH_INIT_DATA);
	  write_pack_addr=1;
}

void FLASH_RW_INIT(void)
{
   //寻找读写数据的存储块
   MAX_PACK=(2048*FLASH_USER_ADDR_NUM/(USER_FLASH_DATA_LENTH*4))-1;
   uint32_t i=0;
	 for(i=0;i<MAX_PACK;i++)
	 { 
	    FLASH_PACK_READ(i,&FALSH_INIT_DATA);
	    write_pack_addr=i;
	     if(FALSH_INIT_DATA.FLASH_WORK!=0X11223344)
			 {
				 
			   write_pack_addr=i;
				 if(write_pack_addr==0) //flash 里面没有初始化数据
				 {
				    FLASH_D FRIST_DATA;
				    FRIST_DATA.FLASH_WORK=0X11223344;
					  FLASH_PACK_WRITE(0,&FRIST_DATA);
					  write_pack_addr=1;
				 }
				 break;
			 }
		}
	 if(write_pack_addr>=(MAX_PACK*0.8f))
	 {
	    //初始化时flash空间使用过多
	    FLASH_RESTART();
	 }
	 
}

uint32_t flash_read(uint32_t addr)
{
	return	*(__IO uint32_t*)( FLASH_USER_START_ADDR+(addr)*4 );
}


void WRITE_FLASH_DATA(FLASH_D * DATA)
{
	  if(write_pack_addr>=MAX_PACK)
		{
		   FLASH_RESTART();
		}
    FLASH_PACK_WRITE(write_pack_addr,DATA);
    write_pack_addr++;
		return ;
}
void READ_FLASH_DATA(FLASH_D * DATA)
{
   FLASH_PACK_READ(write_pack_addr-1,DATA);
	 return ;
}










