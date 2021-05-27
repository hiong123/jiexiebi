#ifndef __W25Q64_H__
#define __W25Q64_H__

#include "tb_type.h"
#include "stm32f10x.h"
#include "tb_gpio.h"

//W25X系列/Q系列芯片列表	   
//W25Q80 ID  0XEF13
//W25Q16 ID  0XEF14
//W25Q32 ID  0XEF15
//W25Q32 ID  0XEF16	

#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16

#define W25Q64_SECTOR_SIZE	0x1000		//4K
#define W25Q64_SECTOR_NUM	2048		//8*1024/4 = 2048


extern u16 W25Q_TYPE;//定义我们使用的flash芯片型号

#define	W25Q_CS PBout(12)  //选中FLASH	
				 
////////////////////////////////////////////////////////////////////////////
 
//指令表
#define W25X_WriteEnable			0x06 		//写使能
#define W25X_WriteDisable			0x04 		//写失能
#define W25X_ReadStatusReg  	0x05 		//读状态寄存器
#define W25X_WriteStatusReg		0x01 		//写状态寄存器
#define W25X_ReadData					0x03 		//读数据
#define W25X_FastReadData			0x0B 		//
#define W25X_FastReadDual			0x3B 
#define W25X_PageProgram			0x02 
#define W25X_BlockErase				0xD8 
#define W25X_SectorErase			0x20 
#define W25X_ChipErase				0xC7 		
#define W25X_PowerDown				0xB9 		//掉电
#define W25X_ReleasePowerDown	0xAB 		//唤醒
#define W25X_DeviceID					0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

void W25Q_Init(void);
void W25Q_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //读取flash
void W25Q_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//写入flash


u16  W25Q_ReadID(void);  	    //读取FLASH ID
u8	 W25Q_ReadSR(void);        //读取状态寄存器 
void W25Q_Write_SR(u8 sr);  	//写状态寄存器
void W25Q_Write_Enable(void);  //写使能 
void W25Q_Write_Disable(void);	//写保护
void W25Q_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void W25Q_Erase_Chip(void);    	  //整片擦除
void W25Q_Erase_Sector(u32 Dst_Addr);//扇区擦除
void W25Q_Wait_Busy(void);           //等待空闲
void W25Q_PowerDown(void);           //进入掉电模式
void W25Q_WAKEUP(void);			  //唤醒


void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节

#define w25x_init() W25Q_Init()
#define w25x_readId()	W25Q_ReadID()
#define w25x_read(buf, addr, len) W25Q_Read(buf, addr, len)
#define w25x_write(buf, addr, len) W25Q_Write(buf, addr, len)
#define w25x_erase_sector(addr) W25Q_Erase_Sector(addr)

#endif
