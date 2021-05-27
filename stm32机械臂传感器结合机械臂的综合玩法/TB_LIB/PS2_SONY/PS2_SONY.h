#ifndef __PS2_SONY_H__
#define __PS2_SONY_H__

#include "stm32f10x_conf.h"

#define START_CMD			0x01
#define ASK_DAT_CMD			0x42

#define PS2_DAT	    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_14)
#define PS2_CMD(x) {x? GPIO_SetBits(GPIOA,1 << 13): GPIO_ResetBits(GPIOA,1 << 13);}
#define PS2_ATT(x) {x? GPIO_SetBits(GPIOA,1 << 12): GPIO_ResetBits(GPIOA,1 << 12);}
#define PS2_CLK(x) {x? GPIO_SetBits(GPIOA,1 << 11): GPIO_ResetBits(GPIOA,1 << 11);}

//#define PS2_ACK(x) if(x) GPIO_SetBits(GPIOC,1 << 15);else GPIO_ResetBits(GPIOC,1 << 15);

void PSX_init(void);
void psx_io_config(void);
unsigned char psx_transfer(unsigned char dat);
void psx_write_read(unsigned char *get_buf);

#endif
