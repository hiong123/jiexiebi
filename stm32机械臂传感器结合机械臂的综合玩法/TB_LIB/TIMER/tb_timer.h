#ifndef __TB_TIMER_H__
#define __TB_TIMER_H__

#include "stm32f10x.h"

void SysTick_Int_Init(void);
void TIM2_Int_Init(u16 arr,u16 psc);


void TIM3_Pwm_Init(u16 arr,u16 psc);
void TIM4_Pwm_Init(u16 arr,u16 psc);
void duoji_inc_handle(u8 index);

#endif
