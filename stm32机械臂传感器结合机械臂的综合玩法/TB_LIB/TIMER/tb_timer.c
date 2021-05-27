#include "tb_timer.h"
#include "tb_gpio.h"
#include "stm32f10x_conf.h"
#include "tb_global.h"
#include "tb_usart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tb_delay.h"

// systick register
#define SYSTICK_TENMS    (*((volatile unsigned long *)0xE000E01C))  
#define SYSTICK_CURRENT  (*((volatile unsigned long *)0xE000E018))  
#define SYSTICK_RELOAD   (*((volatile unsigned long *)0xE000E014))  
#define SYSTICK_CSR       (*((volatile unsigned long *)0xE000E010)) 

//初始化函数
float _abs(float temp) {
	if(temp>0)return temp;
	else {
		return (-temp);
	}
}

void SysTick_Int_Init(void) {
	SYSTICK_CURRENT = 0;
	SYSTICK_RELOAD = 72000;
	SYSTICK_CSR|=0x07;
}
void SysTick_Handler(void) {   
	SYSTICK_CURRENT=0;  
	systick_ms++;
}


void TIM2_Int_Init(u16 arr,u16 psc) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //①时钟 TIM2 使能	
	//定时器 TIM2 初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  //②初始化 TIM2
	TIM_ARRPreloadConfig(TIM2, DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );  //③允许更新中断
	
	//中断优先级 NVIC 设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2 中断
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级 0 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级 2 级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure);  //④初始化 NVIC 寄存器
	TIM_Cmd(TIM2, ENABLE);  //⑤使能 TIM2
}


void TIM3_Pwm_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   //对应CH3通道PB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	TIM_TimeBaseStructure.TIM_Period = arr;  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
	TIM_OCInitStructure.TIM_Pulse=500; 
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);        
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 

	TIM_Cmd(TIM3, ENABLE);  

}

void TIM4_Pwm_Init(u16 arr,u16 psc) {  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;   //对应CH3通道PB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	TIM_TimeBaseStructure.TIM_Period = arr;  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
	TIM_OCInitStructure.TIM_Pulse=500; 
	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM4, &TIM_OCInitStructure);        
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 

	TIM_Cmd(TIM4, ENABLE);  
}

float abs_float(float value) {
	if(value>0) {
		return value;
	}
	return (-value);
}

void duoji_inc_handle(u8 index) {	
	int aim_temp;
	
	if(duoji_doing[index].inc != 0) {
		
		aim_temp = duoji_doing[index].aim;
		
		if(aim_temp > 2500){
			aim_temp = 2500;
		} else if(aim_temp < 500) {
			aim_temp = 500;
		}
	
		if(abs_float(aim_temp - duoji_doing[index].cur) <= abs_float(duoji_doing[index].inc + duoji_doing[index].inc)) {
			duoji_doing[index].cur = aim_temp;
			duoji_doing[index].inc = 0;
		} else {
			duoji_doing[index].cur += duoji_doing[index].inc;
		}
	}
}

void TIM2_IRQHandler(void) {
	static u8 flag = 0;
	static u8 duoji_index1 = 0;
	int temp;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查 TIM2 更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //清除 TIM2 更新中断标志
		
		if(duoji_index1 == 8) {
			duoji_index1 = 0;
		}
		
		if(!flag) {
			TIM2->ARR = ((unsigned int)(duoji_doing[duoji_index1].cur));
			dj_io_set(duoji_index1, 1);
			duoji_inc_handle(duoji_index1);
		} else {
			temp = 2500 - (unsigned int)(duoji_doing[duoji_index1].cur);
			if(temp < 20)temp = 20;
			TIM2->ARR = temp;
			dj_io_set(duoji_index1, 0);
			duoji_index1 ++;
		}
		flag = !flag;
	}
} 
