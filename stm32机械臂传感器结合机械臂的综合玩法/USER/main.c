/***************************************************************
	@笔者：tacbo
	@日期：2017年11月08日
	@所属：杭州众灵科技有限公司
	@功能：实现舵机控制器功能
	功能列表：
	1、单个舵机控制（支持PWM舵机和总线舵机）
	2、多个舵机控制（支持PWM舵机和总线舵机）
	3、手柄控制舵机
	4、串口控制舵机
	5、OLED显示舵机的执行情况
	6、USB一键下载
	7、可支持总线MP3 总线WIFI等设备
	8、上位机图形化编程
	9、控制6自由度机械臂			
***************************************************************/


#include "tb_rcc.h"			//配置时钟文件
#include "tb_gpio.h"		//配置IO口文件
#include "tb_global.h"	//存放全局变量
#include "tb_delay.h"		//存放延时函数
#include "tb_type.h"		//存放类型定义
#include "tb_usart.h"		//存放串口功能文件
#include "tb_timer.h"		//存放定时器功能文件
				
#include "ADC.h"			//存放ADC的
#include "PS2_SONY.h"		//存放索尼手柄
#include "w25q64.h"			//存储芯片的操作
#include "oled_i2c.h"		//OLED文件

#include <stdio.h>			//标准库文件
#include <string.h>			//标准库文件
#include <math.h>			//标准库文件

#define VERSION				20170919	//版本定义
#define CYCLE 				1000		//PWM模块周期
#define PS2_LED_RED  		0x73		//PS2手柄红灯模式
#define PS2_LED_GRN  		0x41		//PS2手柄绿灯模式
#define PSX_BUTTON_NUM 		16			//手柄按键数目
#define PS2_MAX_LEN 		64			//手柄命令最大字节数
#define FLAG_VERIFY 		0x25		//校验标志
#define ACTION_SIZE 		0x80		//一个动作的存储大小

#define W25Q64_INFO_ADDR_SAVE_STR			(((8<<10)-2)<<10)//(8*1024-1)*1024		//eeprom_info结构体存储的位置

void system_init(void);					//系统初始化
void beep_led_dis_init(void);			//开机提示
void handle_nled(void);					//LED工作指示灯提示
void soft_reset(void);					//软件复位

void car_pwm_set(int car_left, int car_right);	//电机控制函数
void handle_ps2(void);							//手柄数据解析
void handle_button(void);						//手柄按键解析
void parse_psx_buf(unsigned char *buf, unsigned char mode);	//手柄按键解析子函数
void handle_car(void);										//摇杆数据解析控制车

void handle_uart(void);					//串口解析
void parse_cmd(u8 *cmd);				//命令解析

void action_save(u8 *str);				//动作保存函数
int get_action_index(u8 *str);			//获取动作组序号
void print_group(int start, int end);	//打印动作组
void int_exchange(int *int1, int *int2);	//两个int互换

void do_group_once(int group_num); 		//执行动作组1次
void handle_action(void);				//处理动作组执行
u8 check_dj_state(void);				//获取舵机的状态

void do_action(u8 *uart_receive_buf);			//执行动作
void replace_char(u8*str, u8 ch1, u8 ch2);		//字符串字母替换
void rewrite_eeprom(void);						//写入eeprom_info结构体
void handle_adc(void);							//处理ADC数据
void handle_oled(void);							//处理OLED数据
void oled_dis_init(void);							//OLED显示初始化
void handle_sensor(void);					//处理传感器		


u8 psx_buf[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 	//存储手柄的数据
const char *pre_cmd_set_red[PSX_BUTTON_NUM] = {					//红灯模式下按键的配置
	"<PS2_RED01:#005P0600T2000!^$DST:5!>",	//L2						  
	"<PS2_RED02:#005P2400T2000!^$DST:5!>",	//R2						  
	"<PS2_RED03:#004P0600T2000!^$DST:4!>",	//L1						  
	"<PS2_RED04:#004P2400T2000!^$DST:4!>",	//R1			
	"<PS2_RED05:#002P2400T2000!^$DST:2!>",	//RU						  
	"<PS2_RED06:#003P2400T2000!^$DST:3!>",	//RR						  
	"<PS2_RED07:#002P0600T2000!^$DST:2!>",	//RD						  
	"<PS2_RED08:#003P0600T2000!^$DST:3!>",	//RL				
	"<PS2_RED09:$DJ_RECORD_DO:1!>",			//SE    执行1次					  
	"<PS2_RED10:$DJ_RECORD_CLEAR!>",		//AL	清除					  
	"<PS2_RED11:$DJ_RECORD!>",				//AR	学习					  
	"<PS2_RED12:$DJR!>",					//ST			
	"<PS2_RED13:#001P0600T2000!^$DST:1!>",	//LU						  
	"<PS2_RED14:#000P0600T2000!^$DST:0!>",	//LR								  
	"<PS2_RED15:#001P2400T2000!^$DST:1!>",	//LD						  
	"<PS2_RED16:#000P2400T2000!^$DST:0!>",	//LL						
};

const char *pre_cmd_set_grn[PSX_BUTTON_NUM] = {					//绿灯模式下按键的配置
	"<PS2_GRN01:#005P0600T2000!^$DST:5!>",	//L2						  
	"<PS2_GRN02:#005P2400T2000!^$DST:5!>",	//R2						  
	"<PS2_GRN03:#004P0600T2000!^$DST:4!>",	//L1						  
	"<PS2_GRN04:#004P2400T2000!^$DST:4!>",	//R1			
	"<PS2_GRN05:#002P2400T2000!^$DST:2!>",	//RU						  
	"<PS2_GRN06:#003P2400T2000!^$DST:3!>",	//RR						  
	"<PS2_GRN07:#002P0600T2000!^$DST:2!>",	//RD						  
	"<PS2_GRN08:#003P0600T2000!^$DST:3!>",	//RL				
	"<PS2_GRN09:$!>",			//SE    执行1次					  
	"<PS2_GRN10:$!>",		//AL	清除					  
	"<PS2_GRN11:$!>",				//AR	学习					  
	"<PS2_GRN12:$DJR!>",					//ST			
	"<PS2_GRN13:#001P0600T2000!^$DST:1!>",	//LU						  
	"<PS2_GRN14:#000P0600T2000!^$DST:0!>",	//LR								  
	"<PS2_GRN15:#001P2400T2000!^$DST:1!>",	//LD						  
	"<PS2_GRN16:#000P2400T2000!^$DST:0!>",	//LL					  
};


/*"D:\DreamSpark\OLED\MP3_UI.bmp",0 图片取模数据
unsigned char MY_PIC[] = {
	0x00,0x03,0x05,0x09,0x11,0xFF,0x11,0x89,0x05,0xC3,0x00,0xE0,0x00,0xF0,0x00,0xF8,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0xFF,0x11,0xAA,0x44,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0x01,0x38,0x44,0x82,0x92,
	0x92,0x74,0x01,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C,0x44,0xC7,0x01,0x7D,
	0x7D,0x7D,0x7D,0x01,0x7D,0x7D,0x7D,0x7D,0x01,0x7D,0x7D,0x7D,0x7D,0x01,0xFF,0x00,
	0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,
	0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x00,0x00,
	0x6D,0x6D,0x6D,0x6D,0x6D,0x00,0x00,0x60,0x60,0x60,0x60,0x60,0x00,0x00,0x40,0x40,
	0x40,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0xDB,0xDB,0xDB,0xDB,0x00,0x00,
	0xDB,0xDB,0xDB,0xDB,0xDB,0x00,0x00,0xDB,0xDB,0xDB,0xDB,0xDB,0x00,0x00,0xDB,0xDB,
	0xDB,0xDB,0xDB,0x00,0x00,0xDA,0xDA,0xDA,0xDA,0xDA,0x00,0x00,0xD8,0xD8,0xD8,0xD8,
	0xD8,0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,
	0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x80,
	0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x00,0x00,
	0x06,0x06,0x06,0x06,0x06,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x00,0x00,0x06,0x06,
	0x06,0x06,0x06,0x00,0x00,0x06,0x06,0x06,0xE6,0x66,0x20,0x00,0x06,0x06,0x86,0x06,
	0x06,0x00,0x00,0x06,0x06,0x06,0x06,0x86,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x00,
	0x00,0x86,0x86,0x86,0x86,0x86,0x80,0x80,0x86,0x86,0x06,0x86,0x86,0xC0,0xC0,0x86,
	0x86,0x86,0x06,0x06,0xD0,0x30,0x76,0x06,0x06,0x06,0x06,0x00,0x00,0x06,0x06,0x06,
	0x06,0x06,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x00,0x00,0x06,0x06,0x06,0x06,0x06,
	0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x1C,0x00,0xFE,0x00,0x01,
	0x02,0x00,0xC4,0x18,0x20,0x02,0x9E,0x63,0xB2,0x0E,0x00,0xFF,0x81,0x81,0xFF,0x00,
	0x00,0x80,0x40,0x30,0x0F,0x00,0x00,0x00,0x00,0xFF,0x00,0x23,0xEA,0xAA,0xBF,0xAA,
	0xEA,0x03,0x3F,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x0C,0x08,0x00,0x00,0x01,0x01,0x01,
	0x01,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x81,0x80,0x80,0x81,0x80,
	0x81,0x80,0x80,0x80,0x80,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
	0x01,0x00,0x01,0x01,0x09,0x0C,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
	0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,
	0x00,0x1E,0x21,0x40,0x40,0x50,0x21,0x5E,0x00,0x1E,0x21,0x40,0x40,0x50,0x21,0x5E,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xC1,0xC1,0xFF,
	0xFF,0xC1,0xC1,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x80,0xFC,0xF3,0xEF,0xF3,0xFC,
	0x80,0xFF,0x80,0xEE,0xEE,0xEE,0xF5,0xFB,0xFF,0x9C,0xBE,0xB6,0xB6,0x88,0xFF,0x00,
};
*/

u8 dbt_flag = 0;
const char *action_pre[] = {
"{G0000#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0001#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0002#000P1500T1000!#001P1705T1000!#002P1859T1000!#003P1936T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0003#000P1500T1000!#001P1705T1000!#002P1859T1000!#003P1936T1000!#004P2000T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0004#000P1500T0500!#001P1705T0500!#002P1859T0500!#003P1936T0500!#004P1000T0500!#005P1500T0500!#006P1500T1000!#007P1500T1000!}",
"{G0005#000P1500T0500!#001P1705T0500!#002P1859T0500!#003P1936T0500!#004P2000T0500!#005P1500T0500!#006P1500T1000!#007P1500T1000!}",
"{G0006#000P1500T0500!#001P1705T0500!#002P1859T0500!#003P1936T0500!#004P1000T0500!#005P1500T0500!#006P1500T1000!#007P1500T1000!}",
"{G0007#000P1500T0500!#001P1705T0500!#002P1859T0500!#003P1936T0500!#004P2000T0500!#005P1500T0500!#006P1500T1000!#007P1500T1000!}",
"{G0008#000P1500T0500!#001P1705T0500!#002P1859T0500!#003P1936T0500!#004P1000T0500!#005P1500T0500!#006P1500T1000!#007P1500T1000!}",
"{G0009#000P1500T0500!#001P1705T0500!#002P1859T0500!#003P1936T0500!#004P2000T0500!#005P1500T0500!#006P1500T1000!#007P1500T1000!}",
"{G0010#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0011#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0012#000P1500T1000!#001P1346T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0013#000P1500T1000!#001P1346T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0014#000P1500T1000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0015#000P2200T1000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0016#000P2200T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0017#000P2200T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0018#000P2200T1000!#001P1800T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0019#000P2200T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0020#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0021#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0022#000P1500T1000!#001P1346T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0023#000P1500T1000!#001P1346T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0024#000P1500T1000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0025#000P0800T1000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0026#000P0800T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0027#000P0800T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0028#000P0800T1000!#001P1800T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0029#000P0800T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0030#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0031#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0032#000P0800T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
"{G0033#000P0800T1000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0034#000P0800T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0035#000P0800T1000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0036#000P2200T2000!#001P2000T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0037#000P2200T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P1000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0038#000P2200T1000!#001P1300T1000!#002P2013T1000!#003P2115T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0039#000P2200T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P2000T1000!#006P1500T1000!#007P1500T1000!}",
"{G0040#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!#006P1500T1000!#007P1500T1000!}",
};




int i;						//常用的一个临时变量
u8 car_dw = 1;				//摇杆档位控制
u8 group_do_ok = 1;			//动作执行完成标志
int do_start_index;			//动作组执行 起始序号
int do_time;				//动作组执行 执行次数
int group_num_start;		//动作组执行 起始序号
int group_num_end;			//动作组执行 终止序号
int group_num_times;		//动作组执行 起始变量
u32 dj_record_time = 1000;	//学习时间默认1000
u8 oled_init_fail = 0;		//OLED初始化成功是否标志
float vol_adc;					//电池电压

/*-------------------------------------------------------------------------------------------------------
*  程序从这里执行				
*  这个启动代码 完成时钟配置 使用外部晶振作为STM32的运行时钟 并倍频到72M最快的执行速率
-------------------------------------------------------------------------------------------------------*/

int main(void) {	
	tb_rcc_init();		//时钟初始化
	tb_gpio_init();		//IO初始化
	tb_global_init();	//全局变量初始化
	nled_init();		//工作指示灯初始化
	beep_init();		//蜂鸣器初始化
	beep_off();			//关闭蜂鸣器
	dj_io_init();		//舵机IO口初始化
	sensor_init();		//传感器初始化
		
	//w25q64 init
	W25Q_Init();				//动作组存储芯片初始化
	if(W25Q_TYPE != W25Q64){	//判断是否是W25Q64芯片
		while(1)beep_on();		//如果不是则长鸣，说明芯片有问题，无法通信
	}
	W25Q_Read((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info));	//读取全局变量
	if(eeprom_info.version != VERSION) {	//判断版本是否是当前版本
		eeprom_info.version = VERSION;		//复制当前版本
		eeprom_info.dj_record_num = 0;		//学习动作组变量赋值0
		rewrite_eeprom();					//写入到存储器
	}

		
	ADC_init();	//ADC初始化
	PSX_init();	//手柄初始化
	TIM2_Int_Init(20000, 71);	//舵机 定时器初始化
	
	//小车 pwm 初始化
	TIM3_Pwm_Init(1000, 239);
	TIM4_Pwm_Init(1000, 239);
	car_pwm_set(0,0);	//设置小车的左右轮速度为0

	
	//串口1初始化
	tb_usart1_init(115200);
	uart1_open();
	
	//串口2初始化
	tb_usart2_init(115200);
	uart2_open();
	
	//串口3初始化
	tb_usart3_init(115200);
	uart3_open();
	
	//总中断打开
	tb_interrupt_open();
	
	
	//三个串口发送测试字符
	uart1_send_str((u8 *)"uart1 check ok!");
	uart2_send_str((u8 *)"uart2 check ok!");
	uart3_send_str((u8 *)"uart3 check ok!");
	
	//总线输出 复位总线舵机
	zx_uart_send_str((u8 *)"#255P1500T2000!");
	
	//系统滴答时钟初始化	
	SysTick_Int_Init();
	
	//OLED初始化 0初始化成功 1初始化失败 handle_oled中会用到
	oled_init_fail = OLED_Init();
		
	if(!oled_init_fail) {
		OLED_CLS();
		mdelay(500);
		oled_dis_init();
		mdelay(3000);
		OLED_CLS();
	}
	
	//蜂鸣器LED 名叫闪烁 示意系统启动
	beep_led_dis_init();
			
	//执行预存命令
	if(eeprom_info.pre_cmd[PRE_CMD_SIZE] == FLAG_VERIFY) {
		if(eeprom_info.pre_cmd[0] == '$') {
			memset(uart_receive_buf, 0, sizeof(uart_receive_buf));	
			strcpy((char *)uart_receive_buf, (char *)eeprom_info.pre_cmd);
			uart1_mode = 1;
			uart1_get_ok = 1;
			uart1_send_str(uart_receive_buf);
		}
	}
		

	
	while(1) {			
		handle_nled();		//处理信号灯
		handle_ps2();		//处理手柄
		handle_button();	//处理手柄按钮
		handle_car();		//处理摇杆控制小车
		handle_uart();		//处理串口接收数据
		handle_action();	//处理动作组
		handle_adc();		//处理ADC
		handle_oled();		//处理OLED显示
		handle_sensor();	//处理传感器
	}
}



//开机指示函数，蜂鸣器和工作指示灯鸣3声作为开机指示
void beep_led_dis_init(void) {
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
}

//工作指示灯处理，间隔1000MS闪烁一次
void handle_nled(void) {
	static u32 time_count=0;
	static u8 flag = 0;
	if(systick_ms-time_count > 1000)  {
		time_count = systick_ms;
		if(flag) {
			nled_on();
		} else {
			nled_off();
		}
		flag= ~flag;
	}
}

//软件复位函数，调用后单片机自动复位
void soft_reset(void) {
	__set_FAULTMASK(1);     
	NVIC_SystemReset();
}

//小车控制函数
//参数 左轮速度和右轮速度 范围 -1000 到 1000
void car_pwm_set(int car_left, int car_right) {
	
	if(car_left >= CYCLE)car_left = CYCLE-1;
	else if(car_left <= -CYCLE)car_left = -CYCLE+1;
	else if(car_left == 0)car_left = 1;
	
	if(car_right >= CYCLE)car_right = CYCLE-1;
	else if(car_right <= -CYCLE)car_right = -CYCLE+1;
	else if(car_right == 0)car_right = 1;
	
	//car_left = car_left/car_dw;
	//car_right = car_right/car_dw;
	car_left = -car_left;
	car_right = -car_right;
	
	if(car_right>0) {
		TIM_SetCompare4(TIM4,1);
		TIM_SetCompare3(TIM4,car_right);
	} else {
		TIM_SetCompare4(TIM4,-car_right);
		TIM_SetCompare3(TIM4,1);
		
	}

	if(car_left>0) {
		TIM_SetCompare4(TIM3,1);
		TIM_SetCompare3(TIM3,car_left);
	} else {
		TIM_SetCompare4(TIM3,-car_left);
		TIM_SetCompare3(TIM3,1);
		
	}	

//	//总线马达设置	
//	sprintf((char *)cmd_return, "#0233P%dT0!#034P%dT0!", 
//	(int)(1500+car_left), (int)(1500+car_right));
//	zx_uart_send_str(cmd_return);
		
	return;
}


//处理手柄
void handle_ps2(void) {
	static u32 systick_ms_bak = 0;
	//每20ms处理1次
	if(systick_ms - systick_ms_bak < 20) {
		return;
	}
	systick_ms_bak = systick_ms;
	//读写手柄数据
	psx_write_read(psx_buf);
	
#if 0
		//测试手柄数据，1为打开 0为关闭
	sprintf((char *)cmd_return, "0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\r\n", 
	(int)psx_buf[0], (int)psx_buf[1], (int)psx_buf[2], (int)psx_buf[3],
	(int)psx_buf[4], (int)psx_buf[5], (int)psx_buf[6], (int)psx_buf[7], (int)psx_buf[8]);
	uart1_send_str(cmd_return);
#endif 	
	
	return;
}

//处理手柄按键
void handle_button(void) {
	static unsigned char psx_button_bak[2] = {0};
	//对比两次获取的按键值是否相同 ，相同就不处理，不相同则处理
	if((psx_button_bak[0] == psx_buf[3])
	&& (psx_button_bak[1] == psx_buf[4])) {				
	} else {
		//处理buf3和buf4两个字节，这两个字节存储这手柄16个按键的状态
		parse_psx_buf(psx_buf+3, psx_buf[1]);
		psx_button_bak[0] = psx_buf[3];
		psx_button_bak[1] = psx_buf[4];
	}
	return;
}


//处理手柄按键字符，buf为字符数组，mode是指模式 主要是红灯和绿灯模式
void parse_psx_buf(unsigned char *buf, unsigned char mode) {
	u8 i, pos = 0;
	static u16 bak=0xffff, temp, temp2;
	temp = (buf[0]<<8) + buf[1];
	
	if(bak != temp) {
		temp2 = temp;
		temp &= bak;
		for(i=0;i<16;i++) {//16个按键一次轮询
			if((1<<i) & temp) {
			} else {
				if((1<<i) & bak) {	//press 表示按键按下了
															
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
					} else if(mode == PS2_LED_GRN) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));
					} else continue;
					
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) uart_receive_buf[pos-1] = '\0';
					if(str_contain_str(uart_receive_buf, (u8 *)"$")) {
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf, (char *)cmd_return);
						uart1_get_ok = 1;
						uart1_open();
						uart1_mode = 1;
					} else if(str_contain_str(uart_receive_buf, (u8 *)"#")) {
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf,(char *) cmd_return);
						uart1_get_ok = 1;
						uart1_open();
						uart1_mode = 2;
					}
					
					//uart1_send_str(uart_receive_buf);
					bak = 0xffff;
				} else {//release 表示按键松开了
										
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
					} else if(mode == PS2_LED_GRN) {
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));
					} else continue;	
					
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) {
						if(str_contain_str(uart_receive_buf+pos, (u8 *)"$")) {
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							parse_cmd(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 1;
						} else if(str_contain_str(uart_receive_buf+pos, (u8 *)"#")) {
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							do_action(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 2;
						}
						//uart1_send_str(uart_receive_buf);
					}	
				}

			}
		}
		bak = temp2;
		beep_on();mdelay(10);beep_off();
	}	
	return;
}

//int型 取绝对值函数
int abs_int(int int1) {
	if(int1 > 0)return int1;
	return (-int1);
}

//处理小车函数 主要处理摇杆的数据 这里 8为左摇杆 6为右摇杆
void handle_car(void) {
	static int car_left, car_right, car_left_bak, car_right_bak;
	
	if(psx_buf[1] != PS2_LED_RED)return;
	
	if(abs_int(127 - psx_buf[8]) < 5 )psx_buf[8] = 127;
	if(abs_int(127 - psx_buf[6]) < 5 )psx_buf[6] = 127;
	
	car_left = (127 - psx_buf[8]) * 8;
	car_right = (127 - psx_buf[6]) * 8;
	
//	if(abs_int(car_left_bak-car_left) < 20 && abs_int(car_right_bak-car_right) < 20)return;
//	if(abs_int(car_left_bak-car_left) > 40)car_left_bak = car_left;
//	if(abs_int(car_right_bak-car_right) > 40)car_right_bak = car_right;
	
	if(car_left != car_left_bak || car_right != car_right_bak) {
		car_pwm_set(car_left, car_right);
		car_left_bak = car_left;
		car_right_bak = car_right;
	}
}

//处理串口接收到的数据
void handle_uart(void) {
	if(uart1_get_ok) {
		if(uart1_mode == 1) {					//命令模式
			//uart1_send_str("cmd:");
			//uart1_send_str(uart_receive_buf);
			parse_cmd(uart_receive_buf);			
		} else if(uart1_mode == 2) {			//单个舵机调试
			//uart1_send_str("sig:");
			//uart1_send_str(uart_receive_buf);
			do_action(uart_receive_buf);
		} else if(uart1_mode == 3) {		//多路舵机调试
			//uart1_send_str("group:");
			//uart1_send_str(uart_receive_buf);
			do_action(uart_receive_buf);
		} else if(uart1_mode == 4) {		//存储模式
			//uart1_send_str("save:");
			//uart1_send_str(uart_receive_buf);
			action_save(uart_receive_buf);
		} 
		uart1_mode = 0;
		uart1_get_ok = 0;
		uart1_open();
	}

	return;
}


/*
	所有舵机停止命令：$DST!
	第x个舵机停止命令：$DST:x!
	单片机重启命令：$RST!
	检查动作组x到y组命令：$CGP:x-y!
	执行第x个动作：$DGS:x!
	执行第x到y组动作z次：$DGT:x-y,z!
	小车左轮x速度，右轮y速度：$DCR:x,y!
	所有舵机复位命令：$DJR!
	获取应答信号：$GETA!
*/
//命令解析函数
void parse_cmd(u8 *cmd) {
	int pos, i, index, int1, int2;
	u8 temp_buf[160];
	u32 long1;
	
	//uart1_send_str(cmd);
	
	if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DST!"), pos) {
		group_do_ok  = 1;
		dbt_flag = 0;
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].inc = 0;	
			duoji_doing[i].aim = duoji_doing[i].cur;
		}
		zx_uart_send_str((u8 *)"#255PDST!");
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DST:"), pos) {
		if(sscanf((char *)cmd, "$DST:%d!", &index)) {
			duoji_doing[index].inc = 0;	
			duoji_doing[index].aim = duoji_doing[index].cur;
			sprintf((char *)cmd_return, "#%03dPDST!\r\n", (int)index);
			zx_uart_send_str(cmd_return);
			memset(cmd_return, 0, sizeof(cmd_return));
		}
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$RST!"), pos) {		
		soft_reset();
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CGP:"), pos) {		
		if(sscanf((char *)cmd, "$CGP:%d-%d!", &int1, &int2)) {
			print_group(int1, int2);
		}
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DGS:"), pos) {		
		if(sscanf((char *)cmd, "$DGS:%d!", &int1)) {
			do_group_once(int1);
			group_do_ok = 1;
		}
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DGT:"), pos) {		
		if(sscanf((char *)cmd, "$DGT:%d-%d,%d!", &group_num_start, &group_num_end, &group_num_times)) {		
			if(group_num_start != group_num_end) {
				do_start_index = group_num_start;
				do_time = group_num_times;
				group_do_ok = 0;
			} else {
				do_group_once(group_num_start);
			}
		}
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DCR:"), pos) {		
		if(sscanf((char *)cmd, "$DCR:%d,%d!", &int1, &int2)) {
			car_pwm_set(int1, int2);	
		}
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DWA!"), pos) {		
		car_dw--;
		if(car_dw == 0)car_dw = 1;
		beep_on();mdelay(100);beep_off();
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DWD!"), pos) {		
		car_dw++;
		if(car_dw == 4)car_dw = 3;
		beep_on();mdelay(100);beep_off();
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_FARWARD!"), pos) {		
		car_pwm_set(1000, 1000);
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_BACKWARD!"), pos) {		
		car_pwm_set(-1000, -1000);
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_LEFT!"), pos) {		
		car_pwm_set(1000, -1000);
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_RIGHT!"), pos) {		
		car_pwm_set(-1000, 1000);
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DJR!"), pos) {	
		zx_uart_send_str((u8 *)"#255P1500T2000!\r\n");
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].aim = 1500;
			duoji_doing[i].time = 2000;
			duoji_doing[i].inc = (duoji_doing[i].aim -  duoji_doing[i].cur) / (duoji_doing[i].time/20.000);
		}
	} else if (pos = str_contain_str(uart_receive_buf, (u8*)"$DJ_RECORD!"), pos) {
		sprintf((char *)temp_buf, "<G%04d", eeprom_info.dj_record_num);
		for(i=0;i<6;i++) {
			sprintf((char *)cmd_return, "#%03dP%04dT%04d!", (int)i, (int)duoji_doing[i].cur, dj_record_time);
			strcat((char *)temp_buf, (char *)cmd_return);
		}		
		strcat((char *)temp_buf, "B000!>");
		eeprom_info.dj_record_num ++;
		rewrite_eeprom();
		uart1_send_str(temp_buf);
		action_save(temp_buf);
		uart1_send_str((u8 *)"$DJ_RECORD OK!");
		beep_times(100,1);
	}else if (pos = str_contain_str(uart_receive_buf, (u8*)"$DJ_RECORD_CLEAR!"), pos) {
		eeprom_info.dj_record_num  = 0;;
		rewrite_eeprom();
		//uart1_send_str(temp_buf);
		action_save(temp_buf);
		uart1_send_str((u8 *)"$DJ_RECORD_CLEAR OK!");
		beep_times(100,1);
	} else if (pos = str_contain_str(uart_receive_buf, (u8*)"$DJ_RECORD_DO:"), pos) {
		if(sscanf((char *)uart_receive_buf, "$DJ_RECORD_DO:%u!", &long1)) {
			if(eeprom_info.dj_record_num==0)return;	

			group_num_start = 0;
			group_num_end = eeprom_info.dj_record_num-1;
			group_num_times = long1;			

			if(group_num_start != group_num_end) {
				do_start_index = group_num_start;
			} else {
				do_group_once(group_num_start);
			}
			do_time = group_num_times;
			group_do_ok = 0;	
		}
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$GETA!"), pos) {		
		uart1_send_str((u8 *)"AAA");
	} else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DBT:"), pos) {		
		if(sscanf((char *)uart_receive_buf, "$DBT:%d,%d!", &int1, &int2)) {
			if(int1 == 1) {
				group_num_start = 1;
				group_num_end = 10;
				group_num_times = int2;
			} else if(int1 == 2) {
				group_num_start = 11;
				group_num_end = 20;
				group_num_times = int2;
			} else if(int1 == 3) {
				group_num_start = 21;
				group_num_end = 30;
				group_num_times = int2;
			} else if(int1 == 4) {
				group_num_start = 31;
				group_num_end = 40;
				group_num_times = int2;
			} else {
				group_num_start = 0;
				group_num_end = 0;
			}
			
			if(group_num_start != group_num_end) {
				do_start_index = group_num_start;
				do_time = group_num_times;
				group_do_ok = 0;
				dbt_flag = 1;
			} else {
				do_group_once(group_num_start);
			}
			
		}
	} else {
		
	}
}


//动作组保存函数
//只有用<>包含的字符串才能在此函数中进行解析
void action_save(u8 *str) {
	int action_index = 0;
	//获取动作的组号
	action_index = get_action_index(str);
	
	//预存命令处理
	if(str[1] == '$' && str[2] == '!') {
		eeprom_info.pre_cmd[PRE_CMD_SIZE] = 0;
		rewrite_eeprom();
		uart1_send_str((u8 *)"@CLEAR PRE_CMD OK!");
		return;
	} else if(str[1] == '$') {
		memset(eeprom_info.pre_cmd, 0, sizeof(eeprom_info.pre_cmd));
		strcpy((char *)eeprom_info.pre_cmd, (char *)str+1);
		eeprom_info.pre_cmd[strlen((char *)str) - 2] = '\0';
		eeprom_info.pre_cmd[PRE_CMD_SIZE] = FLAG_VERIFY;
		rewrite_eeprom();
		uart1_send_str((u8 *)"@SET PRE_CMD OK!");
		return;
	}
	//获取动作的组号如果不正确，或是第6个字符不是#则认为字符串错误
	//<G0000#000P1500T1000!>
	if((action_index == -1) || str[6] != '#'){
		uart1_send_str((u8 *)"E");
		return;
	}
	//把尖括号替换成大括号直接存储到存储芯片里面去，则在执行动作组的时候直接拿出来解析就可以了
	replace_char(str, '<', '{');
	replace_char(str, '>', '}');
	w25x_write(str, action_index*ACTION_SIZE, strlen((char *)str) + 1);
	//反馈一个A告诉上位机我已经接收到了
	uart1_send_str((u8 *)"A");
	return;	
}

//获取动作组的组号，字符串中有组号返回组号，否则返回-1
int get_action_index(u8 *str) {
	u16 index = 0;
	//uart_send_str(str);
	while(*str) {
		if(*str == 'G') {
			str++;
			while((*str != '#') && (*str != '$')) {
				index = index*10 + *str-'0';
				str++;	
			}
			return index;
		} else {
			str++;
		}
	}
	return -1;
}


//打印存储在芯片里的动作组，从串口1中发送出来 $CGP:x-y!这个命令调用
void print_group(int start, int end) {
	if(start > end) {
		int_exchange(&start, &end);
	}
	for(;start<=end;start++) {
		memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
		w25x_read(uart_receive_buf, start*ACTION_SIZE, ACTION_SIZE);
		uart1_send_str(uart_receive_buf);
		uart1_send_str((u8 *)"\r\n");
	}
}

//两个int变量交换
void int_exchange(int *int1, int *int2) {
	int int_temp;
	int_temp = *int1;
	*int1 = *int2;
	*int2 = int_temp;
}

//执行动作组1次
//参数是动作组序号
void do_group_once(int group_num) {
	memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
	//从存储芯片中读取第group_num个动作组
	w25x_read(uart_receive_buf, group_num*ACTION_SIZE, ACTION_SIZE);
	if(dbt_flag) {
		strcpy((char *)uart_receive_buf, action_pre[group_num]);
	}
	//把读取出来的动作组传递到do_action执行
	do_action(uart_receive_buf);
	sprintf((char *)cmd_return, "@DoGroup %d OK!\r\n", group_num);
	uart1_send_str(cmd_return);
}


//处理动作组，主要是$DGT的处理
void handle_action(void) {
	//通过判断舵机是否全部执行完毕 并且是执行动作组group_do_ok尚未结束的情况下进入处理
	if((check_dj_state() == 0) && (group_do_ok == 0)) {
		//调用do_start_index个动作
		do_group_once(do_start_index);
		
		if(group_num_start<group_num_end) {
			if(do_start_index == group_num_end) {
				do_start_index = group_num_start;
				if(group_num_times != 0) {
					do_time--;
					if(do_time == 0) {
						group_do_ok = 1;
						uart1_send_str((u8*)"@GroupDone!");
					}
				}
				return;
			}
			do_start_index++;
		} else {
			if(do_start_index == group_num_end) {
				do_start_index = group_num_start;
				if(group_num_times != 0) {
					do_time--;
					if(do_time == 0) {
						group_do_ok = 1;
						uart1_send_str((u8*)"@GroupDone!");
					}
				}
				return;
			}
			do_start_index--;
		}
	}
	
}

//检查舵机状态，把每个舵机的增量变量想加，只要不等于0就说明舵机在执行中
u8 check_dj_state(void) {
	int i;
	float	inc = 0;
	for(i=0;i<DJ_NUM;i++) {
		inc += duoji_doing[i].inc;
		if(inc)return 1;
	}
	return 0;
}

//处理 #000P1500T1000! 类似的字符串
void do_action(u8 *uart_receive_buf) {
	u16 index, pwm, time,i = 0;
	zx_uart_send_str(uart_receive_buf);
	while(uart_receive_buf[i]) {
		if(uart_receive_buf[i] == '#') {
			index = 0;i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != 'P') {
				index = index*10 + uart_receive_buf[i]-'0';i++;
			}
		} else if(uart_receive_buf[i] == 'P') {
			pwm = 0;i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != 'T') {
				pwm = pwm*10 + uart_receive_buf[i]-'0';i++;
			}
		} else if(uart_receive_buf[i] == 'T') {
			time = 0;i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != '!') {
				time = time*10 + uart_receive_buf[i]-'0';i++;
			}
			
			if(index < DJ_NUM && (pwm<=2500)&& (pwm>=500) && (time<=10000)) {
				//duoji_doing[index].inc = 0;
				if(duoji_doing[index].cur == pwm)pwm += 0.1;
				if(time < 20)time = 20;
				duoji_doing[index].aim = pwm;
				duoji_doing[index].time = time;
				duoji_doing[index].inc = (duoji_doing[index].aim -  duoji_doing[index].cur) / (duoji_doing[index].time/20.000);
				
				//sprintf(cmd_return, "#%03dP%04dT%04d! %f \r\n", index, pwm, time, duoji_doing[index].inc);
				//uart1_send_str(cmd_return);
			}
		} else {
			i++;
		}
	}	
}

//字符串中的字符替代函数 把str字符串中所有的ch1换成ch2
void replace_char(u8*str, u8 ch1, u8 ch2) {
	while(*str) {
		if(*str == ch1) {
			*str = ch2;
		} 
		str++;
	}
	return;
}

//把eeprom_info写入到W25Q64_INFO_ADDR_SAVE_STR位置
void rewrite_eeprom(void) {
	W25Q_Write((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info));
}


//处理ADC数据，主要处理低压报警的AD数据
void handle_adc(void) {
	static u32 systick_ms_bak = 0;
	if(systick_ms - systick_ms_bak < 1000)return;
	systick_ms_bak = systick_ms;
	//经过一个肖特基 有0.3V的压降
	vol_adc = ADC_ConvertedValue[0]/4096.0 * 3.3 * 4 + 0.3;
//	if((vol_adc > 4.8) && (vol_adc < 7.4)) {
//		beep_on();
//		mdelay(100);
//		beep_off();
//	}
	//sprintf((char *)cmd_return, "vol:%.1f \r\n", vol_adc);
	//uart1_send_str(cmd_return);
}

void oled_dis_init(void) {
	u8 i;
	for(i=0;i<4;i++) {
		OLED_ShowCN(16*2+i*16,0,i);//测试显示中文
	}
	OLED_ShowStr(0,3,(u8 *)"ZLTech 2017",1);	//测试6*8字符
	OLED_ShowStr(0,4,(u8 *)"ZLTech 2017",2);	//测试8*16字符	
}

//OLED显示函数
void handle_oled(void) {	
		
#if 0
		u8 i;
		if(oled_init_fail)return;
		OLED_Fill(0xFF);//全屏点亮
		mdelay(2000);
		OLED_Fill(0x00);//全屏灭
		mdelay(2000);
	
		for(i=0;i<4;i++) {
			OLED_ShowCN(22+i*16,0,i);//测试显示中文
		}
		OLED_ShowStr(0,3,(u8 *)"ZLTech 2017",1);//测试6*8字符
		OLED_ShowStr(0,4,(u8 *)"ZLTech 2017",2);				//测试8*16字符
		
		mdelay(2000);
		OLED_CLS();//清屏
		OLED_OFF();//测试OLED休眠
		mdelay(2000);
		OLED_ON();//测试OLED休眠后唤醒
		//OLED_DrawBMP(0,0,128,8, (u8 *)MY_PIC);//测试BMP位图显示
		mdelay(2000);
#else
			u8 i;
	static u32 dj_sum_aim, dj_sum_aim_bak, dj_sum_cur, dj_sum_cur_bak;
	static u8 repeat_count = 0;
	static float vol_adc_bak = 0;

	dj_sum_aim = 0;
	dj_sum_cur = 0;
	for(i=0;i<6;i++) {
		dj_sum_aim += (int)duoji_doing[i].aim;
		dj_sum_cur += (int)duoji_doing[i].cur;
	}
	
	if((dj_sum_aim != dj_sum_aim_bak) || (dj_sum_cur != dj_sum_cur_bak) || ((vol_adc_bak - vol_adc <= -0.1) || (vol_adc_bak - vol_adc >= 0.1))) {
		dj_sum_aim_bak = dj_sum_aim;
		dj_sum_cur_bak = dj_sum_cur;
		vol_adc_bak = vol_adc;
		repeat_count = 0;
		
		sprintf((char *)cmd_return, "+Voltage:%02.1f V+", vol_adc);
		//uart1_send_str(cmd_return);
		OLED_ShowStr(0,0, cmd_return, 2);	//测试6*8字符


		for(i=0;i<6;i++) {
			sprintf((char *)cmd_return, " Aim%d:%04d Cur%d:%04d", (int)i, (int)duoji_doing[i].aim, (int)i, (int)duoji_doing[i].cur);
			//uart1_send_str(cmd_return);
			OLED_ShowStr(0,i+2, cmd_return,1);	//
		}
	} else {
		if(repeat_count < 10){
			sprintf((char *)cmd_return, "+Voltage:%02.1f V+", vol_adc);
			//uart1_send_str(cmd_return);
			OLED_ShowStr(0,0, cmd_return, 2);	//测试6*8字符

			for(i=0;i<6;i++) {
				sprintf((char *)cmd_return, " Aim%d:%04d Cur%d:%04d", (int)i, (int)duoji_doing[i].aim, (int)i, (int)duoji_doing[i].cur);
				//uart1_send_str(cmd_return);
				OLED_ShowStr(0,i+2, cmd_return,1);	//
			}
			repeat_count ++;
		}
	}	
#endif
}

void handle_sensor(void) {
	static u32 systick_ms_bak = 0;
	static u8 sensor_flag = 0;
	//PB11 引脚检测到低电平启动传感器检测
	if(sensor_pa1() == 0) {
		//模拟传感器
		if(systick_ms - systick_ms_bak > 100) {
			systick_ms_bak = systick_ms;
			sprintf((char *)uart_receive_buf, "{#001P%04dT0100!#002P%04dT0100!#003P%04dT0100!}", 
			1500-(ADC_ConvertedValue[1]/10),
			1500+(ADC_ConvertedValue[1]/10),
			1500+(ADC_ConvertedValue[1]/10));
			do_action(uart_receive_buf);
		}
	} else {
		//数字传感器
		if(sensor_pb11() == 0) {
			if(sensor_flag == 0) {
				sensor_flag = 1;
//				sprintf((char *)uart_receive_buf, "{#001P%04dT1000!#002P%04dT1000!#003P%04dT1000!}", 
//				1200,
//				1800,
//				1800);
//				do_action(uart_receive_buf);
				
				group_num_start= 1;
				group_num_end = 3;
				group_num_times =1;
				if(group_num_start != group_num_end) {
					do_start_index = group_num_start;
					do_time = group_num_times;
					group_do_ok = 0;
				} else {
					do_group_once(group_num_start);
				}
				
				while(sensor_pb11() == 0){
				}
				
				beep_on();mdelay(50);beep_off();
			}
			
		} else {
			if(sensor_flag == 1) {
				sensor_flag = 0;
//				sprintf((char *)uart_receive_buf, "{#001P%04dT1000!#002P%04dT1000!#003P%04dT1000!}", 
//				1500,
//				1500,
//				1500);
//				do_action(uart_receive_buf);
				beep_on();mdelay(100);beep_off();
			}
		}
	}
}


