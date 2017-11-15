#ifndef __HARDDRIVE_H__
#define __HARDDRIVE_H__

#include "stc15w4k32s4.h"
#include "unit.h"


//u8  sys_init_state=0;

#define SYSCLK 		11059200UL		//系统时钟频率为22118400Hz


/* 定义控制RunLed的宏命令 */
#define RunLed	P32	   	// LED ='0' 表示亮灯，LED ='1' 表示灭灯

#define RunLedTog()			RunLed=!RunLed	  //LED闪烁
#define RunLedOff()			RunLed=1			    //LED关闭
#define RunLedOn()			RunLed=0			    //LED打开


/* 定义控制导光灯的宏命令 */

#define DaoGuangLed	P35	   	// '1' 表示亮灯 '0'  表示灭灯

#define DaoGuangLedTog()			DaoGuangLed=!DaoGuangLed	  //LED闪烁
#define DaoGuangLedOff()			DaoGuangLed=0			    //LED关闭
#define DaoGuangLedOn()			  DaoGuangLed=1			    //LED打开


/* 定义控制报告灯的宏命令 */

#define BaoGaoLed	P51	   	// '1' 表示亮灯 '0'  表示灭灯

#define BaoGaoLedTog()			BaoGaoLed=!BaoGaoLed	  //LED闪烁
#define BaoGaoLedOff()			BaoGaoLed=0			    //LED关闭
#define BaoGaoLedOn()			  BaoGaoLed=1			    //LED打开


/* 定义加热控制的宏命令 */

#define Heat	P42	   	// '1' 表示开加热 '0'  表示关加热
#define HeatOff()	  	Heat=1			    //关闭
#define HeatOn()		  Heat=0			    //打开


/* 定义加热风扇控制的宏命令 */

#define HeatFan	P43	   	// '1' 表示开加热 '0'  表示关加热


#define HeatFanOff()	  	HeatFan=1			    //关闭
#define HeatFanOn()		    HeatFan=0			    //打开


/* 定义预留风扇控制的宏命令 */

#define Fan2	P36	   	// '1' 表示开加热 '0'  表示关加热


#define Fan2Off()	  	Fan2=0			    //LED关闭
#define Fan2On()		  Fan2=1			    //LED打开



/* 定义蜂鸣器控制宏命令 */

#define Buzzer	P53	   	// '1' 表示开加热 '0'  表示关加热


#define BuzzerOff()	  	Buzzer=0			    //关闭
#define BuzzerOn()		  Buzzer=1			    //打开


/* 定义传感器检测宏命令 */


//IPF510 墨量检测命令

#define  Ipf510_C     P05
#define  IPF510_M     P06
#define  IPF510_Y     P07
#define  IPF510_BK    P10
#define  IPF510_MBK   P11

#define  IPF510_FM    P33

//7280 墨量检测命令

#define  C7280_C     P47
#define  C7280_M     P12
#define  C7280_Y     P13
#define  C7280_BK    P14
#define  C7280_MBK   P15

#define  C7280_FM    P34




/*********************************IPF510 墨量检测函数***********************************/
//读IPF510_C 墨量
u8 ReadIpf510_C(void);

//读IPF510_M 墨量
u8 ReadIpf510_M(void);

//读IPF510_Y 墨量
u8 ReadIpf510_Y(void);

//读IPF510_BK 墨量
u8 ReadIpf510_BK(void);

//读IPF510_MBK 墨量
u8 ReadIpf510_MBK(void);

//读IPF510_FM 墨量
u8 ReadIpf510_FM(void);

/************************************END********************************************/

/*****************************7280 墨量检测函数**************************************/
//读7280_C 墨量
u8 Read7280_C(void);

//读7280_M 墨量
u8 Read7280_M(void);

//读7280_Y 墨量
u8 Read7280_Y(void);

//读7280_BK 墨量
u8 Read7280_BK(void);

//读7280_MBK 墨量
u8 Read7280_MBK(void);

//读7280_FM 墨量
u8 Read7280_FM(void);



/************************************end*******************************************/

/*******************************电机传感器检测**************************************/
//电机传感器检测

#define  MOTO_S8_S0    P52     //感应切刀开始动作

#define  MOTO_S7_S1    P04     //感应切刀结束动作  为出片通道动作开始提供标识。

#define  MOTO_S6_S2    P03     //感应胶片进入出片通道
#define  MOTO_S5_S3    P02     //检测胶片位置
#define  MOTO_S4_S4    P01     //检测胶片位置
#define  MOTO_S3_S5    P00     //感应胶片离开出片通道
#define  MOTO_S2_S6    P46     //感应胶片位置
#define  MOTO_S1_S7    P45 //预留


/*
动作流程：

感应切刀结束动作，为离合马达及同步带马达传达快速出片指令。

当切刀支架切胶片时走到1-P1传感器红外感应处时，离合马达会转到3根离合轴，
使黑色橡胶轮组合压紧，同步带马达在黑色橡胶轴压紧后快速转动，把胶片送出通道。


感应胶片进入出片通道，传达指令给同步带马达进行慢转。
保证通道不卡片。给加热器传达指令，保证加热器正常启动。


1-P3.1-P4传感器作用：
感应黑色轮码盘位置，控制离合马达转动，保证离合轴打开及关闭正常

1-P6传感器作用：
检测同步带马达是正常工作。

1-P5传感器作用：
检测胶片离开出片通道，传达工作结束指令。


*/
u8 MOTO_S8S0(void);
u8 MOTO_S7S1(void);
u8 MOTO_S6S2(void);
u8 MOTO_S5S3(void);
u8 MOTO_S4S4(void);
u8 MOTO_S3S5(void);
u8 MOTO_S2S6(void);
u8 MOTO_S1S7(void);

/************************************end*******************************************/





void Delay100ms(void) ;  //延时函数
//void Delay1s(void) ;

//加热极限控制函数

void HEAT_CONTROL(void);


//心跳控制函数

void heart_control(void);

//测试函数


void  Ml_test(void);



#endif

/**///////////////////////Pino Electronics////////////////////////**/