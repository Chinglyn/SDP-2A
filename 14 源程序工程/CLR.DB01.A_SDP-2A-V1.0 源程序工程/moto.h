/******************************************************************

moto.H  file
  
*******************************************************************/

#ifndef __MOTO_H__
#define __MOTO_H__

#include "stc15w4k32s4.h"
#include "unit.h"


#define  HEATTIME 1  //胶片在通道中加热时间 单位秒

#define  MAPAN_MAX_CONT  30  //后半段胶片加热计数步距离

#define  MAPAN_SEND_PAPER_CONT  1800  //后半段胶片加热计数步距离



/********************************电机控制不调速************************************/
/* 定义控制RunLed的宏命令 */
#define MOTO1_IN1	P37	   	// LED ='0' 表示亮灯，LED ='1' 表示灭灯
#define MOTO1_IN1Tog()		MOTO1_IN1=!MOTO1_IN1	  //LED闪烁
#define MOTO1_IN1Off()		MOTO1_IN1=0			    //LED关闭
#define MOTO1_IN1On()			MOTO1_IN1=1			    //LED打开


/* 定义控制RunLed的宏命令 */
#define MOTO1_IN2	P21	   	// LED ='0' 表示亮灯，LED ='1' 表示灭灯

#define MOTO1_IN2Tog()		MOTO1_IN2=!MOTO1_IN2	  //LED闪烁
#define MOTO1_IN2Off()		MOTO1_IN2=0			    //LED关闭
#define MOTO1_IN2On()			MOTO1_IN2=1			    //LED打开



/* 定义控制RunLed的宏命令 */
#define MOTO2_IN3	P23	   	// LED ='0' 表示亮灯，LED ='1' 表示灭灯

#define MOTO2_IN3Tog()		MOTO2_IN3=!MOTO2_IN3	  //LED闪烁
#define MOTO2_IN3Off()		MOTO2_IN3=0			    //LED关闭
#define MOTO2_IN3On()			MOTO2_IN3=1			    //LED打开

/* 定义控制RunLed的宏命令 */
#define MOTO2_IN4	P22	   	// LED ='0' 表示亮灯，LED ='1' 表示灭灯
#define MOTO2_IN4Tog()		MOTO2_IN4=!MOTO2_IN4	  //LED闪烁
#define MOTO2_IN4Off()		MOTO2_IN4=0			    //LED关闭
#define MOTO2_IN4On()			MOTO2_IN4=1			    //LED打开


/************************************end*******电机控制不调速***********************/

#define FORWARD   1   //正转
#define REVERSE   0   //反转

#define CYCLE     100UL     //定义PWM周期(最大值为32767)  // 周期 9.28us     107.7KHZ



//普通电机控制函数
void MotoDc1Forward(void);
void MotoDc1Reverse(void);
void MotoDc1Stop(void);

void MotoDc2Forward(void);
void MotoDc2Reverse(void);
void MotoDc2Stop(void);
//PWM 电机控制
void MotoDc1Run(u8 time, u8 direction , u8 runtime);
void MotoDc2Run(u8 time, u8 direction ,u8 runtime);
//离合控制
void Moto_DC2_Control(u8 TYPE );
void Moto_control_sys(void);

//20161104  增加PWM控制
void PWM_config(void);

//PWM 输出控制函数
void PWM2_SetPwmWide(u16 Wide);  	
void PWM3_SetPwmWide(u16 Wide);
void PWM4_SetPwmWide(u16 Wide);
void PWM5_SetPwmWide(u16 Wide);
//void PWM6_SetPwmWide(u16 Wide);
//void PWM7_SetPwmWide(u16 Wide);


void Auto_Send_Paper_a4(void);

#endif

/**///////////////////////Pino Electronics////////////////////////**/
