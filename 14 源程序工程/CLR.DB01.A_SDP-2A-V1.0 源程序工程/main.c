/************************************************************************************
*
* CPU：STC15W4K3S4核心板
* 时钟选择：外部时钟11.0592MHz
* 功能说明：SDP-2A电控板主程序
* 作者信息：张 斌
*
************************************************************************************/
#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "ds18b20.h"
#include "moto.h"
#include "comwithpc.h"
#include "time.h"


/********************************************************************
函数功能：将STC15W4K系列的IO口初始化为准双向IO口。		  
入口参数：无。
返    回：无。
备    注：STC15W4K系列的芯片,上电后所有与PWM相关的IO口（P0.6/P0.7/
		  P1.6/P1.7/P2.1/P2.2/P2.3/P2.7/P3.7/P4.2/P4.4/P4.5）均为高
		  阻态,需将这些口设置为准双向口或强推挽模式方可正常使用。
********************************************************************/
void InitIO(void)
{
	
	IP=0X12; //设置中断优先级  定时器 >串口 >定时器2
	
	//初始化 P0 为准双向IO口
	P0M0 = 0x00;
	P0M1 = 0x00;
  //初始化 P1 为准双向IO口
	//其中P1.6  P1.7 为外部晶振引脚 设置为高阻态
	P1M0 = 0x00;  //0b00000000
	P1M1 = 0xc0;  //0b11000000
  //
	P2M0 = 0x0E;  //0B00001110  ( p2.1  p2.2  p2.3  PWM电机 ) （ P2.5  P2.6  DS18B20 ）设置为强推挽模式
	P2M1 = 0x00;  //0B00000000  
  //
	P3M0 = 0xE0;  //0B11100000  ( p3.5 导光灯  p3.6 风扇  p3.7  电机PWM ) 
	P3M1 = 0x00;  //0B00000000
  //
	P4M0 = 0x0C;  //0B00001100  ( p4.2 加热控制  p4.3  加热风扇 )
	P4M1 = 0x00;  //0B00000000
	//
	P5M0 = 0x0A;  //0B00001010  (  P5.1  报告灯 P5.3 BUZZER)
	P5M1 = 0x00;  //0B00000000
	
//初始化电机端口为高阻输出	
    MOTO1_IN1Off();
    MOTO1_IN2Off();		
	  MOTO2_IN3Off();
    MOTO2_IN4Off();			
//	
}
/**///////////////////////Pino Electronics////////////////////////**/

extern u8 sys_init_state; //系统初始化完成标志
extern u8 TemputerHardState;

/********************************************************************
函数功能：主函数。
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
void main (void)
{	
	
	  InitIO();     //初始化IO	
	
    InitUart1();  //初始化串口1
	
	  InitTimer0(); //初始化定时器0
	
	  Timer2Init();  //初始化定时器2    
	
	  PWM_config();  //PWM初始化
	
		Fan2Off();     //心跳控制
	 	HeatFanOn();  // 开启加热风扇
	  HeatOn();     //开启加热
	
	  BuzzerOn();
    //Fan2On();     //开预留风扇	
	  BaoGaoLedOn();   //开报告灯   --送纸电机
	
	  DaoGuangLedOn();  //开导光灯	
	
		Delay100us(10000);  //1S
	  Delay100us(10000);  //1S
		HeatOff();     //关加热
		HeatFanOff();  //关加热风扇    
		
	  BuzzerOff();
	  
	 // DaoGuangLedOff();  //关导光灯
	
	  EA=1;					//允许所有中断	
		
	  //看门狗定时器
	  WDT_CONTR = 0x06;         //看门狗定时器溢出时间计算公式: (12 * 32768 * PS) / FOSC (秒)
                              //设置看门狗定时器分频数为128,溢出时间如下:
                              //11.0592M : 4.55S
    WDT_CONTR |= 0x20;        //启动看门狗	
		
    comwithpc();            //处理一次通信程序	
		/******************************		//异常断电，卡片处理*******************/
		Moto_DC2_Control(0);	    // 离合电机关		
	  MotoDc1Run(100,FORWARD,10); //100占空比 正转10S 送出 由于断电原因存在机器中的胶片	
    Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
		comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
		comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
		comwithpc();            //处理一次通信程序
		
	  BaoGaoLedOff();   //关报告灯---送报告电机 运行5秒
		
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
	  comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
		comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
	  comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
		comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗	
		comwithpc();            //处理一次通信程序
	  Delay100us(10000);  //1S	
		WDT_CONTR |= 0x10;      //喂狗	
	  comwithpc();            //处理一次通信程序
		Delay100us(10000);  //1S
		WDT_CONTR |= 0x10;      //喂狗		
		comwithpc();            //处理一次通信程序
		/******************************************END 异常断电********************/	
	  Moto_DC2_Control(1);	    // 离合电机开	  离合必须始终开启，防止打印的时候顶住胶片
		
	  sys_init_state=0;       //初始化完成
		
	 while(1)
	{	
		WDT_CONTR |= 0x10;      //喂狗			
	  comwithpc();            //串口程序处理		
		WDT_CONTR |= 0x10;      //喂狗		
	  Moto_control_sys();     //出片控制程序		
		WDT_CONTR |= 0x10;      //喂狗		
		HEAT_CONTROL();         //极限温度控制
		heart_control();  //心跳控制
		Auto_Send_Paper_a4();  //报告送纸控制
	}
	
	
}

/**///////////////////////Pino Electronics////////////////////////**/


