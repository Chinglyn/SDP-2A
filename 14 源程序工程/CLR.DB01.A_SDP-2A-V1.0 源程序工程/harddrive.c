/************************************************************************************
*
* CPU：STC15W4K3S4核心板
* 时钟选择：外部时钟11.0592MHz
* 功能说明：相关硬件驱动程序
* 作者信息：张 斌
*************************************************************************************
*更新说明
*20160928 增加通信协议模块---与上位机通信， 串口1  波特率9600
*
************************************************************************************/

#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "moto.h"


/********************************************************************
函数功能：延时函数。
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
volatile	u8 Delay_start=0;  //定时器中PWM 计数值   占空比
volatile	u8 Delay_num=0;  //定时器中PWM 计数值   占空比
volatile	u8 Delay_count=0;  //定时器中PWM 计数值   占空比
volatile	u8 Delay_flag=0;  //定时器中PWM 计数值   占空比

volatile	s16 Heart_Open=1;   //开启心跳标志   
volatile	u8  Heart_flag=1;   //无心跳标志
volatile	u32 Heart_Count=0;  //心跳超时计数

//墨量检测是否屏蔽标志  01 表示屏蔽  02 表示启用
u8 IPF510MT_PB=0x02;
u8 C7280MT_PB=0x02;
//废墨检测
u8 IPF510FM_PB=0x02;
u8 C7280FM_PB=0x02;



/*
void IO_DELAY_5_MS(u8 num)   
{
	Delay_num =num;
	Delay_start=1;
}
*/
/**///////////////////////Pino Electronics////////////////////////**/

/********************************************************************
函数功能：延时函数。
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
/*
void Delay100ms(void)   
{
    u8 a,b,c,n;
    for(c=5;c>0;c--)  //50
        for(b=135;b>0;b--)
            for(a=189;a>0;a--);
								for(n=1;n>0;n--);
}
*/
/**///////////////////////Pino Electronics////////////////////////**/

/*
void Delay1s(void)   
{
    u8 a,b,c,n;
    for(c=108;c>0;c--)
        for(b=135;b>0;b--)
            for(a=189;a>0;a--);
								for(n=1;n>0;n--);
}
*/

/* 定义DS18B20使用的引脚 */
#define DS18B20			P22

#define DQ_1()  		DS18B20=1		/* SDA = 1 */
#define DQ_0()  		DS18B20=0		/* SDA = 0 */
	
#define DQ_READ()  	DS18B20	/* 读SDA线状态 */



//光电传感器 不接IO 为低电平，  有墨水IO为低电平，无墨为高
//电容传感器 不接IO 为高电平，  有墨水IO为低电平，无墨为高

/*********************************IPF510 墨量检测函数***********************************/

//读IPF510_C 墨量
u8 ReadIpf510_C(void)
{
	if(Ipf510_C==0)
	{
		return 0;
  }
	else
	{
   	return 1; 
  }	
}


//读IPF510_M 墨量
u8 ReadIpf510_M(void)
{
	if(IPF510_M==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

//读IPF510_Y 墨量
u8 ReadIpf510_Y(void)
{
	if(IPF510_Y==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


//读IPF510_BK 墨量
u8 ReadIpf510_BK(void)
{
	if(IPF510_BK==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


//读IPF510_MBK 墨量
u8 ReadIpf510_MBK(void)
{
	if(IPF510_MBK==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

//读IPF510_FM 墨量
u8 ReadIpf510_FM(void)
{
	if(IPF510_FM==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


/************************************END********************************************/

//电容传感器 不接IO 为高电平，  有墨水IO为低电平，无墨为高
/*****************************7280 墨量检测函数**************************************/
//读7280_C 墨量
u8 Read7280_C(void)
{
	if(C7280_C==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

//读7280_M 墨量
u8 Read7280_M(void)
{
	if(C7280_M==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

//读7280_Y 墨量
u8 Read7280_Y(void)
{
	if(C7280_Y==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


//读7280_BK 墨量
u8 Read7280_BK(void)
{
	if(C7280_BK==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


//读7280_MBK 墨量
u8 Read7280_MBK(void)
{
	if(C7280_MBK==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

//读7280_FM 墨量
u8 Read7280_FM(void)
{
	if(C7280_FM==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


/************************************end*******************************************/


/*******************    *电机传感器检测函数****************************************/

u8 MOTO_S8S0(void)
{
	if(MOTO_S8_S0==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

u8 MOTO_S7S1(void)
{
	if(MOTO_S7_S1==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


u8 MOTO_S6S2(void)
{
	if(MOTO_S6_S2==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

u8 MOTO_S5S3(void)
{
	if(MOTO_S5_S3==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


u8 MOTO_S4S4(void)
{
	if(MOTO_S4_S4==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

u8 MOTO_S3S5(void)
{
	if(MOTO_S3_S5==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}


u8 MOTO_S2S6(void)
{
	if(MOTO_S2_S6==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}

u8 MOTO_S1S7(void)
{
	if(MOTO_S1_S7==0)
	{
		return 0;

  }
	else
	{
   	return 1; 
  }	
}




//加热器极限控制

extern volatile	s16 temputer1;
extern volatile	s16 temputer2;


extern volatile u8 TemputerHardState1;
extern volatile u8 TemputerHardState2;

void HEAT_CONTROL(void)
{
	WDT_CONTR |= 0x10;      //喂狗		
	
  if(temputer2>60)   //温度超过60度 关加热器
	{
		HeatOff();     //关加热
  }
	
	
	if(TemputerHardState2==0)
	{
		HeatOff();     // 温度传感器异常 关加热
  }	
	
}


//心跳控制 
void heart_control(void)
{
	WDT_CONTR |= 0x10;      //喂狗		

	
	if(Heart_flag==0)
	{
    Fan2On(); //开启控制端口
  }

		if(Heart_flag==1)
	{
    Fan2Off(); //关
  }


	
}






