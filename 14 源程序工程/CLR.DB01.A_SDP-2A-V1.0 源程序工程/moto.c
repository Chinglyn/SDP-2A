
/*************  功能说明    **************

本程序演示使用定时器做软件PWM。  
定时1ms中断定时器2做16位自动重装。

本部分包含，直流电机全速运转驱动，以及PWM运转驱动


******************************************/



#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "ds18b20.h"
#include "moto.h"

// 全局变量 


volatile	u8 Paper_Send_Flag1=0;  //胶片输送状态标志1
volatile	u8 Paper_Send_Flag2=0;  //胶片输送状态标志2
volatile	u8 Paper_Send_Error=0;   //卡片标志
volatile	u8 Paper_Send_OK=1;   //0 就绪，1 正在打印，2 出片，3 卡片标志


volatile	u8 PWM1count=0;  //定时器中PWM 计数值   占空比
volatile	u8 PWM2count=0;  //定时器中PWM 计数值   占空比

volatile	u8 PWM1time=1;  //PWM 占空比值  //占空比改为浮点数 
volatile	u8 PWM2time=1;  //PWM 占空比值  

volatile	u8 PWMDC1Direction=1;  //电机1 方向标志  1 正转  2 反转
volatile	u8 PWMDC2Direction=1;  //电机2 方向标志  1 正转  2 反转

volatile	u8 DC1Run=0;  //电机1 方向标志  1 正转  2 反转
volatile	u8 DC2Run=0;  //电机2 方向标志  1 正转  2 反转

volatile  u8 MOTO_DC1_Run_Time=0;  //电机运行时间
volatile  u8 MOTO_DC2_Run_Time=0;  //电机运行时间

volatile  u8 Heat_Time=0;  //胶片留在通道中的加热时间。 单位 秒
volatile  u8 Heat_Time_Flag=0;  //胶片加热完成标志

volatile  u8 Heat_Time_Flag_start=0;  //胶片加热完成标志
volatile  u8 SendPaper_Time_Flag_start=0;  //胶片加热完成标志

volatile	u32 DC1_Run_Time_count=0;  //定时器中PWM 计数值  电机运行时间
volatile	u32 DC2_Run_Time_count=0;  //定时器中PWM 计数值  电机运行时间


//volatile	u8 DC1DECAY=1;  //电机1 衰减方式标志
//volatile	u8 DC2DECAY=1;  //电机2 衰减方式标志

volatile	u8 MOTO2_LH_Control=0;     //电机2 离合控制
volatile	u8 MOTO2_LH_Flag=3;        //电机2 离合控制

// volatile	u8 MOTO1_SEND_Flag=0;  


volatile  u32 JiuXu_Count=0;  //胶片加热完成标志
volatile  u8 JiuXu_Count_start=0;  //胶片加热完成标志



/****************电机控制系统相关标识定义********************/

//起始切刀
u8 Sensor_P0_flag=0;        //切刀传感器动作标识
u8 Sensor_P0_new_state;     //当前检测状态
u8 Sensor_P0_old_state=1;   //上一状态
//结束切刀
u8 Sensor_P1_flag=0;         //切刀传感器动作标识
u8 Sensor_P1_new_state;      
u8 Sensor_P1_old_state=0;    

//胶片进入通道口传感器
u8 Sensor_P2_flag=0;     //胶片进入通道口标志
u8 Sensor_P2_f10=0;      //状态由1--0 
u8 Sensor_P2_f01=0;        //状态由0--1
u8 Sensor_P2_new_state;    //当前检测状态
u8 Sensor_P2_old_state=0;  //上一状态

u8 Sensor_P2_Moto_Stop_flag=0; //慢转停止标志

//u32 Sensor_P2_f10_count=0;    //完成之后延时两秒计数



//离合关
//u8 Sensor_P3_flag=0;         //离合关传感器状态标识
u8 Sensor_P3_new_state;      //离合关传感器最新状态
u8 Sensor_P3_old_state=0;    //离合关传感器上一次状态

//离合开
//u8 Sensor_P4_flag=0;     //离合开传感器状态
u8 Sensor_P4_new_state;  //离合开传感器最新状态
u8 Sensor_P4_old_state=0;  //离合开传感器上一次状态


u8 Sensor_P5_flag_start=0;    //标识是否需要进行出片检测
u8 Sensor_P5_flag=0;          //出片口传感器  从高--变低--变高  表示胶片正常输出。  1 表示正常输出
u8 Sensor_P5_f10=0;          //出片口传感器  从高--变低 标识
u8 Sensor_P5_new_state=0;    //当前检测状态 
u8 Sensor_P5_old_state=0;    //上一状态

u8 Sensor_P5_OK_flag=0;    //传感器检测完成标志
u8 Sensor_P5_OK_flag_count=0;    //完成之后延时两秒计数


u8 Sensor_P6_flag=0;              //码盘传感标识 --加热阶段到位标志

u8 Sensor_P6_SendPaper_flag=0;    //码盘传感标识 --送片阶段超时标志

u8 Sensor_P6_new_state=0;
u8 Sensor_P6_old_state=0;

u16 Sensor_P6_flag_cont=0;    //加热步数计数器
u8 Sensor_P6_cont_start=0;    //加热行程开始计数标识

u16 Sensor_P6_SendPaper_flag_cont=0; //送纸步数计数器
u16 SendPaper_P6_Time_Flag_start=0; // 送纸超时计数开始标志



/****************************************************************/
volatile  u32 auto_paper_a4_Count=0;  //报告送纸计数
volatile  u32 auto_paper_a4_Count2=0;  //报告送纸计数--超时计数
volatile  u8 auto_paper_a4_Count_Flag=0;  //报告送纸标志

volatile  u8 auto_paper_a4_Max_Flag=0;  //报告送纸标志





/*******************不做PWM控制 电机驱动********************/
/********************************************************************
函数功能：电机1正转
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
/*
void MotoDc1Forward(void)
{
   MOTO1_IN1On();
	 MOTO1_IN2Off();
}
8/
/********************************************************************
函数功能：电机1反转
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
/*
void MotoDc1Referse(void)
{
   MOTO1_IN1Off();
	 MOTO1_IN2On();
}
*/ 
/********************************************************************
函数功能：电机1停止
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
void MotoDc1Stop(void)
{
	 DC1Run=0;
	 MOTO_DC1_Run_Time=0;
   MOTO1_IN1Off();
	 MOTO1_IN2Off();
   PWM2_SetPwmWide(0);     //输出全低电平
   PWM3_SetPwmWide(0);     //输出低电平	
}
 


//20161104  增加PWM控制出片电机



sbit PWM2   =   P3^7;
sbit PWM3   =   P2^1;
sbit PWM4   =   P2^2;
sbit PWM5   =   P2^3;
sbit PWM6   =   P0^7;
sbit PWM7   =   P0^6;

/*


    PWM2_SetPwmWide(0);         //输出全低电平
    PWM3_SetPwmWide(1);         //输出1/2550高电平
    PWM4_SetPwmWide(CYCLE);     //输出全高电平
    PWM5_SetPwmWide(CYCLE-1);   //输出2549/2550低电平
		
    PWM6_SetPwmWide(CYCLE/2);   //输出1/2高电平
    PWM7_SetPwmWide(CYCLE/3);   //输出1/3高电平

*/


void PWM_config(void)
{
	  /*
    P0M0 &= ~0xc0;
    P0M1 &= ~0xc0;
    P0 &= ~0xc0;                    //设置P0.6/.P0.7电平
	  */
	
    P2M0 &= ~0x0e;
    P2M1 &= ~0x0e;
    P2 &= ~0x0e;                    //设置P2.1/P2.2/P2.3电平
	
    P3M0 &= ~0x80;
    P3M1 &= ~0x80;
    P3 &= ~0x80;                    //设置P3.7电平

    P_SW2 |= 0x80;

    PWMCKS = 0x00;
	
    PWMC = CYCLE;                   //设置PWM周期
	
    PWM2T1 = 1;
    PWM2T2 = 0;
	
    PWM2CR = 0x00;                  //PWM2输出到P3.7
    PWM3T1 = 1;
    PWM3T2 = 0;
    PWM3CR = 0x00;                  //PWM3输出到P2.1
    PWM4T1 = 1;
    PWM4T2 = 0;
    PWM4CR = 0x00;                  //PWM4输出到P2.2
    PWM5T1 = 1;
    PWM5T2 = 0;
    PWM5CR = 0x00;                  //PWM5输出到P2.3
		
	  /*
    PWM6T1 = 1;
    PWM6T2 = 0;
    PWM6CR = 0x08;                  //PWM6输出到P0.7
    PWM7T1 = 1;
    PWM7T2 = 0;
    PWM7CR = 0x08;                  //PWM7输出到P0.6
	  */	
		
    PWMCFG = 0x00;                  //配置PWM的输出初始电平
		
    PWMCR = 0x3f;                   //使能PWM信号输出
		
    PWMCR |= 0x80;                  //使能PWM模块
    P_SW2 &= ~0x80;
}


void PWM2_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x01;  //关闭归零中断
        PWM2 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x01;  //关闭归零中断
        PWM2 = 1;
    }
    else
    {
        P_SW2 |= 0x80;   //外设功能切换寄存器
        PWM2T1 = Wide;   //设置翻转点
        P_SW2 &= ~0x80;  //外设功能切换
        PWMCR |= 0x01;   //使能中断
    }
}

void PWM3_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x02;
        PWM3 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x02;
        PWM3 = 1;
    }
    else
    {
        P_SW2 |= 0x80;
        PWM3T1 = Wide;
        P_SW2 &= ~0x80;
        PWMCR |= 0x02;
    }
}

void PWM4_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x04;
        PWM4 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x04;
        PWM4 = 1;
    }
    else
    {
        P_SW2 |= 0x80;
        PWM4T1 = Wide;
        P_SW2 &= ~0x80;
        PWMCR |= 0x04;
    }
}

void PWM5_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x08;
        PWM5 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x08;
        PWM5 = 1;
    }
    else
    {
        P_SW2 |= 0x80;
        PWM5T1 = Wide;
        P_SW2 &= ~0x80;
        PWMCR |= 0x08;
    }
}
/*
void PWM6_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x10;
        PWM6 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x10;
        PWM6 = 1;
    }
    else
    {
        P_SW2 |= 0x80;
        PWM6T1 = Wide;
        P_SW2 &= ~0x80;
        PWMCR |= 0x10;
    }
}

void PWM7_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x20;
        PWM7 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x20;
        PWM7 = 1;
    }
    else
    {
        P_SW2 |= 0x80;
        PWM7T1 = Wide;
        P_SW2 &= ~0x80;
        PWMCR |= 0x20;
    }
}
*/
// 电机2驱动
/********************************************************************
函数功能：电机2正转
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
/*
void MotoDc2Forward(void)
{
   MOTO2_IN3On();
	 MOTO2_IN4Off();
}
*/
/********************************************************************
函数功能：电机2反转
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
/*
void MotoDc2Reverse(void)
{
   MOTO2_IN3Off();
	 MOTO2_IN4On();
}
 */
/********************************************************************
函数功能：电机2停止
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
void MotoDc2Stop(void)
{  
	 DC2Run=0;
	 MOTO_DC2_Run_Time=0;
   MOTO2_IN3Off();
	 MOTO2_IN4Off();	
	 PWM4_SetPwmWide(0);     //输出全低电平
   PWM5_SetPwmWide(0);     //输出低电平	
}
 

//以下为PWM 控制 电机驱动




/********************************************************************
函数功能：电机1 pwm 控制
入口参数：无。 占空比  方向
返    回：无。
备    注：无。
********************************************************************/
void MotoDc1Run(u8 time, u8 direction , u8 runtime)
{
	 DC1Run=1;   //置电机运行标志	
	 MOTO_DC1_Run_Time=runtime;   //运行时间
	 PWM1time=time; //赋占空比值
	 PWMDC1Direction=direction; //方向
	
	if(PWMDC1Direction==1) //正转
	{
		  if(time==100)
			{
			  PWM2_SetPwmWide(CYCLE);     //占空比输出
        PWM3_SetPwmWide(0);             //输出低电平

      }
			else
			{	
				PWM2_SetPwmWide(PWM1time);           //占空比输出
        PWM3_SetPwmWide(0);             //输出低电平
      }	
			
		

  }

	if(PWMDC1Direction==0) //反转
	{
			 if(time==100)
			{
			  PWM3_SetPwmWide(CYCLE);     //占空比输出
        PWM2_SetPwmWide(0);             //输出低电平
      }
			else
			{				
				PWM3_SetPwmWide(PWM1time);           //占空比输出
        PWM2_SetPwmWide(0);             //输出低电平     
      }	
  }
	
}


/********************************************************************
函数功能：电机1 pwm 控制
入口参数：无。 占空比  方向
返    回：无。 32<time<=100
备    注：无。
********************************************************************/
void MotoDc2Run(u8 time, u8 direction ,u8 runtime)
{
	 DC2Run=1;   //置电机运行标志
	 MOTO_DC2_Run_Time=runtime; //运行时间
	
	 PWM2time=time; //赋 占空比值
	
	 PWMDC2Direction=direction;
	
	if(PWMDC2Direction==1) //正转
	{
			 if(time==100)
			{
			  PWM5_SetPwmWide(CYCLE);     //占空比输出
        PWM4_SetPwmWide(0);             //输出低电平
      }
			else
			{				
				PWM5_SetPwmWide(PWM2time);           //占空比输出
        PWM4_SetPwmWide(0);             //输出低电平     
      }	
  }
  
		if(PWMDC2Direction==0) //反转
	{
			 if(time==100)
			{
			  PWM4_SetPwmWide(CYCLE);     //占空比输出
        PWM5_SetPwmWide(0);             //输出低电平
      }
			else
			{				
				PWM4_SetPwmWide(PWM2time);           //占空比输出
        PWM5_SetPwmWide(0);             //输出低电平     
      }	
  }

	
}


/********************************************************************
函数功能：出片通道控制函数
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/


// 电机1为传动电机
// 电机2为离合电机

//离合电机开关控制
/********************************************************************
函数功能：离合电机开关控制
入口参数：TYPE   TYPE =1  OPEN  TYPE =0  CLOSE 
返    回：无。 
备    注：无。
********************************************************************/
void Moto_DC2_Control(u8 TYPE )
{
	 //开电机2 置离合关标志。
   MotoDc2Run(30,1,10); //离合电机运行10S 
	  // MotoDc2Run(20,1,10); //离合电机运行10S   	//	转速太快，离合关闭的时候 夹不紧

	 //离合到位后定时器关闭电机2
   MOTO2_LH_Control=1;  //1 需要进行离合控制
   MOTO2_LH_Flag=TYPE;  //离合控制类型 1 开 松开 ，0 关加紧
}

//胶片输送控制程序
/********************************************************************
函数功能：胶片输送控制程序
入口参数：无
返    回：无。 
备    注：无。
********************************************************************/
u8  test=0xFF;
u8  sensed_time=0;    //卡片再送一次

void Moto_control_sys(void)
{

	u8  Sensor_P0_TMP;  //临时变量，读取切刀位置
	u8  Sensor_P1_TMP;  //临时变量，读取切刀位置
	
	
	WDT_CONTR |= 0x10;      //喂狗

	Sensor_P0_TMP=MOTO_S8S0();  //再次读切刀状态   不能改变中断中   
	Sensor_P1_TMP=MOTO_S7S1();	 //再次读切刀状态	
	Sensor_P2_new_state=MOTO_S6S2();	 //读胶片是否在输送口状态  
	
/* 
	uart1senddata(&DC2_Run_Time_count,4);//1	
	uart1senddata(&Sensor_P0_flag,1);//1
	uart1senddata(&Sensor_P1_flag,1);//1	
	uart1senddata(&Sensor_P2_new_state,1);//1	
	uart1senddata(&Paper_Send_Flag1,1);//1	
	uart1senddata(&Paper_Send_Flag2,1);//1	
	uart1senddata(&Paper_Send_OK,1);//1
	

  uart1senddata(&Sensor_P2_f10,1);//1
	uart1senddata(&Sensor_P2_Moto_Stop_flag,1);//1
	
	*/
	
	if(	(Sensor_P0_TMP==1 ) && (Sensor_P1_TMP==0)) //切刀已经归位
	{
			if( (Sensor_P1_flag==1) && (Sensor_P0_flag==1) ) //发生了切刀动作
			{	
				
				Sensor_P2_new_state=MOTO_S6S2();	 //读胶片是否在输送口状态  
				
				if((Sensor_P2_new_state==0)|| (Paper_Send_Flag1==1) || (Paper_Send_Flag2==1))  //胶片到达通道口,或者处于第一阶段，或者处于第二阶段
				{
					
							//test=0xFF;
							//uart1senddata(&test,1);//1					
							//送片第一阶段 关离合 加热 
							if(Paper_Send_Flag1==0)
							{	
								// test=0xEE;
								// uart1senddata(&test,1);//1
								
							  	 Paper_Send_OK=2;     //开始出片，清出片成功标志
								 
							  	 Sensor_P5_flag_start=1 ; //需要检测出片口传感器	
								
								
							  	 Paper_Send_Flag1=1;  //置位 表示已经进行了加热控制	
								
								   Heat_Time_Flag_start=1;  //加热等待时间计数
								
							    //	 MotoDc1Run(80,FORWARD,1); //60占空比 正转 25S 	 慢转		
								
						     // Sensor_P6_cont_start=1;  //行程码盘计数开始  --注释掉，表示不等待加热时间
								
								 //关离合
								 Moto_DC2_Control(0);	    // 离合电机关
				
								//不加热等待
								// Heat_Time_Flag=1;   
								
							} 
	
	
							//判断胶片是否到了预定的加热位置，到了停止电机	
							 if(Sensor_P6_flag==1) //后半段加热输送距离
							 {						 
								 MotoDc1Stop();           //停止电机 
								 Sensor_P6_cont_start=0;	//清标志
								 Sensor_P6_flag_cont=0;	  //清计数器	 
								 Sensor_P6_flag=0;        //清标志 
							 }	

					 
              //第二阶段  输送胶片
							if(Paper_Send_Flag2==0)
							{							
								 //加热完毕 输送胶片
								 if(Heat_Time_Flag==1)  //判断加热是否完成
								 { 
									 
										Paper_Send_OK=2;     //开始出片，清出片成功标志
									 
										Paper_Send_Flag2=1;  //第二阶段置位标志 
									 
										Heat_Time_Flag_start=0;  //关加热计时开始标志
									 
									 //	Heat_Time_Flag=0;        //清加热完成标志 
									 
									 //胶片停留加热时间完毕，送出胶片
									 
										MotoDc1Run(100,FORWARD,15); //100占空比 正转15S 	  全速
									  
									// 
									 
									 SendPaper_P6_Time_Flag_start=1;  //超时码盘计数	 
									 
								 }
							 
							 }
							
							 //胶片正常输出
							 if(Sensor_P5_flag==1)
							 { 
									SendPaper_P6_Time_Flag_start=0;  //关计数标志
								  Sensor_P6_SendPaper_flag=0;  // 清标志
									Sensor_P5_flag=0;      //清标志
									
								  //MotoDc1Run(100,FORWARD,1); //过了传感器再转一秒钟  保证完全出来
									MotoDc1Stop();
								  //开离合
								  Moto_DC2_Control(1);	    // 离合电机开
								 
									Paper_Send_OK=0x00;       //置位出片成功标志	
								 
								 //就绪计数开始标志
								 
                  JiuXu_Count=0;    //清就绪计数器
								 
                  JiuXu_Count_start=1;  //置就绪计数开始标志
	               
								  sensed_time=0 ;
								  
								  Heat_Time_Flag=0;        //清加热完成标志 

									Sensor_P1_flag=0;      //清切刀标志
									Sensor_P0_flag=0;      //清切刀标志				 
									Paper_Send_Flag1=0;    //清第一阶段标志
									Paper_Send_Flag2=0;    //清第二阶段标志
									Sensor_P5_flag_start=0 ; //清出片口检测标志
									Sensor_P6_SendPaper_flag=0; 
									Sensor_P6_SendPaper_flag_cont=0; //
									SendPaper_P6_Time_Flag_start=0;//送片超时码盘计数  
									
									Sensor_P2_Moto_Stop_flag=0;
									
								  //关加热	
								//	HeatOff();     //开启加热
								 
							 } //正常出片
						
								 
							 if(Sensor_P6_SendPaper_flag==1) //码盘计数超时，表示卡片
							 {
								 
									 Sensor_P6_SendPaper_flag=0;			 
									 // Paper_Send_OK=0x05;  //表示卡片			 									 
									 // MotoDc1Stop();  //停止电机 
									 // 一般情况下卡片是因为离合电机没有加紧
									 //卡片容错处理一次		 								 
									 if(sensed_time==0x01)  //报卡片
									 {
										 
												Heat_Time_Flag=0;        //清加热完成标志 
										 
												SendPaper_P6_Time_Flag_start=0;	 
									 
												Sensor_P6_SendPaper_flag=0; 
									 
												SendPaper_Time_Flag_start=0;		
									 
												Sensor_P6_SendPaper_flag_cont=0;

												Paper_Send_OK=3;  //表示卡片	
                        
										    //关加热
												HeatOff();     //开启加热
										 
									 }
									 
									 //关离合
									 if(sensed_time==0x00)
									 {
											 sensed_time=0x01;
											 Moto_DC2_Control(0);	    // 离合电机关	 	 
											 Paper_Send_Flag2=0 ; //置标志 再送一次纸张
									 }	
 		 						 
							  }	//en  卡片处理
								
							 
							 //卡片后处理
							 if(Paper_Send_OK==3)
							 {
								  //人工干预取片 之后清除相关标志
								   if(Sensor_P2_new_state==1)    //入口无胶片 自动清除卡片标志  //如果卡片在中间，第二张继续打还是会卡在下面。两张一起清除
									 {									
											SendPaper_P6_Time_Flag_start=0;  //关计数标志
											Sensor_P6_SendPaper_flag=0;  // 清标志
										 
											Sensor_P5_flag=0;      //清标志
											MotoDc1Stop();         //停止电机   
											//开离合
											Moto_DC2_Control(1);	    // 离合电机开	 		 
											Paper_Send_OK=0x01;       //清除之后变为就绪
										 //就绪计数开始标志 
											JiuXu_Count=0;    //清就绪计数器
											JiuXu_Count_start=1;  //置就绪计数开始标志  
											sensed_time=0 ;  
											Heat_Time_Flag=0;        //清加热完成标志  
											Sensor_P1_flag=0;      //清切刀标志
											Sensor_P0_flag=0;      //清切刀标志				 
											Paper_Send_Flag1=0;    //清第一阶段标志
											Paper_Send_Flag2=0;    //清第二阶段标志
											Sensor_P5_flag_start=0 ; //清出片口检测标志
											Sensor_P6_SendPaper_flag=0; 
											Sensor_P6_SendPaper_flag_cont=0; //
											SendPaper_P6_Time_Flag_start=0;//送片超时码盘计数  	 
											
											HeatFanOff(); //关加热风扇
								 } 	 
               }
							 
							 
        }	 //有胶片 ，第一阶段，第二阶段
			} //end P0  P1  切刀动作
  }	//end P0  P1  切刀归位
	
	//有胶片无切刀  慢转
	if((Sensor_P2_new_state==0)&&(Sensor_P1_flag==0)&&(Sensor_P0_flag==0))  //胶片到达通道口，无切刀动作
	{		
		if(Sensor_P2_Moto_Stop_flag==0)
		{	
			    //防止再切刀切完没有归位的过程中带走胶片造成报错		
				if(	(Sensor_P0_TMP==1 ) && (Sensor_P1_TMP==0)) //切刀已经归位
				{	
							MotoDc1Run(60,FORWARD,1); //50占空比
				}	
        
        		//有胶片 开加热
				HeatFanOn();  // 开启加热风扇
			//	HeatOn();     //开启加热				
				
    }	
		
		 if(Sensor_P2_Moto_Stop_flag==1)
		{	       		
				HeatFanOff();  // 开启加热风扇
			//	HeatOff();     //开启加热				
				
    }	
		

	
	}
	
} //end  Moto_control_sys


void Auto_Send_Paper_a4(void)
{
	
	if(MOTO_S1S7()==0)  //清超时标志
	{
	  auto_paper_a4_Max_Flag=0;
	}	
	
	
	if(auto_paper_a4_Max_Flag==0)  //不超时
	{
			if(MOTO_S1S7()==1)
			{
				auto_paper_a4_Count_Flag=1;
				//开导光灯---报告电机
				 BaoGaoLedOn();
			}	
			
			if(MOTO_S1S7()==0)
			{
				if(auto_paper_a4_Count_Flag==0)
				{
					 BaoGaoLedOff();
				}
		  }
   
   }

}


/**********************************END Moto_control_sys*************************************/










