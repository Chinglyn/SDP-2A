/************************************************************************************
* 功能说明：
* 定时器处理函数

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
函数功能：初始化定时器0，每5ms产生一个中断。
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
void InitTimer0(void)
{
	
	//AUXR|=0x80;		//设置定时器0为1T模式，默认为12T模式
	
	TMOD&=(~0x03);	//设置Timer0工作在模式0(16位自动重载模式)

	/*系统时钟选择为11.0592M时， 重载值的计算如下：	  
	11059200/12分频/200=4608，即每计数4608耗时为5ms。*/	
	TH0 = (65536-SYSCLK/12/200)>>8;		//初始化Timer0重载值
	TL0 = (65536-SYSCLK/12/200);		//初始化Timer0重载值
	
	ET0=1;  		//Timer0中断使能
	TR0 = 1;		//Timer0开启
}
/**///////////////////////Pino Electronics////////////////////////**/


extern u8 Sensor_P0_flag;       //切刀传感器动作标识
extern u8 Sensor_P0_new_state;  //切刀传感器动作标识
extern u8 Sensor_P0_old_state;  //切刀传感器动作标识

extern u8 Sensor_P1_flag;       //切刀传感器动作标识
extern u8 Sensor_P1_new_state;  //切刀传感器动作标识
extern u8 Sensor_P1_old_state;  //切刀传感器动作标识

//胶片进入通道口传感器
extern u8 Sensor_P2_flag;     //切刀传感器动作标识
extern u8 Sensor_P2_f10;      //状态由1--0 
extern u8 Sensor_P2_f01;      //状态由0--1
extern u8 Sensor_P2_new_state;  //切刀传感器动作标识
extern u8 Sensor_P2_old_state;  //切刀传感器动作标识
//extern u32 Sensor_P2_f10_count;    //完成之后延时停止计数
extern u8 Sensor_P2_Moto_Stop_flag;   //慢转停止标志


//extern u8 Sensor_P3_flag;       //切刀传感器动作标识
extern u8 Sensor_P3_new_state;  //切刀传感器动作标识
extern u8 Sensor_P3_old_state;  //切刀传感器动作标识


//extern u8 Sensor_P4_flag;       //切刀传感器动作标识
extern u8 Sensor_P4_new_state;  //切刀传感器动作标识
extern u8 Sensor_P4_old_state;  //切刀传感器动作标识


extern u8 Sensor_P5_flag_start;   //标识是否需要进行出片检测
extern u8 Sensor_P5_flag;         //出片口传感器  从高--变低--变高  表示胶片正常输出。  1 表示正常输出
extern u8 Sensor_P5_f10;          //出片口传感器  从高--变低 标识

extern u8 Sensor_P5_new_state;    //传感器当前状态
extern u8 Sensor_P5_old_state;    //传感器上一次状态

extern u8 Sensor_P5_OK_flag;    //传感器检测完成标志
extern u8 Sensor_P5_OK_flag_count;    //完成之后延时两秒计数



extern u8 Sensor_P6_flag;            //码盘传感标识 
extern u8 Sensor_P6_SendPaper_flag;  //码盘传感标识 
extern u8 Sensor_P6_new_state;
extern u8 Sensor_P6_old_state;

extern u16 Sensor_P6_flag_cont;     //加热步数计数器
extern u8 Sensor_P6_cont_start;     //加热行程开始计数标识
extern u16 Sensor_P6_SendPaper_flag_cont; //加热步数计数器
extern u16 SendPaper_P6_Time_Flag_start; // 送纸超时计数开始标志
 

extern volatile u8 MOTO_DC1_Run_Time;  //电机运行时间
extern volatile u8 MOTO_DC2_Run_Time;  //电机运行时间

extern volatile	u32 DC1_Run_Time_count;  //定时器中PWM 计数值
extern volatile	u32 DC2_Run_Time_count;  //定时器中PWM 计数值


extern volatile	u8 PWM1count;  //定时器中PWM 计数值
extern volatile	u8 PWM2count;  //定时器中PWM 计数值

extern volatile	u8 PWM1time;  //PWM 占空比值   
extern volatile	u8 PWM2time;  //PWM 占空比值  

extern volatile	u8 PWMDC1Direction;  //电机1 方向标志  1 正转  2 反转
extern volatile	u8 PWMDC2Direction;  //电机2 方向标志  1 正转  2 反转

extern volatile	u8 DC1Run;  //电机1 方向标志  1 正转  2 反转
extern volatile	u8 DC2Run;  //电机2 方向标志  1 正转  2 反转

extern volatile  u8 Heat_Time;  //胶片留在通道中的加热时间。 单位 秒
extern volatile  u8 Heat_Time_Flag;  //胶片加热完成标志
extern volatile  u8 Heat_Time_Flag_start;  //胶片开始加热计时标志

extern volatile	u8 MOTO2_LH_Control;  //电机2 离合控制
extern volatile	u8 MOTO2_LH_Flag;  //电机2 离合控制


extern volatile  u32 JiuXu_Count;  //胶片加热完成标志
extern volatile  u8 JiuXu_Count_start;  //胶片加热完成标志

extern volatile	u8 Paper_Send_OK;   //卡片标志


extern u8 sys_init_state; //系统初始化完成标志


extern volatile	s16 temputer1;
extern volatile	s16 temputer2;

extern volatile	s16 Heart_Open;   //开启心跳标志
extern volatile	u8  Heart_flag;   //无心跳标志
extern volatile	u32 Heart_Count;  //心跳超时计数



extern volatile  u32 auto_paper_a4_Count;  //报告送纸延时计数
extern volatile  u32 auto_paper_a4_Count2;  //报告送纸计数--超时计数
extern volatile  u8 auto_paper_a4_Count_Flag;  //报告送纸标志
extern volatile  u8 auto_paper_a4_Max_Flag;  //报告超时标志


/************************定时器延时变量***********************/
/********************************************************************
函数功能：延时函数。
入口参数：无。
返    回：无。
备    注：无。
********************************************************************/
extern volatile	u8 Delay_start;  //定时器中PWM 计数值   占空比
extern volatile	u8 Delay_num;  //定时器中PWM 计数值   占空比
extern volatile	u8 Delay_count;  //定时器中PWM 计数值   占空比
extern volatile	u8 Delay_flag;  //定时器中PWM 计数值   占空比


/********************************************************************
函数功能：定时器0的中断服务程序。  5ms 一次中断
入口参数：无。
返    回：无。
备    注：中断标志位自动清除。
********************************************************************/
void Timer0ISR(void) interrupt 1
{	
	static u16 count=0;  //心跳灯计数	
	static u16 counttemp=998; //温度传感器计数
  static u16 count3=0;  //切刀检测传感器定时
	
  static u16 HeatCount=0;  //切刀检测传感器定时	
	
	static u16 Sensor_P2_f10_count=0;  //切刀检测传感器定时	
	
	count3++;
	count++;   
	counttemp++; 	

	
	//计数器自动重载
	if(count==200)
	{		//5ms一次中断，接收到200次中断即为1秒
		count=0;
		RunLedTog();		//每1秒闪烁一次LED				
	}	
	
	
	//计数器自动重载
	if(counttemp==1000)
	{		//5ms一次中断，接收到2000次中断即为5秒
		counttemp=0;
		HandleDs18b20ReadTemp();  //读取两路温湿度传感器数据 ，
			
	}	

	
		//需要进行离合控制	---离合传感器 1--P3  1--P4
		if(MOTO2_LH_Control==1)		 
		{
			   //离合关到位控制
			   if(MOTO2_LH_Flag==0) 
				 {
					 		Sensor_P3_old_state= Sensor_P3_new_state ;
							Sensor_P3_new_state=MOTO_S5S3();		//每20ms 读离合关传感器状态
							
							if((Sensor_P3_old_state==1) && (Sensor_P3_new_state==0) ) //从低变成高
							{
									MotoDc2Stop(); 
                  MOTO2_LH_Control=0;								
							}						
         }
					//离合开到位控制
					if(MOTO2_LH_Flag==1) 
				 {
					Sensor_P4_old_state= Sensor_P4_new_state ;
						
					Sensor_P4_new_state=MOTO_S4S4();		//每20ms 读离合关传感器状态
					
					if((Sensor_P4_old_state==1) && (Sensor_P4_new_state==0) ) //从低变成高
					{
							MotoDc2Stop(); 
              MOTO2_LH_Control=0;	   
					}
				}
    }
		//end of  离合控制
	
	
	
   /*
	//延时计数
		if(Delay_start==1)  //判断是否需要码盘计数
		{		
				Delay_count++ ;		
				if(Delay_count >= Delay_num)
				{
					Delay_flag=1;
				}						
    }	

  */
	

		
	
  //计数器自动重载
	if(count3==2)  //每20 ms 查询出片机构相关传感器状态，并置位相关标识
	{		//5ms一次中断，接收到4次中断即为20ms秒
		
		
		count3=0;	//计数器重载
		
		//起始切刀传感器状态处理
	  Sensor_P0_old_state=Sensor_P0_new_state;   //保存老的状态
	  Sensor_P0_new_state=MOTO_S8S0(); 
		if( (Sensor_P0_old_state==1) && (Sensor_P0_new_state==0)) // 从高变成低
		{
       		Sensor_P0_flag=1;      //切刀开始动作
    }
		
		//结束切刀传感器状态处理
		Sensor_P1_old_state= Sensor_P1_new_state ;		
		Sensor_P1_new_state=MOTO_S7S1();		
		if((Sensor_P1_old_state==0) && (Sensor_P1_new_state==1) ) //从低变成高
		{
       	Sensor_P1_flag=1;      //切刀结束动作
    }
		
		//判断通道入口是否有胶片	
		Sensor_P2_old_state= Sensor_P2_new_state ;		
		Sensor_P2_new_state=MOTO_S6S2();	
		
		//从高变低，胶片进入通道口
		if((Sensor_P2_old_state==1) && (Sensor_P2_new_state==0) ) //有胶片进入通道口
		{   
       	Sensor_P2_f10=1;
        Sensor_P2_flag=0;	 
        Sensor_P2_Moto_Stop_flag=0;		 //清除慢转停止标志	
    }
	
			//从低变高
		if((Sensor_P2_old_state==0) && (Sensor_P2_new_state==1) ) //有胶片离开通道入口
		{
       	Sensor_P2_f01=1; 
        Sensor_P2_flag=0;	
			  
			  //清除慢转计数标志
        Sensor_P2_f10=0;
        Sensor_P2_f10_count=0;
			
        Sensor_P2_Moto_Stop_flag=1;		 //清除慢转停止标志				
        			
    }
			
	
		
		//出片报告检测，判断是否正常出片 
		if(Sensor_P5_flag_start==1)  //需要进行出片口报告检测 
		{		
					Sensor_P5_old_state= Sensor_P5_new_state ; //保存旧状态			
					Sensor_P5_new_state=MOTO_S3S5();		//获取新状态	
		      //出片过程 传感器状态为 1----0----1
			    //首先判断 1--0
			    if((Sensor_P5_old_state==1) && (Sensor_P5_new_state==0) ) //从高变低
					{
						
						Sensor_P5_f10=1;  //胶片正在输出
							
				  }	
					
					if(Sensor_P5_f10==1)
					{				
						 
						if((Sensor_P5_old_state==0) && (Sensor_P5_new_state==1) )
						{	
       
              Sensor_P5_OK_flag=1;  //置完成标志  
							
            }

          }	 
					
    }	
			
		//胶片输送完成延时
		if(Sensor_P5_OK_flag==1)
		{
			Sensor_P5_OK_flag_count++;		
			//清超时标志
			SendPaper_P6_Time_Flag_start=0;  //关计数标志
		  Sensor_P6_SendPaper_flag=0;  // 清标志
			
							//计数器自动重载
					if(Sensor_P5_OK_flag_count==200)  //1s
					{	
						//此处延时两秒  等待胶片完全输出	
						Sensor_P5_OK_flag_count=0;
						Sensor_P5_OK_flag=0;
						//置标志
						Sensor_P5_flag=1;      //胶片输出完毕
						Sensor_P5_f10=0;
					}							
    }
		
				
		//码盘  1--P6 计数处理
		if(Sensor_P6_cont_start==1)  //判断是否需要码盘计数
		{		
					Sensor_P6_old_state= Sensor_P6_new_state ; //保存旧状态			
					Sensor_P6_new_state=MOTO_S2S6();		//获取新状态	
		      // 新旧状态不一致 计数器加1
					if( Sensor_P6_old_state != Sensor_P6_new_state) //转一圈 计数 四次
					{
	            Sensor_P6_flag_cont++ ;	
						  if(Sensor_P6_flag_cont == MAPAN_MAX_CONT)
							{
								Sensor_P6_flag_cont=0;
								Sensor_P6_flag=1;  //置码盘标识  联合出片口传感器判断是否卡片。
              }						
							
					}
    }	
		
			
		//码盘  超时卡片计数
		if(SendPaper_P6_Time_Flag_start==1)  //判断是否需要码盘计数
		{		
				Sensor_P6_SendPaper_flag_cont++ ;	
			
				if(Sensor_P6_SendPaper_flag_cont >= MAPAN_SEND_PAPER_CONT)
				{
					Sensor_P6_SendPaper_flag_cont=0;
					SendPaper_P6_Time_Flag_start=0;
					Sensor_P6_SendPaper_flag=1;  //置码盘标识  联合出片口传感器判断是否卡片。
				}						

    }	
					
	}	
	
	
		//慢转延时关闭
		if(Sensor_P2_new_state==0)  //有胶片进入开始计时
		{
			Sensor_P2_f10_count++;
			
			if(Sensor_P2_f10_count == 6000 )  //
			{	
         Sensor_P2_f10_count=0;				
				 Sensor_P2_Moto_Stop_flag=1;
      }		
			
    }


	// 电机运行时间控?
	DC1_Run_Time_count++;
	DC2_Run_Time_count++;
	
	if( DC1_Run_Time_count >= (MOTO_DC1_Run_Time*200))
	{
		DC1Run=0;
		MOTO_DC1_Run_Time=0;
		DC1_Run_Time_count=0;
		MotoDc1Stop();
  }
	
		if( DC2_Run_Time_count >= (MOTO_DC2_Run_Time*200))
	{
		DC2Run=0;
		MOTO_DC2_Run_Time=0;
		DC2_Run_Time_count=0;
		
		MotoDc2Stop();
  }
	

	//胶片停留加热时间控制
		if(Heat_Time_Flag_start==1)
		{   
			 	HeatCount++;
			
				if(HeatCount == 600) // 2000--10s  600--3S
				{
					HeatCount=0;
					
					Heat_Time_Flag=1;
					
					Heat_Time_Flag_start=0;			
				}
    }	
	  
	 	//胶片正常输出到 就绪 时间
		if(JiuXu_Count_start==1)
		{   
			 	JiuXu_Count++;
			
				if(JiuXu_Count >= 1600) // 2000--10s  600--3S
				{
					JiuXu_Count=0;
					
					Paper_Send_OK=1;   //置就绪标志
					//测试语句
					HeatFanOff(); //关加热风扇
					JiuXu_Count_start=0;			
				}
   }	
	 
	 
	 	 	//心跳控制
		if(Heart_Open==1)
		{   
			  Heart_Count++;
			
				//if(Heart_Count >= 20000) // 1000S  16分钟
				if(Heart_Count >= 200000) // 1000S  16分钟	
				{
					
					Heart_flag=0;   
							
				}
   }
	 
	 //送报告电机延时计数
	 if(auto_paper_a4_Count_Flag==1)
	 {
        auto_paper_a4_Count++;
		 
		    if(auto_paper_a4_Count==1600)  //5秒关闭
				{
            auto_paper_a4_Count=0;
					
					  auto_paper_a4_Count_Flag=0;
        }

   }
  
   /*	 
	 //送报告电机超时计数
	  //送报告电机延时计数
	 if(BaoGaoLed==1)  //开启了报告灯，或者开启了电机
	 {
        auto_paper_a4_Count2++;
		    if(auto_paper_a4_Count2 > 3000)  //15S 超时
				{
					  auto_paper_a4_Max_Flag=1;
					  auto_paper_a4_Count2=0;
					  BaoGaoLedOff(); //关电机
        }
   }
	  
	 */
}

/**///////////////////////Pino Electronics////////////////////////**/

/********************************************************************
函数功能：初始化定时器2，每10ms产生一个中断。
入口参数：无。
返    回：无。
备    注：Timer2的工作模式只有一种：16位自动重载模式。
********************************************************************/
/*
void Timer2Init(void)		//1毫秒@11.0592MHz
{
	AUXR |= 0x04;		//定时器时钟1T模式
	T2L = 0xCD;		//设置定时初值
	T2H = 0xD4;		//设置定时初值
	
	IE2|=0x04;  	//Timer2中断使能
	AUXR |= 0x10;		//定时器2开始计时
}

*/


void Timer2Init(void)		//100微秒@11.0592MHz
{
	AUXR |= 0x04;		//定时器时钟1T模式
	T2L = 0xAE;		//设置定时初值
	T2H = 0xFB;		//设置定时初值
	IE2|=0x04;  	//Timer2中断使能
	AUXR |= 0x10;		//定时器2开始计时
}


/********************************************************************
函数功能：定时器2的中断服务程序。
入口参数：无。
返    回：无。
// 在定时器2中做电机PWM输出控制
//主要控制电机占空比等内容。
备    注：中断标志位自动清除。
********************************************************************/
//100微秒@11.0592MHz

//电机控制程序，在定时器2中进行 
void Timer2ISR(void) interrupt 12
{	
	// 采用PWM 控制后不在这里处理
	
}
/**///////////////////////Pino Electronics////////////////////////**/JM













