/************************************************************************************
*
* CPU：STC15W4K3S4核心板
* 时钟选择：外部时钟11.0592MHz
* 功能说明：与上位机通信程序
* 作者信息：张 斌
*************************************************************************************
*按照通信协议，与上位机通信
*
*
************************************************************************************/

#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "moto.h"
#include "comwithpc.h"

//接收数据在串口中断进行，此处只对接收到的数据进行处理
//使用到的外部变量
//extern volatile u8 uart1SendingFlag;	 	  //全局变量，串口是否有数据要发送标志

extern volatile u8 uart1ReceivedFlag;		//全局变量，标志串口已接收到数据

extern volatile u8 uart1ReceiveNum;		  //全局变量，标志串口已接收到数据计数

extern volatile u8 uart1SBuf[UART1_RBUFSIZE];//发送缓冲区，static标识，只在本文件中可以访问

extern volatile u8 uart1rBuf[UART1_RBUFSIZE];//接收缓冲区，static标识，只在本文件中可以访问

extern volatile	u8 Paper_Send_OK;   //卡片标志

extern volatile	s16 Heart_Open;   //开启心跳标志
extern volatile	u8  Heart_flag;   //无心跳标志
extern volatile	u32 Heart_Count;  //心跳超时计数


extern u8 IPF510MT_PB;
extern u8 C7280MT_PB;
//废墨检测
extern u8 IPF510FM_PB;
extern u8 C7280FM_PB;


/**************************************************************************
*校验函数，按照通信协议计算校验位返回校验位的值
*
*输入：需要计算数据的首地址，数据长度
*输出：校验值
*
***************************************************************************/

u8 CRC_value(u8 *p,u8 n)
{
  u8 num1=0,num0=0;
  u8 value0=0;
  for(num1=n;num1>0;num1--)	       
  {
    num0+=*p++;
  }
  value0=num0%256;
  return value0;
}

//生成返回的数据包
//{0x55,0x05,0x00,0x00,0x00,0xfe,0x02,0x01,0xff,0xff,0x00,0xff,0xff,0x0d};	
/**************************************************************************
*校验函数，按照通信协议计算校验位返回校验位的值
*
*输入：数据域长度低位  ，数据长度高位 ，功能码,子功能码 
*输出：校验值
*
***************************************************************************/

void make_data_bus(u8 datal,u8 datah)
{
	u8 i;
	
	uart1SBuf[0]=DATAHEAD;
	
	uart1SBuf[1]=datal; //数据域长度低位
	
	uart1SBuf[2]=datah; //数据长度高位
	
	uart1SBuf[3]=ADDRESSL; //同类板地址L
	
	uart1SBuf[4]=ADDRESSH; //同类板地址H
	
	uart1SBuf[5]=DATADIR; //传送方向
	
	uart1SBuf[6]=uart1rBuf[6]; //功能码
	
  for(i=0;i<4;i++)
	{
   	uart1SBuf[7+i]=uart1rBuf[7+i]; //子功能码 数据域第一个字节
  }
	
	uart1SBuf[(7+datal+datah+1)]=DATAEND; //结束符
	
}

extern volatile	s16 temputer1;
extern volatile	s16 temputer2;

u8 sys_init_state=1; //初始化状态标志

void comwithpc(void)
{
		u8  CRC;  //用于计算校验位
	
    WDT_CONTR |= 0x10;      //喂狗
	
	if(uart1ReceivedFlag==1)  //接收到数据才进行处理
	{
		
		if( (uart1rBuf[0]==0x55)&&(uart1rBuf[5]==0xFF) && (uart1rBuf[uart1ReceiveNum-1]==0x0d) )//判断首尾位以及方向字节是否正确  
		{
			//正确之后才计算校验位
			CRC=CRC_value(uart1rBuf,uart1ReceiveNum-2);
			
			if(CRC==uart1rBuf[uart1ReceiveNum-2])  //校验正确后对数据进行处理
			{
				//uart1senddata(uart1rBuf,uart1rBuf[uart1ReceiveNum-1]);//2
				
				switch(uart1rBuf[6]) //对功能码进行分支处理
        {					
					case 0x00 :		
					
					//未知指令
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //功能码		
					 uart1SBuf[7]=0x00;//功能码	
					 uart1SBuf[8]=0x00;//功能码	
					 uart1SBuf[9]=0x00;//功能码	
					 uart1SBuf[10]=0x00;//功能码						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);					
		
					  break;
						
					//心跳检测
					case 0x06 :
						
					   switch(uart1rBuf[9])
             {
							  //检测
							  case 0x01 :
									
								 //开启器心跳检测
								  Heart_Open=1;
								  Heart_Count=0;  //心跳喂狗
								  Heart_flag=1;
									
									make_data_bus(0x05,0x00);
								
						  		uart1SBuf[11]=0x00;  // 0表示成功
									//计算校验
				       	  uart1SBuf[12]=CRC_value(uart1SBuf,12);	
								  
								  uart1senddata(uart1SBuf,14);
								 break;	
								
								//屏蔽
								case 0x02 :
									
								  //关闭心跳检测
								  Heart_Open=0;
								
								  Heart_Count=0;  //心跳喂狗
								 
								  Heart_flag=1;  //清标志
									
									make_data_bus(0x05,0x00);
						  		uart1SBuf[11]=0x00;  // 0表示成功
        					//计算校验
				       	  uart1SBuf[12]=CRC_value(uart1SBuf,12);		  
								  uart1senddata(uart1SBuf,14);
									
								 break;	
	
								default :
									
								 //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
									
								 break;


               }
						
					  break;
							
					//系统复位
					
					//55 04 00 00 00 FF 04 02 02 00 00 60 0D
					
					case 0x04 :
						
					  if(uart1rBuf[7]==0x02)
						{

						 make_data_bus(0x05,0x00);		

					   uart1SBuf[11]=0x00;  // 0表示成功
											//计算校验
				  	 uart1SBuf[12]=CRC_value(uart1SBuf,12);
					
					   uart1senddata(uart1SBuf,14);	
	
					   IAP_CONTR= 0x20 ;  //系统软件复位
					 }

					  break;
					 	 
					 //清除卡纸状态
					 case 0x07 :
						
					  if(uart1rBuf[7]==0x01)
						{
						 make_data_bus(0x05,0x00);								
						 Paper_Send_OK=1;	    //清状态为就绪		
					   uart1SBuf[11]=0x00;  // 0表示成功
											//计算校验
				  	 uart1SBuf[12]=CRC_value(uart1SBuf,12);				
					   uart1senddata(uart1SBuf,14);	
					  }

					  break;
					 
						
					//查询数据
					case 0x01 :
						
             switch(uart1rBuf[7])
             {
							  //55 04 00 00 00 FF 01 01 00 00 00 5A 0D
							  //55 08 00 00 00 FE 01 01 00 00 00 00 03 10 10 00 00 
							  //查询 版本
							  case 0x01 :
									make_data_bus(0x08,0x00);
								
								  uart1SBuf[11]=HARDVER1;  //
								  uart1SBuf[12]=HARDVER2;  //
								  uart1SBuf[13]=FIREVER;   //
								  uart1SBuf[14]=SOFTVER;   //
								  //计算校验
								  uart1SBuf[15]=CRC_value(uart1SBuf,15);
								  
								 uart1senddata(uart1SBuf,17);
								 break;	
								
								 //查询 纸盒状态
							  case 0x02 :
									
									switch(uart1rBuf[8]) //条件分支，  02---7280
										{
											
											case 0x01 :
												
								         break;	
											
											//7280
											case 0x02 :
												
										  make_data_bus(0x06,0x00);
											
											uart1SBuf[11]=0x01;  // 表示1 个纸盒
											
											uart1SBuf[12]=0x01;  // 00000001 表示无纸张
											
											//无纸低，有纸高
											
											if(MOTO_S1S7()==1)
											{
                        uart1SBuf[12]=0x00; //返回表示有纸张
                      }	
											
										  uart1SBuf[13]=CRC_value(uart1SBuf,13);	
								      uart1senddata(uart1SBuf,15);	
											
								         break;	
										}
									 break;		
								
								
								//查询墨盒状态
								case 03 :
									
										switch(uart1rBuf[8]) //条件分支，02 --7280  03---510
										{
											
											case 0x03 :
											  	
										  make_data_bus(0x06,0x00); 
											
										  uart1SBuf[11]=0x05;  // 表示5 个墨盒   位顺序 C  M Y BK  MBK 
											
											uart1SBuf[12]=0x00;
											
										//	判断是否启用了检测功能
										 if(IPF510MT_PB==0x02)  //如果屏蔽将不检测 直接上报 0x00
										 	{

													if(ReadIpf510_MBK()==1)
													{
														uart1SBuf[12]=uart1SBuf[12]|0x10;
													}	
													
													
													if(ReadIpf510_BK()==1)
													{
														uart1SBuf[12]=uart1SBuf[12]|0x08;
													}	
													
													if(ReadIpf510_Y()==1)
													{
														uart1SBuf[12]=uart1SBuf[12]|0x04;
													}		
													
													if(ReadIpf510_M()==1)
													{
														uart1SBuf[12]=uart1SBuf[12]|0x02;
													
													}	
													
													if(ReadIpf510_C()==1)
													{
														uart1SBuf[12]=uart1SBuf[12]|0x01;
														
													}	
											
											}	
											
											uart1SBuf[13]=CRC_value(uart1SBuf,13);
								      uart1senddata(uart1SBuf,15);	
															
											break;
											
											case 0x02 :
												
											make_data_bus(0x06,0x00); 
											
										  uart1SBuf[11]=0x05;  // 表示5 个墨盒   位顺序 C  M Y BK  MBK 
											
											uart1SBuf[12]=0x00;  //默认为有墨
											
											//判断是否启用了检测功能
											if(C7280MT_PB==0x02)  //如果屏蔽将不检测 直接上报 0x00
											{
											
												if(Read7280_MBK()==1)
												{
													uart1SBuf[12]=uart1SBuf[12]|0x10;
												}	
												if(Read7280_BK()==1)
												{
													uart1SBuf[12]=uart1SBuf[12]|0x08;
												}	
												if(Read7280_Y()==1)
												{
													uart1SBuf[12]=uart1SBuf[12]|0x04;
												}	
												if(Read7280_M()==1)
												{
													uart1SBuf[12]=uart1SBuf[12]|0x02;
												}	
												if(Read7280_C()==1)
												{
													uart1SBuf[12]=uart1SBuf[12]|0x01;
												}	
											
									  	}
											uart1SBuf[13]=CRC_value(uart1SBuf,13);
								      uart1senddata(uart1SBuf,15);	
															
											break;
											
											default :
												
											 //未知指令
											 make_data_bus(0x05,0x00);		
											 uart1SBuf[6]=0xFF; //功能码		
											 uart1SBuf[7]=0x00;//功能码	
											 uart1SBuf[8]=0x00;//功能码	
											 uart1SBuf[9]=0x00;//功能码	
											 uart1SBuf[10]=0x00;//功能码						
											 uart1SBuf[11]=OTHERERROR;  // 
											 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
											 uart1senddata(uart1SBuf,14);
																			
											break;

										}				
							    	break;	
								

					    //传感器状态
							case 0x04 :
								
							
										switch(uart1rBuf[8]) //条件分支，03 --7280  02---510
										{
											
											//出片通道传感器状态
											case 0x02 :
											  	
										  make_data_bus(0x06,0x00); 
											
										  uart1SBuf[11]=0x07;  // 表示 7个传感器
											
											uart1SBuf[12]=0x00;  //首先都置0
											
											//读传感器值
											if(MOTO_S8S0()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x01;
                      }	
											if(MOTO_S7S1()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x02;
                      }	
											if(MOTO_S6S2()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x04;
                      }	
											if(MOTO_S5S3()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x08;
                      }	
											if(MOTO_S4S4()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x10;
                      }
                      if(MOTO_S3S5()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x20;
                      }	
											if(MOTO_S2S6()==1)
											{
                        uart1SBuf[12]=uart1SBuf[12]|0x00;  //计数不做处理统一报0
                      }	
						
											uart1SBuf[13]=CRC_value(uart1SBuf,13);
								      uart1senddata(uart1SBuf,15);	
															
											break;

											default :
												
											 //未知指令
											 make_data_bus(0x05,0x00);		
											 uart1SBuf[6]=0xFF; //功能码		
											 uart1SBuf[7]=0x00;//功能码	
											 uart1SBuf[8]=0x00;//功能码	
											 uart1SBuf[9]=0x00;//功能码	
											 uart1SBuf[10]=0x00;//功能码						
											 uart1SBuf[11]=OTHERERROR;  // 
											 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
											 uart1senddata(uart1SBuf,14);
																			
											break;

										}	
								
							break;
													
								
						  // 查询温度
								case 0x08 :
								
								//查询加热温度 
								
                //读温度	

	              switch(uart1rBuf[8])  //加热器类别
								{
									 case  0x01 :   //整机温度
										 
								   break;
									
									 case  0x02 :   //整机温度
										 
									 	make_data_bus(0x06,0x00);  //温度 六字节 
                    //温度在定时器中读取，每10秒读取一次									 
									  uart1SBuf[11] = temputer2;  //  温度，低位在前
									  uart1SBuf[12] = temputer2 >> 8;  //  温度，高位在后								 
									  uart1SBuf[13]=CRC_value(uart1SBuf,13);
								    uart1senddata(uart1SBuf,15);
				 
								   break;
									
									 case  0x03 :   //加热器温度状态
										 
								   break;
									 
									 case  0x04 :   //整机温度状态
										 
								   break;
									
	 
                 default : 
									 
								 //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
								 break;
								 	
                }//end  查询温度
                break;									
								
								
								//查询 初始化状态  数据长度5
							  case 0x09 :
									
									make_data_bus(0x05,0x00);
								
								  uart1SBuf[11]=sys_init_state;  //直接上报初始化状态
								  
								  //计算校验
								  uart1SBuf[12]=CRC_value(uart1SBuf,12);
								  
								  uart1senddata(uart1SBuf,14);
								
								 break;	
								
								
								//查询加热器状态
							  case 0x0B :
									
									make_data_bus(0x05,0x00);
								 
								 if(Heat==0) 
                 { 
                  uart1SBuf[11]=1;  //1 表示加热器开 
                 }	
                 else
								 {   
									 uart1SBuf[11]=0;  //1 表示加热器关
                 }
               								 
								  //计算校验
								  uart1SBuf[12]=CRC_value(uart1SBuf,12);
								  
								  uart1senddata(uart1SBuf,14);
								
								 break;	
								
								
								case 0x0e :
									
											 switch(uart1rBuf[8])  //废墨类别
											{
												 case  0x01 :   //451
												 //未知指令
												 make_data_bus(0x05,0x00);		
												 uart1SBuf[6]=0xFF; //功能码		
												 uart1SBuf[7]=0x00;//功能码	
												 uart1SBuf[8]=0x00;//功能码	
												 uart1SBuf[9]=0x00;//功能码	
												 uart1SBuf[10]=0x00;//功能码						
												 uart1SBuf[11]=OTHERERROR;  // 
												 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
												 uart1senddata(uart1SBuf,14);
													 
												 break;
												
												 case  0x02 :   //7280
													
												 	make_data_bus(0x05,0x00);	
												 
												 uart1SBuf[11]=0x00;  //高 ，未满
												 
												 if(C7280FM_PB==0x02)  //如果屏蔽将不检测 直接上报 0x00
													{

															if(Read7280_FM()==1)  //无墨 为 高，  有墨 为低 ，低为 废墨满
															{
																uart1SBuf[11]=0x00;  //高 ，未满
															}	
															else
															{
																uart1SBuf[11]=0x01;  //高 ，未满
																
															}
															
													}		
													//计算校验
													uart1SBuf[12]=CRC_value(uart1SBuf,12);
													uart1senddata(uart1SBuf,14);

												 break;
												
												 case  0x03 :   //510
													 
												 	make_data_bus(0x05,0x00);	
												 
												  uart1SBuf[11]=0x00;  //高 ，未满
												 
													if(IPF510FM_PB==0x02)  //如果屏蔽将不检测 直接上报 0x00
													{

														if(ReadIpf510_FM()==1)  //无墨 为 高，  有墨 为低 ，低为 废墨满
														{
															uart1SBuf[11]=0x00;  //高 ，未满
														}	
														else
														{
															uart1SBuf[11]=0x01;  //高 ，未满
															
														}
													}
													//计算校验
													uart1SBuf[12]=CRC_value(uart1SBuf,12);
													uart1senddata(uart1SBuf,14);
													 
												 break;
												 
												 case  0x04 :   //5100
													 
												 //未知指令
												 make_data_bus(0x05,0x00);		
												 uart1SBuf[6]=0xFF; //功能码		
												 uart1SBuf[7]=0x00;//功能码	
												 uart1SBuf[8]=0x00;//功能码	
												 uart1SBuf[9]=0x00;//功能码	
												 uart1SBuf[10]=0x00;//功能码						
												 uart1SBuf[11]=OTHERERROR;  // 
												 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
												 uart1senddata(uart1SBuf,14);
													 
												 break;
												
				 
											 default : 
												 
											  //未知指令
											 make_data_bus(0x05,0x00);		
											 uart1SBuf[6]=0xFF; //功能码		
											 uart1SBuf[7]=0x00;//功能码	
											 uart1SBuf[8]=0x00;//功能码	
											 uart1SBuf[9]=0x00;//功能码	
											 uart1SBuf[10]=0x00;//功能码						
											 uart1SBuf[11]=OTHERERROR;  // 
											 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
											 uart1senddata(uart1SBuf,14);
													 break;
												
											}
								break;
							//end  废墨查询
						
              //查询出片状态											
						    case 0x0F :	
									
									make_data_bus(0x05,0x00);	
									uart1SBuf[11]=Paper_Send_OK;  //高 ，未满

									//计算校验
									uart1SBuf[12]=CRC_value(uart1SBuf,12);
									uart1senddata(uart1SBuf,14);
									break;		
								
								default :
									
								 //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
									
									break;

             }//end  uart1rBuf[7]-数据域第一个数据 判断查询具体内容					
											
					  break;
					//控制指令
						 
					case 0x02 :
						
					 // uart1senddata(uart1rBuf,uart1rBuf[uart1ReceiveNum-1]);//3
					    switch(uart1rBuf[7])
             {
							  //控制加热器
							 case 0x03 :		
							 
								   if(uart1rBuf[9]==0x01)    //开加热器
									 {										 
										  HeatOn();     //低电平开
										 
                     if(Heat==0x00)  //判断是否开成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	
                     
										 make_data_bus(0x05,0x00);					
										//计算校验
										uart1SBuf[12]=CRC_value(uart1SBuf,12);
										uart1senddata(uart1SBuf,14);
									  break;												 
                   }		

									 if(uart1rBuf[9]==0x02)    //关加热器
									 {
										 
										 HeatOff();   //高电平关
										 
                     if(Heat==1)  //判断是否关成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	

											make_data_bus(0x05,0x00);					
											//计算校验
											uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
											 
										 break;											 
                   }

									 break;	

							   //end 加热器控制
									 
								//加热器风扇控制 
								case 0x07 :	
									
								   if(uart1rBuf[9]==0x01)    //开加热器风扇
									 {
										 
										 HeatFanOn();     //低电平开
										 
                     if(HeatFan==0)  //判断是否开成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
											 
                     }	 	

											make_data_bus(0x05,0x00);					
											//计算校验
											uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 if(uart1rBuf[9]==0x02)    //关加热器风扇
									 {
										 
										 HeatFanOff();   //高电平关
										 
                     if(HeatFan==1)  //判断是否关成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	 	

										 make_data_bus(0x05,0x00);					
								    //计算校验
								    uart1SBuf[12]=CRC_value(uart1SBuf,12);
								    uart1senddata(uart1SBuf,14);
								    break;	
 
                   }
									 break;
                //end 风扇控制	 
 
								//导光灯控制
                //加热器风扇控制 
								case 0x01 :
									
								switch(uart1rBuf[8])  //灯类型入口
								{
									 case  0x01 :   //导光灯
										 
									 if(uart1rBuf[9]==0x01)    //开加导光灯
									 {
										 
										 DaoGuangLedOn();     //高电平开
										 
                     if(DaoGuangLed==1)  //判断是否开成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	

 
								    make_data_bus(0x05,0x00);					
								    //计算校验
								    uart1SBuf[12]=CRC_value(uart1SBuf,12);
								    uart1senddata(uart1SBuf,14);
								    break;	
										 
                   }
									
									 
									 if(uart1rBuf[9]==0x02)    //关导光灯
									 {
										 
										 DaoGuangLedOff();   //低电平关
										 
                     if(DaoGuangLed==0)  //判断是否关成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	 

											make_data_bus(0x05,0x00);					
								      //计算校验
								      uart1SBuf[12]=CRC_value(uart1SBuf,12);
								      uart1senddata(uart1SBuf,14);
								      break;											 
                   }
									 
								
							   //end 导光灯控制
									 
								 //报告灯控制	 
								 	case  0x02 :  //报告口灯
										
									 if(uart1rBuf[9]==0x01)    //开报告灯
									 {
										 
										 BaoGaoLedOn();     //高电平开
										 
                     if(BaoGaoLed==1)  //判断是否开成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	


									  	make_data_bus(0x05,0x00);					
								     //计算校验
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 									 
									 if(uart1rBuf[9]==0x02)    //关报告灯
									 {
										 
										 BaoGaoLedOff();   //低电平关
										 
                     if(BaoGaoLed==0)  //判断是否关成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	 

										  make_data_bus(0x05,0x00);					
											//计算校验
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 
									 //未知指令
										 make_data_bus(0x05,0x00);		
										 uart1SBuf[6]=0xFF; //功能码		
										 uart1SBuf[7]=0x00;//功能码	
										 uart1SBuf[8]=0x00;//功能码	
										 uart1SBuf[9]=0x00;//功能码	
										 uart1SBuf[10]=0x00;//功能码						
										 uart1SBuf[11]=OTHERERROR;  // 
										 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
										 uart1senddata(uart1SBuf,14);
										 break;
									 
									
							   //end 导光灯控制
									 
									 
									 
                 default : 
									 
								  //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
								 break;
								 	
                }//end  uart1rBuf[8]-数据域第一个数据 判断控制内容	
                break;
                
								//电机控制
								case 0x0C :			
								switch(uart1rBuf[8])  //电机类型
								{
									 case  0x01 :  

                   //未知指令
									 make_data_bus(0x05,0x00);		
									 uart1SBuf[6]=0xFF; //功能码		
									 uart1SBuf[7]=0x00;//功能码	
									 uart1SBuf[8]=0x00;//功能码	
									 uart1SBuf[9]=0x00;//功能码	
									 uart1SBuf[10]=0x00;//功能码						
									 uart1SBuf[11]=OTHERERROR;  // 
									 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
									 uart1senddata(uart1SBuf,14);									 
										 
									 break;
									 	 
								 //出片点击
								 	case  0x02 :  //出片电机
										
									//55 04 00 00 00 FF 02 0C 02 01 00 69 0D
										
									 if(uart1rBuf[9]==0x01)    //正转
									 {
										 
										 MotoDc1Run(100,FORWARD,10); // 1/2占空比 正转5S 	  全速
										 
                     uart1SBuf[11]=0x00;  // 0表示成功

									  	make_data_bus(0x05,0x00);					
								     //计算校验
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 									 
									 if(uart1rBuf[9]==0x02)    //停止
									 {
										 
										  MotoDc1Stop();     
										 // MotoDc2Stop();

                       uart1SBuf[11]=0;  // 1表示失败
										 
										  make_data_bus(0x05,0x00);					
											//计算校验
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 break;

								  //离合电机
									case 0x03 :
										
									if(uart1rBuf[9]==0x01)    //正转
									 {
										 
										 MotoDc2Run(30,FORWARD,10); //100占空比 正转5S 	  全速
										 
                     uart1SBuf[11]=0x00;  // 0表示成功

									  	make_data_bus(0x05,0x00);					
								     //计算校验
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 									 
									 if(uart1rBuf[9]==0x02)    //停止
									 {
										 
										  //MotoDc1Stop();     
										    MotoDc2Stop();

                       uart1SBuf[11]=0;  // 0表示成功
										 
										  make_data_bus(0x05,0x00);					
											//计算校验
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 break;
									 
                 default :
									  //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
								 break;
								 	
                }//end  uart1rBuf[8]-数据域第一个数据 判断控制内容	
									 //end 电机控制
								break;
								//蜂鸣器控制
								case  0x0D :
									
								switch(uart1rBuf[8])
								{
									
									case 0x00 :
									
									if(uart1rBuf[9]==0x01)    //开蜂鸣器
									 {
										 
										 
										 BuzzerOn();   //低电平关
										 
                     if(Buzzer==1)  //判断是否关成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }	 

									  	make_data_bus(0x05,0x00);					
								     //计算校验
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 
									 if(uart1rBuf[9]==0x02)    //关蜂鸣器
									 {
										 
										 BuzzerOff();   //低电平关
										 
                     if(Buzzer==0)  //判断是否关成功
										 {
											 uart1SBuf[11]=0;  // 0表示成功
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1表示失败
                     }
										 
										  make_data_bus(0x05,0x00);					
											//计算校验
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 break;
								 
								default : 
								 //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14); 
							 
								 break;

                }  //end  buzzer  control
								
								break;
								
								default :
									
								 //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
									break;
             }//end  uart1rBuf[7]-数据域第一个数据 判断控制内容		
						 break;
						 
						 
					//系统清零
					case 0x03 :
						
												//未知指令
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //功能码		
					 uart1SBuf[7]=0x00;//功能码	
					 uart1SBuf[8]=0x00;//功能码	
					 uart1SBuf[9]=0x00;//功能码	
					 uart1SBuf[10]=0x00;//功能码						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
			
					  break;
					
					//0x05  屏蔽
					case 0x05 :
						
					if(uart1rBuf[7]==0x01)  // 类别为墨水
					{	
						
							 switch(uart1rBuf[8])  //墨水类别  0X01-451  0X02--7280  0X03---510  0X04-5100  0X05--废墨
							 {
								 
									//屏蔽墨水
									case 0x03 :   //屏蔽510
										
									 if(uart1rBuf[9]==0x01)  // 屏蔽
									 {
										 IPF510MT_PB=0x01;

                   }
                   if(uart1rBuf[9]==0x02)  // 启用
									 {
										 IPF510MT_PB=0x02;

                   }										 
										make_data_bus(0x05,0x00);
									
										uart1SBuf[11]=0x00;  // 0表示成功
										//计算校验
										uart1SBuf[12]=CRC_value(uart1SBuf,12);	
										
										uart1senddata(uart1SBuf,14);
									 break;	
									
									//屏蔽墨水
									case 0x02 :
										
									  if(uart1rBuf[9]==0x01)  // 屏蔽
									 {
										 C7280MT_PB=0x01;

                   }
                   if(uart1rBuf[9]==0x02)  // 启用
									 {
										 C7280MT_PB=0x02;

                   }	
										make_data_bus(0x05,0x00);
									
										uart1SBuf[11]=0x00;  // 0表示成功
										//计算校验
										uart1SBuf[12]=CRC_value(uart1SBuf,12);	
										
										uart1senddata(uart1SBuf,14);
									 break;	
									
									//屏蔽墨水
									case 0x05 :
										
									 if(uart1rBuf[9]==0x01)  // 屏蔽
									 {
									  IPF510FM_PB=0x01;
                    C7280FM_PB=0x01;		

                   }
                   if(uart1rBuf[9]==0x02)  // 启用
									 {
									  IPF510FM_PB=0x02;
                    C7280FM_PB=0x02;		
                   }	
									 
										make_data_bus(0x05,0x00);
									
										uart1SBuf[11]=0x00;  // 0表示成功
										//计算校验
										uart1SBuf[12]=CRC_value(uart1SBuf,12);	
										
										uart1senddata(uart1SBuf,14);
									 break;	
									

									default :
										
								 //未知指令
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //功能码		
								 uart1SBuf[7]=0x00;//功能码	
								 uart1SBuf[8]=0x00;//功能码	
								 uart1SBuf[9]=0x00;//功能码	
								 uart1SBuf[10]=0x00;//功能码						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
										
									 break;

								 } 	


								 
					 }		
					 break;
					


					//确认/否认
					case 0xFC :
						
					//未知指令
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //功能码		
					 uart1SBuf[7]=0x00;//功能码	
					 uart1SBuf[8]=0x00;//功能码	
					 uart1SBuf[9]=0x00;//功能码	
					 uart1SBuf[10]=0x00;//功能码						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
						
					  break;
					
					//升级更新
					case 0xFD :
					 //未知指令
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //功能码		
					 uart1SBuf[7]=0x00;//功能码	
					 uart1SBuf[8]=0x00;//功能码	
					 uart1SBuf[9]=0x00;//功能码	
					 uart1SBuf[10]=0x00;//功能码						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);	
					  break;
					//主动上报
					case 0xFE :
				 	//未知指令
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //功能码		
					 uart1SBuf[7]=0x00;//功能码	
					 uart1SBuf[8]=0x00;//功能码	
					 uart1SBuf[9]=0x00;//功能码	
					 uart1SBuf[10]=0x00;//功能码						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
						
					  break;
					//错误通知
					case 0xFF :
					 //未知指令
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //功能码		
					 uart1SBuf[7]=0x00;//功能码	
					 uart1SBuf[8]=0x00;//功能码	
					 uart1SBuf[9]=0x00;//功能码	
					 uart1SBuf[10]=0x00;//功能码						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
							
				  break;
					
					default :	
					
				 //未知指令
			   make_data_bus(0x05,0x00);		
         uart1SBuf[6]=0xFF; //功能码		
			 	 uart1SBuf[7]=0x00;//功能码	
				 uart1SBuf[8]=0x00;//功能码	
				 uart1SBuf[9]=0x00;//功能码	
				 uart1SBuf[10]=0x00;//功能码						
			   uart1SBuf[11]=OTHERERROR;  // 
			 	 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
			   uart1senddata(uart1SBuf,14);
				 break;

       }//end SWITCH()    

      }//end if(CRC==uart1rBuf[uart1ReceiveNum-2])
			else
			{
				//校验没过 返回02
			  	make_data_bus(0x05,0x00);
				 uart1SBuf[6]=0xFF;//功能码	
				 uart1SBuf[7]=0x00;//功能码	
				 uart1SBuf[8]=0x00;//功能码	
				 uart1SBuf[9]=0x00;//功能码	
				 uart1SBuf[10]=0x00;//功能码		
			   uart1SBuf[11]=CRCERROR;  // 
			   uart1SBuf[12]=CRC_value(uart1SBuf,12);  
				 uart1senddata(uart1SBuf,14);
      }
    }// end  0x55  0xff  0x0d	
		else
		{		
			//头尾没过  返回03
		 make_data_bus(0x05,0x00);
			
	   uart1SBuf[6]=0xFF;//功能码	
	   uart1SBuf[7]=0x00;//功能码	
		 uart1SBuf[8]=0x00;//功能码	
		 uart1SBuf[9]=0x00;//功能码	
		 uart1SBuf[10]=0x00;//功能码	
			
			if(uart1rBuf[0]!=0x55)
			{	
		   	uart1SBuf[11]=HEADERROR;  //
			}	
			
			if(uart1rBuf[uart1ReceiveNum-1]!=0x0d)
			{					
		    uart1SBuf[11]=ENDERROR;  // 
			}	
		 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
		 uart1senddata(uart1SBuf,14);
    }
  //清除相关标志		
	uart1ReceiveNum=0;
  uart1ReceivedFlag=0;		

  }//END if(uart1ReceivedFlag==1) 
}


