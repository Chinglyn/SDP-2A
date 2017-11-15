/************************************************************************************
*
* CPU��STC15W4K3S4���İ�
* ʱ��ѡ���ⲿʱ��11.0592MHz
* ����˵��������λ��ͨ�ų���
* ������Ϣ���� ��
*************************************************************************************
*����ͨ��Э�飬����λ��ͨ��
*
*
************************************************************************************/

#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "moto.h"
#include "comwithpc.h"

//���������ڴ����жϽ��У��˴�ֻ�Խ��յ������ݽ��д���
//ʹ�õ����ⲿ����
//extern volatile u8 uart1SendingFlag;	 	  //ȫ�ֱ����������Ƿ�������Ҫ���ͱ�־

extern volatile u8 uart1ReceivedFlag;		//ȫ�ֱ�������־�����ѽ��յ�����

extern volatile u8 uart1ReceiveNum;		  //ȫ�ֱ�������־�����ѽ��յ����ݼ���

extern volatile u8 uart1SBuf[UART1_RBUFSIZE];//���ͻ�������static��ʶ��ֻ�ڱ��ļ��п��Է���

extern volatile u8 uart1rBuf[UART1_RBUFSIZE];//���ջ�������static��ʶ��ֻ�ڱ��ļ��п��Է���

extern volatile	u8 Paper_Send_OK;   //��Ƭ��־

extern volatile	s16 Heart_Open;   //����������־
extern volatile	u8  Heart_flag;   //��������־
extern volatile	u32 Heart_Count;  //������ʱ����


extern u8 IPF510MT_PB;
extern u8 C7280MT_PB;
//��ī���
extern u8 IPF510FM_PB;
extern u8 C7280FM_PB;


/**************************************************************************
*У�麯��������ͨ��Э�����У��λ����У��λ��ֵ
*
*���룺��Ҫ�������ݵ��׵�ַ�����ݳ���
*�����У��ֵ
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

//���ɷ��ص����ݰ�
//{0x55,0x05,0x00,0x00,0x00,0xfe,0x02,0x01,0xff,0xff,0x00,0xff,0xff,0x0d};	
/**************************************************************************
*У�麯��������ͨ��Э�����У��λ����У��λ��ֵ
*
*���룺�����򳤶ȵ�λ  �����ݳ��ȸ�λ ��������,�ӹ����� 
*�����У��ֵ
*
***************************************************************************/

void make_data_bus(u8 datal,u8 datah)
{
	u8 i;
	
	uart1SBuf[0]=DATAHEAD;
	
	uart1SBuf[1]=datal; //�����򳤶ȵ�λ
	
	uart1SBuf[2]=datah; //���ݳ��ȸ�λ
	
	uart1SBuf[3]=ADDRESSL; //ͬ����ַL
	
	uart1SBuf[4]=ADDRESSH; //ͬ����ַH
	
	uart1SBuf[5]=DATADIR; //���ͷ���
	
	uart1SBuf[6]=uart1rBuf[6]; //������
	
  for(i=0;i<4;i++)
	{
   	uart1SBuf[7+i]=uart1rBuf[7+i]; //�ӹ����� �������һ���ֽ�
  }
	
	uart1SBuf[(7+datal+datah+1)]=DATAEND; //������
	
}

extern volatile	s16 temputer1;
extern volatile	s16 temputer2;

u8 sys_init_state=1; //��ʼ��״̬��־

void comwithpc(void)
{
		u8  CRC;  //���ڼ���У��λ
	
    WDT_CONTR |= 0x10;      //ι��
	
	if(uart1ReceivedFlag==1)  //���յ����ݲŽ��д���
	{
		
		if( (uart1rBuf[0]==0x55)&&(uart1rBuf[5]==0xFF) && (uart1rBuf[uart1ReceiveNum-1]==0x0d) )//�ж���βλ�Լ������ֽ��Ƿ���ȷ  
		{
			//��ȷ֮��ż���У��λ
			CRC=CRC_value(uart1rBuf,uart1ReceiveNum-2);
			
			if(CRC==uart1rBuf[uart1ReceiveNum-2])  //У����ȷ������ݽ��д���
			{
				//uart1senddata(uart1rBuf,uart1rBuf[uart1ReceiveNum-1]);//2
				
				switch(uart1rBuf[6]) //�Թ�������з�֧����
        {					
					case 0x00 :		
					
					//δָ֪��
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //������		
					 uart1SBuf[7]=0x00;//������	
					 uart1SBuf[8]=0x00;//������	
					 uart1SBuf[9]=0x00;//������	
					 uart1SBuf[10]=0x00;//������						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);					
		
					  break;
						
					//�������
					case 0x06 :
						
					   switch(uart1rBuf[9])
             {
							  //���
							  case 0x01 :
									
								 //�������������
								  Heart_Open=1;
								  Heart_Count=0;  //����ι��
								  Heart_flag=1;
									
									make_data_bus(0x05,0x00);
								
						  		uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
									//����У��
				       	  uart1SBuf[12]=CRC_value(uart1SBuf,12);	
								  
								  uart1senddata(uart1SBuf,14);
								 break;	
								
								//����
								case 0x02 :
									
								  //�ر��������
								  Heart_Open=0;
								
								  Heart_Count=0;  //����ι��
								 
								  Heart_flag=1;  //���־
									
									make_data_bus(0x05,0x00);
						  		uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
        					//����У��
				       	  uart1SBuf[12]=CRC_value(uart1SBuf,12);		  
								  uart1senddata(uart1SBuf,14);
									
								 break;	
	
								default :
									
								 //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
									
								 break;


               }
						
					  break;
							
					//ϵͳ��λ
					
					//55 04 00 00 00 FF 04 02 02 00 00 60 0D
					
					case 0x04 :
						
					  if(uart1rBuf[7]==0x02)
						{

						 make_data_bus(0x05,0x00);		

					   uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
											//����У��
				  	 uart1SBuf[12]=CRC_value(uart1SBuf,12);
					
					   uart1senddata(uart1SBuf,14);	
	
					   IAP_CONTR= 0x20 ;  //ϵͳ�����λ
					 }

					  break;
					 	 
					 //�����ֽ״̬
					 case 0x07 :
						
					  if(uart1rBuf[7]==0x01)
						{
						 make_data_bus(0x05,0x00);								
						 Paper_Send_OK=1;	    //��״̬Ϊ����		
					   uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
											//����У��
				  	 uart1SBuf[12]=CRC_value(uart1SBuf,12);				
					   uart1senddata(uart1SBuf,14);	
					  }

					  break;
					 
						
					//��ѯ����
					case 0x01 :
						
             switch(uart1rBuf[7])
             {
							  //55 04 00 00 00 FF 01 01 00 00 00 5A 0D
							  //55 08 00 00 00 FE 01 01 00 00 00 00 03 10 10 00 00 
							  //��ѯ �汾
							  case 0x01 :
									make_data_bus(0x08,0x00);
								
								  uart1SBuf[11]=HARDVER1;  //
								  uart1SBuf[12]=HARDVER2;  //
								  uart1SBuf[13]=FIREVER;   //
								  uart1SBuf[14]=SOFTVER;   //
								  //����У��
								  uart1SBuf[15]=CRC_value(uart1SBuf,15);
								  
								 uart1senddata(uart1SBuf,17);
								 break;	
								
								 //��ѯ ֽ��״̬
							  case 0x02 :
									
									switch(uart1rBuf[8]) //������֧��  02---7280
										{
											
											case 0x01 :
												
								         break;	
											
											//7280
											case 0x02 :
												
										  make_data_bus(0x06,0x00);
											
											uart1SBuf[11]=0x01;  // ��ʾ1 ��ֽ��
											
											uart1SBuf[12]=0x01;  // 00000001 ��ʾ��ֽ��
											
											//��ֽ�ͣ���ֽ��
											
											if(MOTO_S1S7()==1)
											{
                        uart1SBuf[12]=0x00; //���ر�ʾ��ֽ��
                      }	
											
										  uart1SBuf[13]=CRC_value(uart1SBuf,13);	
								      uart1senddata(uart1SBuf,15);	
											
								         break;	
										}
									 break;		
								
								
								//��ѯī��״̬
								case 03 :
									
										switch(uart1rBuf[8]) //������֧��02 --7280  03---510
										{
											
											case 0x03 :
											  	
										  make_data_bus(0x06,0x00); 
											
										  uart1SBuf[11]=0x05;  // ��ʾ5 ��ī��   λ˳�� C  M Y BK  MBK 
											
											uart1SBuf[12]=0x00;
											
										//	�ж��Ƿ������˼�⹦��
										 if(IPF510MT_PB==0x02)  //������ν������ ֱ���ϱ� 0x00
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
											
										  uart1SBuf[11]=0x05;  // ��ʾ5 ��ī��   λ˳�� C  M Y BK  MBK 
											
											uart1SBuf[12]=0x00;  //Ĭ��Ϊ��ī
											
											//�ж��Ƿ������˼�⹦��
											if(C7280MT_PB==0x02)  //������ν������ ֱ���ϱ� 0x00
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
												
											 //δָ֪��
											 make_data_bus(0x05,0x00);		
											 uart1SBuf[6]=0xFF; //������		
											 uart1SBuf[7]=0x00;//������	
											 uart1SBuf[8]=0x00;//������	
											 uart1SBuf[9]=0x00;//������	
											 uart1SBuf[10]=0x00;//������						
											 uart1SBuf[11]=OTHERERROR;  // 
											 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
											 uart1senddata(uart1SBuf,14);
																			
											break;

										}				
							    	break;	
								

					    //������״̬
							case 0x04 :
								
							
										switch(uart1rBuf[8]) //������֧��03 --7280  02---510
										{
											
											//��Ƭͨ��������״̬
											case 0x02 :
											  	
										  make_data_bus(0x06,0x00); 
											
										  uart1SBuf[11]=0x07;  // ��ʾ 7��������
											
											uart1SBuf[12]=0x00;  //���ȶ���0
											
											//��������ֵ
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
                        uart1SBuf[12]=uart1SBuf[12]|0x00;  //������������ͳһ��0
                      }	
						
											uart1SBuf[13]=CRC_value(uart1SBuf,13);
								      uart1senddata(uart1SBuf,15);	
															
											break;

											default :
												
											 //δָ֪��
											 make_data_bus(0x05,0x00);		
											 uart1SBuf[6]=0xFF; //������		
											 uart1SBuf[7]=0x00;//������	
											 uart1SBuf[8]=0x00;//������	
											 uart1SBuf[9]=0x00;//������	
											 uart1SBuf[10]=0x00;//������						
											 uart1SBuf[11]=OTHERERROR;  // 
											 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
											 uart1senddata(uart1SBuf,14);
																			
											break;

										}	
								
							break;
													
								
						  // ��ѯ�¶�
								case 0x08 :
								
								//��ѯ�����¶� 
								
                //���¶�	

	              switch(uart1rBuf[8])  //���������
								{
									 case  0x01 :   //�����¶�
										 
								   break;
									
									 case  0x02 :   //�����¶�
										 
									 	make_data_bus(0x06,0x00);  //�¶� ���ֽ� 
                    //�¶��ڶ�ʱ���ж�ȡ��ÿ10���ȡһ��									 
									  uart1SBuf[11] = temputer2;  //  �¶ȣ���λ��ǰ
									  uart1SBuf[12] = temputer2 >> 8;  //  �¶ȣ���λ�ں�								 
									  uart1SBuf[13]=CRC_value(uart1SBuf,13);
								    uart1senddata(uart1SBuf,15);
				 
								   break;
									
									 case  0x03 :   //�������¶�״̬
										 
								   break;
									 
									 case  0x04 :   //�����¶�״̬
										 
								   break;
									
	 
                 default : 
									 
								 //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
								 break;
								 	
                }//end  ��ѯ�¶�
                break;									
								
								
								//��ѯ ��ʼ��״̬  ���ݳ���5
							  case 0x09 :
									
									make_data_bus(0x05,0x00);
								
								  uart1SBuf[11]=sys_init_state;  //ֱ���ϱ���ʼ��״̬
								  
								  //����У��
								  uart1SBuf[12]=CRC_value(uart1SBuf,12);
								  
								  uart1senddata(uart1SBuf,14);
								
								 break;	
								
								
								//��ѯ������״̬
							  case 0x0B :
									
									make_data_bus(0x05,0x00);
								 
								 if(Heat==0) 
                 { 
                  uart1SBuf[11]=1;  //1 ��ʾ�������� 
                 }	
                 else
								 {   
									 uart1SBuf[11]=0;  //1 ��ʾ��������
                 }
               								 
								  //����У��
								  uart1SBuf[12]=CRC_value(uart1SBuf,12);
								  
								  uart1senddata(uart1SBuf,14);
								
								 break;	
								
								
								case 0x0e :
									
											 switch(uart1rBuf[8])  //��ī���
											{
												 case  0x01 :   //451
												 //δָ֪��
												 make_data_bus(0x05,0x00);		
												 uart1SBuf[6]=0xFF; //������		
												 uart1SBuf[7]=0x00;//������	
												 uart1SBuf[8]=0x00;//������	
												 uart1SBuf[9]=0x00;//������	
												 uart1SBuf[10]=0x00;//������						
												 uart1SBuf[11]=OTHERERROR;  // 
												 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
												 uart1senddata(uart1SBuf,14);
													 
												 break;
												
												 case  0x02 :   //7280
													
												 	make_data_bus(0x05,0x00);	
												 
												 uart1SBuf[11]=0x00;  //�� ��δ��
												 
												 if(C7280FM_PB==0x02)  //������ν������ ֱ���ϱ� 0x00
													{

															if(Read7280_FM()==1)  //��ī Ϊ �ߣ�  ��ī Ϊ�� ����Ϊ ��ī��
															{
																uart1SBuf[11]=0x00;  //�� ��δ��
															}	
															else
															{
																uart1SBuf[11]=0x01;  //�� ��δ��
																
															}
															
													}		
													//����У��
													uart1SBuf[12]=CRC_value(uart1SBuf,12);
													uart1senddata(uart1SBuf,14);

												 break;
												
												 case  0x03 :   //510
													 
												 	make_data_bus(0x05,0x00);	
												 
												  uart1SBuf[11]=0x00;  //�� ��δ��
												 
													if(IPF510FM_PB==0x02)  //������ν������ ֱ���ϱ� 0x00
													{

														if(ReadIpf510_FM()==1)  //��ī Ϊ �ߣ�  ��ī Ϊ�� ����Ϊ ��ī��
														{
															uart1SBuf[11]=0x00;  //�� ��δ��
														}	
														else
														{
															uart1SBuf[11]=0x01;  //�� ��δ��
															
														}
													}
													//����У��
													uart1SBuf[12]=CRC_value(uart1SBuf,12);
													uart1senddata(uart1SBuf,14);
													 
												 break;
												 
												 case  0x04 :   //5100
													 
												 //δָ֪��
												 make_data_bus(0x05,0x00);		
												 uart1SBuf[6]=0xFF; //������		
												 uart1SBuf[7]=0x00;//������	
												 uart1SBuf[8]=0x00;//������	
												 uart1SBuf[9]=0x00;//������	
												 uart1SBuf[10]=0x00;//������						
												 uart1SBuf[11]=OTHERERROR;  // 
												 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
												 uart1senddata(uart1SBuf,14);
													 
												 break;
												
				 
											 default : 
												 
											  //δָ֪��
											 make_data_bus(0x05,0x00);		
											 uart1SBuf[6]=0xFF; //������		
											 uart1SBuf[7]=0x00;//������	
											 uart1SBuf[8]=0x00;//������	
											 uart1SBuf[9]=0x00;//������	
											 uart1SBuf[10]=0x00;//������						
											 uart1SBuf[11]=OTHERERROR;  // 
											 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
											 uart1senddata(uart1SBuf,14);
													 break;
												
											}
								break;
							//end  ��ī��ѯ
						
              //��ѯ��Ƭ״̬											
						    case 0x0F :	
									
									make_data_bus(0x05,0x00);	
									uart1SBuf[11]=Paper_Send_OK;  //�� ��δ��

									//����У��
									uart1SBuf[12]=CRC_value(uart1SBuf,12);
									uart1senddata(uart1SBuf,14);
									break;		
								
								default :
									
								 //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
									
									break;

             }//end  uart1rBuf[7]-�������һ������ �жϲ�ѯ��������					
											
					  break;
					//����ָ��
						 
					case 0x02 :
						
					 // uart1senddata(uart1rBuf,uart1rBuf[uart1ReceiveNum-1]);//3
					    switch(uart1rBuf[7])
             {
							  //���Ƽ�����
							 case 0x03 :		
							 
								   if(uart1rBuf[9]==0x01)    //��������
									 {										 
										  HeatOn();     //�͵�ƽ��
										 
                     if(Heat==0x00)  //�ж��Ƿ񿪳ɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	
                     
										 make_data_bus(0x05,0x00);					
										//����У��
										uart1SBuf[12]=CRC_value(uart1SBuf,12);
										uart1senddata(uart1SBuf,14);
									  break;												 
                   }		

									 if(uart1rBuf[9]==0x02)    //�ؼ�����
									 {
										 
										 HeatOff();   //�ߵ�ƽ��
										 
                     if(Heat==1)  //�ж��Ƿ�سɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	

											make_data_bus(0x05,0x00);					
											//����У��
											uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
											 
										 break;											 
                   }

									 break;	

							   //end ����������
									 
								//���������ȿ��� 
								case 0x07 :	
									
								   if(uart1rBuf[9]==0x01)    //������������
									 {
										 
										 HeatFanOn();     //�͵�ƽ��
										 
                     if(HeatFan==0)  //�ж��Ƿ񿪳ɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
											 
                     }	 	

											make_data_bus(0x05,0x00);					
											//����У��
											uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 if(uart1rBuf[9]==0x02)    //�ؼ���������
									 {
										 
										 HeatFanOff();   //�ߵ�ƽ��
										 
                     if(HeatFan==1)  //�ж��Ƿ�سɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	 	

										 make_data_bus(0x05,0x00);					
								    //����У��
								    uart1SBuf[12]=CRC_value(uart1SBuf,12);
								    uart1senddata(uart1SBuf,14);
								    break;	
 
                   }
									 break;
                //end ���ȿ���	 
 
								//����ƿ���
                //���������ȿ��� 
								case 0x01 :
									
								switch(uart1rBuf[8])  //���������
								{
									 case  0x01 :   //�����
										 
									 if(uart1rBuf[9]==0x01)    //���ӵ����
									 {
										 
										 DaoGuangLedOn();     //�ߵ�ƽ��
										 
                     if(DaoGuangLed==1)  //�ж��Ƿ񿪳ɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	

 
								    make_data_bus(0x05,0x00);					
								    //����У��
								    uart1SBuf[12]=CRC_value(uart1SBuf,12);
								    uart1senddata(uart1SBuf,14);
								    break;	
										 
                   }
									
									 
									 if(uart1rBuf[9]==0x02)    //�ص����
									 {
										 
										 DaoGuangLedOff();   //�͵�ƽ��
										 
                     if(DaoGuangLed==0)  //�ж��Ƿ�سɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	 

											make_data_bus(0x05,0x00);					
								      //����У��
								      uart1SBuf[12]=CRC_value(uart1SBuf,12);
								      uart1senddata(uart1SBuf,14);
								      break;											 
                   }
									 
								
							   //end ����ƿ���
									 
								 //����ƿ���	 
								 	case  0x02 :  //����ڵ�
										
									 if(uart1rBuf[9]==0x01)    //�������
									 {
										 
										 BaoGaoLedOn();     //�ߵ�ƽ��
										 
                     if(BaoGaoLed==1)  //�ж��Ƿ񿪳ɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	


									  	make_data_bus(0x05,0x00);					
								     //����У��
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 									 
									 if(uart1rBuf[9]==0x02)    //�ر����
									 {
										 
										 BaoGaoLedOff();   //�͵�ƽ��
										 
                     if(BaoGaoLed==0)  //�ж��Ƿ�سɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	 

										  make_data_bus(0x05,0x00);					
											//����У��
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 
									 //δָ֪��
										 make_data_bus(0x05,0x00);		
										 uart1SBuf[6]=0xFF; //������		
										 uart1SBuf[7]=0x00;//������	
										 uart1SBuf[8]=0x00;//������	
										 uart1SBuf[9]=0x00;//������	
										 uart1SBuf[10]=0x00;//������						
										 uart1SBuf[11]=OTHERERROR;  // 
										 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
										 uart1senddata(uart1SBuf,14);
										 break;
									 
									
							   //end ����ƿ���
									 
									 
									 
                 default : 
									 
								  //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
								 break;
								 	
                }//end  uart1rBuf[8]-�������һ������ �жϿ�������	
                break;
                
								//�������
								case 0x0C :			
								switch(uart1rBuf[8])  //�������
								{
									 case  0x01 :  

                   //δָ֪��
									 make_data_bus(0x05,0x00);		
									 uart1SBuf[6]=0xFF; //������		
									 uart1SBuf[7]=0x00;//������	
									 uart1SBuf[8]=0x00;//������	
									 uart1SBuf[9]=0x00;//������	
									 uart1SBuf[10]=0x00;//������						
									 uart1SBuf[11]=OTHERERROR;  // 
									 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
									 uart1senddata(uart1SBuf,14);									 
										 
									 break;
									 	 
								 //��Ƭ���
								 	case  0x02 :  //��Ƭ���
										
									//55 04 00 00 00 FF 02 0C 02 01 00 69 0D
										
									 if(uart1rBuf[9]==0x01)    //��ת
									 {
										 
										 MotoDc1Run(100,FORWARD,10); // 1/2ռ�ձ� ��ת5S 	  ȫ��
										 
                     uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�

									  	make_data_bus(0x05,0x00);					
								     //����У��
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 									 
									 if(uart1rBuf[9]==0x02)    //ֹͣ
									 {
										 
										  MotoDc1Stop();     
										 // MotoDc2Stop();

                       uart1SBuf[11]=0;  // 1��ʾʧ��
										 
										  make_data_bus(0x05,0x00);					
											//����У��
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 break;

								  //��ϵ��
									case 0x03 :
										
									if(uart1rBuf[9]==0x01)    //��ת
									 {
										 
										 MotoDc2Run(30,FORWARD,10); //100ռ�ձ� ��ת5S 	  ȫ��
										 
                     uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�

									  	make_data_bus(0x05,0x00);					
								     //����У��
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 									 
									 if(uart1rBuf[9]==0x02)    //ֹͣ
									 {
										 
										  //MotoDc1Stop();     
										    MotoDc2Stop();

                       uart1SBuf[11]=0;  // 0��ʾ�ɹ�
										 
										  make_data_bus(0x05,0x00);					
											//����У��
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 break;
									 
                 default :
									  //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
								 break;
								 	
                }//end  uart1rBuf[8]-�������һ������ �жϿ�������	
									 //end �������
								break;
								//����������
								case  0x0D :
									
								switch(uart1rBuf[8])
								{
									
									case 0x00 :
									
									if(uart1rBuf[9]==0x01)    //��������
									 {
										 
										 
										 BuzzerOn();   //�͵�ƽ��
										 
                     if(Buzzer==1)  //�ж��Ƿ�سɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }	 

									  	make_data_bus(0x05,0x00);					
								     //����У��
								     uart1SBuf[12]=CRC_value(uart1SBuf,12);
								     uart1senddata(uart1SBuf,14);
								     break;										 
                   }
									 
									 if(uart1rBuf[9]==0x02)    //�ط�����
									 {
										 
										 BuzzerOff();   //�͵�ƽ��
										 
                     if(Buzzer==0)  //�ж��Ƿ�سɹ�
										 {
											 uart1SBuf[11]=0;  // 0��ʾ�ɹ�
                     }
                     else
										 {
                       uart1SBuf[11]=1;  // 1��ʾʧ��
                     }
										 
										  make_data_bus(0x05,0x00);					
											//����У��
										  uart1SBuf[12]=CRC_value(uart1SBuf,12);
											uart1senddata(uart1SBuf,14);
										 break;											 
                   }
									 
									 break;
								 
								default : 
								 //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14); 
							 
								 break;

                }  //end  buzzer  control
								
								break;
								
								default :
									
								 //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
									break;
             }//end  uart1rBuf[7]-�������һ������ �жϿ�������		
						 break;
						 
						 
					//ϵͳ����
					case 0x03 :
						
												//δָ֪��
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //������		
					 uart1SBuf[7]=0x00;//������	
					 uart1SBuf[8]=0x00;//������	
					 uart1SBuf[9]=0x00;//������	
					 uart1SBuf[10]=0x00;//������						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
			
					  break;
					
					//0x05  ����
					case 0x05 :
						
					if(uart1rBuf[7]==0x01)  // ���Ϊīˮ
					{	
						
							 switch(uart1rBuf[8])  //īˮ���  0X01-451  0X02--7280  0X03---510  0X04-5100  0X05--��ī
							 {
								 
									//����īˮ
									case 0x03 :   //����510
										
									 if(uart1rBuf[9]==0x01)  // ����
									 {
										 IPF510MT_PB=0x01;

                   }
                   if(uart1rBuf[9]==0x02)  // ����
									 {
										 IPF510MT_PB=0x02;

                   }										 
										make_data_bus(0x05,0x00);
									
										uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
										//����У��
										uart1SBuf[12]=CRC_value(uart1SBuf,12);	
										
										uart1senddata(uart1SBuf,14);
									 break;	
									
									//����īˮ
									case 0x02 :
										
									  if(uart1rBuf[9]==0x01)  // ����
									 {
										 C7280MT_PB=0x01;

                   }
                   if(uart1rBuf[9]==0x02)  // ����
									 {
										 C7280MT_PB=0x02;

                   }	
										make_data_bus(0x05,0x00);
									
										uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
										//����У��
										uart1SBuf[12]=CRC_value(uart1SBuf,12);	
										
										uart1senddata(uart1SBuf,14);
									 break;	
									
									//����īˮ
									case 0x05 :
										
									 if(uart1rBuf[9]==0x01)  // ����
									 {
									  IPF510FM_PB=0x01;
                    C7280FM_PB=0x01;		

                   }
                   if(uart1rBuf[9]==0x02)  // ����
									 {
									  IPF510FM_PB=0x02;
                    C7280FM_PB=0x02;		
                   }	
									 
										make_data_bus(0x05,0x00);
									
										uart1SBuf[11]=0x00;  // 0��ʾ�ɹ�
										//����У��
										uart1SBuf[12]=CRC_value(uart1SBuf,12);	
										
										uart1senddata(uart1SBuf,14);
									 break;	
									

									default :
										
								 //δָ֪��
								 make_data_bus(0x05,0x00);		
								 uart1SBuf[6]=0xFF; //������		
								 uart1SBuf[7]=0x00;//������	
								 uart1SBuf[8]=0x00;//������	
								 uart1SBuf[9]=0x00;//������	
								 uart1SBuf[10]=0x00;//������						
								 uart1SBuf[11]=OTHERERROR;  // 
								 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
								 uart1senddata(uart1SBuf,14);
										
									 break;

								 } 	


								 
					 }		
					 break;
					


					//ȷ��/����
					case 0xFC :
						
					//δָ֪��
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //������		
					 uart1SBuf[7]=0x00;//������	
					 uart1SBuf[8]=0x00;//������	
					 uart1SBuf[9]=0x00;//������	
					 uart1SBuf[10]=0x00;//������						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
						
					  break;
					
					//��������
					case 0xFD :
					 //δָ֪��
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //������		
					 uart1SBuf[7]=0x00;//������	
					 uart1SBuf[8]=0x00;//������	
					 uart1SBuf[9]=0x00;//������	
					 uart1SBuf[10]=0x00;//������						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);	
					  break;
					//�����ϱ�
					case 0xFE :
				 	//δָ֪��
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //������		
					 uart1SBuf[7]=0x00;//������	
					 uart1SBuf[8]=0x00;//������	
					 uart1SBuf[9]=0x00;//������	
					 uart1SBuf[10]=0x00;//������						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
						
					  break;
					//����֪ͨ
					case 0xFF :
					 //δָ֪��
					 make_data_bus(0x05,0x00);		
					 uart1SBuf[6]=0xFF; //������		
					 uart1SBuf[7]=0x00;//������	
					 uart1SBuf[8]=0x00;//������	
					 uart1SBuf[9]=0x00;//������	
					 uart1SBuf[10]=0x00;//������						
					 uart1SBuf[11]=OTHERERROR;  // 
					 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
					 uart1senddata(uart1SBuf,14);
							
				  break;
					
					default :	
					
				 //δָ֪��
			   make_data_bus(0x05,0x00);		
         uart1SBuf[6]=0xFF; //������		
			 	 uart1SBuf[7]=0x00;//������	
				 uart1SBuf[8]=0x00;//������	
				 uart1SBuf[9]=0x00;//������	
				 uart1SBuf[10]=0x00;//������						
			   uart1SBuf[11]=OTHERERROR;  // 
			 	 uart1SBuf[12]=CRC_value(uart1SBuf,12);  
			   uart1senddata(uart1SBuf,14);
				 break;

       }//end SWITCH()    

      }//end if(CRC==uart1rBuf[uart1ReceiveNum-2])
			else
			{
				//У��û�� ����02
			  	make_data_bus(0x05,0x00);
				 uart1SBuf[6]=0xFF;//������	
				 uart1SBuf[7]=0x00;//������	
				 uart1SBuf[8]=0x00;//������	
				 uart1SBuf[9]=0x00;//������	
				 uart1SBuf[10]=0x00;//������		
			   uart1SBuf[11]=CRCERROR;  // 
			   uart1SBuf[12]=CRC_value(uart1SBuf,12);  
				 uart1senddata(uart1SBuf,14);
      }
    }// end  0x55  0xff  0x0d	
		else
		{		
			//ͷβû��  ����03
		 make_data_bus(0x05,0x00);
			
	   uart1SBuf[6]=0xFF;//������	
	   uart1SBuf[7]=0x00;//������	
		 uart1SBuf[8]=0x00;//������	
		 uart1SBuf[9]=0x00;//������	
		 uart1SBuf[10]=0x00;//������	
			
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
  //�����ر�־		
	uart1ReceiveNum=0;
  uart1ReceivedFlag=0;		

  }//END if(uart1ReceivedFlag==1) 
}


