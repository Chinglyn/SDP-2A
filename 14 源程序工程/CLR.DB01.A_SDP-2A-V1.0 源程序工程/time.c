/************************************************************************************
* ����˵����
* ��ʱ��������

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
�������ܣ���ʼ����ʱ��0��ÿ5ms����һ���жϡ�
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
void InitTimer0(void)
{
	
	//AUXR|=0x80;		//���ö�ʱ��0Ϊ1Tģʽ��Ĭ��Ϊ12Tģʽ
	
	TMOD&=(~0x03);	//����Timer0������ģʽ0(16λ�Զ�����ģʽ)

	/*ϵͳʱ��ѡ��Ϊ11.0592Mʱ�� ����ֵ�ļ������£�	  
	11059200/12��Ƶ/200=4608����ÿ����4608��ʱΪ5ms��*/	
	TH0 = (65536-SYSCLK/12/200)>>8;		//��ʼ��Timer0����ֵ
	TL0 = (65536-SYSCLK/12/200);		//��ʼ��Timer0����ֵ
	
	ET0=1;  		//Timer0�ж�ʹ��
	TR0 = 1;		//Timer0����
}
/**///////////////////////Pino Electronics////////////////////////**/


extern u8 Sensor_P0_flag;       //�е�������������ʶ
extern u8 Sensor_P0_new_state;  //�е�������������ʶ
extern u8 Sensor_P0_old_state;  //�е�������������ʶ

extern u8 Sensor_P1_flag;       //�е�������������ʶ
extern u8 Sensor_P1_new_state;  //�е�������������ʶ
extern u8 Sensor_P1_old_state;  //�е�������������ʶ

//��Ƭ����ͨ���ڴ�����
extern u8 Sensor_P2_flag;     //�е�������������ʶ
extern u8 Sensor_P2_f10;      //״̬��1--0 
extern u8 Sensor_P2_f01;      //״̬��0--1
extern u8 Sensor_P2_new_state;  //�е�������������ʶ
extern u8 Sensor_P2_old_state;  //�е�������������ʶ
//extern u32 Sensor_P2_f10_count;    //���֮����ʱֹͣ����
extern u8 Sensor_P2_Moto_Stop_flag;   //��תֹͣ��־


//extern u8 Sensor_P3_flag;       //�е�������������ʶ
extern u8 Sensor_P3_new_state;  //�е�������������ʶ
extern u8 Sensor_P3_old_state;  //�е�������������ʶ


//extern u8 Sensor_P4_flag;       //�е�������������ʶ
extern u8 Sensor_P4_new_state;  //�е�������������ʶ
extern u8 Sensor_P4_old_state;  //�е�������������ʶ


extern u8 Sensor_P5_flag_start;   //��ʶ�Ƿ���Ҫ���г�Ƭ���
extern u8 Sensor_P5_flag;         //��Ƭ�ڴ�����  �Ӹ�--���--���  ��ʾ��Ƭ���������  1 ��ʾ�������
extern u8 Sensor_P5_f10;          //��Ƭ�ڴ�����  �Ӹ�--��� ��ʶ

extern u8 Sensor_P5_new_state;    //��������ǰ״̬
extern u8 Sensor_P5_old_state;    //��������һ��״̬

extern u8 Sensor_P5_OK_flag;    //�����������ɱ�־
extern u8 Sensor_P5_OK_flag_count;    //���֮����ʱ�������



extern u8 Sensor_P6_flag;            //���̴��б�ʶ 
extern u8 Sensor_P6_SendPaper_flag;  //���̴��б�ʶ 
extern u8 Sensor_P6_new_state;
extern u8 Sensor_P6_old_state;

extern u16 Sensor_P6_flag_cont;     //���Ȳ���������
extern u8 Sensor_P6_cont_start;     //�����г̿�ʼ������ʶ
extern u16 Sensor_P6_SendPaper_flag_cont; //���Ȳ���������
extern u16 SendPaper_P6_Time_Flag_start; // ��ֽ��ʱ������ʼ��־
 

extern volatile u8 MOTO_DC1_Run_Time;  //�������ʱ��
extern volatile u8 MOTO_DC2_Run_Time;  //�������ʱ��

extern volatile	u32 DC1_Run_Time_count;  //��ʱ����PWM ����ֵ
extern volatile	u32 DC2_Run_Time_count;  //��ʱ����PWM ����ֵ


extern volatile	u8 PWM1count;  //��ʱ����PWM ����ֵ
extern volatile	u8 PWM2count;  //��ʱ����PWM ����ֵ

extern volatile	u8 PWM1time;  //PWM ռ�ձ�ֵ   
extern volatile	u8 PWM2time;  //PWM ռ�ձ�ֵ  

extern volatile	u8 PWMDC1Direction;  //���1 �����־  1 ��ת  2 ��ת
extern volatile	u8 PWMDC2Direction;  //���2 �����־  1 ��ת  2 ��ת

extern volatile	u8 DC1Run;  //���1 �����־  1 ��ת  2 ��ת
extern volatile	u8 DC2Run;  //���2 �����־  1 ��ת  2 ��ת

extern volatile  u8 Heat_Time;  //��Ƭ����ͨ���еļ���ʱ�䡣 ��λ ��
extern volatile  u8 Heat_Time_Flag;  //��Ƭ������ɱ�־
extern volatile  u8 Heat_Time_Flag_start;  //��Ƭ��ʼ���ȼ�ʱ��־

extern volatile	u8 MOTO2_LH_Control;  //���2 ��Ͽ���
extern volatile	u8 MOTO2_LH_Flag;  //���2 ��Ͽ���


extern volatile  u32 JiuXu_Count;  //��Ƭ������ɱ�־
extern volatile  u8 JiuXu_Count_start;  //��Ƭ������ɱ�־

extern volatile	u8 Paper_Send_OK;   //��Ƭ��־


extern u8 sys_init_state; //ϵͳ��ʼ����ɱ�־


extern volatile	s16 temputer1;
extern volatile	s16 temputer2;

extern volatile	s16 Heart_Open;   //����������־
extern volatile	u8  Heart_flag;   //��������־
extern volatile	u32 Heart_Count;  //������ʱ����



extern volatile  u32 auto_paper_a4_Count;  //������ֽ��ʱ����
extern volatile  u32 auto_paper_a4_Count2;  //������ֽ����--��ʱ����
extern volatile  u8 auto_paper_a4_Count_Flag;  //������ֽ��־
extern volatile  u8 auto_paper_a4_Max_Flag;  //���泬ʱ��־


/************************��ʱ����ʱ����***********************/
/********************************************************************
�������ܣ���ʱ������
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
extern volatile	u8 Delay_start;  //��ʱ����PWM ����ֵ   ռ�ձ�
extern volatile	u8 Delay_num;  //��ʱ����PWM ����ֵ   ռ�ձ�
extern volatile	u8 Delay_count;  //��ʱ����PWM ����ֵ   ռ�ձ�
extern volatile	u8 Delay_flag;  //��ʱ����PWM ����ֵ   ռ�ձ�


/********************************************************************
�������ܣ���ʱ��0���жϷ������  5ms һ���ж�
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���жϱ�־λ�Զ������
********************************************************************/
void Timer0ISR(void) interrupt 1
{	
	static u16 count=0;  //�����Ƽ���	
	static u16 counttemp=998; //�¶ȴ���������
  static u16 count3=0;  //�е���⴫������ʱ
	
  static u16 HeatCount=0;  //�е���⴫������ʱ	
	
	static u16 Sensor_P2_f10_count=0;  //�е���⴫������ʱ	
	
	count3++;
	count++;   
	counttemp++; 	

	
	//�������Զ�����
	if(count==200)
	{		//5msһ���жϣ����յ�200���жϼ�Ϊ1��
		count=0;
		RunLedTog();		//ÿ1����˸һ��LED				
	}	
	
	
	//�������Զ�����
	if(counttemp==1000)
	{		//5msһ���жϣ����յ�2000���жϼ�Ϊ5��
		counttemp=0;
		HandleDs18b20ReadTemp();  //��ȡ��·��ʪ�ȴ��������� ��
			
	}	

	
		//��Ҫ������Ͽ���	---��ϴ����� 1--P3  1--P4
		if(MOTO2_LH_Control==1)		 
		{
			   //��Ϲص�λ����
			   if(MOTO2_LH_Flag==0) 
				 {
					 		Sensor_P3_old_state= Sensor_P3_new_state ;
							Sensor_P3_new_state=MOTO_S5S3();		//ÿ20ms ����Ϲش�����״̬
							
							if((Sensor_P3_old_state==1) && (Sensor_P3_new_state==0) ) //�ӵͱ�ɸ�
							{
									MotoDc2Stop(); 
                  MOTO2_LH_Control=0;								
							}						
         }
					//��Ͽ���λ����
					if(MOTO2_LH_Flag==1) 
				 {
					Sensor_P4_old_state= Sensor_P4_new_state ;
						
					Sensor_P4_new_state=MOTO_S4S4();		//ÿ20ms ����Ϲش�����״̬
					
					if((Sensor_P4_old_state==1) && (Sensor_P4_new_state==0) ) //�ӵͱ�ɸ�
					{
							MotoDc2Stop(); 
              MOTO2_LH_Control=0;	   
					}
				}
    }
		//end of  ��Ͽ���
	
	
	
   /*
	//��ʱ����
		if(Delay_start==1)  //�ж��Ƿ���Ҫ���̼���
		{		
				Delay_count++ ;		
				if(Delay_count >= Delay_num)
				{
					Delay_flag=1;
				}						
    }	

  */
	

		
	
  //�������Զ�����
	if(count3==2)  //ÿ20 ms ��ѯ��Ƭ������ش�����״̬������λ��ر�ʶ
	{		//5msһ���жϣ����յ�4���жϼ�Ϊ20ms��
		
		
		count3=0;	//����������
		
		//��ʼ�е�������״̬����
	  Sensor_P0_old_state=Sensor_P0_new_state;   //�����ϵ�״̬
	  Sensor_P0_new_state=MOTO_S8S0(); 
		if( (Sensor_P0_old_state==1) && (Sensor_P0_new_state==0)) // �Ӹ߱�ɵ�
		{
       		Sensor_P0_flag=1;      //�е���ʼ����
    }
		
		//�����е�������״̬����
		Sensor_P1_old_state= Sensor_P1_new_state ;		
		Sensor_P1_new_state=MOTO_S7S1();		
		if((Sensor_P1_old_state==0) && (Sensor_P1_new_state==1) ) //�ӵͱ�ɸ�
		{
       	Sensor_P1_flag=1;      //�е���������
    }
		
		//�ж�ͨ������Ƿ��н�Ƭ	
		Sensor_P2_old_state= Sensor_P2_new_state ;		
		Sensor_P2_new_state=MOTO_S6S2();	
		
		//�Ӹ߱�ͣ���Ƭ����ͨ����
		if((Sensor_P2_old_state==1) && (Sensor_P2_new_state==0) ) //�н�Ƭ����ͨ����
		{   
       	Sensor_P2_f10=1;
        Sensor_P2_flag=0;	 
        Sensor_P2_Moto_Stop_flag=0;		 //�����תֹͣ��־	
    }
	
			//�ӵͱ��
		if((Sensor_P2_old_state==0) && (Sensor_P2_new_state==1) ) //�н�Ƭ�뿪ͨ�����
		{
       	Sensor_P2_f01=1; 
        Sensor_P2_flag=0;	
			  
			  //�����ת������־
        Sensor_P2_f10=0;
        Sensor_P2_f10_count=0;
			
        Sensor_P2_Moto_Stop_flag=1;		 //�����תֹͣ��־				
        			
    }
			
	
		
		//��Ƭ�����⣬�ж��Ƿ�������Ƭ 
		if(Sensor_P5_flag_start==1)  //��Ҫ���г�Ƭ�ڱ����� 
		{		
					Sensor_P5_old_state= Sensor_P5_new_state ; //�����״̬			
					Sensor_P5_new_state=MOTO_S3S5();		//��ȡ��״̬	
		      //��Ƭ���� ������״̬Ϊ 1----0----1
			    //�����ж� 1--0
			    if((Sensor_P5_old_state==1) && (Sensor_P5_new_state==0) ) //�Ӹ߱��
					{
						
						Sensor_P5_f10=1;  //��Ƭ�������
							
				  }	
					
					if(Sensor_P5_f10==1)
					{				
						 
						if((Sensor_P5_old_state==0) && (Sensor_P5_new_state==1) )
						{	
       
              Sensor_P5_OK_flag=1;  //����ɱ�־  
							
            }

          }	 
					
    }	
			
		//��Ƭ���������ʱ
		if(Sensor_P5_OK_flag==1)
		{
			Sensor_P5_OK_flag_count++;		
			//�峬ʱ��־
			SendPaper_P6_Time_Flag_start=0;  //�ؼ�����־
		  Sensor_P6_SendPaper_flag=0;  // ���־
			
							//�������Զ�����
					if(Sensor_P5_OK_flag_count==200)  //1s
					{	
						//�˴���ʱ����  �ȴ���Ƭ��ȫ���	
						Sensor_P5_OK_flag_count=0;
						Sensor_P5_OK_flag=0;
						//�ñ�־
						Sensor_P5_flag=1;      //��Ƭ������
						Sensor_P5_f10=0;
					}							
    }
		
				
		//����  1--P6 ��������
		if(Sensor_P6_cont_start==1)  //�ж��Ƿ���Ҫ���̼���
		{		
					Sensor_P6_old_state= Sensor_P6_new_state ; //�����״̬			
					Sensor_P6_new_state=MOTO_S2S6();		//��ȡ��״̬	
		      // �¾�״̬��һ�� ��������1
					if( Sensor_P6_old_state != Sensor_P6_new_state) //תһȦ ���� �Ĵ�
					{
	            Sensor_P6_flag_cont++ ;	
						  if(Sensor_P6_flag_cont == MAPAN_MAX_CONT)
							{
								Sensor_P6_flag_cont=0;
								Sensor_P6_flag=1;  //�����̱�ʶ  ���ϳ�Ƭ�ڴ������ж��Ƿ�Ƭ��
              }						
							
					}
    }	
		
			
		//����  ��ʱ��Ƭ����
		if(SendPaper_P6_Time_Flag_start==1)  //�ж��Ƿ���Ҫ���̼���
		{		
				Sensor_P6_SendPaper_flag_cont++ ;	
			
				if(Sensor_P6_SendPaper_flag_cont >= MAPAN_SEND_PAPER_CONT)
				{
					Sensor_P6_SendPaper_flag_cont=0;
					SendPaper_P6_Time_Flag_start=0;
					Sensor_P6_SendPaper_flag=1;  //�����̱�ʶ  ���ϳ�Ƭ�ڴ������ж��Ƿ�Ƭ��
				}						

    }	
					
	}	
	
	
		//��ת��ʱ�ر�
		if(Sensor_P2_new_state==0)  //�н�Ƭ���뿪ʼ��ʱ
		{
			Sensor_P2_f10_count++;
			
			if(Sensor_P2_f10_count == 6000 )  //
			{	
         Sensor_P2_f10_count=0;				
				 Sensor_P2_Moto_Stop_flag=1;
      }		
			
    }


	// �������ʱ���?
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
	

	//��Ƭͣ������ʱ�����
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
	  
	 	//��Ƭ��������� ���� ʱ��
		if(JiuXu_Count_start==1)
		{   
			 	JiuXu_Count++;
			
				if(JiuXu_Count >= 1600) // 2000--10s  600--3S
				{
					JiuXu_Count=0;
					
					Paper_Send_OK=1;   //�þ�����־
					//�������
					HeatFanOff(); //�ؼ��ȷ���
					JiuXu_Count_start=0;			
				}
   }	
	 
	 
	 	 	//��������
		if(Heart_Open==1)
		{   
			  Heart_Count++;
			
				//if(Heart_Count >= 20000) // 1000S  16����
				if(Heart_Count >= 200000) // 1000S  16����	
				{
					
					Heart_flag=0;   
							
				}
   }
	 
	 //�ͱ�������ʱ����
	 if(auto_paper_a4_Count_Flag==1)
	 {
        auto_paper_a4_Count++;
		 
		    if(auto_paper_a4_Count==1600)  //5��ر�
				{
            auto_paper_a4_Count=0;
					
					  auto_paper_a4_Count_Flag=0;
        }

   }
  
   /*	 
	 //�ͱ�������ʱ����
	  //�ͱ�������ʱ����
	 if(BaoGaoLed==1)  //�����˱���ƣ����߿����˵��
	 {
        auto_paper_a4_Count2++;
		    if(auto_paper_a4_Count2 > 3000)  //15S ��ʱ
				{
					  auto_paper_a4_Max_Flag=1;
					  auto_paper_a4_Count2=0;
					  BaoGaoLedOff(); //�ص��
        }
   }
	  
	 */
}

/**///////////////////////Pino Electronics////////////////////////**/

/********************************************************************
�������ܣ���ʼ����ʱ��2��ÿ10ms����һ���жϡ�
��ڲ������ޡ�
��    �أ��ޡ�
��    ע��Timer2�Ĺ���ģʽֻ��һ�֣�16λ�Զ�����ģʽ��
********************************************************************/
/*
void Timer2Init(void)		//1����@11.0592MHz
{
	AUXR |= 0x04;		//��ʱ��ʱ��1Tģʽ
	T2L = 0xCD;		//���ö�ʱ��ֵ
	T2H = 0xD4;		//���ö�ʱ��ֵ
	
	IE2|=0x04;  	//Timer2�ж�ʹ��
	AUXR |= 0x10;		//��ʱ��2��ʼ��ʱ
}

*/


void Timer2Init(void)		//100΢��@11.0592MHz
{
	AUXR |= 0x04;		//��ʱ��ʱ��1Tģʽ
	T2L = 0xAE;		//���ö�ʱ��ֵ
	T2H = 0xFB;		//���ö�ʱ��ֵ
	IE2|=0x04;  	//Timer2�ж�ʹ��
	AUXR |= 0x10;		//��ʱ��2��ʼ��ʱ
}


/********************************************************************
�������ܣ���ʱ��2���жϷ������
��ڲ������ޡ�
��    �أ��ޡ�
// �ڶ�ʱ��2�������PWM�������
//��Ҫ���Ƶ��ռ�ձȵ����ݡ�
��    ע���жϱ�־λ�Զ������
********************************************************************/
//100΢��@11.0592MHz

//������Ƴ����ڶ�ʱ��2�н��� 
void Timer2ISR(void) interrupt 12
{	
	// ����PWM ���ƺ������ﴦ��
	
}
/**///////////////////////Pino Electronics////////////////////////**/JM













