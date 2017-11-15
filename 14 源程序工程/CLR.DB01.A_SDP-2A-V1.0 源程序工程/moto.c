
/*************  ����˵��    **************

��������ʾʹ�ö�ʱ�������PWM��  
��ʱ1ms�ж϶�ʱ��2��16λ�Զ���װ��

�����ְ�����ֱ�����ȫ����ת�������Լ�PWM��ת����


******************************************/



#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "ds18b20.h"
#include "moto.h"

// ȫ�ֱ��� 


volatile	u8 Paper_Send_Flag1=0;  //��Ƭ����״̬��־1
volatile	u8 Paper_Send_Flag2=0;  //��Ƭ����״̬��־2
volatile	u8 Paper_Send_Error=0;   //��Ƭ��־
volatile	u8 Paper_Send_OK=1;   //0 ������1 ���ڴ�ӡ��2 ��Ƭ��3 ��Ƭ��־


volatile	u8 PWM1count=0;  //��ʱ����PWM ����ֵ   ռ�ձ�
volatile	u8 PWM2count=0;  //��ʱ����PWM ����ֵ   ռ�ձ�

volatile	u8 PWM1time=1;  //PWM ռ�ձ�ֵ  //ռ�ձȸ�Ϊ������ 
volatile	u8 PWM2time=1;  //PWM ռ�ձ�ֵ  

volatile	u8 PWMDC1Direction=1;  //���1 �����־  1 ��ת  2 ��ת
volatile	u8 PWMDC2Direction=1;  //���2 �����־  1 ��ת  2 ��ת

volatile	u8 DC1Run=0;  //���1 �����־  1 ��ת  2 ��ת
volatile	u8 DC2Run=0;  //���2 �����־  1 ��ת  2 ��ת

volatile  u8 MOTO_DC1_Run_Time=0;  //�������ʱ��
volatile  u8 MOTO_DC2_Run_Time=0;  //�������ʱ��

volatile  u8 Heat_Time=0;  //��Ƭ����ͨ���еļ���ʱ�䡣 ��λ ��
volatile  u8 Heat_Time_Flag=0;  //��Ƭ������ɱ�־

volatile  u8 Heat_Time_Flag_start=0;  //��Ƭ������ɱ�־
volatile  u8 SendPaper_Time_Flag_start=0;  //��Ƭ������ɱ�־

volatile	u32 DC1_Run_Time_count=0;  //��ʱ����PWM ����ֵ  �������ʱ��
volatile	u32 DC2_Run_Time_count=0;  //��ʱ����PWM ����ֵ  �������ʱ��


//volatile	u8 DC1DECAY=1;  //���1 ˥����ʽ��־
//volatile	u8 DC2DECAY=1;  //���2 ˥����ʽ��־

volatile	u8 MOTO2_LH_Control=0;     //���2 ��Ͽ���
volatile	u8 MOTO2_LH_Flag=3;        //���2 ��Ͽ���

// volatile	u8 MOTO1_SEND_Flag=0;  


volatile  u32 JiuXu_Count=0;  //��Ƭ������ɱ�־
volatile  u8 JiuXu_Count_start=0;  //��Ƭ������ɱ�־



/****************�������ϵͳ��ر�ʶ����********************/

//��ʼ�е�
u8 Sensor_P0_flag=0;        //�е�������������ʶ
u8 Sensor_P0_new_state;     //��ǰ���״̬
u8 Sensor_P0_old_state=1;   //��һ״̬
//�����е�
u8 Sensor_P1_flag=0;         //�е�������������ʶ
u8 Sensor_P1_new_state;      
u8 Sensor_P1_old_state=0;    

//��Ƭ����ͨ���ڴ�����
u8 Sensor_P2_flag=0;     //��Ƭ����ͨ���ڱ�־
u8 Sensor_P2_f10=0;      //״̬��1--0 
u8 Sensor_P2_f01=0;        //״̬��0--1
u8 Sensor_P2_new_state;    //��ǰ���״̬
u8 Sensor_P2_old_state=0;  //��һ״̬

u8 Sensor_P2_Moto_Stop_flag=0; //��תֹͣ��־

//u32 Sensor_P2_f10_count=0;    //���֮����ʱ�������



//��Ϲ�
//u8 Sensor_P3_flag=0;         //��Ϲش�����״̬��ʶ
u8 Sensor_P3_new_state;      //��Ϲش���������״̬
u8 Sensor_P3_old_state=0;    //��Ϲش�������һ��״̬

//��Ͽ�
//u8 Sensor_P4_flag=0;     //��Ͽ�������״̬
u8 Sensor_P4_new_state;  //��Ͽ�����������״̬
u8 Sensor_P4_old_state=0;  //��Ͽ���������һ��״̬


u8 Sensor_P5_flag_start=0;    //��ʶ�Ƿ���Ҫ���г�Ƭ���
u8 Sensor_P5_flag=0;          //��Ƭ�ڴ�����  �Ӹ�--���--���  ��ʾ��Ƭ���������  1 ��ʾ�������
u8 Sensor_P5_f10=0;          //��Ƭ�ڴ�����  �Ӹ�--��� ��ʶ
u8 Sensor_P5_new_state=0;    //��ǰ���״̬ 
u8 Sensor_P5_old_state=0;    //��һ״̬

u8 Sensor_P5_OK_flag=0;    //�����������ɱ�־
u8 Sensor_P5_OK_flag_count=0;    //���֮����ʱ�������


u8 Sensor_P6_flag=0;              //���̴��б�ʶ --���Ƚ׶ε�λ��־

u8 Sensor_P6_SendPaper_flag=0;    //���̴��б�ʶ --��Ƭ�׶γ�ʱ��־

u8 Sensor_P6_new_state=0;
u8 Sensor_P6_old_state=0;

u16 Sensor_P6_flag_cont=0;    //���Ȳ���������
u8 Sensor_P6_cont_start=0;    //�����г̿�ʼ������ʶ

u16 Sensor_P6_SendPaper_flag_cont=0; //��ֽ����������
u16 SendPaper_P6_Time_Flag_start=0; // ��ֽ��ʱ������ʼ��־



/****************************************************************/
volatile  u32 auto_paper_a4_Count=0;  //������ֽ����
volatile  u32 auto_paper_a4_Count2=0;  //������ֽ����--��ʱ����
volatile  u8 auto_paper_a4_Count_Flag=0;  //������ֽ��־

volatile  u8 auto_paper_a4_Max_Flag=0;  //������ֽ��־





/*******************����PWM���� �������********************/
/********************************************************************
�������ܣ����1��ת
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
/*
void MotoDc1Forward(void)
{
   MOTO1_IN1On();
	 MOTO1_IN2Off();
}
8/
/********************************************************************
�������ܣ����1��ת
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
/*
void MotoDc1Referse(void)
{
   MOTO1_IN1Off();
	 MOTO1_IN2On();
}
*/ 
/********************************************************************
�������ܣ����1ֹͣ
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
void MotoDc1Stop(void)
{
	 DC1Run=0;
	 MOTO_DC1_Run_Time=0;
   MOTO1_IN1Off();
	 MOTO1_IN2Off();
   PWM2_SetPwmWide(0);     //���ȫ�͵�ƽ
   PWM3_SetPwmWide(0);     //����͵�ƽ	
}
 


//20161104  ����PWM���Ƴ�Ƭ���



sbit PWM2   =   P3^7;
sbit PWM3   =   P2^1;
sbit PWM4   =   P2^2;
sbit PWM5   =   P2^3;
sbit PWM6   =   P0^7;
sbit PWM7   =   P0^6;

/*


    PWM2_SetPwmWide(0);         //���ȫ�͵�ƽ
    PWM3_SetPwmWide(1);         //���1/2550�ߵ�ƽ
    PWM4_SetPwmWide(CYCLE);     //���ȫ�ߵ�ƽ
    PWM5_SetPwmWide(CYCLE-1);   //���2549/2550�͵�ƽ
		
    PWM6_SetPwmWide(CYCLE/2);   //���1/2�ߵ�ƽ
    PWM7_SetPwmWide(CYCLE/3);   //���1/3�ߵ�ƽ

*/


void PWM_config(void)
{
	  /*
    P0M0 &= ~0xc0;
    P0M1 &= ~0xc0;
    P0 &= ~0xc0;                    //����P0.6/.P0.7��ƽ
	  */
	
    P2M0 &= ~0x0e;
    P2M1 &= ~0x0e;
    P2 &= ~0x0e;                    //����P2.1/P2.2/P2.3��ƽ
	
    P3M0 &= ~0x80;
    P3M1 &= ~0x80;
    P3 &= ~0x80;                    //����P3.7��ƽ

    P_SW2 |= 0x80;

    PWMCKS = 0x00;
	
    PWMC = CYCLE;                   //����PWM����
	
    PWM2T1 = 1;
    PWM2T2 = 0;
	
    PWM2CR = 0x00;                  //PWM2�����P3.7
    PWM3T1 = 1;
    PWM3T2 = 0;
    PWM3CR = 0x00;                  //PWM3�����P2.1
    PWM4T1 = 1;
    PWM4T2 = 0;
    PWM4CR = 0x00;                  //PWM4�����P2.2
    PWM5T1 = 1;
    PWM5T2 = 0;
    PWM5CR = 0x00;                  //PWM5�����P2.3
		
	  /*
    PWM6T1 = 1;
    PWM6T2 = 0;
    PWM6CR = 0x08;                  //PWM6�����P0.7
    PWM7T1 = 1;
    PWM7T2 = 0;
    PWM7CR = 0x08;                  //PWM7�����P0.6
	  */	
		
    PWMCFG = 0x00;                  //����PWM�������ʼ��ƽ
		
    PWMCR = 0x3f;                   //ʹ��PWM�ź����
		
    PWMCR |= 0x80;                  //ʹ��PWMģ��
    P_SW2 &= ~0x80;
}


void PWM2_SetPwmWide(u16 Wide)
{
    if (Wide == 0)
    {
        PWMCR &= ~0x01;  //�رչ����ж�
        PWM2 = 0;
    }
    else if (Wide == CYCLE)
    {
        PWMCR &= ~0x01;  //�رչ����ж�
        PWM2 = 1;
    }
    else
    {
        P_SW2 |= 0x80;   //���蹦���л��Ĵ���
        PWM2T1 = Wide;   //���÷�ת��
        P_SW2 &= ~0x80;  //���蹦���л�
        PWMCR |= 0x01;   //ʹ���ж�
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
// ���2����
/********************************************************************
�������ܣ����2��ת
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
/*
void MotoDc2Forward(void)
{
   MOTO2_IN3On();
	 MOTO2_IN4Off();
}
*/
/********************************************************************
�������ܣ����2��ת
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
/*
void MotoDc2Reverse(void)
{
   MOTO2_IN3Off();
	 MOTO2_IN4On();
}
 */
/********************************************************************
�������ܣ����2ֹͣ
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
void MotoDc2Stop(void)
{  
	 DC2Run=0;
	 MOTO_DC2_Run_Time=0;
   MOTO2_IN3Off();
	 MOTO2_IN4Off();	
	 PWM4_SetPwmWide(0);     //���ȫ�͵�ƽ
   PWM5_SetPwmWide(0);     //����͵�ƽ	
}
 

//����ΪPWM ���� �������




/********************************************************************
�������ܣ����1 pwm ����
��ڲ������ޡ� ռ�ձ�  ����
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
void MotoDc1Run(u8 time, u8 direction , u8 runtime)
{
	 DC1Run=1;   //�õ�����б�־	
	 MOTO_DC1_Run_Time=runtime;   //����ʱ��
	 PWM1time=time; //��ռ�ձ�ֵ
	 PWMDC1Direction=direction; //����
	
	if(PWMDC1Direction==1) //��ת
	{
		  if(time==100)
			{
			  PWM2_SetPwmWide(CYCLE);     //ռ�ձ����
        PWM3_SetPwmWide(0);             //����͵�ƽ

      }
			else
			{	
				PWM2_SetPwmWide(PWM1time);           //ռ�ձ����
        PWM3_SetPwmWide(0);             //����͵�ƽ
      }	
			
		

  }

	if(PWMDC1Direction==0) //��ת
	{
			 if(time==100)
			{
			  PWM3_SetPwmWide(CYCLE);     //ռ�ձ����
        PWM2_SetPwmWide(0);             //����͵�ƽ
      }
			else
			{				
				PWM3_SetPwmWide(PWM1time);           //ռ�ձ����
        PWM2_SetPwmWide(0);             //����͵�ƽ     
      }	
  }
	
}


/********************************************************************
�������ܣ����1 pwm ����
��ڲ������ޡ� ռ�ձ�  ����
��    �أ��ޡ� 32<time<=100
��    ע���ޡ�
********************************************************************/
void MotoDc2Run(u8 time, u8 direction ,u8 runtime)
{
	 DC2Run=1;   //�õ�����б�־
	 MOTO_DC2_Run_Time=runtime; //����ʱ��
	
	 PWM2time=time; //�� ռ�ձ�ֵ
	
	 PWMDC2Direction=direction;
	
	if(PWMDC2Direction==1) //��ת
	{
			 if(time==100)
			{
			  PWM5_SetPwmWide(CYCLE);     //ռ�ձ����
        PWM4_SetPwmWide(0);             //����͵�ƽ
      }
			else
			{				
				PWM5_SetPwmWide(PWM2time);           //ռ�ձ����
        PWM4_SetPwmWide(0);             //����͵�ƽ     
      }	
  }
  
		if(PWMDC2Direction==0) //��ת
	{
			 if(time==100)
			{
			  PWM4_SetPwmWide(CYCLE);     //ռ�ձ����
        PWM5_SetPwmWide(0);             //����͵�ƽ
      }
			else
			{				
				PWM4_SetPwmWide(PWM2time);           //ռ�ձ����
        PWM5_SetPwmWide(0);             //����͵�ƽ     
      }	
  }

	
}


/********************************************************************
�������ܣ���Ƭͨ�����ƺ���
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/


// ���1Ϊ�������
// ���2Ϊ��ϵ��

//��ϵ�����ؿ���
/********************************************************************
�������ܣ���ϵ�����ؿ���
��ڲ�����TYPE   TYPE =1  OPEN  TYPE =0  CLOSE 
��    �أ��ޡ� 
��    ע���ޡ�
********************************************************************/
void Moto_DC2_Control(u8 TYPE )
{
	 //�����2 ����Ϲر�־��
   MotoDc2Run(30,1,10); //��ϵ������10S 
	  // MotoDc2Run(20,1,10); //��ϵ������10S   	//	ת��̫�죬��Ϲرյ�ʱ�� �в���

	 //��ϵ�λ��ʱ���رյ��2
   MOTO2_LH_Control=1;  //1 ��Ҫ������Ͽ���
   MOTO2_LH_Flag=TYPE;  //��Ͽ������� 1 �� �ɿ� ��0 �ؼӽ�
}

//��Ƭ���Ϳ��Ƴ���
/********************************************************************
�������ܣ���Ƭ���Ϳ��Ƴ���
��ڲ�������
��    �أ��ޡ� 
��    ע���ޡ�
********************************************************************/
u8  test=0xFF;
u8  sensed_time=0;    //��Ƭ����һ��

void Moto_control_sys(void)
{

	u8  Sensor_P0_TMP;  //��ʱ��������ȡ�е�λ��
	u8  Sensor_P1_TMP;  //��ʱ��������ȡ�е�λ��
	
	
	WDT_CONTR |= 0x10;      //ι��

	Sensor_P0_TMP=MOTO_S8S0();  //�ٴζ��е�״̬   ���ܸı��ж���   
	Sensor_P1_TMP=MOTO_S7S1();	 //�ٴζ��е�״̬	
	Sensor_P2_new_state=MOTO_S6S2();	 //����Ƭ�Ƿ������Ϳ�״̬  
	
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
	
	if(	(Sensor_P0_TMP==1 ) && (Sensor_P1_TMP==0)) //�е��Ѿ���λ
	{
			if( (Sensor_P1_flag==1) && (Sensor_P0_flag==1) ) //�������е�����
			{	
				
				Sensor_P2_new_state=MOTO_S6S2();	 //����Ƭ�Ƿ������Ϳ�״̬  
				
				if((Sensor_P2_new_state==0)|| (Paper_Send_Flag1==1) || (Paper_Send_Flag2==1))  //��Ƭ����ͨ����,���ߴ��ڵ�һ�׶Σ����ߴ��ڵڶ��׶�
				{
					
							//test=0xFF;
							//uart1senddata(&test,1);//1					
							//��Ƭ��һ�׶� ����� ���� 
							if(Paper_Send_Flag1==0)
							{	
								// test=0xEE;
								// uart1senddata(&test,1);//1
								
							  	 Paper_Send_OK=2;     //��ʼ��Ƭ�����Ƭ�ɹ���־
								 
							  	 Sensor_P5_flag_start=1 ; //��Ҫ����Ƭ�ڴ�����	
								
								
							  	 Paper_Send_Flag1=1;  //��λ ��ʾ�Ѿ������˼��ȿ���	
								
								   Heat_Time_Flag_start=1;  //���ȵȴ�ʱ�����
								
							    //	 MotoDc1Run(80,FORWARD,1); //60ռ�ձ� ��ת 25S 	 ��ת		
								
						     // Sensor_P6_cont_start=1;  //�г����̼�����ʼ  --ע�͵�����ʾ���ȴ�����ʱ��
								
								 //�����
								 Moto_DC2_Control(0);	    // ��ϵ����
				
								//�����ȵȴ�
								// Heat_Time_Flag=1;   
								
							} 
	
	
							//�жϽ�Ƭ�Ƿ���Ԥ���ļ���λ�ã�����ֹͣ���	
							 if(Sensor_P6_flag==1) //���μ������;���
							 {						 
								 MotoDc1Stop();           //ֹͣ��� 
								 Sensor_P6_cont_start=0;	//���־
								 Sensor_P6_flag_cont=0;	  //�������	 
								 Sensor_P6_flag=0;        //���־ 
							 }	

					 
              //�ڶ��׶�  ���ͽ�Ƭ
							if(Paper_Send_Flag2==0)
							{							
								 //������� ���ͽ�Ƭ
								 if(Heat_Time_Flag==1)  //�жϼ����Ƿ����
								 { 
									 
										Paper_Send_OK=2;     //��ʼ��Ƭ�����Ƭ�ɹ���־
									 
										Paper_Send_Flag2=1;  //�ڶ��׶���λ��־ 
									 
										Heat_Time_Flag_start=0;  //�ؼ��ȼ�ʱ��ʼ��־
									 
									 //	Heat_Time_Flag=0;        //�������ɱ�־ 
									 
									 //��Ƭͣ������ʱ����ϣ��ͳ���Ƭ
									 
										MotoDc1Run(100,FORWARD,15); //100ռ�ձ� ��ת15S 	  ȫ��
									  
									// 
									 
									 SendPaper_P6_Time_Flag_start=1;  //��ʱ���̼���	 
									 
								 }
							 
							 }
							
							 //��Ƭ�������
							 if(Sensor_P5_flag==1)
							 { 
									SendPaper_P6_Time_Flag_start=0;  //�ؼ�����־
								  Sensor_P6_SendPaper_flag=0;  // ���־
									Sensor_P5_flag=0;      //���־
									
								  //MotoDc1Run(100,FORWARD,1); //���˴�������תһ����  ��֤��ȫ����
									MotoDc1Stop();
								  //�����
								  Moto_DC2_Control(1);	    // ��ϵ����
								 
									Paper_Send_OK=0x00;       //��λ��Ƭ�ɹ���־	
								 
								 //����������ʼ��־
								 
                  JiuXu_Count=0;    //�����������
								 
                  JiuXu_Count_start=1;  //�þ���������ʼ��־
	               
								  sensed_time=0 ;
								  
								  Heat_Time_Flag=0;        //�������ɱ�־ 

									Sensor_P1_flag=0;      //���е���־
									Sensor_P0_flag=0;      //���е���־				 
									Paper_Send_Flag1=0;    //���һ�׶α�־
									Paper_Send_Flag2=0;    //��ڶ��׶α�־
									Sensor_P5_flag_start=0 ; //���Ƭ�ڼ���־
									Sensor_P6_SendPaper_flag=0; 
									Sensor_P6_SendPaper_flag_cont=0; //
									SendPaper_P6_Time_Flag_start=0;//��Ƭ��ʱ���̼���  
									
									Sensor_P2_Moto_Stop_flag=0;
									
								  //�ؼ���	
								//	HeatOff();     //��������
								 
							 } //������Ƭ
						
								 
							 if(Sensor_P6_SendPaper_flag==1) //���̼�����ʱ����ʾ��Ƭ
							 {
								 
									 Sensor_P6_SendPaper_flag=0;			 
									 // Paper_Send_OK=0x05;  //��ʾ��Ƭ			 									 
									 // MotoDc1Stop();  //ֹͣ��� 
									 // һ������¿�Ƭ����Ϊ��ϵ��û�мӽ�
									 //��Ƭ�ݴ���һ��		 								 
									 if(sensed_time==0x01)  //����Ƭ
									 {
										 
												Heat_Time_Flag=0;        //�������ɱ�־ 
										 
												SendPaper_P6_Time_Flag_start=0;	 
									 
												Sensor_P6_SendPaper_flag=0; 
									 
												SendPaper_Time_Flag_start=0;		
									 
												Sensor_P6_SendPaper_flag_cont=0;

												Paper_Send_OK=3;  //��ʾ��Ƭ	
                        
										    //�ؼ���
												HeatOff();     //��������
										 
									 }
									 
									 //�����
									 if(sensed_time==0x00)
									 {
											 sensed_time=0x01;
											 Moto_DC2_Control(0);	    // ��ϵ����	 	 
											 Paper_Send_Flag2=0 ; //�ñ�־ ����һ��ֽ��
									 }	
 		 						 
							  }	//en  ��Ƭ����
								
							 
							 //��Ƭ����
							 if(Paper_Send_OK==3)
							 {
								  //�˹���ԤȡƬ ֮�������ر�־
								   if(Sensor_P2_new_state==1)    //����޽�Ƭ �Զ������Ƭ��־  //�����Ƭ���м䣬�ڶ��ż������ǻῨ�����档����һ�����
									 {									
											SendPaper_P6_Time_Flag_start=0;  //�ؼ�����־
											Sensor_P6_SendPaper_flag=0;  // ���־
										 
											Sensor_P5_flag=0;      //���־
											MotoDc1Stop();         //ֹͣ���   
											//�����
											Moto_DC2_Control(1);	    // ��ϵ����	 		 
											Paper_Send_OK=0x01;       //���֮���Ϊ����
										 //����������ʼ��־ 
											JiuXu_Count=0;    //�����������
											JiuXu_Count_start=1;  //�þ���������ʼ��־  
											sensed_time=0 ;  
											Heat_Time_Flag=0;        //�������ɱ�־  
											Sensor_P1_flag=0;      //���е���־
											Sensor_P0_flag=0;      //���е���־				 
											Paper_Send_Flag1=0;    //���һ�׶α�־
											Paper_Send_Flag2=0;    //��ڶ��׶α�־
											Sensor_P5_flag_start=0 ; //���Ƭ�ڼ���־
											Sensor_P6_SendPaper_flag=0; 
											Sensor_P6_SendPaper_flag_cont=0; //
											SendPaper_P6_Time_Flag_start=0;//��Ƭ��ʱ���̼���  	 
											
											HeatFanOff(); //�ؼ��ȷ���
								 } 	 
               }
							 
							 
        }	 //�н�Ƭ ����һ�׶Σ��ڶ��׶�
			} //end P0  P1  �е�����
  }	//end P0  P1  �е���λ
	
	//�н�Ƭ���е�  ��ת
	if((Sensor_P2_new_state==0)&&(Sensor_P1_flag==0)&&(Sensor_P0_flag==0))  //��Ƭ����ͨ���ڣ����е�����
	{		
		if(Sensor_P2_Moto_Stop_flag==0)
		{	
			    //��ֹ���е�����û�й�λ�Ĺ����д��߽�Ƭ��ɱ���		
				if(	(Sensor_P0_TMP==1 ) && (Sensor_P1_TMP==0)) //�е��Ѿ���λ
				{	
							MotoDc1Run(60,FORWARD,1); //50ռ�ձ�
				}	
        
        		//�н�Ƭ ������
				HeatFanOn();  // �������ȷ���
			//	HeatOn();     //��������				
				
    }	
		
		 if(Sensor_P2_Moto_Stop_flag==1)
		{	       		
				HeatFanOff();  // �������ȷ���
			//	HeatOff();     //��������				
				
    }	
		

	
	}
	
} //end  Moto_control_sys


void Auto_Send_Paper_a4(void)
{
	
	if(MOTO_S1S7()==0)  //�峬ʱ��־
	{
	  auto_paper_a4_Max_Flag=0;
	}	
	
	
	if(auto_paper_a4_Max_Flag==0)  //����ʱ
	{
			if(MOTO_S1S7()==1)
			{
				auto_paper_a4_Count_Flag=1;
				//�������---������
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










