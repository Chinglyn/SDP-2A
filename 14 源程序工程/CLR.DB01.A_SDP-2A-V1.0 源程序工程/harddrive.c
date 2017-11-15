/************************************************************************************
*
* CPU��STC15W4K3S4���İ�
* ʱ��ѡ���ⲿʱ��11.0592MHz
* ����˵�������Ӳ����������
* ������Ϣ���� ��
*************************************************************************************
*����˵��
*20160928 ����ͨ��Э��ģ��---����λ��ͨ�ţ� ����1  ������9600
*
************************************************************************************/

#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "moto.h"


/********************************************************************
�������ܣ���ʱ������
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
********************************************************************/
volatile	u8 Delay_start=0;  //��ʱ����PWM ����ֵ   ռ�ձ�
volatile	u8 Delay_num=0;  //��ʱ����PWM ����ֵ   ռ�ձ�
volatile	u8 Delay_count=0;  //��ʱ����PWM ����ֵ   ռ�ձ�
volatile	u8 Delay_flag=0;  //��ʱ����PWM ����ֵ   ռ�ձ�

volatile	s16 Heart_Open=1;   //����������־   
volatile	u8  Heart_flag=1;   //��������־
volatile	u32 Heart_Count=0;  //������ʱ����

//ī������Ƿ����α�־  01 ��ʾ����  02 ��ʾ����
u8 IPF510MT_PB=0x02;
u8 C7280MT_PB=0x02;
//��ī���
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
�������ܣ���ʱ������
��ڲ������ޡ�
��    �أ��ޡ�
��    ע���ޡ�
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

/* ����DS18B20ʹ�õ����� */
#define DS18B20			P22

#define DQ_1()  		DS18B20=1		/* SDA = 1 */
#define DQ_0()  		DS18B20=0		/* SDA = 0 */
	
#define DQ_READ()  	DS18B20	/* ��SDA��״̬ */



//��紫���� ����IO Ϊ�͵�ƽ��  ��īˮIOΪ�͵�ƽ����īΪ��
//���ݴ����� ����IO Ϊ�ߵ�ƽ��  ��īˮIOΪ�͵�ƽ����īΪ��

/*********************************IPF510 ī����⺯��***********************************/

//��IPF510_C ī��
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


//��IPF510_M ī��
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

//��IPF510_Y ī��
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


//��IPF510_BK ī��
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


//��IPF510_MBK ī��
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

//��IPF510_FM ī��
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

//���ݴ����� ����IO Ϊ�ߵ�ƽ��  ��īˮIOΪ�͵�ƽ����īΪ��
/*****************************7280 ī����⺯��**************************************/
//��7280_C ī��
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

//��7280_M ī��
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

//��7280_Y ī��
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


//��7280_BK ī��
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


//��7280_MBK ī��
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

//��7280_FM ī��
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


/*******************    *�����������⺯��****************************************/

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




//���������޿���

extern volatile	s16 temputer1;
extern volatile	s16 temputer2;


extern volatile u8 TemputerHardState1;
extern volatile u8 TemputerHardState2;

void HEAT_CONTROL(void)
{
	WDT_CONTR |= 0x10;      //ι��		
	
  if(temputer2>60)   //�¶ȳ���60�� �ؼ�����
	{
		HeatOff();     //�ؼ���
  }
	
	
	if(TemputerHardState2==0)
	{
		HeatOff();     // �¶ȴ������쳣 �ؼ���
  }	
	
}


//�������� 
void heart_control(void)
{
	WDT_CONTR |= 0x10;      //ι��		

	
	if(Heart_flag==0)
	{
    Fan2On(); //�������ƶ˿�
  }

		if(Heart_flag==1)
	{
    Fan2Off(); //��
  }


	
}






