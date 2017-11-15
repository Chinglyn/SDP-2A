#ifndef __HARDDRIVE_H__
#define __HARDDRIVE_H__

#include "stc15w4k32s4.h"
#include "unit.h"


//u8  sys_init_state=0;

#define SYSCLK 		11059200UL		//ϵͳʱ��Ƶ��Ϊ22118400Hz


/* �������RunLed�ĺ����� */
#define RunLed	P32	   	// LED ='0' ��ʾ���ƣ�LED ='1' ��ʾ���

#define RunLedTog()			RunLed=!RunLed	  //LED��˸
#define RunLedOff()			RunLed=1			    //LED�ر�
#define RunLedOn()			RunLed=0			    //LED��


/* ������Ƶ���Ƶĺ����� */

#define DaoGuangLed	P35	   	// '1' ��ʾ���� '0'  ��ʾ���

#define DaoGuangLedTog()			DaoGuangLed=!DaoGuangLed	  //LED��˸
#define DaoGuangLedOff()			DaoGuangLed=0			    //LED�ر�
#define DaoGuangLedOn()			  DaoGuangLed=1			    //LED��


/* ������Ʊ���Ƶĺ����� */

#define BaoGaoLed	P51	   	// '1' ��ʾ���� '0'  ��ʾ���

#define BaoGaoLedTog()			BaoGaoLed=!BaoGaoLed	  //LED��˸
#define BaoGaoLedOff()			BaoGaoLed=0			    //LED�ر�
#define BaoGaoLedOn()			  BaoGaoLed=1			    //LED��


/* ������ȿ��Ƶĺ����� */

#define Heat	P42	   	// '1' ��ʾ������ '0'  ��ʾ�ؼ���
#define HeatOff()	  	Heat=1			    //�ر�
#define HeatOn()		  Heat=0			    //��


/* ������ȷ��ȿ��Ƶĺ����� */

#define HeatFan	P43	   	// '1' ��ʾ������ '0'  ��ʾ�ؼ���


#define HeatFanOff()	  	HeatFan=1			    //�ر�
#define HeatFanOn()		    HeatFan=0			    //��


/* ����Ԥ�����ȿ��Ƶĺ����� */

#define Fan2	P36	   	// '1' ��ʾ������ '0'  ��ʾ�ؼ���


#define Fan2Off()	  	Fan2=0			    //LED�ر�
#define Fan2On()		  Fan2=1			    //LED��



/* ������������ƺ����� */

#define Buzzer	P53	   	// '1' ��ʾ������ '0'  ��ʾ�ؼ���


#define BuzzerOff()	  	Buzzer=0			    //�ر�
#define BuzzerOn()		  Buzzer=1			    //��


/* ���崫������������ */


//IPF510 ī���������

#define  Ipf510_C     P05
#define  IPF510_M     P06
#define  IPF510_Y     P07
#define  IPF510_BK    P10
#define  IPF510_MBK   P11

#define  IPF510_FM    P33

//7280 ī���������

#define  C7280_C     P47
#define  C7280_M     P12
#define  C7280_Y     P13
#define  C7280_BK    P14
#define  C7280_MBK   P15

#define  C7280_FM    P34




/*********************************IPF510 ī����⺯��***********************************/
//��IPF510_C ī��
u8 ReadIpf510_C(void);

//��IPF510_M ī��
u8 ReadIpf510_M(void);

//��IPF510_Y ī��
u8 ReadIpf510_Y(void);

//��IPF510_BK ī��
u8 ReadIpf510_BK(void);

//��IPF510_MBK ī��
u8 ReadIpf510_MBK(void);

//��IPF510_FM ī��
u8 ReadIpf510_FM(void);

/************************************END********************************************/

/*****************************7280 ī����⺯��**************************************/
//��7280_C ī��
u8 Read7280_C(void);

//��7280_M ī��
u8 Read7280_M(void);

//��7280_Y ī��
u8 Read7280_Y(void);

//��7280_BK ī��
u8 Read7280_BK(void);

//��7280_MBK ī��
u8 Read7280_MBK(void);

//��7280_FM ī��
u8 Read7280_FM(void);



/************************************end*******************************************/

/*******************************������������**************************************/
//������������

#define  MOTO_S8_S0    P52     //��Ӧ�е���ʼ����

#define  MOTO_S7_S1    P04     //��Ӧ�е���������  Ϊ��Ƭͨ��������ʼ�ṩ��ʶ��

#define  MOTO_S6_S2    P03     //��Ӧ��Ƭ�����Ƭͨ��
#define  MOTO_S5_S3    P02     //��⽺Ƭλ��
#define  MOTO_S4_S4    P01     //��⽺Ƭλ��
#define  MOTO_S3_S5    P00     //��Ӧ��Ƭ�뿪��Ƭͨ��
#define  MOTO_S2_S6    P46     //��Ӧ��Ƭλ��
#define  MOTO_S1_S7    P45 //Ԥ��


/*
�������̣�

��Ӧ�е�����������Ϊ�����Ｐͬ������ﴫ����ٳ�Ƭָ�

���е�֧���н�Ƭʱ�ߵ�1-P1�����������Ӧ��ʱ���������ת��3������ᣬ
ʹ��ɫ�������ѹ����ͬ��������ں�ɫ����ѹ�������ת�����ѽ�Ƭ�ͳ�ͨ����


��Ӧ��Ƭ�����Ƭͨ��������ָ���ͬ������������ת��
��֤ͨ������Ƭ��������������ָ���֤����������������


1-P3.1-P4���������ã�
��Ӧ��ɫ������λ�ã�����������ת������֤�����򿪼��ر�����

1-P6���������ã�
���ͬ�������������������

1-P5���������ã�
��⽺Ƭ�뿪��Ƭͨ�������﹤������ָ�


*/
u8 MOTO_S8S0(void);
u8 MOTO_S7S1(void);
u8 MOTO_S6S2(void);
u8 MOTO_S5S3(void);
u8 MOTO_S4S4(void);
u8 MOTO_S3S5(void);
u8 MOTO_S2S6(void);
u8 MOTO_S1S7(void);

/************************************end*******************************************/





void Delay100ms(void) ;  //��ʱ����
//void Delay1s(void) ;

//���ȼ��޿��ƺ���

void HEAT_CONTROL(void);


//�������ƺ���

void heart_control(void);

//���Ժ���


void  Ml_test(void);



#endif

/**///////////////////////Pino Electronics////////////////////////**/