/******************************************************************

moto.H  file
  
*******************************************************************/

#ifndef __MOTO_H__
#define __MOTO_H__

#include "stc15w4k32s4.h"
#include "unit.h"


#define  HEATTIME 1  //��Ƭ��ͨ���м���ʱ�� ��λ��

#define  MAPAN_MAX_CONT  30  //���ν�Ƭ���ȼ���������

#define  MAPAN_SEND_PAPER_CONT  1800  //���ν�Ƭ���ȼ���������



/********************************������Ʋ�����************************************/
/* �������RunLed�ĺ����� */
#define MOTO1_IN1	P37	   	// LED ='0' ��ʾ���ƣ�LED ='1' ��ʾ���
#define MOTO1_IN1Tog()		MOTO1_IN1=!MOTO1_IN1	  //LED��˸
#define MOTO1_IN1Off()		MOTO1_IN1=0			    //LED�ر�
#define MOTO1_IN1On()			MOTO1_IN1=1			    //LED��


/* �������RunLed�ĺ����� */
#define MOTO1_IN2	P21	   	// LED ='0' ��ʾ���ƣ�LED ='1' ��ʾ���

#define MOTO1_IN2Tog()		MOTO1_IN2=!MOTO1_IN2	  //LED��˸
#define MOTO1_IN2Off()		MOTO1_IN2=0			    //LED�ر�
#define MOTO1_IN2On()			MOTO1_IN2=1			    //LED��



/* �������RunLed�ĺ����� */
#define MOTO2_IN3	P23	   	// LED ='0' ��ʾ���ƣ�LED ='1' ��ʾ���

#define MOTO2_IN3Tog()		MOTO2_IN3=!MOTO2_IN3	  //LED��˸
#define MOTO2_IN3Off()		MOTO2_IN3=0			    //LED�ر�
#define MOTO2_IN3On()			MOTO2_IN3=1			    //LED��

/* �������RunLed�ĺ����� */
#define MOTO2_IN4	P22	   	// LED ='0' ��ʾ���ƣ�LED ='1' ��ʾ���
#define MOTO2_IN4Tog()		MOTO2_IN4=!MOTO2_IN4	  //LED��˸
#define MOTO2_IN4Off()		MOTO2_IN4=0			    //LED�ر�
#define MOTO2_IN4On()			MOTO2_IN4=1			    //LED��


/************************************end*******������Ʋ�����***********************/

#define FORWARD   1   //��ת
#define REVERSE   0   //��ת

#define CYCLE     100UL     //����PWM����(���ֵΪ32767)  // ���� 9.28us     107.7KHZ



//��ͨ������ƺ���
void MotoDc1Forward(void);
void MotoDc1Reverse(void);
void MotoDc1Stop(void);

void MotoDc2Forward(void);
void MotoDc2Reverse(void);
void MotoDc2Stop(void);
//PWM �������
void MotoDc1Run(u8 time, u8 direction , u8 runtime);
void MotoDc2Run(u8 time, u8 direction ,u8 runtime);
//��Ͽ���
void Moto_DC2_Control(u8 TYPE );
void Moto_control_sys(void);

//20161104  ����PWM����
void PWM_config(void);

//PWM ������ƺ���
void PWM2_SetPwmWide(u16 Wide);  	
void PWM3_SetPwmWide(u16 Wide);
void PWM4_SetPwmWide(u16 Wide);
void PWM5_SetPwmWide(u16 Wide);
//void PWM6_SetPwmWide(u16 Wide);
//void PWM7_SetPwmWide(u16 Wide);


void Auto_Send_Paper_a4(void);

#endif

/**///////////////////////Pino Electronics////////////////////////**/
