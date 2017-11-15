/******************************************************************

DS18B20.H  file
  
*******************************************************************/
#include "stc15w4k32s4.h"
#include "unit.h"
#include "harddrive.h"
#include "uart1.h"
#include "ds18b20.h"
#include "moto.h"





#ifndef __COMWITHPC_H__
#define __COMWITHPC_H__

#define ADDRESSH 0X00  //ͬ����ӵ�ַ��ţ�ֻ��һ����� ��ַΪ0
#define ADDRESSL 0X00  //ͬ����ӵ�ַ��ţ�ֻ��һ����� ��ַΪ0
#define DATAHEAD 0x55  //֡ͷ

#define DATAEND  0X0D  //������
#define DATADIR  0XFE  //������

#define HARDVER1  0X00 //������
#define HARDVER2  0X03 //������

#define FIREVER  0X12  //�̼��汾   1.0 ������������1.1 �������� ��1.2 ���ӳ�ֽͨ��
#define SOFTVER  0X10  //����汾 Ԥ��

#define CRCERROR 0X02  //У�����

#define OTHERERROR  0x03 //δʶ��ָ��

#define HEADERROR  0x01 //δʶ��ָ��
#define ENDERROR  0x08 //δʶ��ָ��


void comwithpc(void);


#endif

/**///////////////////////Pino Electronics////////////////////////**/


