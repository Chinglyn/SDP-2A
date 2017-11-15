/******************************************************************

DS18B20.H  file
  
*******************************************************************/
#include "my_type.h"

#ifndef __DS18B20_H__
#define __DS18B20_H__

/*
//��һ·DS18B20��λ
u8 Ds18_1Reset(void);
//����һ·�¶����� ��һ�ε�ת������
u8 DS18_1_ReadTemperature(s16 *pTemp);
//������һ·�¶�ת��
u8 DS18_1_StartConv(void);
u8 Ds18_2Reset(void);
u8 DS18_2_ReadTemperature(s16 *pTemp);
u8 DS18_2_StartConv(void);

*/

u8 Ds18_2Reset(void);  //�¶ȴ�������λ����������Ƿ��н��¶ȴ�����

void Delay100us(u32 x);   //��� -0.008318865741us

void HandleDs18b20ReadTemp(void);





#endif

/**///////////////////////Pino Electronics////////////////////////**/


