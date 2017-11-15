/******************************************************************

DS18B20.H  file
  
*******************************************************************/
#include "my_type.h"

#ifndef __DS18B20_H__
#define __DS18B20_H__

/*
//第一路DS18B20复位
u8 Ds18_1Reset(void);
//读第一路温度数据 上一次的转换数据
u8 DS18_1_ReadTemperature(s16 *pTemp);
//开启第一路温度转换
u8 DS18_1_StartConv(void);
u8 Ds18_2Reset(void);
u8 DS18_2_ReadTemperature(s16 *pTemp);
u8 DS18_2_StartConv(void);

*/

u8 Ds18_2Reset(void);  //温度传感器复位，用来检测是否有接温度传感器

void Delay100us(u32 x);   //误差 -0.008318865741us

void HandleDs18b20ReadTemp(void);





#endif

/**///////////////////////Pino Electronics////////////////////////**/


