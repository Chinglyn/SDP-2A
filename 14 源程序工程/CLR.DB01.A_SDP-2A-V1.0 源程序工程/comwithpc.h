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

#define ADDRESSH 0X00  //同类板子地址编号，只有一块板子 地址为0
#define ADDRESSL 0X00  //同类板子地址编号，只有一块板子 地址为0
#define DATAHEAD 0x55  //帧头

#define DATAEND  0X0D  //结束符
#define DATADIR  0XFE  //结束符

#define HARDVER1  0X00 //结束符
#define HARDVER2  0X03 //结束符

#define FIREVER  0X12  //固件版本   1.0 不含蜂鸣器，1.1 含蜂鸣器 ，1.2 增加出纸通道
#define SOFTVER  0X10  //软件版本 预留

#define CRCERROR 0X02  //校验错误

#define OTHERERROR  0x03 //未识别指令

#define HEADERROR  0x01 //未识别指令
#define ENDERROR  0x08 //未识别指令


void comwithpc(void);


#endif

/**///////////////////////Pino Electronics////////////////////////**/


