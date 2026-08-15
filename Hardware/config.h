#ifndef CONFIG_H
#define CONFIG_H
#include "stm32f4xx.h"
/**************************************************************
基础类型重命名
**************************************************************/
#define UINT8  unsigned char
#define INT8   char
#define UINT16 unsigned short
#define INT16  short
#define UINT32 unsigned long
#define INT32  long
/**************************************************************
C语言无NULL，宏定义NULL
**************************************************************/
#ifndef NULL
    #define NULL  (void*)0
#endif
/**************************************************************
C语言未提供BOOL型数据。
**************************************************************/
typedef enum
{
    FALSE,
    TRUE
}BOOL;
#endif
