/****************************************Copyright (c)**************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:          Type.h
** Last modified Date: 2007-06-04
** Last Version:       1.0
** Description:        类型定义头文件
** 
**------------------------------------------------------------------------------------------------------
** Created By:         马亮
** Created date:       2007-06-04
** Version:            1.0
** Descriptions:       The original version 初始版本
**
**------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
********************************************************************************************************/

#ifndef  __TYPE_H__
#define  __TYPE_H__


/********************************************************************************************************
*                       Date types(Compiler specific)  数据类型（和编译器相关）                         *                 
********************************************************************************************************/
typedef unsigned char   uint8;          // Unsigned  8 bit quantity  无符号8位整型变量      
typedef signed   char   int8;           // Signed    8 bit quantity  有符号8位整型变量   
typedef unsigned short  uint16;         // Unsigned 16 bit quantity  无符号16位整型变量
typedef signed   short  int16;          // Signed   16 bit quantity  有符号16位整型变量 
typedef unsigned int    uint32;         // Unsigned 32 bit quantity  无符号32位整型变量 
typedef signed   int    int32;          // Signed   32 bit quantity  有符号32位整型变量
typedef float           fp32;           // Single precision floating point 单精度浮点数（32位长度）    
typedef double          fp64;           // Double precision floating point 双精度浮点数（64位长度）  

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL  0
#endif



#endif
/*********************************************************************************************************
*                                        End Of File                                                     *
*********************************************************************************************************/









