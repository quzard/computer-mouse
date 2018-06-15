/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               Zlg7289.h
** Last modified Date:      2008/01/03
** Last Version:            V1.00
** Description:             数码管显示与键盘管理芯片ZLG7289 针对MicroMouse615的驱动程序
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              廖茂刚
** Created date: 
** Version: 
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


#include "Zlg7289.h"


/*********************************************************************************************************
** Function name:       __delayNuS
** Descriptions:        延时N个微秒
** input parameters:    iTime: 延时时间
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __delayNuS (int32 iTime)
{
    iTime = SysCtlClockGet() * iTime / 2000000;                         /*  根据系统时钟速率确定延时    */
    while (--iTime != 0);
}

/*********************************************************************************************************
** Function name:       __zlg7289SPIWrite
** Descriptions:        向SPI 总线写入1 个字节的数据。
** input parameters:    cDat：要写入的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __zlg7289SPIWrite (int8 cDat)
{
    int8 cT = 8;
    GPIODirModeSet(GPIO_PORTA_BASE, ZLG7289_DIO, GPIO_DIR_MODE_OUT);    /*  设置DIO端口为输出模式       */
    /*
     *  循环写一个字节的数据
     */
    do {
        if((cDat & 0x80) == 0x80) {
            GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_DIO, 0xff);
        } else {
            GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_DIO, 0x00);
        }
        cDat <<= 1;
        GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CLK, 0xff);
        __delayNuS(5);
        GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CLK, 0x00);
        __delayNuS(5);
    } while (--cT != 0);
}


/*********************************************************************************************************
** Function name:       __zlg7289SPIRead
** Descriptions:        从SPI 总线读取1 个字节的数据。
** input parameters:    无
** output parameters:   无
** Returned value:      读取到的数据
*********************************************************************************************************/
int8 __zlg7289SPIRead (void)
{
    int8 cDat = 0;
    int8 cT   = 8;
    GPIODirModeSet(GPIO_PORTA_BASE, ZLG7289_DIO, GPIO_DIR_MODE_IN);     /* 设置DIO端口为输出模式        */
    /*
     *  循环读一个字节的数据
     */
    do {
        GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CLK, 0xff);
        __delayNuS(5);
        cDat <<= 1;
        if (GPIOPinRead(GPIO_PORTA_BASE, ZLG7289_DIO)) {
            cDat++;
        }
        GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CLK, 0x00);
        __delayNuS(5);
    } while (--cT != 0);
    return cDat;
}


/*********************************************************************************************************
** Function name:       zlg7289Cmd
** Descriptions:        执行ZLG7289 纯指令。
** input parameters:    cCmd：命令字
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void zlg7289Cmd (int8  cCmd)
{
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS, 0x00);
    __delayNuS(25);
    __zlg7289SPIWrite(cCmd);
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS, 0xff);
    __delayNuS(5);
}


/*********************************************************************************************************
** Function name:       zlg7289CmdDat
** Descriptions:        执行ZLG7289 带数据指令。
** input parameters:    cCmd：命令字
**                      cDat：数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void zlg7289CmdDat (uint8  cCmd, int8  cDat)
{
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS, 0x00);
    __delayNuS(25);
    __zlg7289SPIWrite(cCmd);
    __delayNuS(15);
    __zlg7289SPIWrite(cDat);
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS, 0xff);
    __delayNuS(5);
}


/*********************************************************************************************************
** Function name:       zlg7289Download
** Descriptions:        下载数据。
** input parameters:    ucMod=0： 下载数据且按方式0 译码
**                      ucMod=1： 下载数据且按方式1 译码
**                      ucMod=2： 下载数据但不译码
**                      cX：      数码管编号（横坐标），取值0～7
**                      cDp=0：   小数点不亮
**                      cDp=1：   小数点亮
**                      cDat：    要显示的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void zlg7289Download (uint8  ucMod, int8  cX, int8  cDp, int8  cDat)
{
    uint8 ucModDat[3] = {0x80,0xC8,0x90};
    uint8 ucD1;
    uint8 ucD2;
    
    if (ucMod > 2) {
        ucMod = 2;
    }
    
    ucD1  = ucModDat[ucMod];
    cX   &= 0x07;
    ucD1 |= cX;
    ucD2  = cDat & 0x7F;
    
    if (cDp  == 1) {
        ucD2 |= 0x80;
    }
    zlg7289CmdDat(ucD1, ucD2);
}


/*********************************************************************************************************
** Function name:       zlg7289Key
** Descriptions:        执行ZLG7289 键盘命令。
** input parameters:    无
** output parameters:   无
** Returned value:      返回读到的按键值：0～63。如果返回0xFF 则表示没有键按下
*********************************************************************************************************/
int8 zlg7289Key (void)
{
    int8 cKey;
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS, 0x00);
    __delayNuS(25);
    __zlg7289SPIWrite(0x15);
    __delayNuS(15);
    cKey = __zlg7289SPIRead();
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS, 0xff);
    __delayNuS(5);
    return cKey;
}


/*********************************************************************************************************
** Function name:       ZLG7289_Init
** Descriptions:        ZLG7289 初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void zlg7289Init (void)
{
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );                      /*  使能GPIO A口外设            */
    
    GPIODirModeSet(GPIO_PORTA_BASE,
                     ZLG7289_CS
                    |ZLG7289_CLK
                    |ZLG7289_DIO,
                     GPIO_DIR_MODE_OUT);                                /*  设置I/O口为输出模式         */
    
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_DIO, 0xff);
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CLK, 0x00);
    GPIOPinWrite(GPIO_PORTA_BASE, ZLG7289_CS , 0xff);
    
    zlg7289Reset();                                                     /*  复位ZLG7289                 */
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
