/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse_Drive.c
** Last modified Date: 
** Last Version: 
** Description:         电脑鼠底层驱动
** 
**--------------------------------------------------------------------------------------------------------
** Created By: 
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


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Mouse_Drive.h"
#include "Maze.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0};              /*  保存电脑鼠当前位置坐标      */

uint8             GucMouseDir                     = UP;                 /*  保存电脑鼠当前方向          */

uint8             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
                                                                        /*  x,横坐标;y,纵坐标;          */
                                                                        /*  bit3~bit0分别代表左下右上   */
                                                                        /*  0:该方向无路，1:该方向有路  */

static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0};    /*  定义并初始化左电机状态      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0};    /*  定义并初始化右电机状态      */

static uint8    __GucMouseState                   = __STOP;             /*  保存电脑鼠当前运行状态      */
       uint32   __GuiAccelTable[400]              = {0};                /*  电机加减速各阶段定时器值    */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  保存允许运行的最大速度      */
static uint8    __GucDistance[5]                  = {0};                /*  记录传感器状态              */

uint8 		speedcontrol 			= 0 ;			/*电机控制方式*/
int8		motormethod			= 0;			/*电机转动方式*/
int8		turnflag	     		= 0 ;
uint8		turnbackspeed			= 45;   //？？？ 这个值是什么用？ 转弯时老在用
uint32  Acc0_temp=0;

/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __delay (uint32  uiD)
{
    for (; uiD; uiD--);
}

/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        右步进电机驱动时序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __rightMotorContr (void)
{
	static int8 cStep = 0;                                              /*  保存电机当前位置            */  
	if(motormethod)
	{
		switch(__GmRight.cDir)
		{
			case __MOTORGOAHEAD:
				cStep = (cStep + 2) % 8;
			break;

			case __MOTORGOBACK:
				cStep = (cStep + 6) % 7;

			default:
			break;
		}
		if(cStep % 2)
			cStep --;
	}
	else
	{
		switch (__GmRight.cDir)
		{
			case __MOTORGOAHEAD:                                                /*  向前步进                    */
				cStep = (cStep + 1) % 8;
			break;

			case __MOTORGOBACK:                                                 /*  向后步进                    */
					cStep = (cStep + 7) % 8;
			break;

			default:
			break;
		}
	}
	switch (cStep)
	{
		case 0:                                                             /*  A2B2                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2);
		break;

		case 1:                                                             /*  B2                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRA1 | __PHRA2);
		break;

		case 2:                                                             /*  A1B2                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRA1 | __PHRA2 | __PHRB2);
		break;

		case 3:                                                             /*  A1                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRB2);
		break;

		case 4:                                                             /*  A1B1                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRA2 | __PHRB2);
		break;

		case 5:                                                             /*  B1                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRA2);
		break;

		case 6:                                                             /*  A2B1                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRA2 | __PHRB1 | __PHRB2);
		break;

		case 7:                                                             /*  A2                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
			__PHRB1 | __PHRB2);
		break;

		default:
		break;
	}
}


/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        左步进电机驱动时序
** input parameters:    __GmLeft.cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __leftMotorContr (void)
{
	static int8 cStep = 0;                                              /*  保存电机当前位置            */
	if(motormethod)
	{
		switch(__GmLeft.cDir)
		{
			case __MOTORGOAHEAD:
				cStep = (cStep + 2) % 8;
			break;

			case __MOTORGOBACK:
				cStep = (cStep + 6) % 7;

			default:
			break;
		}
		if(cStep % 2)
			cStep --;
	}
	else
	{
		switch (__GmLeft.cDir)
		{
			case __MOTORGOAHEAD:                                                /*  向前步进                    */
				cStep = (cStep + 1) % 8;
			break;

			case __MOTORGOBACK:                                                 /*  向后步进                    */
				cStep = (cStep + 7) % 8;
			break;

			default:
			break;
		}
	}
	switch (cStep)
	{
		case 0:                                                             /*  A2B2                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2);
		break;

		case 1:                                                             /*  B2                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLB1 | __PHLB2);
		break;

		case 2:                                                             /*  A1B2                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLA2 | __PHLB1 | __PHLB2);
		break;

		case 3:                                                             /*  A1                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLA2);
		break;

		case 4:                                                             /*  A1B1                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLA2 | __PHLB2);
		break;

		case 5:                                                             /*  B1                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLB2);
		break;

		case 6:                                                             /*  A2B1                        */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLA1 | __PHLA2 | __PHLB2);
		break;

		case 7:                                                             /*  A2                          */
			GPIOPinWrite(GPIO_PORTD_BASE,
			__PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
			__PHLA1 | __PHLA2);
		break;

		default:
		break;
	}
}


/*********************************************************************************************************
** Function name:       __speedContrR
** Descriptions:        右电机速度调节
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __speedContrR (void)
{ 
	int32 iDPusle;
	switch (speedcontrol)
	{
		case 0:
			iDPusle = __GmRight.uiPulse - __GmRight.uiPulseCtr;           /*  统计电机还剩余的步数        */
			if (iDPusle <= __GmRight.iSpeed)
				__GmRight.iSpeed--;
			else if(__GmRight.iSpeed < __GiMaxSpeed)                    /*  非减速区间，则加速到最大值  */
					__GmRight.iSpeed++;
			if (__GmRight.iSpeed < 0)                                   /*  设置速度下限                */
				__GmRight.iSpeed = 0;
		break;

		case 1:
			if(__GmRight.iSpeed < __GiMaxSpeed)
				__GmRight.iSpeed ++;
			if(__GmRight.iSpeed > __GiMaxSpeed)
				__GmRight.iSpeed --;
		break;

		case 2:
			iDPusle = __GmRight.uiPulse - __GmRight.uiPulseCtr;          /*  统计电机还剩余的步数        */
			if(__GmRight.iSpeed >= (iDPusle + 2 * turnbackspeed+10))
				__GmRight.iSpeed --;
			else  if (__GmRight.iSpeed < __GiMaxSpeed)
					__GmRight.iSpeed ++;
		break;

		case 3:
			if(__GmRight.iSpeed < 80)
				__GmRight.iSpeed ++;
			if(__GmRight.iSpeed > 80)
				__GmRight.iSpeed --;
		break;

		default:
		break;
	}
	TimerLoadSet(TIMER0_BASE,TIMER_A,__GuiAccelTable[__GmRight.iSpeed]);/*  设置定时时间                */
}



/*********************************************************************************************************
** Function name:       __speedContrL
** Descriptions:        左电机速度调节
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __speedContrL (void)
{
	int32 iDPusle;
	switch (speedcontrol)
	{
		case 0:
			iDPusle = __GmLeft.uiPulse - __GmLeft.uiPulseCtr;                   /*  统计电机还剩余的步数        */
			if (iDPusle <= __GmLeft.iSpeed)
				__GmLeft.iSpeed--;
			else  if(__GmLeft.iSpeed < __GiMaxSpeed)                      /*  非减速区间，则加速到最大值  */
					__GmLeft.iSpeed++;
			if (__GmLeft.iSpeed < 0)                                          /*  设置速度下限                */
				__GmLeft.iSpeed = 0;
		break;

		case 1:
			if(__GmLeft.iSpeed < __GiMaxSpeed)
				__GmLeft.iSpeed ++;
			if(__GmLeft.iSpeed > __GiMaxSpeed)
				__GmLeft.iSpeed --;
		break;

		case 2:
			iDPusle = __GmLeft.uiPulse - __GmLeft.uiPulseCtr;                   /*  统计电机还剩余的步数        */
			if(__GmLeft.iSpeed >= (iDPusle + 2 * turnbackspeed+10))
				__GmLeft.iSpeed --;
			else if (__GmLeft.iSpeed < __GiMaxSpeed)
					__GmLeft.iSpeed ++;
		break;

		case 3:
			if(__GmLeft.iSpeed < 80)
				__GmLeft.iSpeed ++;
			if(__GmLeft.iSpeed > 80)
				__GmLeft.iSpeed --;
		break;

		default:
		break;
	}
	TimerLoadSet(TIMER1_BASE,TIMER_A,__GuiAccelTable[__GmLeft.iSpeed]); /*  设置定时时间                */
}


/*********************************************************************************************************
** Function name:       Timer0A_ISR
** Descriptions:        Timer0中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer0A_ISR(void)
{
	static int8 n = 0,m = 0;
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                     /*  清除定时器0中断。           */
	switch (__GmRight.cState)
	{  
		case __MOTORSTOP:                                                   /*  停止，同时清零脉冲值  */
			__GmRight.uiPulse    = 0;
			__GmRight.uiPulseCtr = 0;
		break;

		case __WAITONESTEP:                                                 /*  暂停一步                    */
			__GmRight.cState     = __MOTORRUN;
		break;

		case __MOTORRUN:                                                    /*  电机运行                    */
			if (__GucMouseState == __GOAHEAD)			/*  根据传感器状态微调电机位置  */
			{                            
				if (__GucDistance[__FRONTL] && (__GucDistance[__FRONTR] == 0))
				{
					if (n == 1)
						__GmRight.cState = __WAITONESTEP;
					n++;
					n %= 2;
				}
				else
					n = 0;

				if ((__GucDistance[__RIGHT] == 1) && (__GucDistance[__LEFT] == 0))
				{
					if(m == 3)
						__GmRight.cState = __WAITONESTEP;
					m++;
					m %= 6;
				}
				else
					m  = 0;
			}
			__rightMotorContr();                                            /*  电机驱动程序                */
		break;

		default:
		break;
	}
	/*
	*  是否完成任务判断
	*/
	if (__GmRight.cState != __MOTORSTOP)
	{
		__GmRight.uiPulseCtr++;                                         /*  运行脉冲计数                */
		__speedContrR();                                                /*  速度调节                    */
		if (__GmRight.uiPulseCtr >= __GmRight.uiPulse)
		{
			__GmRight.cState      = __MOTORSTOP;
			__GmRight.uiPulseCtr  = 0;
			__GmRight.uiPulse     = 0;
                     //此处删去了速度值检测为0   __GmRight.iSpeed      = 0;

		}
	}
}


/*********************************************************************************************************
** Function name:       Timer1A_ISR
** Descriptions:        Timer1中断服务函数
** input parameters:    __GmLeft.cState :驱动步进电机的时序状态
**                      __GmLeft.cDir   :步进电机运动的方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer1A_ISR(void)
{
    static int8 n = 0, m = 0;
    
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                     /*  清除定时器1中断。           */
    switch (__GmLeft.cState)
	{
		case __MOTORSTOP:                                                   /*  停止，同时清零脉冲值  */
			__GmLeft.uiPulse    = 0;
                        //此处删去了速度值 __GmLeft.iSpeed     = 0;
			__GmLeft.uiPulseCtr = 0;
        break;
        
		case __WAITONESTEP:                                                 /*  暂停一步                    */
		  __GmLeft.cState     = __MOTORRUN;
        break;

		case __MOTORRUN:                                                    /*  电机运行                    */
			if (__GucMouseState == __GOAHEAD)                            /*  根据传感器状态微调电机位置  */
			{ 
				if (__GucDistance[__FRONTR] &&(__GucDistance[__FRONTL]==0))
				{
					if (n == 1)
				      __GmLeft.cState = __WAITONESTEP;
					n++;
					 n %= 2;
				}
				else 
					n = 0;

				if ((__GucDistance[__LEFT] == 1) && (__GucDistance[__RIGHT] == 0))
				{
					if(m == 3)
						__GmLeft.cState = __WAITONESTEP;
					m++;
					m %= 6;
				}
				else
					m  = 0;
			}
		 __leftMotorContr();                                             /*  电机驱动程序                */
		break;

		default:
        break;
    }
    /*
     *  是否完成任务判断
     */
    if (__GmLeft.cState != __MOTORSTOP)
	{
        __GmLeft.uiPulseCtr++;                                          /*  运行脉冲计数                */
        __speedContrL();                                                /*  速度调节                    */
        if (__GmLeft.uiPulseCtr >= __GmLeft.uiPulse)
		{
            __GmLeft.cState      = __MOTORSTOP;
            __GmLeft.uiPulseCtr  = 0;
            __GmLeft.uiPulse     = 0;
            //__GmLeft.iSpeed     = 0;
        }
    }
}

void acc_change(int kai){
      uint16 n = 0;
//     kai=1581139;
    __GuiAccelTable[0] = kai;//2236068
    __GuiAccelTable[1] = 0.4142*kai;
    for(n = 2; n < 400; n++) {
        __GuiAccelTable[n] = __GuiAccelTable[n - 1] - (2 * __GuiAccelTable[n - 1] / (4 * n + 1));
    }
}
/****************************************************************
** Function name:   backTurnright 
** Descriptions:   在返回过程中探测右边挡板
		测到挡板后根据不同情况确定应前进的步数
** output parameters:   无
** Returned value:      无
****************************************************************/
void backTurnright ()
{
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);//__MOTORSTOP 已经没有速度值为0
	/*
	 *  开始右转
	 */
//       zlg7289Reset();
//       zlg7289Download(1, 0, 0, 0x0B);
//       zlg7289Download(0, 1, 0, 0x0E);
//        acc_change(2001139);
//      acc_change(2236068);

	__GucMouseState   = __TURNRIGHT;
	__GmLeft.uiPulse = 87;//86
	__GmLeft.iSpeed = 80;
	__GmLeft.cState   = __MOTORRUN;
        
	while ((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) > 3);//剩余步数大于3
	__GmRight.uiPulse  = (turnbackspeed - 40)/3 +7; //??? 这个又疑惑，不懂
	__GmRight.cState   = __MOTORRUN;
	while (__GmRight.cState != __MOTORSTOP);
	__GmLeft.iSpeed = 40;
//        acc_change(1901139);
//      acc_change(2236068);

}

/****************************************************************
** Function name: backTurnleft
** Descriptions:  在返回过程中探测左边挡板
		测到挡板后根据不同情况确定应前进的步数
** output parameters:   无
** Returned value:      无
****************************************************************/
void backTurnleft ()
{
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	/*
	 *  开始左转
	 */
//        zlg7289Reset();
//        zlg7289Download(1, 0, 0, 0x0B);
//        zlg7289Download(0, 1, 0, 0x0D);
//        acc_change(2001139);
//      acc_change(2236068);
        
        
	__GucMouseState   = __TURNLEFT;
	__GmRight.uiPulse = 87;//86
	__GmRight.iSpeed = 80;
	__GmRight.cState  = __MOTORRUN;
	while ((__GmRight.uiPulse - __GmRight.uiPulseCtr) > 3);
	__GmLeft.uiPulse  = (turnbackspeed - 40)/3 +7;
	__GmLeft.cState   = __MOTORRUN;
	while (__GmLeft.cState  != __MOTORSTOP);
	__GmRight.iSpeed = 40;
//        acc_change(1901139);
//      acc_change(2236068);

}

/****************************************************************
** Function name:		  backturnback
** Descriptions:    		在返回过程中探测左边挡板
					测到挡板后根据不同情况确定应前进的步数
** output parameters:   无
** Returned value:      无
****************************************************************/

void backTurnback(void)
{
	/*
	 *  等待停止
	 */
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	/*
	 *  开始后转
	 */
//        __GmLeft.iSpeed=0;
//        __GmRight.iSpeed=0;
        Acc0_temp=__GuiAccelTable[0];
        acc_change(1881139);//可能会打滑？ 回到起点 1381139
//        acc_change(2236068);
//        zlg7289Reset();
//        zlg7289Download(1, 0, 0, 0x0B);
//        zlg7289Download(1, 1, 0, 0x0B);
	 motormethod = 1;
	__GiMaxSpeed = 15;
	__GucMouseState   = __TURNBACK;
	__GmRight.cDir    = __MOTORGOBACK;
	__GmRight.uiPulse = 45;//44
	__GmLeft.uiPulse  = 45;//44
	 speedcontrol = 0;
	__GmLeft.cState   = __MOTORRUN;
	__GmRight.cState  = __MOTORRUN;
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	motormethod = 0;
	__GiMaxSpeed = MAXSPEED;
	__GmRight.cDir = __MOTORGOAHEAD;
        
        acc_change(Acc0_temp);
}



/*********************************************************************************************************
** Function name:       objectGoTo
** Descriptions:        使电脑鼠按最快方式运动到指定坐标
** input parameters:    cXdst: 目的地的横坐标
**                      cYdst: 目的地的纵坐标
** output parameters:   无
** Returned value:      无
** GuiStep[][]:		第一个参数：向前应该走的格数
			第二个参数：0：不转弯；1：向右转；2：向左转；3：向后转
*********************************************************************************************************/
//void objectGoTo (int8  cXdst, int8  cYdst, uint8 mode )
//{
//	uint8   i = 1, cL = 0, cR = 0, offset = 0;
//        int jiasu=0;//5*spurt_time;
//        
//	static uint8 gapflag = 0; //??? 这个参数没有看懂
//        
//	mapStepEdit(cXdst,cYdst);	 /*  制作等高图，建立返回数组*/  
//	if( GucDirFlag == 1)		   //GucDirFlag  冲刺区别      /*是否重新冲刺*/
//	{
//		GuiStep[0][1]=0;
//		GucDirFlag = 0;
//	}
//    switch (mode)		/*根据不同的返回模式，采取不同的返回速度*/
//    {
//		case __BACK:
//			turnbackspeed = 45 + jiasu;//45;
//		break;
//
//		case __START:
//                  if(gapflag == 1){
//				turnbackspeed = 40 + jiasu;//40;
//
//
//                  }
//                        
//                  else{
//				turnbackspeed = 45+ jiasu;//45;     
//                  }
//		break;
//
//		case __END:
//			turnbackspeed = 50+ jiasu;//50;
//			gapflag = 1;
//		break;
//
//		default:
//		break;
//		
//	}
//	switch (GuiStep[0][1])		/*转弯前先转弯*/
//	{
//		case 0:
//			if(!mode)
//			{
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 50;
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 50 ;
//				speedcontrol = 1;
//				__GmLeft.cState = __MOTORRUN;
//				__GmRight.cState = __MOTORRUN;
//				while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) > 5);
//			}
//			else
//				GucDirFlag = 0;
//		break;
//		
//		case 1:
//			backTurnright();
//		break;
//
//		case 2:
//			backTurnleft();
//		break;
//
//		case 3:
// 			if((!(__GucDistance[__LEFT] && __GucDistance[__RIGHT])))	/*左右至少有一边存在缺口,向前走40步*/
// 			{
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 38;//40;
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 38;//40 ;
//				speedcontrol = 0;
//				__GmLeft.cState = __MOTORRUN;
//				__GmRight.cState = __MOTORRUN;
//				while (__GmLeft.cState  != __MOTORSTOP);
//				while (__GmRight.cState != __MOTORSTOP);
//				__GmLeft.iSpeed = 0;
//				__GmRight.iSpeed = 0;
// 			}
// 			backTurnback();
//			speedcontrol = 1;
//		break;
//
//		default:
//		break;
//	}
//	while(GuiStep[i][0] != 0)	/*根据返回数组，开始执行返回*/
//	{
//		__GucMouseState   = __GOAHEAD;
//		offset = 0;
//		if(GuiStep[i][0] > 1)	/*快速向前走GuiStep[i][0]-1个格子，目标位置为最后一个格子的中央*/
//		{	
//			__GiMaxSpeed  =  3 *  turnbackspeed + jiasu;////////////////////////////////
//			__GmRight.uiPulse =  (GuiStep[i][0] - 1) * ONEBLOCK - 20; 
//			__GmLeft.uiPulse  =  (GuiStep[i][0] - 1) * ONEBLOCK - 20;
//			if((i == 1) && ((GuiStep[0][1] == 3) || (GuiStep[0][1] == 0)))
//			{
//				__GmLeft.uiPulse += 20;
//				__GmRight.uiPulse += 20;
//			}
//			speedcontrol = 2;
//			__GmRight.cState  = __MOTORRUN;
//			__GmLeft.cState   = __MOTORRUN;
//			while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) > 2);
//		}
//		__GiMaxSpeed      =  SEARCHSPEED + jiasu;///////////////////////////////////////////
//		__GmRight.uiPulseCtr = 0;
//		__GmLeft.uiPulseCtr = 0;
//		switch (GuiStep[i][1])	/*准备转弯*/
//		{
//			case 0:		/*不需要转弯，说明已到达目标点*/
//				switch (mode)
//				{
//					case __START:
//	 					__GmRight.uiPulse =  ONEBLOCK;
//						__GmLeft.uiPulse  =  ONEBLOCK;
//						__GiMaxSpeed      =  turnbackspeed + jiasu;/////////////////////////////////
//						speedcontrol = 0;
//						__GmRight.cState  = __MOTORRUN;
//						__GmLeft.cState   = __MOTORRUN;
//						while(__GmLeft.cState != __MOTORSTOP)
//						{
//							if (__GucDistance[__FRONT] == 0x03)   /*根据前方挡板判断是否到达迷宫出发点*/
//							{ 
//								__GmRight.uiPulse = __GmRight.uiPulseCtr + 60;
//								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 60;
//								while (__GucDistance[ __FRONT] == 0x03)
//								{
//									if((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) <5)
//										return;
//								}
//							}
//						}
//					break;
//
//					case __END:
//					case __BACK:
//						speedcontrol = 3;
//						if((GuiStep[i][0] > 1) || (GuiStep[i-1][1] ==0))/*走直线到达目标点,此时速度较快，根据左右缺口判断是否到达目标点*/
//						{
//							__GmRight.uiPulse =  2 * ONEBLOCK;
//							__GmLeft.uiPulse  = 2 * ONEBLOCK;
//							__GmRight.cState  = __MOTORRUN;
//							__GmLeft.cState   = __MOTORRUN;
//							while(__GmLeft.cState != __MOTORSTOP)
//							{
//								if (cL) 	/* 是否允许检测左边*/
//								{                                                
//									if (!__GucDistance[ __LEFT])   	/*左边有支路，则跳出程序*/
//									{         
//										speedcontrol = 4;
//										__GmLeft.iSpeed = 40;
//										__GmRight.iSpeed = 40;
//										__GmRight.uiPulse = __GmRight.uiPulseCtr + 43;//45
//										__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 43;//45
//										while (!__GucDistance[ __LEFT] )	/*连续检测，防止误判*/
//										{
//											if ((__GmLeft.uiPulseCtr + 5) > __GmLeft.uiPulse)/*确定到达目标点，跳出*/
//												return;
//										}
//										__GmRight.uiPulse =  ONEBLOCK;	/*误判，恢复*/
//										__GmLeft.uiPulse  = ONEBLOCK;
//									}
//								} 
//								else if ( __GucDistance[ __LEFT] ) 	/*左边有墙时开始允许检测左边*/
//									cL = 1;
//								if (cR)            /*是否允许检测右边*/
//								{
//									if (!__GucDistance[ __RIGHT] )/*右边有支路，则跳出程序*/
//									{
//										speedcontrol = 4;
//										__GmLeft.iSpeed = 40;
//										__GmRight.iSpeed = 40;
//										__GmRight.uiPulse = __GmRight.uiPulseCtr + 43;//45
//										__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 43;//45
//										while (!__GucDistance[ __RIGHT])/*连续检测，防止误判*/
//										{
//											if ((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) < 10)/*确定到大目标点，跳出*/
//												return;
//										}
//										__GmRight.uiPulse = ONEBLOCK;	/*误判，恢复*/
//										__GmLeft.uiPulse  = ONEBLOCK;
//									}
//								} 
//								else if ( __GucDistance[__RIGHT] )	/*右边有墙时开始允许检测右边*/               
//									cR = 1;
//							}
//						}
//						else if((GuiStep[i-1][1] == 1) || (GuiStep[i-1][1] == 2))/*通过左转或右转到达目标点，此时速度较慢，且可能转过头*/
//						{
//							__GmRight.uiPulse =  ONEBLOCK;
//							__GmLeft.uiPulse  =  ONEBLOCK ;
//							speedcontrol = 4;
//							__GmLeft.iSpeed = 40;		/*控制速度为40，和下次转弯速度匹配*/
//							__GmRight.iSpeed = 40;
//							__GmRight.cState  = __MOTORRUN;
//							__GmLeft.cState   = __MOTORRUN;
//							while (__GmLeft.cState != __MOTORSTOP)
//							{ 
//								if(!__GucDistance[__LEFT])	/*左边有支路，则跳出程序*/
//								{
//									__GmRight.uiPulse = __GmRight.uiPulseCtr + 46;//48
//									__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 46;//48
//									while(!__GucDistance[__LEFT])		/*连续检测，防止误判*/
//									{
//										if((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) < 10)/*确定到大目标点，跳出*/
//											return;
//									} 
//									__GmRight.uiPulse =  ONEBLOCK;	/*误判，恢复*/
//									__GmLeft.uiPulse  =  ONEBLOCK ;
//								}
//								if(!__GucDistance[__RIGHT])		/*右边有支路，则跳出程序*/
//								{ 
//									__GmRight.uiPulse = __GmRight.uiPulseCtr + 46;//48
//									__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 46;//48
//									while(!__GucDistance[__RIGHT] )		/*连续检测，防止误判*/
//									{
//								 		if((__GmRight.uiPulse - __GmRight.uiPulseCtr ) < 10)	/*确定到大目标点，跳出*/
//											return;
//									} 
//									__GmRight.uiPulse =  ONEBLOCK;		/*误判，恢复*/
//									__GmLeft.uiPulse  =  ONEBLOCK ;
//								}
//							}
//						}
//						else if(GuiStep[i-1][1] == 3)			/*向后转弯后到达目标点*/
//						{
//							__GmRight.uiPulse =  ONEBLOCK;
//							__GmLeft.uiPulse  =  ONEBLOCK;
//							speedcontrol = 1;
//							__GmRight.cState  = __MOTORRUN;
//							__GmLeft.cState   = __MOTORRUN;
//							while (__GmLeft.cState != __MOTORSTOP)
//							{ 
//								if(cL)					/*是否允许检测左边*/
//								{
//									if(!__GucDistance[__LEFT])
//									{
//										__GmRight.uiPulse = __GmRight.uiPulseCtr + 44;//45
//										 __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 44;//45
//										while(!__GucDistance[__LEFT])
//										{
//											if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*控制速度为不大于40，尽量和下次转弯速度匹配*/
//											{
//												speedcontrol = 4;
//												__GmLeft.iSpeed = 40;
//												__GmRight.iSpeed = 40;
//											}
//											if((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) < 10)	/*确定到大目标点，跳出*/
//												return;
//										} 
//									}
//								}
//								else if ( __GucDistance[__LEFT] )		/*左边有墙时开始允许检测左边*/               
//									cL = 1;
//								if(cR)					/*是否允许检测右边*/
//								{
//									if(!__GucDistance[__RIGHT])
//									{ 
//										__GmRight.uiPulse = __GmRight.uiPulseCtr + 44;//45
//										__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 44;//45
//										while(!__GucDistance[__RIGHT] )
//										{
//											if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*控制速度为不大于40，尽量和下次转弯速度匹配*/
//											{
//												speedcontrol = 4;
//												__GmLeft.iSpeed = 40;
//												__GmRight.iSpeed = 40;
//											}
//										 	if((__GmRight.uiPulse - __GmRight.uiPulseCtr ) < 10)	/*确定到大目标点，跳出*/
//												return;
//										} 
//									}
//								}
//								else if ( __GucDistance[__RIGHT] )			/*右边有墙时开始允许检测右边*/               
//									cR = 1;
//							}
//						}
//					break;
//
//					default:
//					break;
//				}
//			break;
//			
//			case 1:								/*向右转*/
//				if((GuiStep[i][0] > 1) || (GuiStep[i-1][1] == 0) || (GuiStep[i-1][1] == 3))  /*此时电脑鼠所在位置不会超出可检测范围，容易通过挡板校正*/
//				{
//					__GmRight.uiPulse = 2 *  ONEBLOCK;
//					__GmLeft.uiPulse  =  2 * ONEBLOCK;
//					__GmRight.cState  = __MOTORRUN;
//					__GmLeft.cState   = __MOTORRUN;
//					speedcontrol = 3;			/*使速度向80靠近*/
//					if(!__GucDistance[__RIGHT])		/*右边有缺口，不是目标缺口，等待下一个缺口出现*/
//					{
//						while(__GmRight.cState != __MOTORSTOP)
//						{
//							if(__GucDistance[__RIGHT])
//								break;
//						}
//					}
//					while(__GmRight.cState != __MOTORSTOP)
//					{
//						if(!__GucDistance[__RIGHT])	/*检测到右边有缺口，准备转弯*/
//						{ 
//							if((GuiStep[i-1][1] == 3) && (GuiStep[i][0] == 1))
//							{
//								__GmRight.uiPulse = __GmRight.uiPulseCtr + 43;//45
//								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 43;//45
//							}
//							else					/*如果直线加速到达该点，由于惯性，转弯应向前走的走数较少*/
//							{
//								__GmRight.uiPulse = __GmRight.uiPulseCtr + 65 - turnbackspeed/2;
//								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 65 - turnbackspeed/2;
//							}
//							while(!__GucDistance[__RIGHT])		/*连续检测，防止误判*/
//							{
//								if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*保证转弯前速度不大于40*/
//								{
//									speedcontrol = 4;
//									__GmLeft.iSpeed =  40;
//									__GmRight.iSpeed =  40;
//								}
//					 			if((__GmRight.uiPulseCtr + 2) > __GmRight.uiPulse)
//								{
//									backTurnright();
//									break;
//								}
//							} 
//						}
//					}
//				}
//				else if((GuiStep[i-1][1] == 1) || (GuiStep[i-1][1] == 2))/*转弯前也是转弯，说明是连续转弯，此时有可能超出也检测范围而无法校正*/
//				{
//					speedcontrol = 4;
//					__GmLeft.iSpeed =  40;
//					__GmRight.iSpeed =  40;
//					__GmRight.uiPulse = ONEBLOCK;
//					__GmLeft.uiPulse  =  ONEBLOCK;
//					__GmRight.cState  = __MOTORRUN;
//					__GmLeft.cState   = __MOTORRUN;
//					while(__GmRight.cState != __MOTORSTOP)
//					{
//						if(!__GucDistance[__RIGHT])
//						{ 
//							__GmRight.uiPulse = __GmRight.uiPulseCtr + 61 + offset - turnbackspeed/2;//62
//							__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 61 + offset - turnbackspeed/2;//62
//							while(!__GucDistance[__RIGHT])
//							{
//						 		if((__GmRight.uiPulseCtr + 2) > __GmRight.uiPulse)
//								{
//									if(GuiStep[i-1][1] == 2)
//										__GmLeft.uiPulse += 5;
//									backTurnright();
//									break;
//								}
//							} 
//							__GmRight.uiPulse = ONEBLOCK;
//							__GmLeft.uiPulse  =  ONEBLOCK;
//						}
//						offset = 5;			/*电脑鼠还未超出可检测范围，适当增加应向前走的步数*/
//					}
//				}
//			break;
//			
//			case 2:						/*向左转*/
//				if((GuiStep[i][0] > 1) || (GuiStep[i-1][1] == 0) || (GuiStep[i-1][1] == 3))  /*此时电脑鼠所在位置不会超出可检测范围，容易通过挡板校正*/
//				{
//					__GmRight.uiPulse = 2 *  ONEBLOCK;
//					__GmLeft.uiPulse  =  2 * ONEBLOCK;
//					__GmRight.cState  = __MOTORRUN;
//					__GmLeft.cState   = __MOTORRUN;
//					speedcontrol = 3;				/*使速度向80靠近*/
//					if(!__GucDistance[__LEFT])			/*左边有缺口，不是目标缺口，等待下一个缺口出现*/
//					{	
//						while(__GmLeft.cState != __MOTORSTOP)
//						{
//							if(__GucDistance[__LEFT])
//								break;
//						}
//					}
//					while(__GmLeft.cState != __MOTORSTOP)
//					{
//						if(!__GucDistance[__LEFT])		/*检测到左边有缺口，准备转弯*/
//						{ 
//							if((GuiStep[i-1][1] == 3) && (GuiStep[i][0] == 1))
//							{
//								__GmRight.uiPulse = __GmRight.uiPulseCtr + 43;//45
//								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 43;//45
//							}
//							else						/*如果直线加速到达该点，由于惯性，转弯应向前走的走数较少*/
//							{
//								__GmRight.uiPulse = __GmRight.uiPulseCtr + 65 - turnbackspeed/2;
//								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 65 - turnbackspeed/2;
//							}
//							while(!__GucDistance[__LEFT])		/*连续检测，防止误判*/
//							{
//								if((__GmLeft.iSpeed >  40) && (__GmRight.iSpeed > 40))	/*保证转弯前速度不大于40*/
//								{
//									speedcontrol = 4;
//									__GmLeft.iSpeed =  40;
//									__GmRight.iSpeed =  40;
//								}
//					 			if((__GmLeft.uiPulseCtr + 2) > __GmLeft.uiPulse)
//								{
//									backTurnleft();
//									break;
//								}
//							} 
//						}
//					}
//		             	}
//				else if((GuiStep[i-1][1] == 1) || (GuiStep[i-1][1] == 2))	/*转弯前也是转弯，说明是连续转弯，此时有可能超出也检测范围而无法校正*/
//				{
//					speedcontrol = 4;
//					__GmLeft.iSpeed =  40;
//					__GmRight.iSpeed =  40;
//					__GmRight.uiPulse = ONEBLOCK;
//					__GmLeft.uiPulse  =  ONEBLOCK;
//					__GmRight.cState  = __MOTORRUN;
//					__GmLeft.cState   = __MOTORRUN;
//					while(__GmLeft.cState != __MOTORSTOP)
//					{
//						if(!__GucDistance[__LEFT])
//						{ 
//							__GmRight.uiPulse = __GmRight.uiPulseCtr + 62 + offset - turnbackspeed/2;
//							__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 62 + offset - turnbackspeed/2;
//							while(!__GucDistance[__LEFT])
//							{
//						 		if((__GmLeft.uiPulseCtr + 2) > __GmLeft.uiPulse)
//								{
//									if(GuiStep[i-1][1] == 1)
//										__GmRight.uiPulse += 5;
//									backTurnleft();
//									break;
//								}
//							} 
//							__GmRight.uiPulse = ONEBLOCK;
//							__GmLeft.uiPulse  =  ONEBLOCK;
//						}
//						offset = 5;				/*电脑鼠还未超出可检测范围，适当增加应向前走的步数*/
//					}
//				}
//			break;
//
//			default:
//			break;
//		}
//		i++;
//	}	
//}

void objectGoTo (int8  cXdst, int8  cYdst, uint8 mode)
{
	uint8   i = 1, cL = 0, cR = 0, offset = 0;
        int jiasu=0;
        if(spurt_time==1) {jiasu=4;}
        else if(spurt_time==2){ jiasu=jiasu+3;}
        else {
          jiasu+=2;
        }
//        zlg7289Download(1, 5, 0, jiasu / 10);
//        zlg7289Download(1, 6, 0, jiasu % 10);

//        if(jiasu>6) jiasu=6;
        
	static uint8 gapflag = 0; //??? 这个参数没有看懂
        if((2136068-spurt_time*130000)>2000000) acc_change(2136068-spurt_time*100000);
        else acc_change(1990000);
        
	mapStepEdit(cXdst,cYdst);	 /*  制作等高图，建立返回数组*/  
	if( GucDirFlag == 1)		   //GucDirFlag  冲刺区别      /*是否重新冲刺*/
	{
		GuiStep[0][1]=0;
		GucDirFlag = 0;
	}
    switch (mode)		/*根据不同的返回模式，采取不同的返回速度*/
    {
		case __BACK:
			turnbackspeed = 45+jiasu;//45
		break;
		case __START:
                  if(gapflag == 1){
				turnbackspeed = 40+jiasu;//40
                  }
                        
                  else{
				turnbackspeed = 45+jiasu;//45     
                  }
		break;

		case __END:
			turnbackspeed = 50+jiasu;//50
			gapflag = 1;
		break;

		default:
		break;
		
    }
    switch (GuiStep[0][1])		/*转弯前先转弯*/
    {
                case 0:
			if(!mode)
			{
				__GmRight.uiPulse = __GmRight.uiPulseCtr + 50;
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 50 ;
				speedcontrol = 1;
				__GmLeft.cState = __MOTORRUN;
				__GmRight.cState = __MOTORRUN;
				while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) > 5);
			}
			else
				GucDirFlag = 0;
		break;
		
		case 1:
			backTurnright();
		break;

		case 2:
			backTurnleft();
		break;

		case 3:
 			if((!(__GucDistance[__LEFT] && __GucDistance[__RIGHT])))	/*左右至少有一边存在缺口,向前走40步*/
 			{
				__GmRight.uiPulse = __GmRight.uiPulseCtr + 40;
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 40 ;
				speedcontrol = 0;
				__GmLeft.cState = __MOTORRUN;
				__GmRight.cState = __MOTORRUN;
				while (__GmLeft.cState  != __MOTORSTOP);
				while (__GmRight.cState != __MOTORSTOP);
				__GmLeft.iSpeed = 0;
				__GmRight.iSpeed = 0;
 			}
 			backTurnback();
			speedcontrol = 1;
		break;

		default:
		break;
	}
	while(GuiStep[i][0] != 0)	/*根据返回数组，开始执行返回*/
	{
		__GucMouseState   = __GOAHEAD;
		offset = 0;
		if(GuiStep[i][0] > 1)	/*快速向前走GuiStep[i][0]-1个格子，目标位置为最后一个格子的中央*/
		{	
			__GiMaxSpeed  =  3 *  turnbackspeed;
			__GmRight.uiPulse =  (GuiStep[i][0] - 1) * ONEBLOCK - 20; 
			__GmLeft.uiPulse  =  (GuiStep[i][0] - 1) * ONEBLOCK - 20;
			if((i == 1) && ((GuiStep[0][1] == 3) || (GuiStep[0][1] == 0)))
			{
				__GmLeft.uiPulse += 20;
				__GmRight.uiPulse += 20;
			}
			speedcontrol = 2;
			__GmRight.cState  = __MOTORRUN;
			__GmLeft.cState   = __MOTORRUN;
			while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) > 2);
		}
		__GiMaxSpeed      =  SEARCHSPEED;
		__GmRight.uiPulseCtr = 0;
		__GmLeft.uiPulseCtr = 0;
		switch (GuiStep[i][1])	/*准备转弯*/
		{
			case 0:		/*不需要转弯，说明已到达目标点*/
				switch (mode)
				{
					case __START:
	 					__GmRight.uiPulse =  ONEBLOCK;
						__GmLeft.uiPulse  =  ONEBLOCK;
						__GiMaxSpeed      =  turnbackspeed;
						speedcontrol = 0;
						__GmRight.cState  = __MOTORRUN;
						__GmLeft.cState   = __MOTORRUN;
						while(__GmLeft.cState != __MOTORSTOP)
						{
							if (__GucDistance[__FRONT] == 0x03)   /*根据前方挡板判断是否到达迷宫出发点*/
							{ 
								__GmRight.uiPulse = __GmRight.uiPulseCtr + 60;
								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 60;
								while (__GucDistance[ __FRONT] == 0x03)
								{
									if((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) <5)
										return;
								}
							}
						}
					break;

					case __END:
					case __BACK:
						speedcontrol = 3;
						if((GuiStep[i][0] > 1) || (GuiStep[i-1][1] ==0))/*走直线到达目标点,此时速度较快，根据左右缺口判断是否到达目标点*/
						{
							__GmRight.uiPulse =  2 * ONEBLOCK;
							__GmLeft.uiPulse  = 2 * ONEBLOCK;
							__GmRight.cState  = __MOTORRUN;
							__GmLeft.cState   = __MOTORRUN;
							while(__GmLeft.cState != __MOTORSTOP)
							{
								if (cL) 	/* 是否允许检测左边*/
								{                                                
									if (!__GucDistance[ __LEFT])   	/*左边有支路，则跳出程序*/
									{         
										speedcontrol = 4;
										__GmLeft.iSpeed = 40;
										__GmRight.iSpeed = 40;
										__GmRight.uiPulse = __GmRight.uiPulseCtr + 42;//45
										__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 42;//45
										while (!__GucDistance[ __LEFT] )	/*连续检测，防止误判*/
										{
											if ((__GmLeft.uiPulseCtr + 5) > __GmLeft.uiPulse)/*确定到达目标点，跳出*/
												return;
										}
										__GmRight.uiPulse =  ONEBLOCK;	/*误判，恢复*/
										__GmLeft.uiPulse  = ONEBLOCK;
									}
								} 
								else if ( __GucDistance[ __LEFT] ) 	/*左边有墙时开始允许检测左边*/
									cL = 1;
								if (cR)            /*是否允许检测右边*/
								{
									if (!__GucDistance[ __RIGHT] )/*右边有支路，则跳出程序*/
									{
										speedcontrol = 4;
										__GmLeft.iSpeed = 40;
										__GmRight.iSpeed = 40;
										__GmRight.uiPulse = __GmRight.uiPulseCtr + 41;//45
										__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 41;//45
										while (!__GucDistance[ __RIGHT])/*连续检测，防止误判*/
										{
											if ((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) < 10)/*确定到大目标点，跳出*/
												return;
										}
										__GmRight.uiPulse = ONEBLOCK;	/*误判，恢复*/
										__GmLeft.uiPulse  = ONEBLOCK;
									}
								} 
								else if ( __GucDistance[__RIGHT] )	/*右边有墙时开始允许检测右边*/               
									cR = 1;
							}
						}
						else if((GuiStep[i-1][1] == 1) || (GuiStep[i-1][1] == 2))/*通过左转或右转到达目标点，此时速度较慢，且可能转过头*/
						{
							__GmRight.uiPulse =  ONEBLOCK;
							__GmLeft.uiPulse  =  ONEBLOCK ;
							speedcontrol = 4;
							__GmLeft.iSpeed = 40;		/*控制速度为40，和下次转弯速度匹配*/
							__GmRight.iSpeed = 40;
							__GmRight.cState  = __MOTORRUN;
							__GmLeft.cState   = __MOTORRUN;
							while (__GmLeft.cState != __MOTORSTOP)
							{ 
								if(!__GucDistance[__LEFT])	/*左边有支路，则跳出程序*/
								{
									__GmRight.uiPulse = __GmRight.uiPulseCtr + 48;
									__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 48;
									while(!__GucDistance[__LEFT])		/*连续检测，防止误判*/
									{
										if((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) < 10)/*确定到大目标点，跳出*/
											return;
									} 
									__GmRight.uiPulse =  ONEBLOCK;	/*误判，恢复*/
									__GmLeft.uiPulse  =  ONEBLOCK ;
								}
								if(!__GucDistance[__RIGHT])		/*右边有支路，则跳出程序*/
								{ 
									__GmRight.uiPulse = __GmRight.uiPulseCtr + 48;//48
									__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 48;//48
									while(!__GucDistance[__RIGHT] )		/*连续检测，防止误判*/
									{
								 		if((__GmRight.uiPulse - __GmRight.uiPulseCtr ) < 10)	/*确定到大目标点，跳出*/
											return;
									} 
									__GmRight.uiPulse =  ONEBLOCK;		/*误判，恢复*/
									__GmLeft.uiPulse  =  ONEBLOCK ;
								}
							}
						}
						else if(GuiStep[i-1][1] == 3)			/*向后转弯后到达目标点*/
						{
							__GmRight.uiPulse =  ONEBLOCK;
							__GmLeft.uiPulse  =  ONEBLOCK;
							speedcontrol = 1;
							__GmRight.cState  = __MOTORRUN;
							__GmLeft.cState   = __MOTORRUN;
							while (__GmLeft.cState != __MOTORSTOP)
							{ 
								if(cL)					/*是否允许检测左边*/
								{
									if(!__GucDistance[__LEFT])
									{
										__GmRight.uiPulse = __GmRight.uiPulseCtr + 45;
										 __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 45;
										while(!__GucDistance[__LEFT])
										{
											if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*控制速度为不大于40，尽量和下次转弯速度匹配*/
											{
												speedcontrol = 4;
												__GmLeft.iSpeed = 40;
												__GmRight.iSpeed = 40;
											}
											if((__GmLeft.uiPulse - __GmLeft.uiPulseCtr ) < 10)	/*确定到大目标点，跳出*/
												return;
										} 
									}
								}
								else if ( __GucDistance[__LEFT] )		/*左边有墙时开始允许检测左边*/               
									cL = 1;
								if(cR)					/*是否允许检测右边*/
								{
									if(!__GucDistance[__RIGHT])
									{ 
										__GmRight.uiPulse = __GmRight.uiPulseCtr + 44;//45
										__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 44;//45
										while(!__GucDistance[__RIGHT] )
										{
											if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*控制速度为不大于40，尽量和下次转弯速度匹配*/
											{
												speedcontrol = 4;
												__GmLeft.iSpeed = 40;
												__GmRight.iSpeed = 40;
											}
										 	if((__GmRight.uiPulse - __GmRight.uiPulseCtr ) < 10)	/*确定到大目标点，跳出*/
												return;
										} 
									}
								}
								else if ( __GucDistance[__RIGHT] )			/*右边有墙时开始允许检测右边*/               
									cR = 1;
							}
						}
					break;

					default:
					break;
				}
			break;
			
			case 1:								/*向右转*/
				if((GuiStep[i][0] > 1) || (GuiStep[i-1][1] == 0) || (GuiStep[i-1][1] == 3))  /*此时电脑鼠所在位置不会超出可检测范围，容易通过挡板校正*/
				{
					__GmRight.uiPulse = 2 *  ONEBLOCK;
					__GmLeft.uiPulse  =  2 * ONEBLOCK;
					__GmRight.cState  = __MOTORRUN;
					__GmLeft.cState   = __MOTORRUN;
					speedcontrol = 3;			/*使速度向80靠近*/
					if(!__GucDistance[__RIGHT])		/*右边有缺口，不是目标缺口，等待下一个缺口出现*/
					{
						while(__GmRight.cState != __MOTORSTOP)
						{
							if(__GucDistance[__RIGHT])
								break;
						}
					}
					while(__GmRight.cState != __MOTORSTOP)
					{
						if(!__GucDistance[__RIGHT])	/*检测到右边有缺口，准备转弯*/
						{ 
							if((GuiStep[i-1][1] == 3) && (GuiStep[i][0] == 1))
							{
								__GmRight.uiPulse = __GmRight.uiPulseCtr + 45;
								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 45;
							}
							else					/*如果直线加速到达该点，由于惯性，转弯应向前走的走数较少*/
							{
								__GmRight.uiPulse = __GmRight.uiPulseCtr + 65 - turnbackspeed/2;
								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 65 - turnbackspeed/2;
							}
							while(!__GucDistance[__RIGHT])		/*连续检测，防止误判*/
							{
								if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*保证转弯前速度不大于40*/
								{
									speedcontrol = 4;
									__GmLeft.iSpeed =  40;
									__GmRight.iSpeed =  40;
								}
					 			if((__GmRight.uiPulseCtr + 2) > __GmRight.uiPulse)
								{
									backTurnright();
									break;
								}
							} 
						}
					}
				}
				else if((GuiStep[i-1][1] == 1) || (GuiStep[i-1][1] == 2))/*转弯前也是转弯，说明是连续转弯，此时有可能超出也检测范围而无法校正*/
				{
					speedcontrol = 4;
					__GmLeft.iSpeed =  40;
					__GmRight.iSpeed =  40;
					__GmRight.uiPulse = ONEBLOCK;
					__GmLeft.uiPulse  =  ONEBLOCK;
					__GmRight.cState  = __MOTORRUN;
					__GmLeft.cState   = __MOTORRUN;
					while(__GmRight.cState != __MOTORSTOP)
					{
						if(!__GucDistance[__RIGHT])
						{ 
							__GmRight.uiPulse = __GmRight.uiPulseCtr + 62 + offset - turnbackspeed/2;
							__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 62 + offset - turnbackspeed/2;
							while(!__GucDistance[__RIGHT])
							{
						 		if((__GmRight.uiPulseCtr + 2) > __GmRight.uiPulse)
								{
									if(GuiStep[i-1][1] == 2)//左转加右转
										__GmLeft.uiPulse += 5;
									backTurnright();
									break;
								}
							} 
							__GmRight.uiPulse = ONEBLOCK;
							__GmLeft.uiPulse  =  ONEBLOCK;
						}
						offset = 5;			/*电脑鼠还未超出可检测范围，适当增加应向前走的步数*/
					}
				}
			break;
			
			case 2:						/*向左转*/
				if((GuiStep[i][0] > 1) || (GuiStep[i-1][1] == 0) || (GuiStep[i-1][1] == 3))  /*此时电脑鼠所在位置不会超出可检测范围，容易通过挡板校正*/
				{
					__GmRight.uiPulse = 2 *  ONEBLOCK;
					__GmLeft.uiPulse  =  2 * ONEBLOCK;
					__GmRight.cState  = __MOTORRUN;
					__GmLeft.cState   = __MOTORRUN;
					speedcontrol = 3;				/*使速度向80靠近*/
					if(!__GucDistance[__LEFT])			/*左边有缺口，不是目标缺口，等待下一个缺口出现*/
					{	
						while(__GmLeft.cState != __MOTORSTOP)
						{
							if(__GucDistance[__LEFT])
								break;
						}
					}
					while(__GmLeft.cState != __MOTORSTOP)
					{
						if(!__GucDistance[__LEFT])		/*检测到左边有缺口，准备转弯*/
						{ 
							if((GuiStep[i-1][1] == 3) && (GuiStep[i][0] == 1))
							{
								__GmRight.uiPulse = __GmRight.uiPulseCtr + 43;//45
								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 43;//45
							}
							else						/*如果直线加速到达该点，由于惯性，转弯应向前走的走数较少*/
							{
								__GmRight.uiPulse = __GmRight.uiPulseCtr + 65 - turnbackspeed/2;
								__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 65 - turnbackspeed/2;
							}
							while(!__GucDistance[__LEFT])		/*连续检测，防止误判*/
							{
								if((__GmLeft.iSpeed >  40) && (__GmRight.iSpeed > 40))	/*保证转弯前速度不大于40*/
								{
									speedcontrol = 4;
									__GmLeft.iSpeed =  40;
									__GmRight.iSpeed =  40;
								}
					 			if((__GmLeft.uiPulseCtr + 2) > __GmLeft.uiPulse)
								{
									backTurnleft();
									break;
								}
							} 
						}
					}
		             	}
				else if((GuiStep[i-1][1] == 1) || (GuiStep[i-1][1] == 2))	/*转弯前也是转弯，说明是连续转弯，此时有可能超出也检测范围而无法校正*/
				{
					speedcontrol = 4;
					__GmLeft.iSpeed =  40;
					__GmRight.iSpeed =  40;
					__GmRight.uiPulse = ONEBLOCK;
					__GmLeft.uiPulse  =  ONEBLOCK;
					__GmRight.cState  = __MOTORRUN;
					__GmLeft.cState   = __MOTORRUN;
					while(__GmLeft.cState != __MOTORSTOP)
					{
						if(!__GucDistance[__LEFT])
						{ 
							__GmRight.uiPulse = __GmRight.uiPulseCtr + 62 + offset - turnbackspeed/2;
							__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 62 + offset - turnbackspeed/2;
							while(!__GucDistance[__LEFT])
							{
						 		if((__GmLeft.uiPulseCtr + 2) > __GmLeft.uiPulse)
								{
									if(GuiStep[i-1][1] == 1)//右转加左转
										__GmRight.uiPulse += 5;
									backTurnleft();
									break;
								}
							} 
							__GmRight.uiPulse = ONEBLOCK;
							__GmLeft.uiPulse  =  ONEBLOCK;
						}
						offset = 5;				/*电脑鼠还未超出可检测范围，适当增加应向前走的步数*/
					}
				}
			break;

			default:
			break;
		}
		i++;
	}	
}





/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
//void mazeSearch(void)
//{
//  int jiasu=0;//2*spurt_time;
//   int8  cL = 0, cR = 0, sL = 0, sR = 0;
//   __GucMouseState   = __GOAHEAD;
//  if((turnflag == 1) || (turnflag == 2))	/*刚转完弯，与走直道分开处理*/
//  {
//		speedcontrol = 4;
//		__GmLeft.iSpeed = 40;
//		__GmRight.iSpeed = 40;
//		__GmRight.uiPulse = 75;
//		__GmLeft.uiPulse = 75;
//		__GmRight.cState  = __MOTORRUN;
//		__GmLeft.cState  = __MOTORRUN;
//		while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) > 5)
//		{ 
//			if(!__GucDistance[__LEFT] )   		/*左边有支路 */
//			{
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 41;//45;//38
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 41;//45;
//				while(!__GucDistance[__LEFT])	/*连续检测，防止误判*/
//				{
//					if((__GmRight.uiPulseCtr + 10) > __GmRight.uiPulse)	/*确定有支路，跳出*/
//					{
//						__mouseCoorUpdate();
//						if(turnflag == 1)//先右转了左转	/*在转弯函数里用到，判断是否为不同方向的连续转弯*/
//							turnflag = 3;
//						else//左转了之后左转
//							turnflag = 4;
//						return;
//					}
//				} 
//				__GmRight.uiPulse = 75;			/*误判，恢复*/
//				__GmLeft.uiPulse  = 75;
//			}
//			if(!__GucDistance[__RIGHT])			/*右边有支路 */
//			{ 
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 38;//45//35
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 38;//45
//				while(!__GucDistance[__RIGHT])		/*连续检测，防止误判*/
//				{
//					if((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定有支路，跳出*/
//					{
//						__mouseCoorUpdate();
//						if(turnflag == 1)//右转了右转		/*在转弯函数里用到，判断是否为不同方向的连续转弯*/
//							turnflag = 3;
//						else
//							turnflag = 4;//左转了右转
//						return;
//					}
//				} 
//				__GmRight.uiPulse = 75;			 /*误判，恢复*/
//				__GmLeft.uiPulse  = 75;
//			}
//			if(__GucDistance[__FRONT] == 0x03)		/*前方有墙，则跳出程序*/
//			{
//				speedcontrol = 0;
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 60;
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 60;
//				while(__GucDistance[__FRONT] == 0x03)		/*连续检测，防止误判*/
//				{
//					if((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定无路可走，跳出*/
//					{
//						__mouseCoorUpdate();
//						turnflag = 0;
//						return;
//					}
//				} 
//				speedcontrol = 4;					/*误判，恢复之前状态*/
//				__GmLeft.iSpeed = 40;
//				__GmRight.iSpeed = 40;
//    			__GiMaxSpeed = SEARCHSPEED + jiasu;///////////////////////////////////
//				__GmRight.uiPulse = 75;
//				__GmLeft.uiPulse  = 75;
//			}
//		}
//		__mouseCoorUpdate();						/*更新坐标*/
//	} 
//  	else if (__GmLeft.cState)
//  	{
//  		speedcontrol = 1;
//		__GmRight.uiPulse = __GmRight.uiPulseCtr + 30;
//		__GmLeft.uiPulse = __GmLeft.uiPulseCtr + 30;
//		while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) > 2);
//	}
//	turnflag = 0;
//	__GmLeft.uiPulseCtr = 0;
//	__GmRight.uiPulseCtr = 0;
//	__GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
//	__GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;
//	speedcontrol = 1;
//    __GiMaxSpeed = SEARCHSPEED + jiasu;/////////////////////////////////////////////////
//	__GmRight.cState  = __MOTORRUN;
//	__GmLeft.cState   = __MOTORRUN; 
//	while (__GmLeft.cState != __MOTORSTOP)
//	{
//		if (__GmLeft.uiPulseCtr >= ONEBLOCK)    	/*判断是否走完一格*/
//		{                    
//			__GmLeft.uiPulse    -= ONEBLOCK;
//			__GmLeft.uiPulseCtr -= ONEBLOCK;
//			__mouseCoorUpdate();			/*更新坐标*/
//		}
//		if (__GmRight.uiPulseCtr >= ONEBLOCK)		/*判断是否走完一格*/
//		{                      
//			__GmRight.uiPulse    -= ONEBLOCK;
//			__GmRight.uiPulseCtr -= ONEBLOCK;
//		}
//		if (cL) 					/*是否允许检测左边*/
//		{                                                
//			if (!__GucDistance[ __LEFT])   		/*左边有支路，则跳出程序*/
//			{         
//				sL = __GmLeft.iSpeed;
//				sR = __GmRight.iSpeed;
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 37;//42
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 37;//42
//				while (!__GucDistance[ __LEFT] )	/*连续检测，防止误判*/
//				{
//					if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*调整为转弯速度*/
//					{
//						speedcontrol = 4;
//						__GmLeft.iSpeed = 40;
//						__GmRight.iSpeed = 40;
//					}
//					if ((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定有支路，跳出*/
//					{
//						__mouseCoorUpdate();
//						return;
//					}
//				}
//				if((__GmLeft.iSpeed == 40) && (__GmRight.iSpeed == 40)&&(sL != 0)&&(sR != 0)) /*误判，恢复之前状态*/
//				{
//					__GmLeft.iSpeed = sL;
//					__GmRight.iSpeed = sR;
//					speedcontrol = 1;
//					__GmRight.uiPulse = MAZETYPE * ONEBLOCK;
//					__GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
//				}
//			}
//		} 
//		else if ( __GucDistance[ __LEFT] ) 			/*左边有墙时开始允许检测左边  */
//			cL = 1;
//		if (cR)							 /*是否允许检测右边*/
//		{
//			if (!__GucDistance[ __RIGHT] )			/*右边有支路，则跳出程序*/
//			{
//				sL = __GmLeft.iSpeed;
//				sR = __GmRight.iSpeed;
//				__GmRight.uiPulse = __GmRight.uiPulseCtr + 32;//42
//				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 32;//42
//				while (!__GucDistance[ __RIGHT])		/*连续检测，防止误判*/
//				{
//					if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))/*调整为转弯速度*/
//					{
//						speedcontrol = 4;
//						__GmLeft.iSpeed = 40;
//						__GmRight.iSpeed = 40;
//					}
//					if ((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定有支路，跳出*/
//					{
//						__mouseCoorUpdate();
//						return;
//					}
//				}
//				if((__GmLeft.iSpeed == 40) && (__GmRight.iSpeed == 40)&&(sL != 0)&&(sR != 0)) /*误判，恢复之前状态*/
//				{
//					__GmLeft.iSpeed = sL;
//					__GmRight.iSpeed = sR;
//					speedcontrol = 1;
//					__GmRight.uiPulse = MAZETYPE * ONEBLOCK;
//					__GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
//				}
//			}
//		} 
//		else if ( __GucDistance[__RIGHT] )			/*  右边有墙时开始允许检测右边*/               
//			cR = 1;
//		if (__GucDistance[__FRONT] == 0x03)			/*前方有墙，则跳出程序*/
//		{ 
//			speedcontrol = 0;
//			sL = __GmLeft.iSpeed;
//			sR = __GmRight.iSpeed;
//			__GmRight.uiPulse = __GmRight.uiPulseCtr + 60;
//			__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 60;
//			while (__GucDistance[ __FRONT] == 0x03)			/*连续检测，防止误判*/
//			{
//				if((__GmLeft.iSpeed > 55) && (__GmRight.iSpeed > 55))/*调整为转弯速度*/
//				{
//					__GmLeft.iSpeed = 55;
//					__GmRight.iSpeed = 55;
//				}
//				if ((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定无路可走，跳出*/
//				{
//					__mouseCoorUpdate();
//					return;
//				}
//			}
//			if((__GmLeft.iSpeed == 55) && (__GmRight.iSpeed == 55)&&(sL != 0)&&(sR != 0)) /*误判，恢复之前状态*/
//			{
//				__GmLeft.iSpeed = sL;
//				__GmRight.iSpeed = sR;
//				speedcontrol = 1;
//				__GmRight.uiPulse = MAZETYPE * ONEBLOCK;
//				__GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
//			}
//		}
//     }
//}


void mazeSearch(void)
{
   int8  cL = 0, cR = 0, sL = 0, sR = 0;
   __GucMouseState   = __GOAHEAD;
  if((turnflag == 1) || (turnflag == 2))	/*刚转完弯，与走直道分开处理*/
  {
		speedcontrol = 4;
		__GmLeft.iSpeed = 40;
		__GmRight.iSpeed = 40;
		__GmRight.uiPulse = 75;
		__GmLeft.uiPulse = 75;
		__GmRight.cState  = __MOTORRUN;
		__GmLeft.cState  = __MOTORRUN;
		while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) > 5)
		{ 
			if(!__GucDistance[__LEFT] )   		/*左边有支路 */
			{
				__GmRight.uiPulse = __GmRight.uiPulseCtr + 40;//前一次转弯，接着左转
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 40;
				while(!__GucDistance[__LEFT])	/*连续检测，防止误判*/
				{
					if((__GmRight.uiPulseCtr + 10) > __GmRight.uiPulse)	/*确定有支路，跳出*/
					{
						__mouseCoorUpdate();
						if(turnflag == 1)//之前右转	/*在转弯函数里用到，判断是否为不同方向的连续转弯*/
							turnflag = 3;//转弯后左转
						else//之前左转
							turnflag = 4;//turnflag==2(左转) 
						return;
					}
				} 
				__GmRight.uiPulse = 75;			/*误判，恢复*/
				__GmLeft.uiPulse  = 75;
			}
			if(!__GucDistance[__RIGHT])			/*右边有支路 */
			{ 
				__GmRight.uiPulse = __GmRight.uiPulseCtr + 38;//45 前一次转弯，接着右转
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 38;//45
				while(!__GucDistance[__RIGHT])		/*连续检测，防止误判*/
				{
					if((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定有支路，跳出*/
					{
						__mouseCoorUpdate();
						if(turnflag == 1)		/*在转弯函数里用到，判断是否为不同方向的连续转弯*/
							turnflag = 3;
						else
							turnflag = 4;
						return;
					}
				} 
				__GmRight.uiPulse = 75;			 /*误判，恢复*/
				__GmLeft.uiPulse  = 75;
			}
			if(__GucDistance[__FRONT] == 0x03)		/*前方有墙，则跳出程序*/
			{
				speedcontrol = 0;

				__GmRight.uiPulse = __GmRight.uiPulseCtr + 60;//60;
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 60;//60;
                                
				while(__GucDistance[__FRONT] == 0x03)		/*连续检测，防止误判*/
				{
					if((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定无路可走，跳出*/
					{
						__mouseCoorUpdate();
						turnflag = 0;
						return;
					}
				} 
				speedcontrol = 4;					/*误判，恢复之前状态*/
				__GmLeft.iSpeed = 40;
				__GmRight.iSpeed = 40;
    			__GiMaxSpeed = SEARCHSPEED;
				__GmRight.uiPulse = 75;
				__GmLeft.uiPulse  = 75;
			}
		}
		__mouseCoorUpdate();						/*更新坐标*/
	} 
  	else if (__GmLeft.cState)
  	{
  		speedcontrol = 1;
		__GmRight.uiPulse = __GmRight.uiPulseCtr + 30;
		__GmLeft.uiPulse = __GmLeft.uiPulseCtr + 30;
		while((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) > 2);
	}
	turnflag = 0;
	__GmLeft.uiPulseCtr = 0;
	__GmRight.uiPulseCtr = 0;
	__GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
	__GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;
	speedcontrol = 1;
        __GiMaxSpeed = SEARCHSPEED;
	__GmRight.cState  = __MOTORRUN;
	__GmLeft.cState   = __MOTORRUN; 
	while (__GmLeft.cState != __MOTORSTOP)
	{
		if (__GmLeft.uiPulseCtr >= ONEBLOCK)    	/*判断是否走完一格*/
		{                    
			__GmLeft.uiPulse    -= ONEBLOCK;
			__GmLeft.uiPulseCtr -= ONEBLOCK;
			__mouseCoorUpdate();			/*更新坐标*/
		}
		if (__GmRight.uiPulseCtr >= ONEBLOCK)		/*判断是否走完一格*/
		{                      
			__GmRight.uiPulse    -= ONEBLOCK;
			__GmRight.uiPulseCtr -= ONEBLOCK;
		}
		if (cL) 					/*是否允许检测左边*/
		{                                                
			if (!__GucDistance[ __LEFT])   		/*左边有支路，则跳出程序*/
			{         
				sL = __GmLeft.iSpeed;
				sR = __GmRight.iSpeed;
				__GmRight.uiPulse = __GmRight.uiPulseCtr + 38;//42
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 38;//42
				while (!__GucDistance[ __LEFT] )	/*连续检测，防止误判*/
				{
					if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))	/*调整为转弯速度*/
					{
						speedcontrol = 4;
						__GmLeft.iSpeed = 40;
						__GmRight.iSpeed = 40;
					}
					if ((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定有支路，跳出*/
					{
						__mouseCoorUpdate();
						return;
					}
				}
				if((__GmLeft.iSpeed == 40) && (__GmRight.iSpeed == 40)&&(sL != 0)&&(sR != 0)) /*误判，恢复之前状态*/
				{
					__GmLeft.iSpeed = sL;
					__GmRight.iSpeed = sR;
					speedcontrol = 1;
					__GmRight.uiPulse = MAZETYPE * ONEBLOCK;
					__GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
				}
			}
		} 
		else if ( __GucDistance[ __LEFT] ) 			/*左边有墙时开始允许检测左边  */
			cL = 1;
		if (cR)							 /*是否允许检测右边*/
		{
			if (!__GucDistance[ __RIGHT] )			/*右边有支路，则跳出程序*/
			{
				sL = __GmLeft.iSpeed;
				sR = __GmRight.iSpeed;

				__GmRight.uiPulse = __GmRight.uiPulseCtr +34;//42
				__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  +34;//42
				while (!__GucDistance[ __RIGHT])		/*连续检测，防止误判*/
				{
					if((__GmLeft.iSpeed > 40) && (__GmRight.iSpeed > 40))/*调整为转弯速度*/
					{
						speedcontrol = 4;
						__GmLeft.iSpeed = 40;
						__GmRight.iSpeed = 40;
					}
					if ((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定有支路，跳出*/
					{
						__mouseCoorUpdate();
						return;
					}
				}
				if((__GmLeft.iSpeed == 40) && (__GmRight.iSpeed == 40)&&(sL != 0)&&(sR != 0)) /*误判，恢复之前状态*/
				{
					__GmLeft.iSpeed = sL;
					__GmRight.iSpeed = sR;
					speedcontrol = 1;
					__GmRight.uiPulse = MAZETYPE * ONEBLOCK;
					__GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
				}
			}
		} 
		else if ( __GucDistance[__RIGHT] )			/*  右边有墙时开始允许检测右边*/               
			cR = 1;
		if (__GucDistance[__FRONT] == 0x03)			/*前方有墙，则跳出程序*/
		{ 
			speedcontrol = 0;
			sL = __GmLeft.iSpeed;
			sR = __GmRight.iSpeed;
			__GmRight.uiPulse = __GmRight.uiPulseCtr + 60;//60
			__GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 60;//60
			while (__GucDistance[ __FRONT] == 0x03)			/*连续检测，防止误判*/
			{
				if((__GmLeft.iSpeed > 55) && (__GmRight.iSpeed > 55))/*调整为转弯速度*/
				{
					__GmLeft.iSpeed = 55;
					__GmRight.iSpeed = 55;
				}
				if ((__GmLeft.uiPulseCtr + 10) > __GmLeft.uiPulse)	/*确定无路可走，跳出*/
				{
					__mouseCoorUpdate();
					return;
				}
			}
			if((__GmLeft.iSpeed == 55) && (__GmRight.iSpeed == 55)&&(sL != 0)&&(sR != 0)) /*误判，恢复之前状态*/
			{
				__GmLeft.iSpeed = sL;
				__GmRight.iSpeed = sR;
				speedcontrol = 1;
				__GmRight.uiPulse = MAZETYPE * ONEBLOCK;
				__GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
			}
		}
     }
}





/*********************************************************************************************************
** Function name:       SysTick_ISR
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SysTick_ISR(void)
{
    static int32 iL = 0, iR = 0;
    if (__GmLeft.cState == __MOTORSTOP) 
    {
        iL++;
    } 
    else 
    {
        iL = 0;
    }
    if (iL >= 500) 
    {
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     0x00);
    }
    if (__GmRight.cState == __MOTORSTOP)
    {
        iR++;
    } 
    else 
    {
        iR = 0;
    }
    if (iR >= 500) 
    {
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     0x00);
    }
    __irCheck();
}


/*********************************************************************************************************
** Function name:       __irSendFreq
** Descriptions:        发送红外线。
** input parameters:    __uiFreq:  红外线调制频率
**                      __cNumber: 选择需要设置的PWM模块
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irSendFreq (uint32  __uiFreq, int8  __cNumber)
{
    __uiFreq = SysCtlClockGet() / __uiFreq;
    switch (__cNumber) {

    case 1:
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                 /*  设置PWM发生器1的周期        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, __uiFreq / 2);            /*  设置PWM2输出的脉冲宽度      */
        PWMGenEnable(PWM_BASE, PWM_GEN_1);                              /*  使能PWM发生器1              */
        break;

    case 2:
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, __uiFreq);                 /*  设置PWM发生器2的周期        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, __uiFreq / 2);            /*  设置PWM4输出的脉冲宽度      */
        PWMGenEnable(PWM_BASE, PWM_GEN_2);                              /*  使能PWM发生器2              */
        break;

    default:
        break;
    }
}




/*********************************************************************************************************
** Function name:       __irCheck
** Descriptions:        红外线传感器检测。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irCheck (void)
{
	static uint8 ucState = 0;
	static uint8 ucIRCheck;

	switch (ucState)
	{
		case 0:
			__irSendFreq(32200, 2);                                         /*  探测左右两测近距            */
			__irSendFreq(35000, 1);                                         /*  驱动斜角上的传感器检测      */
		break;

		case 1:
			ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x3e);                 /*  读取传感器状态              */
			PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  禁止PWM发生器2              */
			PWMGenDisable(PWM_BASE, PWM_GEN_1);                             /*  禁止PWM发生器1              */
			if (ucIRCheck & __RIGHTSIDE)
				__GucDistance[__RIGHT]  &= 0xfd;
			else 
				__GucDistance[__RIGHT]  |= 0x02;
			if (ucIRCheck & __LEFTSIDE)
				__GucDistance[__LEFT]   &= 0xfd;
			else 
				__GucDistance[__LEFT]   |= 0x02;
			if (ucIRCheck & __FRONTSIDE_R)
				__GucDistance[__FRONTR]  = 0x00;
			else
				__GucDistance[__FRONTR]  = 0x01;
			if (ucIRCheck & __FRONTSIDE_L)
				__GucDistance[__FRONTL]  = 0x00;
			else
				__GucDistance[__FRONTL]  = 0x01;
		break;

		case 2:
			__irSendFreq(35000, 2);   
		break;

		case 3:
			ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x3e);
			PWMGenDisable(PWM_BASE, PWM_GEN_2);  
			if (ucIRCheck & __FRONTSIDE)
				__GucDistance[__FRONT] &= 0xfd;
			else
				__GucDistance[__FRONT]  |= 0x02;
		break;

		case 4:
			__irSendFreq(36000, 2);                                         /*  驱动检测左前右三个方向远距  */
		break;

		case 5:
			ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x2a);                 /*  读取传感器状态              */
			PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  禁止PWM发生器2              */
		break;

		case 6:
			__irSendFreq(36000, 2);                                         /*  重复检测左前右三个方向远距  */
		break;

		case 7:
			ucIRCheck &= GPIOPinRead(GPIO_PORTB_BASE, 0x2a);                /*  读取传感器状态              */
			PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  禁止PWM发生器2              */
			if (ucIRCheck & __RIGHTSIDE)
				__GucDistance[__RIGHT] &= 0xfe;
			else
				__GucDistance[__RIGHT] |= 0x01;
			if (ucIRCheck & __LEFTSIDE)
				__GucDistance[__LEFT]  &= 0xfe;
			else 
				__GucDistance[__LEFT]  |= 0x01;
			if (ucIRCheck & __FRONTSIDE)
				__GucDistance[__FRONT] &= 0xfe;
			else
				__GucDistance[__FRONT] |= 0x01;
		break;

		default:
		break;
	}
	ucState = (ucState + 1) % 8;//8                                        /*  循环检测    */
}



/*********************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        右转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnright(void)
{
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	/*
	 *  开始右转
	 */
//        zlg7289Reset();
//        zlg7289Download(0, 1, 0, 0x0E);
        
//        acc_change(2001139);
//      acc_change(2236068);        
	__GucMouseState   = __TURNRIGHT;
	__GmLeft.uiPulse = 84;//86
	if(turnflag == 4)//左转再右转
		__GmLeft.uiPulse += 2;//3;
	if(turnflag == 3)//右转再右转
		__GmLeft.uiPulse += 2;//3;        
	__GmLeft.iSpeed = 80;
	__GmLeft.cState  = __MOTORRUN;
	GucMouseDir  = (GucMouseDir + 1) % 4;                            /*  方向标记                    */
	while ((__GmLeft.uiPulse - __GmLeft.uiPulseCtr) > 3);
	__GmRight.uiPulse  = 7;
	__GmRight.cState  = __MOTORRUN;
	while (__GmRight.cState  != __MOTORSTOP);
	__GmLeft.iSpeed = 40;
	turnflag = 1;
//      acc_change(1581139);
//        acc_change(2236068);
}


/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnleft(void)
{
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	/*
	 *  开始左转
	 */
//        zlg7289Reset();
//        zlg7289Download(0, 1, 0, 0x0D);
//        acc_change(2001139);
//      acc_change(2236068);
	__GucMouseState   = __TURNLEFT;
	__GmRight.uiPulse = 84;//86
	if(turnflag == 3)//右转再左转
	__GmRight.uiPulse += 2;//3
        if(turnflag == 4)//左转再左转
	__GmRight.uiPulse += 2;

	__GmRight.iSpeed = 80;
	__GmRight.cState  = __MOTORRUN;
	GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  方向标记                    */
	while ((__GmRight.uiPulse - __GmRight.uiPulseCtr) > 3);
	__GmLeft.uiPulse  = 7;
	__GmLeft.cState   = __MOTORRUN;
	while (__GmLeft.cState != __MOTORSTOP);
	__GmRight.iSpeed = 40;
	turnflag = 2;
//    acc_change(1581139);
//      acc_change(2236068);
}


/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        后转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnback(void)
{
	/*
	 *  等待停止
	 */
	 speedcontrol = 0;
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	/*
	 *  开始后转
	 */
        __GmLeft.iSpeed=0;
        __GmRight.iSpeed=0;

        Acc0_temp=__GuiAccelTable[0];
        acc_change(1801139);//1801139

//        zlg7289Reset();
//        zlg7289Download(1, 1, 0, 0x0B);
	__GucMouseState   = __TURNBACK;
	__GmRight.cDir    = __MOTORGOBACK;
	__GmRight.uiPulse = 88;//86
	__GmLeft.uiPulse  = 88;//86
	__GmLeft.cState   = __MOTORRUN;
	__GmRight.cState  = __MOTORRUN;
	GucMouseDir = (GucMouseDir + 2) % 4;                                /*  方向标记                    */
	while (__GmLeft.cState  != __MOTORSTOP);
	while (__GmRight.cState != __MOTORSTOP);
	__GmRight.cDir = __MOTORGOAHEAD;
        acc_change(Acc0_temp);        
}

/*********************************************************************************************************
** Function name:       __mouseCoorUpdate
** Descriptions:        根据当前方向更新坐标值
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mouseCoorUpdate (void)
{
    switch (GucMouseDir)
	{
		case 0:
			GmcMouse.cY++;                        
//    zlg7289Download(1,6,1,GmcMouse.cX);
//    zlg7289Download(1,7,0,GmcMouse.cY);

		break;

		case 1:
			GmcMouse.cX++;
//    zlg7289Download(1,6,1,GmcMouse.cX);
//    zlg7289Download(1,7,0,GmcMouse.cY);
		break;

		case 2:
			GmcMouse.cY--;
//    zlg7289Download(1,6,1,GmcMouse.cX);
//    zlg7289Download(1,7,0,GmcMouse.cY);
		break;

		case 3:
			GmcMouse.cX--;
//    zlg7289Download(1,6,1,GmcMouse.cX);
//    zlg7289Download(1,7,0,GmcMouse.cY);
		break;

		default:
		break;
    }
/*    
    if(  (spurt_time==0) &&
         (GmcMouse.cX==XDST0 || GmcMouse.cX==XDST1) &&
         (GmcMouse.cY==YDST0 || GmcMouse.cY==YDST1)
      )
       {
         GucXGoal=GmcMouse.cX;
         GucYGoal=GmcMouse.cY;
         zlg7289Reset();
         zlg7289Download(0, 4, 1, GucXGoal);
         zlg7289Download(0, 5, 0, GucYGoal);
                               
         objectGoTo(GucXStart,GucYStart,__START);                         
 	 GmcMouse.cX = GucXStart;
	 GmcMouse.cY = GucYStart; 
         //zlg7289Reset();
         zlg7289Download(0, 6, 1, GucXStart);
         zlg7289Download(0, 7, 0, GucYStart);
         mouseTurnback();
         GucMouseTask = SPURT;                               //  电脑鼠将开始冲刺状态  

       }
*/
    
   __mazeInfDebug();
    __wallCheck();
}


/*********************************************************************************************************
** Function name:       __wallCheck
** Descriptions:        根据传感器检测结果判断是否存在墙壁
** input parameters:    无
** output parameters:   无
** Returned value:      cValue: 低三位从左到右一次代表左前右。1为有墙，0为没墙。
*********************************************************************************************************/
void __wallCheck (void)
{
    uint8 ucMap = 0;
    ucMap |= MOUSEWAY_B;
    
    if (__GucDistance[__LEFT]  & 0x01)
        ucMap &= ~MOUSEWAY_L;
    else
        ucMap |=  MOUSEWAY_L;

    if (__GucDistance[__FRONT] & 0x01)
        ucMap &= ~MOUSEWAY_F;
    else
        ucMap |=  MOUSEWAY_F;

    if (__GucDistance[__RIGHT] & 0x01)
        ucMap &= ~MOUSEWAY_R;
    else
        ucMap |=  MOUSEWAY_R;

//    zlg7289Reset();
//    zlg7289Download(1,4,0,GucMapBlock[GmcMouse.cX][GmcMouse.cY]);
//    zlg7289Download(1,5,0,ucMap);
    
    if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] == 0x00)  //??? 看不懂
    {
        GucMapBlock[GmcMouse.cX][GmcMouse.cY] = ucMap;
        Gucstep++;//搜索的步数
/*        if(GmcMouse.cX >=7 && GmcMouse.cX <=8 && GmcMouse.cY >=7  && GmcMouse.cY<=8)
        {
           GucMapBlock[GmcMouse.cX][GmcMouse.cY] =  MOUSEWAY_B ;       
           GucXGoal = GmcMouse.cX;
           GucYGoal = GmcMouse.cY;
          zlg7289Reset();
//        zlg7289Download(0, 0, 0, 0x0E);
//        zlg7289Download(0, 1, 0, 0x0E);
//        zlg7289Download(0, 2, 0, 0x0E);
//        zlg7289Download(0, 3, 0, 0x0E);
          zlg7289Download(0, 4, 1, GucXGoal);
          zlg7289Download(0, 5, 0, GucYGoal);
//        zlg7289Download(0, 6, 0, 0x0E);
//        zlg7289Download(0, 7, 0, 0x0E);

        }
*/       
    }

  //注销掉了显示器显示，节省CPU资源  
    else {
        if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] != ucMap) {
            zlg7289Download(1,4,0,GucMapBlock[GmcMouse.cX][GmcMouse.cY]);
            zlg7289Download(1,5,0,ucMap);
            //while(keyCheck() == false);
        }
    }
   
}


/*********************************************************************************************************
** Function name:       SensorDebug
** Descriptions:        用数码管显示出传感器状态，方便调试
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void sensorDebug (void)
{
    zlg7289Download(2, 0, 0, __GucDistance[__LEFT  ]);
    zlg7289Download(2, 1, 0, __GucDistance[__FRONTL]);
    zlg7289Download(2, 2, 0, __GucDistance[__FRONT ]);
    zlg7289Download(2, 3, 0, __GucDistance[__FRONTR]);    
    zlg7289Download(2, 4, 0, __GucDistance[__RIGHT ]);    
}


/*********************************************************************************************************
** Function name:       __mazeInfDebug
** Descriptions:        用数码管显示出当前电脑鼠前进方向和坐标
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mazeInfDebug (void)
{
    /*
     *  显示方向
     */
    switch (GucMouseDir) {
        
    case 0:
       zlg7289Download(2, 3, 0, 0x47);                                /*  向前，用F表示               */
       break;
        
    case 1:
      zlg7289Download(2, 3, 0, 0x77);                                /*  向右，用R表示               */
       break;
        
   case 2:
       zlg7289Download(2, 3, 0, 0x1f);                                /*  向后，用b表示               */
        break;
        
   case 3:
        zlg7289Download(2, 3, 0, 0x0e);                                /*  向左，用L表示               */
       break;
        
   default :
       zlg7289Download(2, 3, 0, 0x4f);                                /*  错误，用E表示               */
       break;
    }
    
      //显示坐标
     
    zlg7289Download(1, 0, 0, GmcMouse.cX / 10);
   zlg7289Download(1, 1, 0, GmcMouse.cX % 10);
    zlg7289Download(1, 6, 0, GmcMouse.cY / 10);
    zlg7289Download(1, 7, 0, GmcMouse.cY % 10);
}


/*********************************************************************************************************
** Function name:       keyCheck
** Descriptions:        读取按键
** input parameters:    无
** output parameters:   无
** Returned value:      true:  按键已按下
**                      false: 按键未按下
*********************************************************************************************************/
uint8 keyCheck (void)
{
    if (GPIOPinRead(GPIO_PORTC_BASE, __KEY) == 0)
	{
		__delay(50);
        while(GPIOPinRead(GPIO_PORTC_BASE, __KEY) == 0);
        return(true);
    }
	else
        return(false);
}


/*********************************************************************************************************
** Function name:       voltageDetect
** Descriptions:        电压检测，检测结果在7289 EX BOARD 上显示出来
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void voltageDetect (void)
{
    unsigned long ulVoltage;
    
    ADCProcessorTrigger(ADC_BASE, 0);                                   /*  处理器触发一次A/D转换       */
    while (!ADCIntStatus(ADC_BASE, 0, false));                          /*  等待转换结束                */
    ADCIntClear(ADC_BASE, 0);                                           /*  清除中断标准位              */
    ADCSequenceDataGet(ADC_BASE, 0, &ulVoltage);                        /*  读取转换结果                */
    
    ulVoltage = ulVoltage * 3000 / 1023;                                /*  计算实际检测到的电压值(mV)  */
    ulVoltage = ulVoltage * 3 + 350;                                    /*  计算电池电压值(mV)          */
    
    zlg7289Download(0,6,1,(ulVoltage % 10000) / 1000);                  /*  显示电压值整数部分，单位V   */
    zlg7289Download(0,7,0,(ulVoltage % 1000 ) / 100 );                  /*  显示电压值小数部分，单位V   */
}


/*********************************************************************************************************
** Function name:       mouseInit
** Descriptions:        对LM3S615处理器进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseInit (void)
{
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_6MHZ );                                 /*  使能PLL，50M                */

    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );                      /*  使能GPIO B口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );                      /*  使能GPIO C口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );                      /*  使能GPIO D口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );                      /*  使能GPIO E口外设            */
    __keyInit();                                                        /*  按键初始化                  */
    __sensorInit();                                                     /*  传感器初始化                */
    __stepMotorIint();                                                  /*  步进电机控制初始化          */
    __sysTickInit();                                                    /*  系统时钟初始化              */
    __ADCInit();
    GucMapBlock[0][0] = 0x01;
    Gucstep++;
}


/*********************************************************************************************************
** Function name:       __sensorInit
** Descriptions:        传感器控制初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __sensorInit (void)
{
    /*
     *  设置连接到传感器信号输出脚的I/O口为输入模式
     */
    GPIODirModeSet(GPIO_PORTB_BASE,
                   __LEFTSIDE    |
                   __FRONTSIDE_L |
                   __FRONTSIDE   |
                   __FRONTSIDE_R |
                   __RIGHTSIDE,  
                   GPIO_DIR_MODE_IN);
    /*
     *  用PWM驱动红外线发射头产生调制的红外线信号
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  使能PWM模块                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                 /*  PWM时钟配置：不分频         */
    /*
     *  初始化PWM2，该PWM驱动斜角红外发射头
     */
    GPIOPinTypePWM(GPIO_PORTB_BASE, __IRSEND_BEVEL);                    /*  PB0配置为PWM功能            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_1,                                /*  配置PWM发生器1              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */

    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, true);                      /*  使能PWM2输出                */
    PWMGenDisable(PWM_BASE, PWM_GEN_1);                                 /*  禁止PWM发生器1              */
    /*
     *  初始化PWM4，该PWM驱动左前右正方向红外发射头
     */
    GPIOPinTypePWM(GPIO_PORTE_BASE, __IRSEND_SIDE);                     /*  PE0配置为PWM功能            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_2,                                /*  配置PWM发生器2              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */

    PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);                      /*  使能PWM4输出                */
    PWMGenDisable(PWM_BASE, PWM_GEN_2);                                 /*  禁止PWM发生器2              */
}


/*********************************************************************************************************
** Function name:       __stepMotorIint
** Descriptions:        步进电机控制初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __stepMotorIint (void)
{
    uint16 n = 0;
    /*
     *  设置驱动步进电机的八个I/O口为输出模式
     */
    GPIODirModeSet(GPIO_PORTD_BASE,
                   __PHRA1 |
                   __PHRA2 |
                   __PHRB1 |
                   __PHRB2 |
                   __PHLA1 |
                   __PHLA2 |
                   __PHLB1 |
                   __PHLB2,
                   GPIO_DIR_MODE_OUT);
    /*
     *  对左右电机转动的位置初始化
     */
    GPIOPinWrite(GPIO_PORTD_BASE,
                 __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                 __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2);
    
    GPIOPinWrite(GPIO_PORTD_BASE,
                 __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                 __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2);
    /*
     *  初始化加速/减速时定时器加载值的数据表
     */
 //   int kaishi=2236068;
    __GuiAccelTable[0] = 2136068;//2236068
    __GuiAccelTable[1] = 2136068*0.4142;//906949
    for(n = 2; n < 400; n++) {
        __GuiAccelTable[n] = __GuiAccelTable[n - 1] - (2 * __GuiAccelTable[n - 1] / (4 * n + 1));
    }
    /*
     *  初始化定时器0，用来控制右电机的转速
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                       /*  使能定时器0模块             */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);                  /*  配置为32位周期计数模式      */
    TimerLoadSet(TIMER0_BASE, TIMER_A, __GuiAccelTable[0]);             /*  设置定时时间                */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                    /*  设置为溢出中断              */

    IntEnable(INT_TIMER0A);                                             /*  使能定时器0中断             */
    TimerEnable(TIMER0_BASE, TIMER_A);                                  /*  使能定时器0                 */
    
    /*
     *  初始化定时器1，用来控制电机的转速
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);                       /*  使能定时器1模块             */
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);                  /*  配置为32位周期计数模式      */
    TimerLoadSet(TIMER1_BASE, TIMER_A, __GuiAccelTable[0]);             /*  设置定时时间                */
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                    /*  设置为溢出中断              */

    IntEnable(INT_TIMER1A);                                             /*  使能定时器1中断             */
    TimerEnable(TIMER1_BASE, TIMER_A);                                  /*  使能定时器1                 */
}


/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        对连接按键的GPIO口初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __keyInit (void)
{
    GPIODirModeSet(GPIO_PORTC_BASE, __KEY, GPIO_DIR_MODE_IN);           /*  设置按键口为输入            */
}


/*********************************************************************************************************
** Function name:       __sysTickInit
** Descriptions:        系统节拍定时器初始化。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __sysTickInit (void)
{
    SysTickPeriodSet(SysCtlClockGet() / 1600);                          /*  设置定时时钟为1ms           */
    SysTickEnable();                                                    /*  使能系统时钟                */
    SysTickIntEnable();                                                 /*  使能系统时钟中断            */
}


/*********************************************************************************************************
** Function name:       __ADCInit
** Descriptions:        对连接按键的GPIO口初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __ADCInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);                          /*  使能ADC模块                 */
    SysCtlADCSpeedSet(SYSCTL_ADCSPEED_125KSPS);                         /*  125KSps采样率               */

    ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);        /*  序列0为处理器触发，优先级为0*/
    ADCSequenceStepConfigure(ADC_BASE, 0, 0,
                             ADC_CTL_CH0  | 
                             ADC_CTL_IE   | 
                             ADC_CTL_END);                              /*  配置采样序列发生器的步进    */
    
    ADCHardwareOversampleConfigure(ADC_BASE, 16);                       /*  设置ADC采样平均控制寄存器   */
    ADCSequenceEnable(ADC_BASE, 0);                                     /*  使能采样序列0               */
}

//uint8 sanmianqiang(uint8 x,uint8 y){
//  if(__GucDistance[__FRONT] == 0x03) return 1;
//  else return 0;
//}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
