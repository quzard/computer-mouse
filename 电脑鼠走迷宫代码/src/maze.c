/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           maze.c
** Last modified Date:  2010/08/26
** Last Version:        V1.0
** Description:         根据底层程序取得的迷宫信息，经过该智能算法控制电脑鼠的下一状态，并送往底层驱动程
**                      序执行。
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          Liao Maogang
** Created date:        2007/09/08
** Version:             V1.0
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
#include "Maze.h"
#include "Mouse_Drive.h"
/*********************************************************************************************************
  全局变量定义  
*********************************************************************************************************/
       uint8    GucXStart                           = 0;//  static             
       uint8    GucYStart                           = 0;//  static                /*  起点X，Y坐标                */

       uint8    GucXGoal                            = XDST0;
       uint8    GucYGoal                            = YDST0;                /*  终点X,Y坐标                 */
//static uint8    GucXGoal0                            = XDST0;
//static uint8    GucYGoal0                            = YDST0;                /*  终点X,Y坐标                 */
//static uint8    GucXGoal1                           = XDST1;           /*  终点Y坐标，有两个值         */
//static uint8    GucYGoal1                           = YDST1;


       uint8    GucMouseTask                        = WAIT;//static             /*  状态机，初始状态为等待      */

static uint16   GucMapStep[MAZETYPE][MAZETYPE]      = {0xffff};         /*  保存等高值                  */

static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  在等高图作堆栈使用          */
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()中暂存未走过支路坐标  */

       uint8    Gucstep                             = 0 ;               //搜索的步数
       uint8    GucDirFlag                          = 0 ;               //冲刺区别
       uint8    spurt_time = 0;//冲刺的次数
//       uint8    maze_time=0;//maze状态执行次数
       uint8    temp_x=0;//临时保存X坐标
       uint8    temp_y=0;//临时保存y坐标
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       uint8    GuiStep[MAZETYPE * MAZETYPE][2]     ={0};              //objectto中记录返回信息
                     //GuiStep[256][2]
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8    GucMapLoad[17][17]                   = {0};        //0：up
      						                   //1：right
							           //2：down
    							           //3：left
static MAZECOOR GucAboveLoad[17*17]                  = {0};        //起点到终点坐标
static uint8    loadcount                            =  0 ;        //起点到终点坐标数
static uint8    uStartFlag                           =  0 ;        //标记起点X if（0）if（15） 
static uint8    ushieldflag                          =  0 ;        //是否有路口屏蔽
static uint8    Gucsave[17*17]                       = {0};        //保存岔路下标
static uint8    GucMapFlag[17][17]                   = {0};        //标记走过的路

static  int8    crosslenth                           =  0 ;
static MAZECOOR Gmcshield[MAZETYPE]                  = {0};
//int cor_changed = 0;//转表转换标记


/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}

/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    cX:    目的地横坐标
**                      cY:    目的地纵坐标
** output parameters:   GucMapStep[][]:  各坐标上的等高值
** Returned value:      无
*********************************************************************************************************/
void mapStepEdit (int8  cX, int8  cY)
{
    uint8  n= 0,m=0;                                   /*  GmcStack[]下标              */
    uint16  ucStep = 1;                              /*  等高值                      */
    uint8  ucStat = 0;                              /*  统计可前进的方向数          */
    int8   i=0,j=0;
    int8   cNBlock =0, cDirTemp;
    uint8  cXcur=cX,cYcur=cY;
    
    uint8  Dircur=0;                                  //方向
    uint16 ucsteptemp;				      //
    uint8  choice;
    
    GmcStack[n].cX  = cXcur;                             /*  起点X值入栈                 */
    GmcStack[n].cY  = cYcur;                             /*  起点Y值入栈                 */
    Gucsave[n]=0;
    n++;
    
    for(m = 0; m < 255; m++)   //初始化返回信息
    {
	GuiStep[m][0] = 0;
	GuiStep[m][1] = 0;
    }
    m = 0;

    
    for (i = 0; i<16; i++)     //初始化等高值
        for (j = 0; j<16; j++)
            GucMapStep[i][j] = 0xffff;       

    while (n)                  //制作等高图
    {
        GucMapStep[cXcur][cYcur] = ucStep;                     
        ucStat = 0;
        if (GucMapBlock[cXcur][cYcur] & 0x08)            /*  左方有路                    */
	{    
	    if(Dircur==0x08||Dircur==0)
		ucsteptemp=ucStep+STRWEIGHT;
	    else
		ucsteptemp=ucStep+CURVEWEIGHT;
	    if(GucMapStep[cXcur - 1][cYcur] > (ucsteptemp))
	    {
		ucStat++;
		choice=0x08;
	    }
        }

        if (GucMapBlock[cXcur][cYcur] & 0x04)            /*  下方有路                    */
	{  
            if(Dircur==0x04||Dircur==0)
                    ucsteptemp=ucStep+STRWEIGHT;
            else
                    ucsteptemp=ucStep+CURVEWEIGHT;
            if(GucMapStep[cXcur][cYcur - 1] > (ucsteptemp))
            {
                    ucStat++;
                    choice=0x04;
            }
         }

	if (GucMapBlock[cXcur][cYcur] & 0x02)          /*  右方有路                    */
	{    
            if(Dircur==0x02||Dircur==0)
                    ucsteptemp=ucStep+STRWEIGHT;
            else
                    ucsteptemp=ucStep+CURVEWEIGHT;
            if(GucMapStep[cXcur + 1][cYcur] > (ucsteptemp))
            {
                    ucStat++;
                    choice=0x02;
            }
         }

	if (GucMapBlock[cXcur][cYcur] & 0x01)        /*  上方有路                    */      
	{     
            if(Dircur==0x01||Dircur==0)
                    ucsteptemp=ucStep+STRWEIGHT;
            else
                    ucsteptemp=ucStep+CURVEWEIGHT;
            if(GucMapStep[cXcur][cYcur + 1] > (ucsteptemp))
            {
                    ucStat++;  
                    choice=0x01;
            }
        }
       
        if (ucStat == 0) 
        {
            n--;
            cXcur = GmcStack[n].cX;
            cYcur = GmcStack[n].cY;
	    Dircur = Gucsave[n];
            ucStep = GucMapStep[cXcur][cYcur];
        } 
        else 
        {
	    if (ucStat > 1) 
            {                                   
                GmcStack[n].cX = cXcur;            
                GmcStack[n].cY = cYcur; 
		Gucsave[n]=Dircur;
                n++;
            }
	    switch(choice)
	    {
		case 0x01:
			if(Dircur==0x01||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x01;
			cYcur++;                                    
			break;
		case 0x02:
			if(Dircur==0x02||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x02;
			cXcur++;                                      
			break;
		case 0x04:
			if(Dircur==0x04||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x04;
			cYcur--;                                     
			break;
		case 0x08:
			if(Dircur==0x08||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x08;
			cXcur--;                                       
			break;
	    }
        }
    }
    //等高图制作完毕
    cXcur = GmcMouse.cX;
    cYcur = GmcMouse.cY;
    ucsteptemp=0xffff;
    ucStep = GucMapStep[cXcur][cYcur];
    if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
        (GucMapBlock[cXcur][cYcur+1]!=0) &&
        (GucMapStep[cXcur][cYcur + 1] < ucStep)) 
            ucsteptemp=GucMapStep[cXcur][cYcur + 1];

    if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
        (GucMapBlock[cXcur+1][cYcur]!=0) &&
        (GucMapStep[cXcur + 1][cYcur] < ucStep)) 
            if(GucMapStep[cXcur + 1][cYcur] < ucsteptemp)
                    ucsteptemp=GucMapStep[cXcur+1][cYcur];

    if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
        (GucMapBlock[cXcur][cYcur-1]!=0) &&
        (GucMapStep[cXcur][cYcur - 1] < ucStep)) 
            if(GucMapStep[cXcur][cYcur -1] < ucsteptemp)
                    ucsteptemp=GucMapStep[cXcur][cYcur -1];

    if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
        (GucMapBlock[cXcur-1][cYcur]!=0) &&
        (GucMapStep[cXcur - 1][cYcur] < ucStep))
            if(GucMapStep[cXcur -1][cYcur] < ucsteptemp)
                    ucsteptemp=GucMapStep[cXcur -1][cYcur];


    if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
        (GucMapBlock[cXcur][cYcur+1]!=0) &&
        (GucMapStep[cXcur][cYcur + 1]==ucsteptemp)) 
    {                      
        cDirTemp = UP;                                      
    }
    if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
        (GucMapBlock[cXcur+1][cYcur]!=0) &&
        (GucMapStep[cXcur + 1][cYcur]==ucsteptemp)) 
    {            
        cDirTemp = RIGHT;                                   
    }
    if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
        (GucMapBlock[cXcur][cYcur-1]!=0) &&
        (GucMapStep[cXcur][cYcur - 1]==ucsteptemp)) 
    {           
        cDirTemp = DOWN;                                   
    }
    if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
        (GucMapBlock[cXcur-1][cYcur]!=0) &&
        (GucMapStep[cXcur - 1][cYcur]==ucsteptemp))
    {               
        cDirTemp = LEFT;                                    
    }
   cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;  
    switch (cDirTemp) 
    {
            case 1:
                GuiStep[0][1] = 0x1;   //右转
                break;
	     case 2:
               GuiStep[0][1] = 0x3;    //后转
                break;
            case 3:
                GuiStep[0][1] = 0x2;   //左转
                break;
            default:
                break;
    }
    GucMouseDir = (GucMouseDir+cDirTemp)%4;
    m++; 
    
    while ((cXcur != cX) || (cYcur != cY)) 
    {
        ucsteptemp=0xffff;
        ucStep = GucMapStep[cXcur][cYcur];
	if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
            (GucMapBlock[cXcur][cYcur+1]!=0) &&
            (GucMapStep[cXcur][cYcur + 1] < ucStep)) 
		ucsteptemp=GucMapStep[cXcur][cYcur + 1];

        if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
            (GucMapBlock[cXcur+1][cYcur]!=0) &&
            (GucMapStep[cXcur + 1][cYcur] < ucStep)) 
		if(GucMapStep[cXcur + 1][cYcur] < ucsteptemp)
			ucsteptemp=GucMapStep[cXcur+1][cYcur];

	if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
            (GucMapBlock[cXcur][cYcur-1]!=0) &&
            (GucMapStep[cXcur][cYcur - 1] < ucStep)) 
		if(GucMapStep[cXcur][cYcur -1] < ucsteptemp)
			ucsteptemp=GucMapStep[cXcur][cYcur -1];

	if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
            (GucMapBlock[cXcur-1][cYcur]!=0) &&
            (GucMapStep[cXcur - 1][cYcur] < ucStep))
		if(GucMapStep[cXcur -1][cYcur] < ucsteptemp)
			ucsteptemp=GucMapStep[cXcur -1][cYcur];


        if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
            (GucMapBlock[cXcur][cYcur+1]!=0) &&
            (GucMapStep[cXcur][cYcur + 1]==ucsteptemp)) 
        {                      
            cDirTemp = UP;                                      
            if (cDirTemp == GucMouseDir) 
            { 
                cNBlock++;                                     
                cYcur++;
                continue;                                      
            }
        }
        if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
            (GucMapBlock[cXcur+1][cYcur]!=0) &&
            (GucMapStep[cXcur + 1][cYcur]==ucsteptemp)) 
	{            
            cDirTemp = RIGHT;                                   
            if (cDirTemp == GucMouseDir)
	    {         
                cNBlock++;                                      
                cXcur++;
                continue;                                       
            }
        }
        if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
            (GucMapBlock[cXcur][cYcur-1]!=0) &&
            (GucMapStep[cXcur][cYcur - 1]==ucsteptemp)) 
	{           
            cDirTemp = DOWN;                                   
            if (cDirTemp == GucMouseDir) 
	    {    
                cNBlock++;                                      
                cYcur--;
                continue;                                       
            }
        }
        if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
            (GucMapBlock[cXcur-1][cYcur]!=0) &&
            (GucMapStep[cXcur - 1][cYcur]==ucsteptemp))
	{               
            cDirTemp = LEFT;                                    
            if (cDirTemp == GucMouseDir) 
	    {      
                cNBlock++;                                      
                cXcur--;
                continue;                                       
            }
        }
       cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;              /*  计算方向偏移量              */
       GucMouseDir=(GucMouseDir+cDirTemp)%4;
       GuiStep[m][0]=cNBlock;                                  /*  前进cNBlock步               */       
       cNBlock = 0;                                            /*  任务清零                    */
       
       switch (cDirTemp) 
       {
            case 1:
                GuiStep[m][1] = 0x1;   //右转
                break;
	     case 2:
               GuiStep[m][1] = 0x3;    //后转
                break;
            case 3:
                GuiStep[m][1] = 0x2;   //左转
                break;
            default:
                break;
       }
       m++;
    }
    GuiStep[m][0] = cNBlock;    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//////mapStepEdit_check/////
/*********************************************************************************************************
** Function name:       mapStepEdit_check
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    cX:    目的地横坐标
**                      cY:    目的地纵坐标
** output parameters:   GucMapStep[][]:  各坐标上的等高值
** Returned value:      无
*********************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////
//void mapStepEdit_check (int8  cX, int8  cY)
//{
//    uint8  n=0;//,m=0;                                   /*  GmcStack[]下标              */
//    uint16 ucStep = 1;                              /*  等高值                      */
//    uint8  ucStat = 0;                              /*  统计可前进的方向数          */
//    int8   i=0,j=0;
////    int8   cNBlock =0, cDirTemp;
//    uint8  cXcur=cX,cYcur=cY;//
////    uint8  tempX=GmcMouse.cX,tempY=GmcMouse.cY;
//
////    GmcMouse.cX = GucXStart;
////    GmcMouse.cY = GucYStart;
//
//    
//    uint8  Dircur=0;                                  //方向
//    uint16 ucsteptemp;				      //
//    uint8  choice;
//    
//    GmcStack[n].cX  = cXcur;                             /*  起点X值入栈                 */
//    GmcStack[n].cY  = cYcur;                             /*  起点Y值入栈                 */
//    Gucsave[n]=0;
//    n++;
//    
////    for(m = 0; m < 255; m++)   //初始化返回信息
////    {
////	GuiStep[m][0] = 0;
////	GuiStep[m][1] = 0;
////    }
////    m = 0;
//
//    
//    for (i = 0; i<16; i++)     //初始化等高值
//        for (j = 0; j<16; j++)
//            GucMapStep[i][j] = 0xffff;       
//
//    while (n)                  //制作等高图
//    {
//        GucMapStep[cXcur][cYcur] = ucStep;                     
//        ucStat = 0;
//        if (GucMapBlock[cXcur][cYcur] & 0x08)            /*  左方有路                    */
//	{    
//	    if(Dircur==0x08||Dircur==0)
//		ucsteptemp=ucStep+STRWEIGHT;
//	    else
//		ucsteptemp=ucStep+CURVEWEIGHT;
//	    if(GucMapStep[cXcur - 1][cYcur] > (ucsteptemp))
//	    {
//		ucStat++;
//		choice=0x08;
//	    }
//        }
//
//        if (GucMapBlock[cXcur][cYcur] & 0x04)            /*  下方有路                    */
//	{  
//            if(Dircur==0x04||Dircur==0)
//                    ucsteptemp=ucStep+STRWEIGHT;
//            else
//                    ucsteptemp=ucStep+CURVEWEIGHT;
//            if(GucMapStep[cXcur][cYcur - 1] > (ucsteptemp))
//            {
//                    ucStat++;
//                    choice=0x04;
//            }
//         }
//
//	if (GucMapBlock[cXcur][cYcur] & 0x02)          /*  右方有路                    */
//	{    
//            if(Dircur==0x02||Dircur==0)
//                    ucsteptemp=ucStep+STRWEIGHT;
//            else
//                    ucsteptemp=ucStep+CURVEWEIGHT;
//            if(GucMapStep[cXcur + 1][cYcur] > (ucsteptemp))
//            {
//                    ucStat++;
//                    choice=0x02;
//            }
//         }
//
//	if (GucMapBlock[cXcur][cYcur] & 0x01)        /*  上方有路                    */      
//	{     
//            if(Dircur==0x01||Dircur==0)
//                    ucsteptemp=ucStep+STRWEIGHT;
//            else
//                    ucsteptemp=ucStep+CURVEWEIGHT;
//            if(GucMapStep[cXcur][cYcur + 1] > (ucsteptemp))
//            {
//                    ucStat++;  
//                    choice=0x01;
//            }
//        }
//       
//        if (ucStat == 0) 
//        {
//            n--;
//            cXcur = GmcStack[n].cX;
//            cYcur = GmcStack[n].cY;
//	    Dircur = Gucsave[n];
//            ucStep = GucMapStep[cXcur][cYcur];
//        } 
//        else 
//        {
//	    if (ucStat > 1) 
//            {                                   
//                GmcStack[n].cX = cXcur;            
//                GmcStack[n].cY = cYcur; 
//		Gucsave[n]=Dircur;
//                n++;
//            }
//	    switch(choice)
//	    {
//		case 0x01:
//			if(Dircur==0x01||Dircur==0)
//			    ucStep+=STRWEIGHT;
//			else
//			    ucStep+=CURVEWEIGHT;
//			Dircur=0x01;
//			cYcur++;                                    
//			break;
//		case 0x02:
//			if(Dircur==0x02||Dircur==0)
//			    ucStep+=STRWEIGHT;
//			else
//			    ucStep+=CURVEWEIGHT;
//			Dircur=0x02;
//			cXcur++;                                      
//			break;
//		case 0x04:
//			if(Dircur==0x04||Dircur==0)
//			    ucStep+=STRWEIGHT;
//			else
//			    ucStep+=CURVEWEIGHT;
//			Dircur=0x04;
//			cYcur--;                                     
//			break;
//		case 0x08:
//			if(Dircur==0x08||Dircur==0)
//			    ucStep+=STRWEIGHT;
//			else
//			    ucStep+=CURVEWEIGHT;
//			Dircur=0x08;
//			cXcur--;                                       
//			break;
//	    }
//        }
//    }
//    //等高图制作完毕
////    cXcur = GmcMouse.cX;
////    cYcur = GmcMouse.cY;
////    ucsteptemp=0xffff;
////    ucStep = GucMapStep[cXcur][cYcur];
////    if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
////        (GucMapBlock[cXcur][cYcur+1]!=0) &&
////        (GucMapStep[cXcur][cYcur + 1] < ucStep)) 
////            ucsteptemp=GucMapStep[cXcur][cYcur + 1];
////
////    if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
////        (GucMapBlock[cXcur+1][cYcur]!=0) &&
////        (GucMapStep[cXcur + 1][cYcur] < ucStep)) 
////            if(GucMapStep[cXcur + 1][cYcur] < ucsteptemp)
////                    ucsteptemp=GucMapStep[cXcur+1][cYcur];
////
////    if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
////        (GucMapBlock[cXcur][cYcur-1]!=0) &&
////        (GucMapStep[cXcur][cYcur - 1] < ucStep)) 
////            if(GucMapStep[cXcur][cYcur -1] < ucsteptemp)
////                    ucsteptemp=GucMapStep[cXcur][cYcur -1];
////
////    if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
////        (GucMapBlock[cXcur-1][cYcur]!=0) &&
////        (GucMapStep[cXcur - 1][cYcur] < ucStep))
////            if(GucMapStep[cXcur -1][cYcur] < ucsteptemp)
////                    ucsteptemp=GucMapStep[cXcur -1][cYcur];
////
////
////    if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
////        (GucMapBlock[cXcur][cYcur+1]!=0) &&
////        (GucMapStep[cXcur][cYcur + 1]==ucsteptemp)) 
////    {                      
////        cDirTemp = UP;                                      
////    }
////    if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
////        (GucMapBlock[cXcur+1][cYcur]!=0) &&
////        (GucMapStep[cXcur + 1][cYcur]==ucsteptemp)) 
////    {            
////        cDirTemp = RIGHT;                                   
////    }
////    if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
////        (GucMapBlock[cXcur][cYcur-1]!=0) &&
////        (GucMapStep[cXcur][cYcur - 1]==ucsteptemp)) 
////    {           
////        cDirTemp = DOWN;                                   
////    }
////    if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
////        (GucMapBlock[cXcur-1][cYcur]!=0) &&
////        (GucMapStep[cXcur - 1][cYcur]==ucsteptemp))
////    {               
////        cDirTemp = LEFT;                                    
////    }
////   cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;  
////    switch (cDirTemp) 
////    {
////            case 1:
////                GuiStep[0][1] = 0x1;   //右转
////                break;
////	     case 2:
////               GuiStep[0][1] = 0x3;    //后转
////                break;
////            case 3:
////                GuiStep[0][1] = 0x2;   //左转
////                break;
////            default:
////                break;
////    }
////    GucMouseDir = (GucMouseDir+cDirTemp)%4;
////    m++; 
////    
////    while ((cXcur != cX) || (cYcur != cY)) 
////    {
////        ucsteptemp=0xffff;
////        ucStep = GucMapStep[cXcur][cYcur];
////	if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
////            (GucMapBlock[cXcur][cYcur+1]!=0) &&
////            (GucMapStep[cXcur][cYcur + 1] < ucStep)) 
////		ucsteptemp=GucMapStep[cXcur][cYcur + 1];
////
////        if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
////            (GucMapBlock[cXcur+1][cYcur]!=0) &&
////            (GucMapStep[cXcur + 1][cYcur] < ucStep)) 
////		if(GucMapStep[cXcur + 1][cYcur] < ucsteptemp)
////			ucsteptemp=GucMapStep[cXcur+1][cYcur];
////
////	if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
////            (GucMapBlock[cXcur][cYcur-1]!=0) &&
////            (GucMapStep[cXcur][cYcur - 1] < ucStep)) 
////		if(GucMapStep[cXcur][cYcur -1] < ucsteptemp)
////			ucsteptemp=GucMapStep[cXcur][cYcur -1];
////
////	if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
////            (GucMapBlock[cXcur-1][cYcur]!=0) &&
////            (GucMapStep[cXcur - 1][cYcur] < ucStep))
////		if(GucMapStep[cXcur -1][cYcur] < ucsteptemp)
////			ucsteptemp=GucMapStep[cXcur -1][cYcur];
////
////
////        if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
////            (GucMapBlock[cXcur][cYcur+1]!=0) &&
////            (GucMapStep[cXcur][cYcur + 1]==ucsteptemp)) 
////        {                      
////            cDirTemp = UP;                                      
////            if (cDirTemp == GucMouseDir) 
////            { 
////                cNBlock++;                                     
////                cYcur++;
////                continue;                                      
////            }
////        }
////        if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
////            (GucMapBlock[cXcur+1][cYcur]!=0) &&
////            (GucMapStep[cXcur + 1][cYcur]==ucsteptemp)) 
////	{            
////            cDirTemp = RIGHT;                                   
////            if (cDirTemp == GucMouseDir)
////	    {         
////                cNBlock++;                                      
////                cXcur++;
////                continue;                                       
////            }
////        }
////        if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
////            (GucMapBlock[cXcur][cYcur-1]!=0) &&
////            (GucMapStep[cXcur][cYcur - 1]==ucsteptemp)) 
////	{           
////            cDirTemp = DOWN;                                   
////            if (cDirTemp == GucMouseDir) 
////	    {    
////                cNBlock++;                                      
////                cYcur--;
////                continue;                                       
////            }
////        }
////        if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
////            (GucMapBlock[cXcur-1][cYcur]!=0) &&
////            (GucMapStep[cXcur - 1][cYcur]==ucsteptemp))
////	{               
////            cDirTemp = LEFT;                                    
////            if (cDirTemp == GucMouseDir) 
////	    {      
////                cNBlock++;                                      
////                cXcur--;
////                continue;                                       
////            }
////        }
////       cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;              /*  计算方向偏移量              */
////       GucMouseDir=(GucMouseDir+cDirTemp)%4;
////       GuiStep[m][0]=cNBlock;                                  /*  前进cNBlock步               */       
////       cNBlock = 0;                                            /*  任务清零                    */
////       
////       switch (cDirTemp) 
////       {
////            case 1:
////                GuiStep[m][1] = 0x1;   //右转
////                break;
////	     case 2:
////               GuiStep[m][1] = 0x3;    //后转
////                break;
////            case 3:
////                GuiStep[m][1] = 0x2;   //左转
////                break;
////            default:
////                break;
////       }
////       m++;
////    }
////    GuiStep[m][0] = cNBlock; 
//    
//    
////    GmcMouse.cX = tempX;
////    GmcMouse.cY = tempY;
//
//}
//

/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        根据相对方向，取出下一点上迷宫格的墙壁资料
** input parameters:    ucDir: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      GucMapBlock[cX][cY] : 墙壁资料
*********************************************************************************************************/
uint8 mazeBlockDataGet (uint8  ucDirTemp)
{
    int8 cX = 0,cY = 0;
    switch (ucDirTemp) 
    {
        case MOUSEFRONT:
            ucDirTemp = GucMouseDir;
            break;
        case MOUSELEFT:
            ucDirTemp = (GucMouseDir + 3) % 4;
            break;
        case MOUSERIGHT:
            ucDirTemp = (GucMouseDir + 1) % 4;
            break;
        default:
            break;
    }
    switch (ucDirTemp) 
    {
        case 0:
            cX = GmcMouse.cX;
            cY = GmcMouse.cY + 1;
            break;   
        case 1:
            cX = GmcMouse.cX + 1;
            cY = GmcMouse.cY;
            break;   
        case 2:
            cX = GmcMouse.cX;
            cY = GmcMouse.cY - 1;
            break;    
        case 3:
            cX = GmcMouse.cX - 1;
            cY = GmcMouse.cY;
            break;   
        default:
            break;
    } 
    return(GucMapBlock[cX][cY]);                                    
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
uint8 process1(uint8 cXsur,uint8 cYsur,int8 cXYflag,uint8 k)
{
	int8 count;
	uint8 i;
	if( cXsur==GmcStack[0].cX && (cXYflag==1||cXYflag==3) ||
	    cYsur==GmcStack[0].cY && (cXYflag==0||cXYflag==2) )
	{
		if((cXsur-GmcStack[0].cX)<0)
			return 0;
		if(cXYflag==0)                 //绝对方向为上的路口      
		{
			crosslenth=cXsur-GmcStack[0].cX;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX+count][GmcMouse.cY]&0x08))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cY<GmcStack[0].cY)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX+count;
				Gmcshield[count].cY=GmcMouse.cY;
			}
		}
		if(cXYflag==2)                 //绝对方向为下的路口      
		{
			crosslenth=cXsur-GmcStack[0].cX;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX+count][GmcMouse.cY]&0x08))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cY>GmcStack[0].cY)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX+count;
				Gmcshield[count].cY=GmcMouse.cY;
			}
		}


		if((cYsur-GmcStack[0].cY)<0)
			return 0;
		if(cXYflag==1)               //绝对方向为右的路口
		{
			crosslenth=cYsur-GmcStack[0].cY;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX][GmcMouse.cY+count]&0x04))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cX<GmcStack[0].cX)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX;
				Gmcshield[count].cY=GmcMouse.cY+count;
			}
		}
		if(cXYflag==3)               //绝对方向为左的路口
		{
			crosslenth=cYsur-GmcStack[0].cY;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX][GmcMouse.cY+count]&0x04))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cX>GmcStack[0].cX)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX;
				Gmcshield[count].cY=GmcMouse.cY+count;
			}
		}
		for(i=0;i<k;i++)
		{
			 if( (GmcStack[0].cY<=8&&GmcStack[0].cX<=8 && GucAboveLoad[i].cX>=8&&GucAboveLoad[i].cY>=8) ||   \
                             (GmcStack[0].cY<=8&&GmcStack[0].cX>=8 && GucAboveLoad[i].cX<=8&&GucAboveLoad[i].cY>=8) ||   \
                             (GmcStack[0].cY>=8&&GmcStack[0].cX<=8 && GucAboveLoad[i].cX>=8&&GucAboveLoad[i].cY<=8) ||   \
                             (GmcStack[0].cY>=8&&GmcStack[0].cX>=8 && GucAboveLoad[i].cX<=8&&GucAboveLoad[i].cY<=8)  )
                                                    return 0;
		}
		return 1;
	}
	return 0;
 }



uint8 process2(uint8 cXsur,uint8 cYsur,int8 cXYflag,uint8 k)
{
	int8 count;
	uint8 i;
	if( cXsur==GmcStack[0].cX && (cXYflag==1||cXYflag==3) ||
	    cYsur==GmcStack[0].cY && (cXYflag==0||cXYflag==2) )
	{
		if(GmcStack[0].cX-cXsur<0)
			return 0;
		if(cXYflag==0)                 //绝对方向为上的路口      
		{
		    crosslenth=GmcStack[0].cX-cXsur;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX-count][GmcMouse.cY]&0x02))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cY<GmcStack[0].cY)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX-count;
				Gmcshield[count].cY=GmcMouse.cY;
			}
		}
		if(cXYflag==2)                 //绝对方向为下的路口      
		{
			crosslenth=GmcStack[0].cX-cXsur;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX-count][GmcMouse.cY]&0x02))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cY>GmcStack[0].cY)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX-count;
				Gmcshield[count].cY=GmcMouse.cY;
			}
		}

		if(GmcStack[0].cY-cYsur<0)
			return 0;
		if(cXYflag==1)               //绝对方向为右的路口
		{
			crosslenth=GmcStack[0].cY-cYsur;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX][GmcMouse.cY-count]&0x01))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cX<GmcStack[0].cX)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX;
				Gmcshield[count].cY=GmcMouse.cY-count;
			}
		}
		if(cXYflag==3)               //绝对方向为左的路口
		{
			crosslenth=GmcStack[0].cY-cYsur;
			count=crosslenth-1;
			while(count)
			{
				if(!(GucMapBlock[GmcMouse.cX][GmcMouse.cY-count]&0x01))
						return 0;
				count--;
			}
			if(crosslenth!=1)
				for(i=0;i<k;i++)
				{
					if(GucAboveLoad[i].cX>GmcStack[0].cX)
							return 0;
				}
			for(count=0;count<crosslenth;count++)
			{
				Gmcshield[count].cX=GmcMouse.cX;
				Gmcshield[count].cY=GmcMouse.cY-count;
			}
		}
		for(i=0;i<k;i++)
		{
			 if( (GmcStack[0].cY<=8&&GmcStack[0].cX<=8 && GucAboveLoad[i].cX>8&&GucAboveLoad[i].cY>8) ||   \
                             (GmcStack[0].cY<=8&&GmcStack[0].cX>=8 && GucAboveLoad[i].cX<8&&GucAboveLoad[i].cY>8) ||   \
                             (GmcStack[0].cY>=8&&GmcStack[0].cX<=8 && GucAboveLoad[i].cX>8&&GucAboveLoad[i].cY<8) ||   \
                             (GmcStack[0].cY>=8&&GmcStack[0].cX>=8 && GucAboveLoad[i].cX<8&&GucAboveLoad[i].cY<8)  )
							return 0;
		}
		return 1;
	}
	return 0;
 }

/********************************************************************************************************
**Function name:        ShieldLoad
**Descriptions:         屏蔽没必要的路
**input parameters：    相对方向
**output parameters：   逻辑变量
*********************************************************************************************************/
uint8 ShieldLoad(uint8  ucDir)                              //在转弯函数中
{
    int16 i,j;
    int16 n=0;
    uint8 k=0;
    uint8 ucDirTemp=ucDir;
    uint8 ucStat = 0;                                   //统计可前进的方向数   
    uint8 cXsur,cYsur;
    uint8 cXdst,cYdst;
    int8  cXYflag=0;               
        
    for(i=0;i<16;i++)
      for(j=0;j<16;j++)
      {
          if(GucMapBlock[i][j]==0)
             GucMapFlag[i][j]=0xff;
          else
             GucMapFlag[i][j]=GucMapBlock[i][j];
      }

    if(!uStartFlag)					   //四角初始化
    {
       GucMapLoad[0][0] = 0x00 | 0x00; 
       GucMapLoad[16][0] = 0x01 | 0x08;
    }
    else
    {
       GucMapLoad[0][0] = 0x01 | 0x02; 
       GucMapLoad[16][0] = 0x00 | 0x00;
    }
    GucMapLoad[0][16] = 0x02 | 0x04;
    GucMapLoad[16][16] = 0x04 | 0x08;
    
    
    for(j=1;j<16;j++)                                 //四边初始化
    {
            GucMapLoad[0][j] = 0x01 | 0x04 |  ((~GucMapFlag[0][j-1])&0x01)<<1 | ((~GucMapFlag[0][j])&0x04) >>1 ;  
            //              上   下                             右
            GucMapLoad[16][j] = 0x01 | 0x04 | ((~GucMapFlag[15][j-1])&0x01)<<3 | ((~GucMapFlag[15][j])&0x04)<<1 ;
            //               上     下                          左
    }
    for(i=1;i<16;i++)
    {
            GucMapLoad[i][0] = 0x02 | 0x08 | ((~GucMapFlag[i-1][0])&0x02)>>1 | ((~GucMapFlag[i][0])&0x08)>>3 ;
            //              右    左                           上
            GucMapLoad[i][16] = 0x02 | 0x08 | ((~GucMapFlag[i-1][15])&0x02)<<1 | ((~GucMapFlag[i][15])&0x08)>>1 ;
            //               右    左                          下
    }
    for(j=1;j<16;j++)                                                  
      for(i=1;i<16;i++)
      {
          GucMapLoad[i][j] = ((~GucMapFlag[i-1][j])&0x02)>>1 | ((~GucMapFlag[i][j])&0x08)>>3      |  \
     //                                    上
                             ((~GucMapFlag[i][j-1])&0x01)<<1 | ((~GucMapFlag[i][j])&0x04) >>1     |  \
             //                                    右                
                             ((~GucMapFlag[i-1][j-1])&0x02)<<1 | ((~GucMapFlag[i][j-1])&0x08)>>1  |  \
             //                                    下
                             ((~GucMapFlag[i-1][j-1])&0x01)<<3 | ((~GucMapFlag[i-1][j])&0x04)<<1  ;
             //                                    左
      }
    GucMapLoad[8][8]=0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//确定源，目的坐标
    switch (ucDirTemp) 
    {
		case MOUSEFRONT:
			ucDirTemp = GucMouseDir;
			break;
		case MOUSELEFT:
			ucDirTemp = (GucMouseDir + 3) % 4;
			break;
		case MOUSERIGHT:
			ucDirTemp = (GucMouseDir + 1) % 4;
			break;
		default:
			break;
    }
    switch (ucDirTemp) 
    {
		case 0:
			cXsur = GmcMouse.cX;
			cYsur = GmcMouse.cY + 1;
			cXdst = GmcMouse.cX + 1;
			cYdst = GmcMouse.cY + 1;
			cXYflag= 0;
			break; 
		case 1:
			cXsur = GmcMouse.cX+1;
			cYsur = GmcMouse.cY;
			cXdst = GmcMouse.cX + 1;
			cYdst = GmcMouse.cY + 1;
			cXYflag=1; 
			break;       
		case 2:
			cXsur = GmcMouse.cX;
			cYsur = GmcMouse.cY;
			cXdst = GmcMouse.cX + 1;
			cYdst = GmcMouse.cY;
			cXYflag=2;
			break;    
		case 3:
			cXsur = GmcMouse.cX;
			cYsur = GmcMouse.cY;
			cXdst = GmcMouse.cX;
			cYdst = GmcMouse.cY + 1;
			cXYflag=3;
			break;     
		default:
			break;
     }
/////////////////////////////////////////////////////////////////////////////////////////////////
	//确定是否形成环路
        GmcStack[n].cX  = cXsur;                             /*  起点X值入栈                 */
        GmcStack[n].cY  = cYsur;                             /*  起点Y值入栈                 */
        n++;
        
        GucAboveLoad[k].cX=cXsur;                            /*  起点到终点路径              */
        GucAboveLoad[k].cY=cYsur;
        k++;
        
        for(i=0;i<17;i++)
          for(j=0;j<17;j++)
            GucMapFlag[i][j]=0;
        GucMapFlag[cXsur][cYsur]=1;
        
        while (n) 
        {
            ucStat = 0;
            if ((GucMapLoad[cXsur][cYsur] & 0x01) &&     
                (GucMapFlag[cXsur][cYsur+ 1]==0))              
					  ucStat++;                           
            if ((GucMapLoad[cXsur][cYsur] & 0x02) &&     
                (GucMapFlag[cXsur + 1][cYsur]==0))                                 
				      ucStat++;                            
            if ((GucMapLoad[cXsur][cYsur] & 0x04) &&
                (GucMapFlag[cXsur][cYsur - 1]==0))
			            ucStat++;                            
            if ((GucMapLoad[cXsur][cYsur] & 0x08) &&
                (GucMapFlag[cXsur - 1][cYsur]==0))
				         ucStat++;                            
            if (ucStat == 0) 
            {
                n--;
                cXsur = GmcStack[n].cX;
                cYsur = GmcStack[n].cY;
                k=Gucsave[n];
            } 
            else 
            {
                loadcount=k+1;
                if (ucStat > 1) 
                {                                       
                    GmcStack[n].cX = cXsur;             
                    GmcStack[n].cY = cYsur;             
                    Gucsave[n]=k;
                    n++;
                }
                if ((GucMapLoad[cXsur][cYsur] & 0x01) &&            /*  上方有路                    */
                    (GucMapFlag[cXsur][cYsur+ 1]==0)) 
                {   
                    cYsur++;                                      
                    GucAboveLoad[k].cX=cXsur;
                    GucAboveLoad[k].cY=cYsur;
                    k++;
                    GucMapFlag[cXsur][cYsur]=1;
                    if(process1(cXsur,cYsur,cXYflag,k))
                            return 1;
                    continue;
                }
                if ((GucMapLoad[cXsur][cYsur] & 0x02) &&            /*  右方有路                    */
                    (GucMapFlag[cXsur + 1][cYsur]==0)) 
                {   
                    cXsur++;                                    
                    GucAboveLoad[k].cX=cXsur;
                    GucAboveLoad[k].cY=cYsur;
                    k++;
                    GucMapFlag[cXsur][cYsur]=1;
					if(process1(cXsur,cYsur,cXYflag,k))
						return 1;
					continue;
                }
                if ((GucMapLoad[cXsur][cYsur] & 0x04) &&            /*  下方有路                    */
                    (GucMapFlag[cXsur][cYsur - 1]==0))
                {   
                    cYsur--;                                     
                    GucAboveLoad[k].cX=cXsur;
                    GucAboveLoad[k].cY=cYsur;
                    k++;
                    GucMapFlag[cXsur][cYsur]=1;
					if(process1(cXsur,cYsur,cXYflag,k))
						return 1;
					continue; 
                }
                if ((GucMapLoad[cXsur][cYsur] & 0x08) &&             /*  左方有路                    */
                    (GucMapFlag[cXsur - 1][cYsur]==0))
                {      
                    cXsur--;                                         
                    GucAboveLoad[k].cX=cXsur;
                    GucAboveLoad[k].cY=cYsur;
                    k++;
                    GucMapFlag[cXsur][cYsur]=1;
					if(process1(cXsur,cYsur,cXYflag,k))
						return 1;
					continue;
                }
            }
        }
/////////////////////////////////////////////////////////////////////////////////////////
        //改变起点跟终点
		ucStat=cXsur;
		cXsur=cXdst;
		cXdst=ucStat;
		ucStat=cYsur;
		cYsur=cYdst;
		cYdst=ucStat;

		n=0;
		GmcStack[n].cX  = cXsur;                             /*  起点X值入栈                 */
		GmcStack[n].cY  = cYsur;                             /*  起点Y值入栈                 */
		n++;
		k=0;
		GucAboveLoad[k].cX=cXsur;                       
		GucAboveLoad[k].cY=cYsur;
		k++;

		for(i=0;i<17;i++)                                   
		  for(j=0;j<17;j++)
			GucMapFlag[i][j]=0;
		GucMapFlag[cXsur][cYsur]=1;  

		while (n) 
		{
			ucStat = 0;
			if ((GucMapLoad[cXsur][cYsur] & 0x01) &&     
				(GucMapFlag[cXsur][cYsur+ 1]==0))              
					  ucStat++;                           
			if ((GucMapLoad[cXsur][cYsur] & 0x02) &&     
				(GucMapFlag[cXsur + 1][cYsur]==0))                                 
					  ucStat++;                            
			if ((GucMapLoad[cXsur][cYsur] & 0x04) &&
				(GucMapFlag[cXsur][cYsur - 1]==0))
						ucStat++;                            
			if ((GucMapLoad[cXsur][cYsur] & 0x08) &&
				(GucMapFlag[cXsur - 1][cYsur]==0))
						 ucStat++; 
			if (ucStat == 0) 
			{
				n--;
				cXsur = GmcStack[n].cX;
				cYsur = GmcStack[n].cY;
				k=Gucsave[n];
			} 
			else 
			{
                                loadcount=k+1;
				if (ucStat > 1) 
				{                                       
					GmcStack[n].cX = cXsur;            
					GmcStack[n].cY = cYsur;            
					Gucsave[n]=k;                        //保存下标
					n++;
				}
				if ((GucMapLoad[cXsur][cYsur] & 0x01) &&            /*  上方有路 */
					(GucMapFlag[cXsur][cYsur+ 1]==0)) 
				{    
					cYsur++;                                     
					GucAboveLoad[k].cX=cXsur;
					GucAboveLoad[k].cY=cYsur;
					k++;
					GucMapFlag[cXsur][cYsur]=1;
					if(process2(cXsur,cYsur,cXYflag,k))
						return 1;
					continue;
				}
				if ((GucMapLoad[cXsur][cYsur] & 0x02) &&            /*  右方有路                    */
					(GucMapFlag[cXsur + 1][cYsur]==0)) 
					{   
						cXsur++;                                 
						GucAboveLoad[k].cX=cXsur;
						GucAboveLoad[k].cY=cYsur;
						k++;
						GucMapFlag[cXsur][cYsur]=1;
						if(process2(cXsur,cYsur,cXYflag,k))
							return 1;
						continue;
					}
				if ((GucMapLoad[cXsur][cYsur] & 0x04) &&            /*  下方有路                    */
					(GucMapFlag[cXsur][cYsur - 1]==0))
				{   
					cYsur--;                                    
					GucAboveLoad[k].cX=cXsur;
					GucAboveLoad[k].cY=cYsur;
					k++;
					GucMapFlag[cXsur][cYsur]=1;
					if(process2(cXsur,cYsur,cXYflag,k))
						return 1;
					continue;
				}
				if ((GucMapLoad[cXsur][cYsur] & 0x08) &&             /*  左方有路                    */
					(GucMapFlag[cXsur - 1][cYsur]==0))
				{      
					cXsur--;                                      
					GucAboveLoad[k].cX=cXsur;
	         		GucAboveLoad[k].cY=cYsur;
	      			k++;
					GucMapFlag[cXsur][cYsur]=1;
					if(process2(cXsur,cYsur,cXYflag,k))
						return 1;
					continue;
				}
			}
		}
	return 0;    //没有形成环路
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//uint8 ShieldLoad(uint8  ucDir)                              //在转弯函数中
//{
//    int16 i,j;
//    int16 n=0;
//    uint8 k=0;
//    uint8 ucDirTemp=ucDir;
//    uint8 ucStat = 0;                                   //统计可前进的方向数   
//    uint8 cXsur,cYsur;
//    uint8 cXdst,cYdst;
//    int8  cXYflag=0;               
//        
//    for(i=0;i<8;i++)
//      for(j=0;j<8;j++)
//      {
//          if(GucMapBlock[i][j]==0)
//             GucMapFlag[i][j]=0xff;
//          else
//             GucMapFlag[i][j]=GucMapBlock[i][j];
//      }
//
//    if(!uStartFlag)					   //四角初始化
//    {
//       GucMapLoad[0][0] = 0x00 | 0x00; 
//       GucMapLoad[8][0] = 0x01 | 0x08;
//    }
//    else
//    {
//       GucMapLoad[0][0] = 0x01 | 0x02; 
//       GucMapLoad[8][0] = 0x00 | 0x00;
//    }
//    GucMapLoad[0][8] = 0x02 | 0x04;
//    GucMapLoad[8][8] = 0x04 | 0x08;
//    
//    
//    for(j=1;j<8;j++)                                 //四边初始化
//    {
//            GucMapLoad[0][j] = 0x01 | 0x04 |  ((~GucMapFlag[0][j-1])&0x01)<<1 | ((~GucMapFlag[0][j])&0x04) >>1 ;  
//            //              上   下                             右
//            GucMapLoad[8][j] = 0x01 | 0x04 | ((~GucMapFlag[7][j-1])&0x01)<<3 | ((~GucMapFlag[7][j])&0x04)<<1 ;
//            //               上     下                          左
//    }
//    for(i=1;i<8;i++)
//    {
//            GucMapLoad[i][0] = 0x02 | 0x08 | ((~GucMapFlag[i-1][0])&0x02)>>1 | ((~GucMapFlag[i][0])&0x08)>>3 ;
//            //              右    左                           上
//            GucMapLoad[i][8] = 0x02 | 0x08 | ((~GucMapFlag[i-1][7])&0x02)<<1 | ((~GucMapFlag[i][7])&0x08)>>1 ;
//            //               右    左                          下
//    }
//    for(j=1;j<8;j++)                                                  
//      for(i=1;i<8;i++)
//      {
//          GucMapLoad[i][j] = ((~GucMapFlag[i-1][j])&0x02)>>1 | ((~GucMapFlag[i][j])&0x08)>>3      |  \
//     //                                    上
//                             ((~GucMapFlag[i][j-1])&0x01)<<1 | ((~GucMapFlag[i][j])&0x04) >>1     |  \
//             //                                    右                
//                             ((~GucMapFlag[i-1][j-1])&0x02)<<1 | ((~GucMapFlag[i][j-1])&0x08)>>1  |  \
//             //                                    下
//                             ((~GucMapFlag[i-1][j-1])&0x01)<<3 | ((~GucMapFlag[i-1][j])&0x04)<<1  ;
//             //                                    左
//      }
//    GucMapLoad[8][8]=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	//确定源，目的坐标
//    switch (ucDirTemp) 
//    {
//		case MOUSEFRONT:
//			ucDirTemp = GucMouseDir;
//			break;
//		case MOUSELEFT:
//			ucDirTemp = (GucMouseDir + 3) % 4;
//			break;
//		case MOUSERIGHT:
//			ucDirTemp = (GucMouseDir + 1) % 4;
//			break;
//		default:
//			break;
//    }
//    switch (ucDirTemp) 
//    {
//		case 0:
//			cXsur = GmcMouse.cX;
//			cYsur = GmcMouse.cY + 1;
//			cXdst = GmcMouse.cX + 1;
//			cYdst = GmcMouse.cY + 1;
//			cXYflag= 0;
//			break; 
//		case 1:
//			cXsur = GmcMouse.cX+1;
//			cYsur = GmcMouse.cY;
//			cXdst = GmcMouse.cX + 1;
//			cYdst = GmcMouse.cY + 1;
//			cXYflag=1; 
//			break;       
//		case 2:
//			cXsur = GmcMouse.cX;
//			cYsur = GmcMouse.cY;
//			cXdst = GmcMouse.cX + 1;
//			cYdst = GmcMouse.cY;
//			cXYflag=2;
//			break;    
//		case 3:
//			cXsur = GmcMouse.cX;
//			cYsur = GmcMouse.cY;
//			cXdst = GmcMouse.cX;
//			cYdst = GmcMouse.cY + 1;
//			cXYflag=3;
//			break;     
//		default:
//			break;
//     }
///////////////////////////////////////////////////////////////////////////////////////////////////
//	//确定是否形成环路
//        GmcStack[n].cX  = cXsur;                             /*  起点X值入栈                 */
//        GmcStack[n].cY  = cYsur;                             /*  起点Y值入栈                 */
//        n++;
//        
//        GucAboveLoad[k].cX=cXsur;                            /*  起点到终点路径              */
//        GucAboveLoad[k].cY=cYsur;
//        k++;
//        
//        for(i=0;i<9;i++)
//          for(j=0;j<9;j++)
//            GucMapFlag[i][j]=0;
//        GucMapFlag[cXsur][cYsur]=1;
//        
//        while (n) 
//        {
//            ucStat = 0;
//            if ((GucMapLoad[cXsur][cYsur] & 0x01) &&     
//                (GucMapFlag[cXsur][cYsur+ 1]==0))              
//					  ucStat++;                           
//            if ((GucMapLoad[cXsur][cYsur] & 0x02) &&     
//                (GucMapFlag[cXsur + 1][cYsur]==0))                                 
//				      ucStat++;                            
//            if ((GucMapLoad[cXsur][cYsur] & 0x04) &&
//                (GucMapFlag[cXsur][cYsur - 1]==0))
//			            ucStat++;                            
//            if ((GucMapLoad[cXsur][cYsur] & 0x08) &&
//                (GucMapFlag[cXsur - 1][cYsur]==0))
//				         ucStat++;                            
//            if (ucStat == 0) 
//            {
//                n--;
//                cXsur = GmcStack[n].cX;
//                cYsur = GmcStack[n].cY;
//                k=Gucsave[n];
//            } 
//            else 
//            {
//                loadcount=k+1;
//                if (ucStat > 1) 
//                {                                       
//                    GmcStack[n].cX = cXsur;             
//                    GmcStack[n].cY = cYsur;             
//                    Gucsave[n]=k;
//                    n++;
//                }
//                if ((GucMapLoad[cXsur][cYsur] & 0x01) &&            /*  上方有路                    */
//                    (GucMapFlag[cXsur][cYsur+ 1]==0)) 
//                {   
//                    cYsur++;                                      
//                    GucAboveLoad[k].cX=cXsur;
//                    GucAboveLoad[k].cY=cYsur;
//                    k++;
//                    GucMapFlag[cXsur][cYsur]=1;
//                    if(process1(cXsur,cYsur,cXYflag,k))
//                            return 1;
//                    continue;
//                }
//                if ((GucMapLoad[cXsur][cYsur] & 0x02) &&            /*  右方有路                    */
//                    (GucMapFlag[cXsur + 1][cYsur]==0)) 
//                {   
//                    cXsur++;                                    
//                    GucAboveLoad[k].cX=cXsur;
//                    GucAboveLoad[k].cY=cYsur;
//                    k++;
//                    GucMapFlag[cXsur][cYsur]=1;
//					if(process1(cXsur,cYsur,cXYflag,k))
//						return 1;
//					continue;
//                }
//                if ((GucMapLoad[cXsur][cYsur] & 0x04) &&            /*  下方有路                    */
//                    (GucMapFlag[cXsur][cYsur - 1]==0))
//                {   
//                    cYsur--;                                     
//                    GucAboveLoad[k].cX=cXsur;
//                    GucAboveLoad[k].cY=cYsur;
//                    k++;
//                    GucMapFlag[cXsur][cYsur]=1;
//					if(process1(cXsur,cYsur,cXYflag,k))
//						return 1;
//					continue; 
//                }
//                if ((GucMapLoad[cXsur][cYsur] & 0x08) &&             /*  左方有路                    */
//                    (GucMapFlag[cXsur - 1][cYsur]==0))
//                {      
//                    cXsur--;                                         
//                    GucAboveLoad[k].cX=cXsur;
//                    GucAboveLoad[k].cY=cYsur;
//                    k++;
//                    GucMapFlag[cXsur][cYsur]=1;
//					if(process1(cXsur,cYsur,cXYflag,k))
//						return 1;
//					continue;
//                }
//            }
//        }
///////////////////////////////////////////////////////////////////////////////////////////
//        //改变起点跟终点
//		ucStat=cXsur;
//		cXsur=cXdst;
//		cXdst=ucStat;
//		ucStat=cYsur;
//		cYsur=cYdst;
//		cYdst=ucStat;
//
//		n=0;
//		GmcStack[n].cX  = cXsur;                             /*  起点X值入栈                 */
//		GmcStack[n].cY  = cYsur;                             /*  起点Y值入栈                 */
//		n++;
//		k=0;
//		GucAboveLoad[k].cX=cXsur;                       
//		GucAboveLoad[k].cY=cYsur;
//		k++;
//
//		for(i=0;i<9;i++)                                   
//		  for(j=0;j<9;j++)
//			GucMapFlag[i][j]=0;
//		GucMapFlag[cXsur][cYsur]=1;  
//
//		while (n) 
//		{
//			ucStat = 0;
//			if ((GucMapLoad[cXsur][cYsur] & 0x01) &&     
//				(GucMapFlag[cXsur][cYsur+ 1]==0))              
//					  ucStat++;                           
//			if ((GucMapLoad[cXsur][cYsur] & 0x02) &&     
//				(GucMapFlag[cXsur + 1][cYsur]==0))                                 
//					  ucStat++;                            
//			if ((GucMapLoad[cXsur][cYsur] & 0x04) &&
//				(GucMapFlag[cXsur][cYsur - 1]==0))
//						ucStat++;                            
//			if ((GucMapLoad[cXsur][cYsur] & 0x08) &&
//				(GucMapFlag[cXsur - 1][cYsur]==0))
//						 ucStat++; 
//			if (ucStat == 0) 
//			{
//				n--;
//				cXsur = GmcStack[n].cX;
//				cYsur = GmcStack[n].cY;
//				k=Gucsave[n];
//			} 
//			else 
//			{
//                                loadcount=k+1;
//				if (ucStat > 1) 
//				{                                       
//					GmcStack[n].cX = cXsur;            
//					GmcStack[n].cY = cYsur;            
//					Gucsave[n]=k;                        //保存下标
//					n++;
//				}
//				if ((GucMapLoad[cXsur][cYsur] & 0x01) &&            /*  上方有路 */
//					(GucMapFlag[cXsur][cYsur+ 1]==0)) 
//				{    
//					cYsur++;                                     
//					GucAboveLoad[k].cX=cXsur;
//					GucAboveLoad[k].cY=cYsur;
//					k++;
//					GucMapFlag[cXsur][cYsur]=1;
//					if(process2(cXsur,cYsur,cXYflag,k))
//						return 1;
//					continue;
//				}
//				if ((GucMapLoad[cXsur][cYsur] & 0x02) &&            /*  右方有路                    */
//					(GucMapFlag[cXsur + 1][cYsur]==0)) 
//					{   
//						cXsur++;                                 
//						GucAboveLoad[k].cX=cXsur;
//						GucAboveLoad[k].cY=cYsur;
//						k++;
//						GucMapFlag[cXsur][cYsur]=1;
//						if(process2(cXsur,cYsur,cXYflag,k))
//							return 1;
//						continue;
//					}
//				if ((GucMapLoad[cXsur][cYsur] & 0x04) &&            /*  下方有路                    */
//					(GucMapFlag[cXsur][cYsur - 1]==0))
//				{   
//					cYsur--;                                    
//					GucAboveLoad[k].cX=cXsur;
//					GucAboveLoad[k].cY=cYsur;
//					k++;
//					GucMapFlag[cXsur][cYsur]=1;
//					if(process2(cXsur,cYsur,cXYflag,k))
//						return 1;
//					continue;
//				}
//				if ((GucMapLoad[cXsur][cYsur] & 0x08) &&             /*  左方有路                    */
//					(GucMapFlag[cXsur - 1][cYsur]==0))
//				{      
//					cXsur--;                                      
//					GucAboveLoad[k].cX=cXsur;
//	         		GucAboveLoad[k].cY=cYsur;
//	      			k++;
//					GucMapFlag[cXsur][cYsur]=1;
//					if(process2(cXsur,cYsur,cXYflag,k))
//						return 1;
//					continue;
//				}
//			}
//		}
//	return 0;    //没有形成环路
//}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        右手法则，优先向右前进
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) 
    {                       
        if(ShieldLoad(MOUSERIGHT))
        {
           ushieldflag = 1; 
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_R));
                --crosslenth;
           }
           return;
        }
        mouseTurnright();                                             
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) 
    {                      
       if(ShieldLoad(MOUSEFRONT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_F));
                --crosslenth;
           }
           return;
        }
        return;                                                        
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) 
    {                     
       if(ShieldLoad(MOUSELEFT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_L));
                --crosslenth;
           }
           return;
        }
        mouseTurnleft();                                            
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        左手法则，优先向左运动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) 
    {                       
       if(ShieldLoad(MOUSELEFT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_L));
                --crosslenth;
           }
           return;
        }
        mouseTurnleft();                                           
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) 
    {                      
       if(ShieldLoad(MOUSEFRONT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_F));
                --crosslenth;
           }
           return;
        }
        return;                                                        
    }
     if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) 
    {                       
        if(ShieldLoad(MOUSERIGHT))
        {
           ushieldflag = 1; 
           while(crosslenth)
           {
             GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_R));
                --crosslenth;
           }
           return;
        }
        mouseTurnright();                                              
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        中右法则，优先向前运行，其次向右
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontRightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) 
    {                     
       if(ShieldLoad(MOUSEFRONT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_F));
                --crosslenth;
           }
           return;
        }
        return;                                                      
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) 
    {                      
        if(ShieldLoad(MOUSERIGHT))
        {
           ushieldflag = 1; 
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_R));
                --crosslenth;
           }
           return;
        }
        mouseTurnright();                                               
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) 
    {                      
       if(ShieldLoad(MOUSELEFT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_L));
                --crosslenth;
           }
           return;
        }
        mouseTurnleft();                                                
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        中左法则，优先向前运行，其次向左
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) 
    {                      
       if(ShieldLoad(MOUSEFRONT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_F));
                --crosslenth;
           }
           return;
        }
        return;                                                        
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) 
    {                    
       if(ShieldLoad(MOUSELEFT))
        {
           ushieldflag = 1 ;
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_L));
                --crosslenth;
           }
           return;
        }
        mouseTurnleft();                                               
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) 
    {                      
        if(ShieldLoad(MOUSERIGHT))
        {
           ushieldflag = 1; 
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_R));
                --crosslenth;
           }
           return;
        }
        mouseTurnright();                                             
        return;
    }
}
/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        中心法则，根据电脑鼠目前在迷宫中所处的位置觉定使用何种搜索法则
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的右上角
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的右下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的左上角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                leftMethod();                                           /*  左手法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的左下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                rightMethod();                                          /*  右手法则                    */
                break;

            default:
                break;
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       calculatemap
** Descriptions:        计算某些未走过坐标的挡板情况
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void calculmap(void)
{
        uint8 i,j;
        if(GucMapBlock[0][0]==0 && GucMapBlock[0][1]!=0 && GucMapBlock[1][0]!=0)                              //[0,0]
            GucMapBlock[0][0]=(GucMapBlock[0][1]&0x04)>>2 | (GucMapBlock[1][0]&0x08)>>2;
        if(GucMapBlock[0][15]==0 && GucMapBlock[0][14]!=0 && GucMapBlock[1][15]!=0)                           //[0,15]
            GucMapBlock[0][15]=(GucMapBlock[0][14]&0x01)<<2 | (GucMapBlock[1][15]&0x08)>>2;
        if(GucMapBlock[15][15]==0 && GucMapBlock[15][14]!=0 && GucMapBlock[14][15]!=0)                        //[15,15]
            GucMapBlock[15][15]=(GucMapBlock[15][14]&0x01)<<2 | (GucMapBlock[14][15]&0x02)<<2;
        if(GucMapBlock[15][0]==0 && GucMapBlock[15][1]!=0 && GucMapBlock[14][0]!=0)                           //[15,0]
            GucMapBlock[15][0]=(GucMapBlock[15][1]&0x04)>>2 | (GucMapBlock[14][0]&0x02)<<2;
        
        
	for(i=1;i<15;i++)                                 //四边初始化
	{
            if(GucMapBlock[i][0]==0 && GucMapBlock[i-1][0]!=0 && GucMapBlock[i+1][0]!=0 && GucMapBlock[i][1]!=0)
                GucMapBlock[i][0] = (GucMapBlock[i-1][0]&0x02)<<2 | (GucMapBlock[i][1]&0x04)>>2 | (GucMapBlock[i+1][0]&0x08)>>2 ;  
		//                        左                              上                           右
            if(GucMapBlock[i][15]==0 && GucMapBlock[i-1][15]!=0 && GucMapBlock[i+1][15]!=0 && GucMapBlock[i][14]!=0)
                GucMapBlock[i][15] = (GucMapBlock[i-1][15]&0x02)<<2 | (GucMapBlock[i][14]&0x01)<<2 | (GucMapBlock[i+1][15]&0x08)>>2 ; 
		//                        左                              下                           右
        }
	for(j=1;j<15;j++)
	{
            if(GucMapBlock[0][j]==0 && GucMapBlock[0][j-1]!=0 && GucMapBlock[1][j]!=0 && GucMapBlock[0][j+1]!=0 )
                GucMapBlock[0][j] = (GucMapBlock[0][j-1]&0x01)<<2 | (GucMapBlock[1][j]&0x08)>>2 | (GucMapBlock[0][j+1]&0x04)>>2;  
		//                        下                              右                           上
            if(GucMapBlock[15][j]==0 && GucMapBlock[15][j-1]!=0 && GucMapBlock[14][j]!=0 && GucMapBlock[15][j+1]!=0)
                GucMapBlock[15][j] = (GucMapBlock[15][j-1]&0x01)<<2 | (GucMapBlock[14][j]&0x02)<<2 | (GucMapBlock[0][j+1]&0x04)>>2 ; 
		//                        下                              左                           下
        }
	
        
      for(i=1;i<15;i++)                                                  
	 for(j=1;j<15;j++)
	 {
              if(GucMapBlock[i][j]==0 && GucMapBlock[i][j+1]!=0 && GucMapBlock[i+1][j]!=0 && GucMapBlock[i][j-1]!=0 && GucMapBlock[i-1][j]!=0)
                    GucMapBlock[i][j] = (GucMapBlock[i][j+1]&0x04)>>2 | (GucMapBlock[i+1][j]&0x08)>>2 | (GucMapBlock[i][j-1]&0x01)<<2 | (GucMapBlock[i-1][j]&0x02)<<2;
		 //                        上                              右                           下                                左
	 }
}

//////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
//void calculmap(void)
//{
//        uint8 i,j;
//        if(GucMapBlock[0][0]==0 && GucMapBlock[0][1]!=0 && GucMapBlock[1][0]!=0)                              //[0,0]
//            GucMapBlock[0][0]=(GucMapBlock[0][1]&0x04)>>2 | (GucMapBlock[1][0]&0x08)>>2;
//        if(GucMapBlock[0][7]==0 && GucMapBlock[0][6]!=0 && GucMapBlock[1][7]!=0)                           //[0,15]
//            GucMapBlock[0][7]=(GucMapBlock[0][6]&0x01)<<2 | (GucMapBlock[1][7]&0x08)>>2;
//        if(GucMapBlock[7][7]==0 && GucMapBlock[7][6]!=0 && GucMapBlock[6][7]!=0)                        //[15,15]
//            GucMapBlock[7][7]=(GucMapBlock[7][6]&0x01)<<2 | (GucMapBlock[6][7]&0x02)<<2;
//        if(GucMapBlock[7][0]==0 && GucMapBlock[7][1]!=0 && GucMapBlock[6][0]!=0)                           //[15,0]
//            GucMapBlock[7][0]=(GucMapBlock[7][1]&0x04)>>2 | (GucMapBlock[6][0]&0x02)<<2;
//        
//        
//	for(i=1;i<7;i++)                                 //四边初始化
//	{
//            if(GucMapBlock[i][0]==0 && GucMapBlock[i-1][0]!=0 && GucMapBlock[i+1][0]!=0 && GucMapBlock[i][1]!=0)
//                GucMapBlock[i][0] = (GucMapBlock[i-1][0]&0x02)<<2 | (GucMapBlock[i][1]&0x04)>>2 | (GucMapBlock[i+1][0]&0x08)>>2 ;  
//		//                        左                              上                           右
//            if(GucMapBlock[i][7]==0 && GucMapBlock[i-1][7]!=0 && GucMapBlock[i+1][7]!=0 && GucMapBlock[i][6]!=0)
//                GucMapBlock[i][7] = (GucMapBlock[i-1][7]&0x02)<<2 | (GucMapBlock[i][6]&0x01)<<2 | (GucMapBlock[i+1][7]&0x08)>>2 ; 
//		//                        左                              下                           右
//        }
//	for(j=1;j<7;j++)
//	{
//            if(GucMapBlock[0][j]==0 && GucMapBlock[0][j-1]!=0 && GucMapBlock[1][j]!=0 && GucMapBlock[0][j+1]!=0 )
//                GucMapBlock[0][j] = (GucMapBlock[0][j-1]&0x01)<<2 | (GucMapBlock[1][j]&0x08)>>2 | (GucMapBlock[0][j+1]&0x04)>>2;  
//		//                        下                              右                           上
//            if(GucMapBlock[7][j]==0 && GucMapBlock[7][j-1]!=0 && GucMapBlock[6][j]!=0 && GucMapBlock[7][j+1]!=0)
//                GucMapBlock[7][j] = (GucMapBlock[7][j-1]&0x01)<<2 | (GucMapBlock[6][j]&0x02)<<2 | (GucMapBlock[0][j+1]&0x04)>>2 ; 
//		//                        下                              左                           下
//        }
//	
//        
//      for(i=1;i<7;i++)                                                  
//	 for(j=1;j<7;j++)
//	 {
//              if(GucMapBlock[i][j]==0 && GucMapBlock[i][j+1]!=0 && GucMapBlock[i+1][j]!=0 && GucMapBlock[i][j-1]!=0 && GucMapBlock[i-1][j]!=0)
//                    GucMapBlock[i][j] = (GucMapBlock[i][j+1]&0x04)>>2 | (GucMapBlock[i+1][j]&0x08)>>2 | (GucMapBlock[i][j-1]&0x01)<<2 | (GucMapBlock[i-1][j]&0x02)<<2;
//		 //                        上                              右                           下                                左
//	 }
//}

/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        统计某坐标存在还未走过的支路数
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      ucCt，未走过的支路数
*********************************************************************************************************/
uint8 crosswayCheck (int8  cX, int8  cY)
{
    uint8 ucCt = 0;
    calculmap();        //计算挡板信息
    if ((GucMapBlock[cX][cY] & 0x01) &&                                
        (GucMapBlock[cX][cY + 1]) == 0x00) 
    {                            
        ucCt++;                                                      
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                               
        (GucMapBlock[cX + 1][cY]) == 0x00) 
    {                          
        ucCt++;                                                       
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                               
        (GucMapBlock[cX][cY - 1]) == 0x00) 
    {                         
        ucCt++;                                                     
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                               
        (GucMapBlock[cX - 1][cY]) == 0x00) 
    {                           
        ucCt++;                                                       
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        选择一条支路作为前进方向
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
////////////////////////////////////原始函数
void crosswayChoice (void)
{
    switch (SEARCHMETHOD) {
        
    case RIGHTMETHOD:
        rightMethod();
        break;
    
    case LEFTMETHOD:
        leftMethod();
        break;
    
    case CENTRALMETHOD:
        centralMethod();
        break;

    case FRONTRIGHTMETHOD:
        frontRightMethod();
        break;

    case FRONTLEFTMETHOD:
        frontLeftMethod();
        break;

    default:
        break;
    }
}

////////////////////////////////////彭文诺函数

//void crosswayChoice1 (void)//转了
//{
//    switch (GucMouseDir) {
//      
//    case UP:                                                            /*  当前电脑鼠向上 */
//        leftMethod();                                                   /*  左手法则 */
//        break;
//        
//    case RIGHT:                                                         /*  当前电脑鼠向右*/
//        leftMethod();                                                   /*  左手法则  */
//        break;
//        
//    case DOWN:                                                          /*  当前电脑鼠向下*/
//        rightMethod();                                                  /*  右手法则  */                                                     
//        break;
//        
//    case LEFT:                                                          /*  当前电脑鼠向左*/
//        frontRightMethod();                                             /*  中右法则*/
//        break;  
//        
//    default:
//        break;
//    }
//}
//
//void crosswayChoice2 (void)//没转换
//{
//     switch (GucMouseDir) {
//       
//     case UP:                                                           /*  当前电脑鼠向上*/
//         rightMethod();                                                 /*  右手法则 */
//         break;
//                
//     case RIGHT:                                                        /*  当前电脑鼠向右*/
//         frontLeftMethod();                                             /*  中左手法则*/
//         break; 
//                
//     case DOWN:                                                         /*  当前电脑鼠向下*/
//         leftMethod();                                                  /*  左手法则*/
//         break;
//                
//     case LEFT:                                                         /*  当前电脑鼠向左*/
//         rightMethod();                                                 /*  右手法则*/
//         break;
//                
//     default:
//         break;
//    }
//}

//void crosswayChoice (void)
//{
//    switch (cor_changed) {
//        
//    case 0: crosswayChoice2();
//        break;
//        
//    default: crosswayChoice1();
//        break;
//    }
//}

//////////////////////////////////////向心法则1
//void crosswayChoice (void)
//{
//  int sm=0;//方向
//  
//  if(GucXStart==15)
//     if( GmcMouse.cY+GmcMouse.cX>=15)
//  {
//    if(GucMouseDir==UP) sm=3;  //左
//    if(GucMouseDir==DOWN) sm=0;//右
//    if(GucMouseDir==RIGHT) sm=3;//左
//    if(GucMouseDir==LEFT) sm=2;// 前右
//  }  
//   if( GmcMouse.cY+GmcMouse.cX<15)
//  {
//    if(GucMouseDir==UP) sm=3; //  左
//    if(GucMouseDir==DOWN) sm=0;// 右
//    if(GucMouseDir==RIGHT) sm=3;// 左
//    if(GucMouseDir==LEFT) sm=0;//  右
//  }
//    if(GucXStart == 0)
//   {
//     if( GmcMouse.cY>=GmcMouse.cX)
//     {
//       if(GucMouseDir==UP) sm=0;  //右
//       if(GucMouseDir==DOWN) sm=3;//左
//       if(GucMouseDir==RIGHT) sm=1;//前左
//       if(GucMouseDir==LEFT) sm=0;// 右
//     }  
//    if( GmcMouse.cY<GmcMouse.cX)
//    {
//    if(GucMouseDir==UP) sm=0; //  右
//    if(GucMouseDir==DOWN) sm=3;// 左
//    if(GucMouseDir==RIGHT) sm=3;// 左
//    if(GucMouseDir==LEFT) sm=0;//  右
//    }
//   }
//  switch(sm){
//  case 0: rightMethod(); break;
//  case 1: frontLeftMethod();break;
//  case 2: frontRightMethod();break;
//  case 3: leftMethod();break;
//  }
//}

//////////////////////////////////////向心法则2
//void crosswayChoice (void)
//{
//  if (GmcMouse.cX & 0x08)
//	{
//        if (GmcMouse.cY & 0x08)   //此时电脑鼠在迷宫的右上角
//		{               
//			if(GmcMouse.cY>=GmcMouse.cX)
//		{switch (GucMouseDir) {              
//            case UP:
//		rightMethod();                 
//                break;
//            case RIGHT: 
//		rightMethod(); 
//                break;
//            case DOWN: 
//                frontLeftMethod();   
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                leftMethod();
//                break;
//            default:
//                break;
//            }}
//			else
//		{switch (GucMouseDir) {             
//            case UP:
//	        leftMethod();         
//                break;
//            case RIGHT: 
//		rightMethod(); 
//                break;
//            case DOWN: 
//		rightMethod(); 
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                frontLeftMethod();   
//                break;
//            default:
//                break;
//            }}
//            
//        } 
//		else                     // 此时电脑鼠在迷宫的右下角
//		{
//			if(GmcMouse.cY+GmcMouse.cX>=15)
//		{switch (GucMouseDir) {           
//            case UP:
//		leftMethod();			                               // 左                    
//                break;
//            case RIGHT: 
//	        leftMethod();
//                break;
//            case DOWN: 
//		rightMethod(); 
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                frontLeftMethod();   
//                break;
//            default:
//                break;
//            }}
//			else
//		{switch (GucMouseDir) {
//            case UP:
//	        frontLeftMethod();                
//                break;
//            case RIGHT: 
//                leftMethod();
//                break;
//            case DOWN: 
//                rightMethod();
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                rightMethod();   
//                break;
//            default:
//                break;
//            }}                        
//        }
//    }
//	else 
//	{
//        if (GmcMouse.cY & 0x08) //此时电脑鼠在迷宫的左上角
//		{   
//			if(GmcMouse.cY+GmcMouse.cX>=15)
//		{switch (GucMouseDir) {
//            case UP:
//                leftMethod();                
//                break;
//            case RIGHT: 
//                rightMethod();
//                break;
//            case DOWN:
//		frontLeftMethod();  		
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                leftMethod();   
//                break;
//            default:
//                break;
//            }}
//			else
//		{switch (GucMouseDir) {
//            case UP:
//	        rightMethod();               
//                break;
//            case RIGHT: 
//                frontLeftMethod();
//                break;
//            case DOWN:
//                leftMethod();
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                rightMethod();   
//                break;
//            default:
//                break;
//            }}
//            
//        }
//		else                    //此时电脑鼠在迷宫的左下角
//		{
//			if(GmcMouse.cY>GmcMouse.cX)
//		{switch (GucMouseDir) {
//            case UP:
//	        rightMethod();               
//                break;
//            case RIGHT: 
//	        frontLeftMethod();
//                break;
//            case DOWN:
//                leftMethod();
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                rightMethod();   
//                break;
//            default:
//                break;
//            }}
//			else
//		{switch (GucMouseDir) {
//            case UP:
//	        frontRightMethod();               
//                break;
//            case RIGHT: 
//	  	leftMethod();
//                break;
//            case DOWN:
//                leftMethod();    
//                break;
//            case LEFT:                                                  /*  当前电脑鼠向左              */
//                rightMethod(); 
//                break;
//            default:
//                break;
//            }}
//        }
//    }
//}




/////////////////////////////////////////////////////////////////////////////////////
//////delect no effect statck node
/////////////////////////////////////////////////////////////////////////////////////
uint8 delnode(uint8 n)
{
    int i=0,j=0,k=0;
    uint8 flag[4]={0};
    uint8 count=n;
    for(j=0;j<count;)
    {
        flag[0]=0;
        flag[1]=0;
        flag[2]=0;
        flag[3]=0;
        for(i=0;i<loadcount;i++)
        {
           if(GmcCrossway[j].cX>=GucAboveLoad[i].cX && GmcCrossway[j].cY>=GucAboveLoad[i].cY)
             flag[0]=1;
           if(GmcCrossway[j].cX>=GucAboveLoad[i].cX && GmcCrossway[j].cY<GucAboveLoad[i].cY)
             flag[1]=1;
           if(GmcCrossway[j].cX<GucAboveLoad[i].cX && GmcCrossway[j].cY<GucAboveLoad[i].cY)
             flag[2]=1;
           if(GmcCrossway[j].cX<GucAboveLoad[i].cX && GmcCrossway[j].cY>=GucAboveLoad[i].cY)
             flag[3]=1;
        }
        if(flag[0]&&flag[1]&&flag[2]&&flag[3])
        {
           k=j;
           while(k<count)
           {
             GmcCrossway[k].cX = GmcCrossway[k+1].cX;
             GmcCrossway[k].cY = GmcCrossway[k+1].cY;
             k++;
           }
           count--;
           continue;
        }
        j++;
    }
    return count;
}
/*********************************************************************************************************
** Function name:       objecttest
** Descriptions:        测试是否必要返回函数
** input parameters:    目的坐标
** output parameters:   无
** Returned value:      无//有返回值
*********************************************************************************************************/
uint8 objecttest (int8  cX, int8  cY, uint8 count)
{
    uint8 n=0;                                        /*  GmcStack[]下标              */
    uint16 ucStep    = 1;                              /*  等高值                      */
    uint8 ucStat    = 0;                              /*  统计可前进的方向数          */
    int8  i=0,j=0;
    int8  cDirTemp;  
    uint8 cXcur=cX,cYcur=cY;
    uint8 MouseDirTemp  = GucMouseDir;
    uint8 cXtemp = GmcMouse.cX,cYtemp = GmcMouse.cY;
    
    uint8  counttemp=count;
    uint8  choice; 
    uint8  Dircur=0;                                  //方向
    uint16 ucsteptemp;				      //
    
    
    
    GmcStack[n].cX  = cXcur;                             /*  起点X值入栈                 */
    GmcStack[n].cY  = cYcur;                             /*  起点Y值入栈                 */
    Gucsave[n]=0;
    n++;
//////////////////////////////////////////////////////////////////////////////////////////////
    //制作等高图目的是为找出返回后的方向  改进：将objectto放置到该文件，可将其融合在其中
    //改进：不要方向 
    for (i = 0; i<16; i++)     //初始化等高值
        for (j = 0; j<16; j++)
            GucMapStep[i][j] = 0xffff;       

    while (n)                  //制作等高图
    {
        GucMapStep[cXcur][cYcur] = ucStep;                     
        ucStat = 0;
        if (GucMapBlock[cXcur][cYcur] & 0x08)            /*  左方有路                    */
	{    
	    if(Dircur==0x08||Dircur==0)
		ucsteptemp=ucStep+STRWEIGHT;
	    else
		ucsteptemp=ucStep+CURVEWEIGHT;
	    if(GucMapStep[cXcur - 1][cYcur] > (ucsteptemp))
	    {
		ucStat++;
		choice=0x08;
	    }
        }

        if (GucMapBlock[cXcur][cYcur] & 0x04)            /*  下方有路                    */
	{  
            if(Dircur==0x04||Dircur==0)
                    ucsteptemp=ucStep+STRWEIGHT;
            else
                    ucsteptemp=ucStep+CURVEWEIGHT;
            if(GucMapStep[cXcur][cYcur - 1] > (ucsteptemp))
            {
                    ucStat++;
                    choice=0x04;
            }
         }

	if (GucMapBlock[cXcur][cYcur] & 0x02)          /*  右方有路                    */
	{    
            if(Dircur==0x02||Dircur==0)
                    ucsteptemp=ucStep+STRWEIGHT;
            else
                    ucsteptemp=ucStep+CURVEWEIGHT;
            if(GucMapStep[cXcur + 1][cYcur] > (ucsteptemp))
            {
                    ucStat++;
                    choice=0x02;
            }
         }

	if (GucMapBlock[cXcur][cYcur] & 0x01)        /*  上方有路                    */      
	{     
            if(Dircur==0x01||Dircur==0)
                    ucsteptemp=ucStep+STRWEIGHT;
            else
                    ucsteptemp=ucStep+CURVEWEIGHT;
            if(GucMapStep[cXcur][cYcur + 1] > (ucsteptemp))
            {
                    ucStat++;  
                    choice=0x01;
            }
        }
       
        if (ucStat == 0) 
        {
            n--;
            cXcur = GmcStack[n].cX;
            cYcur = GmcStack[n].cY;
			Dircur = Gucsave[n];
            ucStep = GucMapStep[cXcur][cYcur];
        } 
        else 
        {
	    if (ucStat > 1) 
            {                                   
                GmcStack[n].cX = cXcur;            
                GmcStack[n].cY = cYcur; 
		Gucsave[n]=Dircur;
                n++;
            }
	    switch(choice)
	    {
		case 0x01:
			if(Dircur==0x01||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x01;
			cYcur++;                                    
			break;
		case 0x02:
			if(Dircur==0x02||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x02;
			cXcur++;                                      
			break;
		case 0x04:
			if(Dircur==0x04||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x04;
			cYcur--;                                     
			break;
		case 0x08:
			if(Dircur==0x08||Dircur==0)
			    ucStep+=STRWEIGHT;
			else
			    ucStep+=CURVEWEIGHT;
			Dircur=0x08;
			cXcur--;                                       
			break;
	    }
        }
    }
    //等高图制作完毕
    cXcur = GmcMouse.cX;
    cYcur = GmcMouse.cY;
    ucsteptemp=0xffff;
    ucStep = GucMapStep[cXcur][cYcur];
    if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
        (GucMapBlock[cXcur][cYcur+1]!=0) &&
        (GucMapStep[cXcur][cYcur + 1] < ucStep)) 
            ucsteptemp=GucMapStep[cXcur][cYcur + 1];

    if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
        (GucMapBlock[cXcur+1][cYcur]!=0) &&
        (GucMapStep[cXcur + 1][cYcur] < ucStep)) 
            if(GucMapStep[cXcur + 1][cYcur] < ucsteptemp)
                    ucsteptemp=GucMapStep[cXcur+1][cYcur];

    if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
        (GucMapBlock[cXcur][cYcur-1]!=0) &&
        (GucMapStep[cXcur][cYcur - 1] < ucStep)) 
            if(GucMapStep[cXcur][cYcur -1] < ucsteptemp)
                    ucsteptemp=GucMapStep[cXcur][cYcur -1];

    if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
        (GucMapBlock[cXcur-1][cYcur]!=0) &&
        (GucMapStep[cXcur - 1][cYcur] < ucStep))
            if(GucMapStep[cXcur -1][cYcur] < ucsteptemp)
                    ucsteptemp=GucMapStep[cXcur -1][cYcur];


    if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
        (GucMapBlock[cXcur][cYcur+1]!=0) &&
        (GucMapStep[cXcur][cYcur + 1]==ucsteptemp)) 
    {                      
        cDirTemp = UP;                                      
    }
    if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
        (GucMapBlock[cXcur+1][cYcur]!=0) &&
        (GucMapStep[cXcur + 1][cYcur]==ucsteptemp)) 
    {            
        cDirTemp = RIGHT;                                   
    }
    if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
        (GucMapBlock[cXcur][cYcur-1]!=0) &&
        (GucMapStep[cXcur][cYcur - 1]==ucsteptemp)) 
    {           
        cDirTemp = DOWN;                                   
    }
    if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
        (GucMapBlock[cXcur-1][cYcur]!=0) &&
        (GucMapStep[cXcur - 1][cYcur]==ucsteptemp))
    {               
        cDirTemp = LEFT;                                    
    }
    cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;              /*  计算方向偏移量              */
    GucMouseDir=(GucMouseDir+cDirTemp)%4; 
    while ((cXcur != cX) || (cYcur != cY)) 
    {
        ucsteptemp=0xffff;
        ucStep = GucMapStep[cXcur][cYcur];
	if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
            (GucMapBlock[cXcur][cYcur+1]!=0) &&
            (GucMapStep[cXcur][cYcur + 1] < ucStep)) 
		ucsteptemp=GucMapStep[cXcur][cYcur + 1];

        if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
            (GucMapBlock[cXcur+1][cYcur]!=0) &&
            (GucMapStep[cXcur + 1][cYcur] < ucStep)) 
		if(GucMapStep[cXcur + 1][cYcur] < ucsteptemp)
			ucsteptemp=GucMapStep[cXcur+1][cYcur];

	if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
            (GucMapBlock[cXcur][cYcur-1]!=0) &&
            (GucMapStep[cXcur][cYcur - 1] < ucStep)) 
		if(GucMapStep[cXcur][cYcur -1] < ucsteptemp)
			ucsteptemp=GucMapStep[cXcur][cYcur -1];

	if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
            (GucMapBlock[cXcur-1][cYcur]!=0) &&
            (GucMapStep[cXcur - 1][cYcur] < ucStep))
		if(GucMapStep[cXcur -1][cYcur] < ucsteptemp)
			ucsteptemp=GucMapStep[cXcur -1][cYcur];


        if ((GucMapBlock[cXcur][cYcur] & 0x01) &&               /*  上方有路                    */
            (GucMapBlock[cXcur][cYcur+1]!=0) &&
            (GucMapStep[cXcur][cYcur + 1]==ucsteptemp)) 
        {                      
            cDirTemp = UP;                                      
            if (cDirTemp == GucMouseDir) 
            {                                     
                cYcur++;
                continue;                                      
            }
        }
        if ((GucMapBlock[cXcur][cYcur] & 0x02) &&               /*  右方有路                    */
            (GucMapBlock[cXcur+1][cYcur]!=0) &&
            (GucMapStep[cXcur + 1][cYcur]==ucsteptemp)) 
	{            
            cDirTemp = RIGHT;                                   
            if (cDirTemp == GucMouseDir)
	    {                                              
                cXcur++;
                continue;                                       
            }
        }
        if ((GucMapBlock[cXcur][cYcur] & 0x04) &&               /*  下方有路                    */
            (GucMapBlock[cXcur][cYcur-1]!=0) &&
            (GucMapStep[cXcur][cYcur - 1]==ucsteptemp)) 
	{           
            cDirTemp = DOWN;                                   
            if (cDirTemp == GucMouseDir) 
	    {                                       
                cYcur--;
                continue;                                       
            }
        }
        if ((GucMapBlock[cXcur][cYcur] & 0x08) &&               /*  左方有路                    */
            (GucMapBlock[cXcur-1][cYcur]!=0) &&
            (GucMapStep[cXcur - 1][cYcur]==ucsteptemp))
	{               
            cDirTemp = LEFT;                                    
            if (cDirTemp == GucMouseDir) 
	    {                                          
                cXcur--;
                continue;                                       
            }
        }
       cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;              /*  计算方向偏移量              */
       GucMouseDir=(GucMouseDir+cDirTemp)%4; 
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
    GmcMouse.cX = cX;
    GmcMouse.cY = cY;
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路 */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) 
    {                   
        if(ShieldLoad(MOUSERIGHT))
        {
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY]&= (~(MOUSEWAY_R));
                --crosslenth;
           }
           counttemp=delnode(counttemp);
        }
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) 
    {                   
       if(ShieldLoad(MOUSEFRONT))
        {
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &=(~(MOUSEWAY_F));
                --crosslenth;
           }
           counttemp=delnode(counttemp);
        }                                                          
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) 
    {                     
       if(ShieldLoad(MOUSELEFT))
        {
           while(crosslenth)
           {
                GucMapBlock[Gmcshield[crosslenth-1].cX][Gmcshield[crosslenth-1].cY] &= (~(MOUSEWAY_L));
                --crosslenth;
           }
        }
       counttemp=delnode(counttemp);
    }
    GmcMouse.cX = cXtemp;
    GmcMouse.cY = cYtemp;
    GucMouseDir = MouseDirTemp;
    return counttemp;
}

//void stop(){
//                while (1) 
//                {
//                    if (keyCheck() == true) 
//                    {
//                        break;
//                    }
//                    sensorDebug();
//                    delay(20000);
//                }
//}
//

//void GOTO_SPURT(){
//  goto spurtcase;
//}

/*********************************************************************************************************
** Function name:       main
** Descriptions:        主函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/

 main (void)
{
  
    uint8 n          = 0;                                               /*  GmcCrossway[]下标           */
    uint8 ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
    uint8 ucTemp     = 0;                                               /*  用于START状态中坐标转换     */
    mouseInit();                                                        /*  底层驱动的初始化            */
    zlg7289Init();                                                      /*  显示模块初始化              */
//    uint8 maze_time_ctr=0;//记录mazesearch状态进入的次数
//    uint8 denggaozhi = 0xff;
//    uint8 denggaozhi_temp = 0xff;

    while (1) 
    {
        switch (GucMouseTask) 
        {                                           
            case WAIT:
                sensorDebug(); //循环测试红外线
                voltageDetect();
                //zlg7289Download(1, 5, 1, 0x08);
                delay(100000);
                if (keyCheck() == true) 
                {                              
                    zlg7289Reset();                                       
                    GucMouseTask = START;
                }
                break;
                
            case START:                                                     /*  判断电脑鼠起点的横坐标      */
                //zlg7289Download(0, 0, 0, 0x0C);
                //zlg7289Download(0, 1, 0, 0x0B);
                //zlg7289Download(0, 2, 0, 0x0D);
                //zlg7289Download(0, 3, 0, 0x0D);
                //zlg7289Download(0, 4, 1, 0x00);

                mazeSearch();                                               /*  向前搜索      */
//                backTurnback();
//                stop();
//                mouseTurnright();
//                mouseTurnleft();
//                stop();
//                mazeSearch();                                               /*  向前搜索                    */
//                mouseTurnright();
//                stop();
//                mazeSearch();
//                mouseTurnback();
//                stop();
//                backTurnleft();
//                mazeSearch();
//                mouseTurnright();
//                mazeSearch();                                               /*  向前搜索                    */
//                mouseTurnright();

//                 stop();
                if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) 
                {         
                    uStartFlag = 1;                      //标志，用在断开起点
                    GucXStart   = MAZETYPE - 1;                       
                    GmcMouse.cX = MAZETYPE - 1;                       
                    ucTemp = GmcMouse.cY;
                    GucXGoal=8;
                    GucYGoal=7;
//                    cor_changed=1;
                    
                    do 
                    {
                        GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                        GucMapBlock[0 ][ucTemp] = 0;
                    }while (ucTemp--);
                    GmcCrossway[n].cX = MAZETYPE - 1;
                    GmcCrossway[n].cY = 0;
                    n++;
                    GucMouseTask = MAZESEARCH;                           
                }
                if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) 
                {        
                    GmcCrossway[n].cX = 0;
                    GmcCrossway[n].cY = 0;
                    n++;
                    GucMouseTask = MAZESEARCH;                            
                }
                break;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////             
            case MAZESEARCH:
//              if(spurt_time!=0){
//                mapStepEdit_check(GucXGoal,GucYGoal);
//                denggaozhi_temp=GucMapStep[GucXStart][GucYStart];
//                zlg7289Reset();
//                 zlg7289Download(1, 4, 0, denggaozhi_temp / 10);
//                 zlg7289Download(1, 5, 0, denggaozhi_temp % 10);
//               
//              }
//              maze_time_ctr++;
//                zlg7289Download(1, 4, 0, maze_time_ctr / 10);
//                zlg7289Download(1, 5, 0, maze_time_ctr % 10);
              

//              if(spurt_time!=0) maze_time++;
//              
//                zlg7289Download(1, 6, 0, maze_time / 10);
//                zlg7289Download(1, 7, 0, maze_time % 10);
                
                //zlg7289Download(1, 7, 0, spurt_time );
                
                ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  统计可前进的支路数          */
                
    //           if( GmcMouse.cX == 4 && GmcMouse.cY == 0) //调试用的
    //           ucTemp++;
                if (ucRoadStat) 
                {                                                           /*  有可前进方向                */
                    crosswayChoice(); //crosswayChoice是法则选择，现在是中心法则    /*  用右手法则搜索选择前进方向  */
                    if(ushieldflag == 1)
                    {
                       ushieldflag = 0;
                       n=delnode(n);
                       break;
                    }
                    if (ucRoadStat > 1) 
                    {                                                       /*  有多条可前进方向，保存坐标  */
                        GmcCrossway[n].cX = GmcMouse.cX;
                        GmcCrossway[n].cY = GmcMouse.cY;
                        n++;
                    }
                    mazeSearch();                                           /*  前进一格                    */
                } 
                else 
                {                                                           /*  没有可前进方向，回到最近支路*/
                    while (--n) 
                    {
;                        
                       // if(Gucstep>=STEPNUM)
                       // {
                       //       if( GucMapBlock[8][8]!=0 || GucMapBlock[8][7]!=0 || GucMapBlock[7][8]!=0 || GucMapBlock[7][7]!=0 )
                       //       {
                       //              Gucstep = 0 ;
                       //              n=1;
                       //              continue;
                       //       }
                       //  }
;
                        ucRoadStat = crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);
                        if (ucRoadStat) 
                        {  
                            n=objecttest(GmcCrossway[n].cX,GmcCrossway[n].cY,n);
                            ucRoadStat = crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);
                            if(ucRoadStat==0)
                            {       
                               continue;
                            }         
                            objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY,__BACK);
		 	    GmcMouse.cX = GmcCrossway[n].cX;
		            GmcMouse.cY = GmcCrossway[n].cY;
;                                        
//                          crosswayChoice();            //此时路口在objecttest中计算，因此ushieldflag ！= 1 仅转弯！
//                          if (ucRoadStat > 1) 
//                          {
//                            n++;
//                          }
//                          mazeSearch();
//;
                            break;
                        }
                    }
                }
                 if (n == 0){                                                     //  走完了所有的支路，回到起点  
                   Gucstep = 0 ; 
                   objectGoTo(GucXStart,GucYStart,__START);//GmcCrossway[0].cX, GmcCrossway[0].cY ,__START
                   GmcMouse.cX = GucXStart;//GmcCrossway[0].cX;
                   GmcMouse.cY = GucYStart;//GmcCrossway[0].cY;
                   //delay(2000000);
                   mouseTurnback();
		   // delay(1000000);
                   GucMouseTask = SPURT;                               //  电脑鼠将开始冲刺状态        
                 }    
                
//                if((spurt_time==0) && 
 //                  ((GmcMouse.cX==GucXGoal) && (GmcMouse.cY==GucYGoal)) 
//                       ){
                   //      GucXGoal=GmcMouse.cX;
                   //      GucYGoal=GmcMouse.cY;
                         
//                         zlg7289Reset();
//                         zlg7289Download(0, 4, 1, GucXGoal);
//                         zlg7289Download(0, 5, 0, GucYGoal);
                //mapStepEdit_check(GucXGoal,GucYGoal);
                //denggaozhi=GucMapStep[GucXStart][GucYStart];
                //zlg7289Reset();
                //zlg7289Download(1, 0, 0, denggaozhi / 10);
                //zlg7289Download(1, 1, 0, denggaozhi % 10);
                         
 //                        objectGoTo(GucXStart,GucYStart,__START);                          /*  回到起点                    */
//                         GmcMouse.cX = GucXStart;
//                         GmcMouse.cY = GucYStart; 
                         //mouseTurnback();第一次检测到终点回起点，加mouseturnback会有小问题
//                         mouseTurnback();//不加入也出问题！！！
                         //backTurnback();//不能加入这个函数，会转身后再转撞！！！
 //                        GucMouseTask = SPURT;                               //  电脑鼠将开始冲刺状态  
 //                      }
//                if(n!=0 &&  maze_time==60){//(denggaozhi_temp>denggaozhi || maze_time==37)){
////                  temp_x=GmcMouse.cX;
////                  temp_y=GmcMouse.cY;
////                  objectGoTo(GucXStart,GucYStart,__START);                          /*  回到起点                    */
////                  GmcMouse.cX = GucXStart;
////                  GmcMouse.cY = GucYStart; 
////                  mouseTurnback();
////                  GucMouseTask = SPURT;                               //  电脑鼠将开始冲刺状态  
//                  
//                  if(!sanmianqiang(GmcMouse.cX,GmcMouse.cY)){
//                      temp_x=GmcMouse.cX;
//                      temp_y=GmcMouse.cY;
//                      objectGoTo(GucXStart,GucYStart,__START);                          /*  回到起点                    */
//                      GmcMouse.cX = GucXStart;
//                      GmcMouse.cY = GucYStart; 
//                      mouseTurnback();
//                      GucMouseTask = SPURT;                               //  电脑鼠将开始冲刺状态  
//                  }
//                  else maze_time-=8;
//
//                }
//spurtcase: 
          break;
  
        case SPURT:
 //       zlg7289Reset();
   //     zlg7289Download(0, 0, 0, 0x0E);
     //   zlg7289Download(0, 1, 0, 0x0E);
       // zlg7289Download(0, 2, 0, 0x0E);
      //  zlg7289Download(0, 3, 0, 0x0E);
 //       zlg7289Download(0, 4, 0, 0x0E);
   //     zlg7289Download(0, 5, 0, 0x0E);
     //   zlg7289Download(0, 6, 0, 0x0E);
       // zlg7289Download(0, 7, 0, 0x0E);
          
//          switch(spurt_time)
//          {
//            acc_change(2236068-spurt_time*200000);
//          case 0:
            //objectGoTo(GucXGoal,GucYGoal,__START);
            objectGoTo(GucXGoal,GucYGoal,__START);                                            /*  运行到指定目标点            */
            GmcMouse.cX = GucXGoal;
            GmcMouse.cY = GucYGoal;
            mouseTurnback();

//            GucMouseTask = MAZESEARCH;
//            maze_time=0;
//            spurt_time++;
//            break;
            
//          default:
//            objectGoTo(GucXGoal,GucYGoal,__START);//objectGoTo(GucXGoal,GucYGoal,__END);                                            /*  运行到指定目标点            */
//            GmcMouse.cX = GucXGoal;
//            GmcMouse.cY = GucYGoal;
//            mouseTurnback();
//            objectGoTo(temp_x,temp_y,__BACK);                          /*  回到起点                    */
//            GmcMouse.cX = temp_x;
//            GmcMouse.cY = temp_y;
//            GucMouseTask = MAZESEARCH;
////            maze_time=0;
//            spurt_time++;
//            break;               
//          }
              
//          objectGoTo(GucXGoal,GucYGoal,__END); //GucXGoal,GucYGoal,__END                           /*  以最优路径冲向终点          */
//          GmcMouse.cX = GucXGoal;
//          GmcMouse.cY = GucYGoal;
//          mouseTurnback();  

                  
//          delay(500000);
//                
        objectGoTo(GucXStart,GucYStart,__START);                          /*  回到起点                    */
          GmcMouse.cX = GucXStart;
         GmcMouse.cY = GucYStart;
         delay(1000000);
          mouseTurnback();                                            /*  向后转，恢复出发姿势        */
         GucMouseDir = (GucMouseDir + 2) % 4; 
         GucDirFlag = 1;
              while (1) 
                {
                    if (keyCheck() == true) 
                    {
                        break;
                    }
                    sensorDebug();
                    delay(20000);
                }
                break;
        
 
        default: 
          break;
        }
    }
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

