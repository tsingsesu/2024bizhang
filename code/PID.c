/*
 * @file       PID.c
 */
#include "zf_common_headfile.h"

PID Point_PID, AD_PID;  //定义舵机的PID参数结构体
MotorPid MotorPID, RMPID, LMPID;  // 普通增量式PID
Motor_Add_Pid ;  // 专家增量式PID
int16 Rspeed,Lspeed;  // 滤波后的速度
int16 Rrealspeed; // 真实的 左右轮速度
int16 Lrealspeed;
int16 Rcount=0;
int16 Lcount=0;
int32  LMotorSpeed;
int32  RMotorSpeed;
int32  MotorSpeed;
int32 iError;
uint8 L_Mode,R_Mode;

/*
 *  @brief      舵机PD
 */
//位置式PID舵机控制
int32 PlacePID_Control(PID *sprt, int32 NowPiont, int32 SetPoint)
{
      int32  Actual;    //最后得出的实际输出值
      iError = SetPoint - NowPiont; //计算当前误差
         if (iError > 0)   sprt->KP = (20.0*iError)/100;    // var1 =  270
         else  sprt->KP =(-20.0*iError)/100;
        sprt->KD = 160/100.0*sprt->KP;      //   230
     Actual = sprt->KP*iError + sprt->KD * (iError - sprt->LastError);
     sprt->LastError = iError;      //更新上次误差
     return Steer_Mid - Actual;   // [2026-10-22] 用可校准的中位
}





// 普通增量式电机PID
int32 RMotorPid_Col(int32 SpeedSet)
{
    RMPID.e2 = RMPID.e1;       //变量更迭
    RMPID.e1 = RMPID.e0;
    RMPID.e0 = SpeedSet - Rrealspeed;

    RMPID.Kp = 40;   //  ★降增益：原 127 是按目标 105~210 调的，目标 30 时太猛会来回振荡
    RMPID.Ki = 6;    //  ★原 23
    RMPID.Kd = 1;    //  ★原 5，速度反馈量化噪声 ±1，D 太大会放大噪声

    RMPID.Pe = RMPID.e0 - RMPID.e1;
    RMPID.Ie = RMPID.e0;
    RMPID.De = RMPID.e0 - 2.0 * RMPID.e1 + RMPID.e2;


    RMPID.DefaultSum = (int)(RMPID.Kp * RMPID.Pe + RMPID.Ki * RMPID.Ie + RMPID.Kd*RMPID.De);
    RMPID.Sum += RMPID.DefaultSum;


    if(RMPID.Sum > MotorMax)      //限幅操作
      RMPID.Sum = MotorMax;
    if(RMPID.Sum < MotorMin)
      RMPID.Sum = MotorMin;

    return RMPID.Sum;
}

//uint8 LBangBang_Flag = 0;
int32 LMotorPid_Col(int32 SpeedSet)
{

    LMPID.e2 = LMPID.e1;       //变量更迭
    LMPID.e1 = LMPID.e0;
    LMPID.e0 = SpeedSet - Lrealspeed;

    LMPID.Kp = 40;   //  ★降增益，原因同右轮
    LMPID.Ki = 6;    //  ★原 25
    LMPID.Kd = 1;    //  ★原 5

    LMPID.Pe = LMPID.e0 - LMPID.e1;
    LMPID.Ie = LMPID.e0;
    LMPID.De = LMPID.e0 - 2.0 * LMPID.e1 + LMPID.e2;


    LMPID.DefaultSum = (int)(LMPID.Kp * LMPID.Pe + LMPID.Ki * LMPID.Ie + LMPID.Kd*LMPID.De);
    LMPID.Sum += LMPID.DefaultSum;

    if(LMPID.Sum > MotorMax)      //限幅操作
      LMPID.Sum = MotorMax;
    if(LMPID.Sum < MotorMin)
      LMPID.Sum = MotorMin;

    return LMPID.Sum;
}

//int32 Motor_PID_Control(int32 SpeedSet)
//{
//    MotorPID.e2 = MotorPID.e1;       //变量更迭
//    MotorPID.e1 = MotorPID.e0;
//    MotorPID.e0 = SpeedSet - Realspeed;
//
//    MotorPID.Kp = 80*1.0 ;  //  127 23 5
//    MotorPID.Ki = 5*1.0 ;    // 300     80 40 25
//    MotorPID.Kd = 10*1.0;
//
//    MotorPID.Pe = MotorPID.e0 - MotorPID.e1;
//    MotorPID.Ie = MotorPID.e0;
//    MotorPID.De = MotorPID.e0 - 2.0 * MotorPID.e1 + MotorPID.e2;
//
//
//    MotorPID.DefaultSum = (int)(MotorPID.Kp * MotorPID.Pe + MotorPID.Ki * MotorPID.Ie + MotorPID.Kd*MotorPID.De);
//    MotorPID.Sum += MotorPID.DefaultSum;
//
//
//    if(MotorPID.Sum > MotorMax)      //限幅操作
//      MotorPID.Sum = MotorMax;
//    if(MotorPID.Sum < MotorMin)
//      MotorPID.Sum = MotorMin;
//
//    return MotorPID.Sum;
//}




















