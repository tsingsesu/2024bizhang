#ifndef __PID_H__
#define __PID_H__
#include "zf_common_headfile.h"
typedef struct PID
{
    int32 LastError;	//Error[-1]
    float KP;                              //比例常数 Proportional Const
    float KI;                                //积分常数 Integral Const
    float KD;                              //微分常数 Derivative Const
} PID;

typedef struct
{
    float Kp;             //P系数
    float Ki;             //I系数
    float Kd;             //D系数
    int32 Pe;            // p 的误差
    int32 Ie;               // I 的误差
    int32 De;              // D 的误差
    int32 e0;             //当前误差
    int32 e1;             //前次误差
    int32 e2;             //前前次误差
    int32 Sum;            //最终值
    int32 DefaultSum;     //增量值
    int32 Speed;

}MotorPid;

/*
    专家增量式 PID 结构体
*/
typedef struct
{
    float Kp;             //P系数
    float Ki;             //I系数
    float Kd;             //D系数
    int32 e0;             //当前误差
    int32 e1;             //前次误差
    int32 e2;             //前前次误差
    int32 de0;           //  本次误差 减 上次误差
    int32 de1;           //  上次误差 减 上上次误差
    int32 Sum;            //最终值
    int32 DefaultSum;     //增量值
    int32 Speed;

}Motor_Add_Pid;

extern PID Point_PID,AD_PID;	//定义舵机PID参数结构体
extern Motor_Add_Pid LAMPID,RAMPID; 
extern MotorPid MotorPID, RMPID, LMPID;  // 普通增量式PID

extern uint8 L_Mode,R_Mode;
extern int16 Rrealspeed,Lrealspeed; // 真实的 左右轮速度
extern int16 Rcount;
extern int16 Lcount;
extern int32  RMotorSpeed;
extern int32  LMotorSpeed;
extern int32  MotorSpeed;
extern int32 iError;
extern int32 MotorPid_Col(int32 SpeedSet);
extern int32 PlacePID_Control(PID *sprt, int32 NowPiont, int32 SetPoint);
extern int32 ADCPID_Control(PID *sprt, int32 Error);
extern int32 Motor_PID_Control(int32 SpeedSet);
void RAPID_Init();
void LAPID_Init();
int32  LMotorAddPid(int32 SpeedSet);
int32  RMotorAddPid(int32 SpeedSet);











#endif
