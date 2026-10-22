#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"

extern int32 RSpeedPWM,LSpeedPWM;

extern uint16 Pwm_Max,Pwm_Min;
extern int16 Steer_Trim;   // [2026-10-22] 转向微调，正值往左
extern double Run_Odo;   // [2026-10-22] 发车后绝对里程(cm)



extern uint8    Point_Mid;
extern uint8    Point_Left;
extern uint8    Point_Right;
extern int32    SD5_Duty;      //       //4320  4740 5180
extern uint8 Line_Point;
extern uint8    Point;	//目标点横坐标
extern char    Point_Error;
extern float Mid_Ka, Mid_Kb;   // 拟合的赛道中线的斜率
extern uint8 Fictitious_Midline[61];  // 虚拟赛道中线
extern int32 AD_Error;


extern uint8  Distance_ON;//默认关闭距离记录
extern double Distance;    //距离保存变量，勿动
extern uint8  Distance_Test_ON;  
extern double Distance_Test;   



extern char  Track_Line_Num;
extern uint8 Set_Speed;
extern uint8 Stop;
extern uint8 StartStopLine;
extern void S3010_Control(void);
extern void Motor_Control(void);
extern void Point_Control(void);
extern void Least_Sqaure_Method(float *Ka, float *Kb,uint8 start, uint8 end);
extern int16 Fictitious_Beeline(float Ka, float Kb);
extern void Track_Line(void);
extern void Adjust_Speed();
extern void Control_Speed();
extern int16   speedshow[4];
extern uint8  showpoint[2] ;
extern uint16   Speed_max,Speed_min;
void Light(void);

extern int32 RSpeedPWM,LSpeedPWM;



#endif
