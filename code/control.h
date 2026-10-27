#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"

extern int32 RSpeedPWM,LSpeedPWM;

extern uint16 Pwm_Max,Pwm_Min;
extern uint8 steer_mode;
extern int32 steer_raw_target;
extern int16 lane_k;
extern volatile uint32 vision_processed_frames;
extern volatile uint32 tof_sample_tick;
extern volatile uint8 tof_has_sample;
extern uint16 vision_age_ticks, track_bad_ticks;
extern uint8 steer_look_row;
extern uint16 steer_lane_left, steer_lane_right;
extern uint8 drive_inhibit, mk_finish_reason;
extern uint16 mk_stop_elapsed_ms, mk_clear_remaining_mm;
extern double run_distance_cm;
uint16 Tof_Get_Distance(void);
uint32 Tof_Get_Age_Ms(void);
extern int16 Steer_Trim;   // [2026-10-22] 转向微调，正值往左
extern int16 Steer_Mid;    // [2026-10-22] 运行时舵机中位（掉电保存）
void Steer_Set_Mid(int16 m);   // [2026-10-22] 设中位并同步左右限幅
extern double Run_Odo;   // [2026-10-22] 发车后绝对里程(cm)
extern volatile uint8 steer_settle;   // [2026-10-22] 起跑后中位保护期



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
