#ifndef __IMG_ANALYSE_H__
#define __IMG_ANALYSE_H__
#include "zf_common_headfile.h"

#define Poor_Sum(x,y)  (((x-y)<<7)/(x+y))   // 计算差 比 和  放大 127 倍
#define Black_Write   40       // 黑白赛道 分界线
#define Gray_Num  2

/* ===== [2026-10-20] reflection (glare) suppression ===== */
/* Floor reflections are saturated (>=200) and much wider than the 20 mm
   tape.  Every scan starts at the lane centre and walks outward, so the
   FIRST white run it meets on the floor IS the reflection.  A real edge
   line (tape) gives a white run of roughly 2 px in this 120x70 image. */
#define AGL_EN              1      // 0 = old behaviour, for A/B testing
#define AGL_NEAR_ROW        22     // rows >= this are near-field
/* Absolute check: below the lane centre the two edge lines must never
   cross.  This catches a row where a reflection grabbed the FIRST
   transition on both sides (blobs in the image centre). */
#define AGL_CROSS_GUARD     1
#define AGL_RUN_NEAR        8      // white run wider than this (near)  = glare, skip
#define AGL_RUN_FAR         5      // white run wider than this (far)   = glare, skip
#define AGL_EDGE_JUMP_NEAR  8      // max px a lane edge may jump per row (near)
#define AGL_EDGE_JUMP_FAR   5      // max px a lane edge may jump per row (far)
#define AGL_MIN_INTERIOR    3      // min white px inside a valid lane row (mid/near)
#define AGL_MAX_INTERIOR    60     // max white px inside a real lane (all-white = bad threshold)
extern uint8 agl_rejected_rows;   // diag: rows whose edges were rejected as glare
extern uint8 agl_blob_pair;       // diag: two glare blobs seen in the same row
void Glare_Suppress(uint8 *data);
uint8 LinkCnt_Valid_Rows(uint8 *data, uint8 i);
  //  灰度间隔

typedef enum
{
  NoRing,//不是环岛
  FindRing,//发现环岛
  AgainRing,//环岛确定
  GoRing,//进环岛
  TempRing,//进环和在环中间变量
  InRing,//在环岛中
  OutRing//出环岛
} RingFalg;

extern void Zebra_Crossing(uint8 *data);

uint8 Width(uint8 data);
uint8 Prefect_Road(uint8 *data);//完美赛道
void Find_Rhombus_Out(uint8 *data);
void  Find_Stop(uint8 *data);

void Find_Rhombus(uint8 *data);
void Find_Zebra(uint8 *data);
void Find_Decelerate(uint8 *data);
void Turn_Left(void);

void Turn_Right(void);

void Overtaking(void);
void Will_car(void);
void Decelerate(uint8 *data);
void  Tiao_Light(uint8 *data);

extern uint32 time_dl1b;
extern uint8 time_dl1b_on;
extern uint32 time_dl1b_s;
extern uint8 mode_2024,tiaoguang;
extern volatile uint8 start_armed;   // [2026-10-22] 发车布防（上电确认前方无障碍）
extern volatile uint8 start_trigger;   // [2026-10-22] 连续遮挡 2 秒才发车

extern uint8 Cycle_Flag,Lin_Flag,rhombus_Flag,Distance_Flag,rhombus_Flag1;
extern uint8 Stadiometry_Flag1,Stadiometry_Flag2,Stdiometry_Flag3;
extern uint8 Distance_Left_Flag,Distance_Right_Flag;
extern uint32 distance_0,distance_1,distance_2;                //转换出来的距离  单位为厘米
extern uint8 Zuo_Flag,Zhuan_Flag,Left_Flag1,Right_Flag1,Stop1,Right_Flag2;
extern uint8 white_count,n,k,j,a,b,c,i,a1,b1;
extern uint8 Cone_bucket;
extern uint8 Right_Max,Left_Min,Right_Flag,Left_Flag,Right_Flag1;
extern uint8 zuobianjie_Flag,youbianjie_Flag;
extern uint8 White_Flag;
extern uint8 result_screen;
extern uint8 Flag_stop,Light_Flag,Light_Flag1,Num_Turn_R;
extern int32 Time_Cal,Change_Light;
extern uint8 aaa,Rhombus_Out,Stop_Out;
extern uint8 stop,Stop_1;
extern uint8  Start_Line_Flag;
extern uint8 Rhombus_Flag;
extern uint8 Rhombus_Out_Flag;
extern uint8 Flag_turn;
extern uint8 Deal_Flag,Time_banmaxian,Time_banmaxian_Flag,Dis_Flag;
extern uint16 Distances;
extern uint8 Guaidian,NUM;
// 用于时间计数
extern uint8   Time_ON;
extern uint32 Time_Count;


//外部变量声明
extern int8 Out_Side;
extern int8 Line_Count;
extern uint8 Result_Far_Row;
/* [2026-10-22] 2026 标线识别 */
extern volatile uint8 mk_cw_seen, mk_cw_near, mk_cw_score;
extern volatile uint8 mk_dia_seen, mk_dia_near, mk_dia_score;
extern volatile uint8 mk_state;
void Marker_Scan(uint8 *data);
extern int16 Left_Max, Right_Min;
extern uint16 Left_Line[61];
extern uint16 Right_Line[61];
extern uint16 Mid_Line[61];
extern uint16 Left_Add_Line[61];
extern uint16 Right_Add_Line[61];
extern uint16 Width_Real[61];
extern uint8 Left_Add_Flag[61], Right_Add_Flag[61];
extern int16 Width_Add[61];

extern uint8 Left_Add_Start, Right_Add_Start;   // 左右补线起始行坐标
extern uint8 Left_Add_Stop, Right_Add_Stop; // 左右补线结束行坐标
extern uint8 Left_Line_Start, Right_Line_Start;    //  左右 实线 起始点 坐标
extern uint8 Left_Line_Break, Right_Line_Break;   //  左右实线 断点坐标
extern uint8 Left_Linear_Flag, Right_Linear_Flag;


extern uint8 Car_Dirc;        // 车库方向
extern int16 Width_Min;

extern uint8 Flag1;          //0直道  1 環島底部  2 環島第二入口  3 環島中部
extern uint8 Dirc;             //1左  2右
extern uint8 turnleft;
extern int Count;

extern uint8 LDrop_Line_Num , RDrop_Line_Num;


extern uint32 g_fDirectionSum_Max,g_fDirectionSum_Min;
extern uint32 last_g_fDirectionSum;
extern uint8 g_fcount;   // 计数值用于 可能的结果判断

void Line_Repair(uint8 Start, uint8 Stop, uint8 *data, uint16 *Line, uint16 *Line_Add, uint8 *Add_Flag, uint8 Mode);
extern void SupplementaryLine(uint16 *Line1,uint16 *Line, uint8 start, uint8 end);
void Stop_stop(uint8 *data);
extern uint8 Straight_Line(uint16 *Line, uint8 top, uint8 botton, uint8 mode);

void In_Ring_Add_Line(uint8 *data, uint8 mode);   // 进环 补线
void Out_Ring_Add_Line(uint8 *data, uint8 mode); //  出环 补线
uint8 Find_J_Black(uint8 *data, uint8 j,uint8 start,uint8 end);     //2020.7.15

uint8 Start_Line_Judg(uint8 *data);  // 斑马线判别
uint8 intial_Start_Line_Judg(uint8 *data);
void Garage_Add_Line(uint8 *data, uint8 mode);
uint8 Find_Cone_bucket(uint8 *data);//辅助判别锥桶

void Point_Repair_Line(uint8 Start, uint8 Stop,uint16 position_start, uint16 position_stop ,uint16 *Line_Add);
//  俩点 补线


uint8 Direct_path_judgment(uint8 *data);
uint8 Ring_judgment(uint16 *Line, uint8 Dirc);  // 2020.7.9

extern uint32 Test_Center(uint8 Num);
void Image_Para_Init(void);
void Image_Handle(uint8 *data);

float Slope_Weight(uint8 *Mid);
char Calculate_Angle(uint8 Point_1, uint8 Point_2, uint8 Point_3);
void Curve_Fitting(float *Ka, float *Kb, uint8 *Start, uint16 *Line, uint8 *Add_Flag, uint8 Mode);
void Curve2_Fitting(float *Ka, float *Kb, uint8 Start,uint8 End, uint16 *Line, uint8 Mode);
uint8 Calculate_Add(uint8 i, float Ka, float Kb);

uint8 First_Line_Handle(uint8 *data);
uint8 Traversal_Left(uint8 i, uint8 *data, uint8 *Mid, uint8 Left_Min, uint8 Right_Max);
uint8 Traversal_Right(uint8 i, uint8 *data, uint8 *Mid, uint8 Left_Min, uint8 Right_Max);
uint8 Traversal_Left_Line(uint8 i, uint8 *data, uint16 *Left_Line, uint16 *Right_Line);
uint8 Traversal_Right_Line(uint8 i, uint8 *data, uint16 *Left_Line, uint16 *Right_Line);
void Traversal_Mid_Line(uint8 i, uint8 *data, uint8 Mid, uint8 Left_Min, uint8 Right_Max, uint16 *Left_Line, uint16 *Right_Line, uint16 *Left_Add_Line, uint16 *Right_Add_Line);

int32 abs_int(int32 i);
uint16 Dynamic_Point_Weight(int8 begin,int8 end);

int32 range_protect(int32 duty, int32 min, int32 max);




uint8 Ramp_Judg(uint8 *data);
void Find_rhombus(uint8 *data);//判断拐点
void Find_rhombus_Out(uint8 *data);
float Right_Add_Line_Weight(int8 begin,int8 end);
float Left_Add_Line_Weight(int8 begin,int8 end);
float Right_Line_curvity(uint8 i,uint8 gap);
float Left_Line_curvity(uint8 i,uint8 gap);
float Mid_Line_curvity(uint8 i,uint8 gap);
extern float Mid_qulv;
extern float Right_qulv;
extern float Left_qulv;
uint8 Find_In_Trigeminal_Jump(uint8 *data);
uint8 Find_Out_Trigeminal_Jump(uint8 *data);
void In_Fork_Add_Line(uint8 *data, uint8 mode);
void Out_Fork_Add_Line(uint8 *data, uint8 mode);
void Ramp(uint8 *data);
extern uint8 Ramp_Flag ;
extern uint8 Ramp_Flag3;
extern uint8 Ring_Flag;
extern uint8 Ring_Enter1;
extern uint8 Ring_Enter;
extern uint8 LDrop_Line_Num1, RDrop_Line_Num1;
uint8 Ring_Mid_judgment(uint16 *Line, uint8 mode);
uint8 Fork_Judg(uint8 *data,uint8 mode);
extern uint8 Fork_judg1;
uint8 Ring_Judg(uint8 *data);
extern void New_boundary(uint8 *data, float *Ka, float *Kb);
void Decelerate_2(uint8 *data);
//记忆算法变量
extern uint8 Ring_Count;
extern uint8 Fork_Flag;
extern uint8 Fork_Count;
extern uint8 Fork_res;
extern  uint8 Ramp_Count;
void Buzzer();
uint8 zuobianjie(uint8 *data,uint8 Left_Min, uint8 Right_Max);
uint8 youbianjie(uint8 *data,uint8 Left_Min, uint8 Right_Max);
void Traversal_Mid_Line_Ling(uint8 i, uint8 *data, uint8 Mid, uint8 Left_Min, uint8 Right_Max, uint16 *Left_Line, uint16 *Right_Line, uint16 *Left_Add_Line, uint16 *Right_Add_Line);

/* [2026-10-20] edge lines captured before Glare_Suppress (for page1 overlay) */
extern uint16 agl_L_before[61];
extern uint16 agl_R_before[61];
extern uint16 agl_M_before[61];

#endif
