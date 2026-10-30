#include "zf_common_headfile.h"
#include "System.h"

/* ★舵机保护（新增）：视野里没有可用赛道时，别让 PID 把舵机顶到极限位置（会一直堵转发热）。
   判据两条，满足任一条就回中位：
     (1) Point == 0                  —— 图像链根本没产出结果（例如摄像头没出帧）
     (2) Line_Count >= 阈值          —— 基本没识别到赛道行
   Line_Count 在 Img_analyse.c 里维护（识别到的赛道行数，越大表示识别到的越少），
   等摄像头通了以后，在表达式窗口里看一眼 Line_Count 的正常范围，再回来调这个阈值。*/
#define STEER_NO_LINE_LINECOUNT     (58)
/* [2026-10-20] minimum rows needed before the fitted mid-line is trusted */
#define STEER_MIN_VALID_ROWS        (40)
/* ★宽度归一化（2026-10-24）
   Traversal_Mid_Line 是从画面中心向左右各找"第一处白色跳变"：
     虚线这一行有白色   → 扫到的是一条道              ≈ 43px
     虚线这一行是断口   → 整条虚线被跳过 → 整条路     ≈ 90~116px
   同一个 lane_k 套在两种宽度上算的根本不是同一个位置，车就会在两道之间来回穿梭。 */
#define STEER_ONE_LANE_MAX   (70)    /* [2026-10-26] 85->70 已回退：改成 85 后 lane_w_ema 学到 76，整条路阈值变 121，lane_k 选道永远不触发，绕行不换道 */
#define STEER_LANE_W_DEF     (43)    /* [2026-10-26] 75->43 已回退。实测一条道确实约 76px，但改成 75 会让 lane_k 失效；真要改对得连选道逻辑一起重写 */
/* ★模式迟滞（2026-10-24）：实测 W 是连续分布（58~116），70 这条分界正好切在
   中间，抖动一下就在两种算法之间切换，目标点单次最大跳 92px。
   所以 >85 才进"整条路"，<60 才回"单道"，中间保持上一次的状态。 */
#define STEER_WIDE_ON        (85)    /* 超过它 → 认为扫到了整条路 */
#define STEER_STRAIGHT_RANGE   (45)   /* [2026-10-28] 10->45: 10 只够直道调试, 弯道转不够直接冲出跑道 */
/* [2026-10-28] 过横道时的方向补偿（占空比单位）：正 = 往左（duty 变大），负 = 往右。
   实测过横道时中位 933 走 1.2 米右偏 12~15cm（约 7°），先补 16。
   注意这个宏必须定义在 S3010_Control 之前，否则 E272 undeclared identifier。 */
#define MK_CROSS_TRIM         (0)   /* [2026-10-31] 16->10->0：现场日志过横道时 L=43/R=117（车道中心在 x=80，画面中心 60），说明车已经偏左，固定左偏只会更偏 */
/* [2026-10-31] 过横道用的"远处那条白线"。行号越小越远（0 是画面最上方），默认 14 行，
   现场看那条白线更远就改小（10/8/5），更近就改大（18）。
   这两条边线要同时落在画面内、宽度在 [MIN, MAX] 才采信，否则退回固定中位。 */
#define MK_XING_ROW           (14)
#define MK_XING_W_MIN         (35)    /* 正常一条车道约 76px，40~110 都可信 */
#define MK_XING_W_MAX         (110)
#define MK_XING_GAIN          (20)    /* 偏差 1px 给 2 个 duty（20/10），越大打舵越狠 */
#define MK_XING_DUTY          (30)    /* 纠偏最多补多少 duty（满舵 ±70）*/

/* [2026-10-31] ToF 转向：过横道/终点段，ToF 测到前方够近就把目标点偏一点。
   EN 默认 0 —— 现在 ToF 照的是地面（有效读数只有 26~154mm），先装好、
   确认日志里 TOF 行的 max 能稳定到 1000 以上，再把 EN 改成 1。 */
#define MK_TOF_TURN_EN        (0)     /* 0 = 关，1 = 开 */
#define MK_TOF_MIN_MM         (200)   /* 近于这个当噪音/地面，忽略 */
#define MK_TOF_TURN_MM        (600)   /* 前方近于这个距离就补转向 */
#define MK_TOF_FRESH_MS       (200)   /* 读数超过这么久没更新就作废 */
#define MK_TOF_TURN_PX        (12)    /* 目标点最大偏移(px)：>0 往右打，<0 往左打 */
#define MK_TOF_PX_GAIN        (6)     /* 每近 100mm 加多少 px（600->500 加 6px）*/
#define STEER_TARGET_STEP      (2)    /* [2026-10-28] 1->2: 每 5ms 2px, 目标点 60->105 约 110ms */
#define STEER_START_TICKS     (200)   /* 1 second, timed by the steering ISR. */
#define CONE_CONFIRM_FRAMES   (3)
#define CONE_ARM_CM           (100)  /* First course cones are at least 2 m from start. */
#define CONE_MAX_CM           (780)  /* [2026-10-28] 绕障只在前 6 米内触发。实测两组锥桶都在 1.9~4.2 米；10 米那次 CN=9/cone_near=55/GP=4/87/22 是斑马线白条，被当成锥桶组，会让车在横道前先往右甩。0 = 关闭。 */
#define VISION_STALE_TICKS    (40)    /* 200 ms without a processed frame. */
#define TRACK_LOSS_TICKS       (400)  /* [2026-10-28] 40->400: 200ms 就停车, 分道线断口一路停; 改 2 秒 */
#define TOF_FRESH_TICKS       (40)
#define STEER_WIDE_OFF       (60)    /* 低于它 → 认为只是一条道     */

extern volatile uint8 mk_state;   // 标线状态：0空闲 1减速 2停车
extern volatile uint8 cone_ok, cone_cnt;
extern volatile int16 cone_x;
extern volatile uint8 cone_near;   /* [2026-10-26] 最近一个锥桶 blob 所在的行 */

/* ★选道 lane_k：只在"扫到整条路"（虚线断口）那一帧起作用
   <=50 = 左道中心    >50 = 右道中心 */
int16 lane_k = 75;   // ★选道：<=50 = 左道中心，>50 = 右道中心

/* ★电机测试模式（新增）—— 接电机驱动之前一定先看一眼这里
   0 = 正常：PID 算出来什么就直接给电机（默认，行为不变）
   1 = 输出锁死：电机 PWM 恒 0，但编码器照读、PID 照算
       -> 用来验证编码器、转向符号、左右对应，轮子一动不动
   2 = 开环固定 PWM：不走 PID，直接输出 MOTOR_TEST_PWM
       -> 第一次接驱动时，用小 PWM 慢慢试正反转
   注意：编码器没反馈时 PID 会一路顶满（实测 RSpeedPWM/LSpeedPWM 已到 8000），
        所以接驱动之前务必先设成 1 或 2。 */
#define MOTOR_TEST_MODE             (0)          // ★正式模式（发车逻辑 + 悄转减速）
#define MOTOR_TEST_PWM              (-2000)      // 模式2用的固定PWM = 20%，用于判断右轮是不是PWM不够

/* ★★电机硬限幅（新增）★★ —— 不管 PID 算出多少、不管模式是几，
   最终送到驱动板的 PWM 绝对值永远不会超过这个数。
   这是最后一道保险：就算闭环失控、符号反了，车速也被锁死在这里。
   10000 = 100%。 1200 = 12%。 想更慢就调小。 */
#define MOTOR_PWM_MAX               (3000)

/* 模式 3 = 闭环测试（新增）：正常跑 PID，但目标速度固定，不受 Flag_stop / 发车逻辑影响。
   这样不用先让车"发车"，就能验证 PID 是否稳定、会不会失控。 */
#define MOTOR_TEST_TARGET_SPEED     (30)        // 模式3 的固定目标速度
// ★软启动：目标速度每 MOTOR_RAMP_TICKS 个 5ms 增加 1，慢慢爬上去
//   40 x 5ms = 200ms 加一点；目标 30 大约 6 秒爬完
//   想更慢就把 MOTOR_RAMP_TICKS 调大（80 = 12 秒）
#define MOTOR_RAMP_TICKS            (40)

/* ★速度总缩放（新增）：100 = 原速，50 = 一半，30 = 三成
   在 Control_Speed() 末尾统一乘一遍，这样所有档位一起变慢，不用一个个改。
   注意：这个只改「目标速度」；只要编码器反馈还是 0（轮子不转），PID 照样会顶到 8000。
        想让电机真的不动，用上面的 MOTOR_TEST_MODE = 1。 */
#define SPEED_PERCENT               (14)   /* [2026-10-28] 26->20：0.5m/s 时循迹（L=1 单边补线）抖得厉害，降到约 0.42m/s，仍高于龟行车 0.3m/s */
                                                 //   跑熟了再往上加：25 -> 52，50 -> 105
       /****************************    舵机控制变量    ****************************/
uint8    Point_Mid = MT9V03X_W/2;
/* [2026-10-20] 3-frame median on Point; reflection blobs make single-frame
   jumps.  A real lane change is gradual, so this costs one frame of lag. */
/* [2026-10-20] 1 = ignore the image and steer to the mathematical centre.
   Use it to split "perception is biased" from "the car itself drifts". */
#define STEER_FORCE_CENTER    0
/* [2026-10-20] 1 = pure lane following: ignore Flag_turn / mode_2024 and
   always steer from the fitted mid-line.  mode_2024 starts at 0 and its
   "competition" branch forces Flag_turn=3 (straight) for the whole first
   segment, which is why the car ignored the image. */
#define STEER_LANE_FOLLOW     1
#define AGL_POINT_MEDIAN_EN   0   // [2026-10-20] off until the glare pass is validated:
                                   // a steady lane offset would look like a "jump"
                                   // and pin the steering to the centre.
static uint8 agl_point_hist[3] = {60, 60, 60};
static uint8 agl_point_i = 0;

static uint8 agl_point_median(uint8 v)
{
    uint8 a, b, c, t;
    agl_point_hist[agl_point_i] = v;
    agl_point_i = (uint8)((agl_point_i + 1) % 3);
    a = agl_point_hist[0]; b = agl_point_hist[1]; c = agl_point_hist[2];
    if(a > b) { t = a; a = b; b = t; }
    if(b > c) { t = b; b = c; c = t; }
    if(a > b) { t = a; a = b; b = t; }
    return b;                                  /* median */
}
uint8    Point_Left = 43;//43
uint8    Point_Right = 37;//37
int32    SD5_Duty =  SteerMiddle;
/* [2026-10-22] 舵机中位改成运行时变量：上电默认 SteerMiddle（代码里的标定值），
   掉电保存在 flash 第 17 页；page3 上可以用 S3/S4 现场微调，不用重新编译。
   注意：拉杆/舵机臂重新装过之后，857 就不再是中位了，必须重新校准。 */
/* ★舵机行程（2026-10-24）：中位往两边各能打多少。
   70  = 原来实测的值，约 ±15 度，绕锥桶时不够用。
   先试 100，不行再 120。加大后必须空转测试舵机不堵转。 */
#define STEER_DUTY_RANGE            (70)

/* ★锥形标绕行参数（2026-10-24）：看到锥桶就变道并锁存，不再打一下就回正 */
#define CONE_LANE_CM        (107)  /* [2026-10-26] 变道要走够的里程(cm)，改成按里程算。
                                      原来固定 1.0 秒：实测车速约 140cm/秒，全速时正好走 140cm；
                                      遇菱形减速到 50% 后 1 秒只走 70cm，横移量砍半 -> 撞锥桶。
                                      现在按里程，速度怎么变横移量都一样。调大=转得更狠，140~180 之间试。 */
#define CONE_RETURN_TAPER_FROM (85)  /* [2026-10-28] 120->90: 第二次绕障冲过右边界（V38 实测绕完 R 从 110 退到 48），提前收舵、少转一点。 */
#define CONE_HOLD_MAXTICKS  (800)  /* 兜底 4 秒：打滑/编码器不动时强制退出，别一直满舵 */
#define CONE_CLEAR_TICKS    (200)  /* 5ms × 200 = 1 秒看不见锥桶，才算这一组过完 */
#define CONE_COOL_TICKS     (300)  /* 5ms × 300 = 1.5 秒冷却；第二组锥桶间隔 2~4 米，够触发 */
#define CONE_LANE_HALF      (18)   /* [2026-10-26] 22->14 太小清不过锥桶，14->18 取中间。
                                      目标点 42、duty 约 980。
                                      22=满舵987(太猛出跑道) / 14=972(太小会撞) */
/* [2026-10-26] 锥桶偏离本道中心超过这么多像素，就"往另一边"走；
   基本在正前方时分不出来，按交错顺序走。 */
/* [2026-10-26] 锥桶必须近到这个行号才允许触发绕行。
   实测原来 O=11cm（车刚起步）就触发了：那时障碍还在画面最上面（cone_near≈28），
   车提前 3~4 米就做完 140cm 的变道、又回到中线，真正到锥桶时已经回正 -> 撞。 */
#define CONE_TRIG_ROW       (36)
/* [2026-10-26] 这条赛道总共只有两组障碍（左右各一处）。
   第 3 次以后的触发全是误报（实测终点附近还在绕），所以直接限次数。
   第一次往左、第二次往右，做完就再也不触发，lane_k 停在右道。 */
/* [2026-10-26] 终点停车：跑到 end_stop_cm 厘米就断电机。0 = 关闭。
   为什么不用标线判据：实测 Find_Stop 那个窗口（第 10~60 列）正好夹在两条
   车道线之间，平时本来就是黑的。「全黑」和「全白」在赛道中间都大量出现
   （全黑最长连续 45 帧、全白 A=1836 也出现在 O=4100 附近），判不出来。
   设置方法：跑完整一圈，看日志最后一行的 O=（cm），填到 end_stop_cm。 */
uint16 end_stop_cm = 0;          /* ★终点里程(cm)，0 = 不停车 */
uint8  end_stopped = 0;          /* 已经停了（发车前清零）*/

#define CONE_MAX_GROUPS     (2)
#define CONE_OFF_PX         (12)

/* 交错方向：这条赛道两排锥桶左右交错摆 —— 右道的在前、左道的在后，
   车从右道出发。所以第一次绕行往【左】，之后左右交替。
   每次发车前复位成 -1（左）。 */
static int8 slalom_dir = -1;
uint8 cone_group_cnt = 0;   /* 已经绕过的组数 */

int16    Steer_Mid = 917;   /* [2026-10-22] 实测中位(原代码 857 偏右约 5°) */

/* [2026-10-22] 设置舵机中位，并让左右限幅跟着中位走。
   之前 Pwm_Min/Max 写死 785/915，一旦真实中位超过 915（实测 917），
   舵机就永远无法往左打，车只能单方向转。 */
void Steer_Set_Mid(int16 m)
{
    if(m < 600) m = 600;
    if(m > 1100) m = 1100;
    Steer_Mid = m;
    /* [2026-10-24] 行程从硬编码 70 改成宏，绕锥桶时发现 ±70 不够用
       （duty 已经顶到 987 还是转不过去）。加大前先在架子上确认舵机不会
       顶到机械限位（会嗡鸣/发烫/电流大）。 */
    Pwm_Min   = (uint16)((m > STEER_DUTY_RANGE) ? (m - STEER_DUTY_RANGE) : 0);
    Pwm_Max   = (uint16)((m < 990) ? (m + STEER_DUTY_RANGE) : 1000);
    SteerSet(m);
}
int16    Steer_Trim = 16;      // 转向微调：中位对正后用不到，只补很小的直道偏差
uint16   steer_guard_bits = 0;
uint16 vision_age_ticks = 0, track_bad_ticks = 0;
uint8 steer_look_row = 22;
uint16 steer_lane_left = 0, steer_lane_right = 0;
uint8 drive_inhibit = 0, drive_inhibit_reason = 0;
double run_distance_cm = 0;  /* Nominal physical distance; legacy Run_Odo is preserved. */
uint16 mk_stop_elapsed_ms = 0, mk_clear_remaining_mm = 500;
uint8 mk_finish_reason = 0;
extern volatile uint8 mk_near_white;
extern volatile uint16 tof_sample_mm;
extern volatile uint32 tof_sample_tick;
extern volatile uint8 tof_has_sample;
uint8    steer_mode = 0;      /* 0 waiting, 1 settling, 2 lane, 3 cone, 4 invalid, 5 parked */
int32 steer_normal_target = -1;
uint8 cone_turn_percent = 0;
uint8 cone_active = 0;        /* [2026-10-31] 正在绕障：这期间不要被白漆兜底抢走转向 */
int32    steer_raw_target = -1;
int32    steer_target = -1;    // 本次计算出的目标点（诊断用）   // ★诊断：保护条件位掩码，0=PID正在跑
uint8    showpoint[2] = {0};
uint16   Speed_max = 150,Speed_min = 20;
uint16   Pwm_Max = 915,Pwm_Min = 785;

uint8     Line_Point;    // 目标点 纵坐标
/* [2026-10-20] start at the image centre.  0 would be read by the
   steering loop as "lane is far to one side" before the first frame. */
uint8     Point = MT9V03X_W / 2;
char      Point_Error;
float     Mid_Ka = 0, Mid_Kb = 0;   // 拟合的赛道中线的斜率
uint8     Fictitious_Midline[61] ;  // 虚拟赛道中线


int32 AD_Error;
/***************************  电机控制变量 *******************************/
volatile uint32 isr_tick_5ms = 0;
int32   RSpeedPWM,LSpeedPWM;
/* [2026-10-22] 发车后的绝对里程(cm)，不随标线状态机清零，用于日志里倒推行号-距离对照 */
double Run_Odo = 0;
uint8   Distance_ON = 0;         //默认关闭距离记录 用于行车过程的记录
double  Distance;                //距离保存变量，勿动
uint8   Distance_Test_ON = 0;  
double  Distance_Test;                //距离保存变量，勿动

uint8   Stop = 0;                 //停车
uint8   StartStopLine = 0;
int16   speedshow[4]={0};

/***************************** 赛道识别 ************************************/
char   Track_Line_Num=0,Track_Line_Time=0;
int16  Fictitious_Beeline_Num=0;
char   Last_Error[5];
int16  Max_Speed=0,Min_Speed=0;
uint8  Foresight;
uint8  Set_Speed;        //速度挡位  超过8为全程匀速


// 舵机控制
/* Use only an actual corridor or a plausible single edge, never both
   clipped defaults. Called only for rows produced in the current frame. */
static uint8 Mai_Lane_Row_Usable(uint8 row)
{
    int32 l = Left_Line[row], r = Right_Line[row];
    uint8 l_ok = (uint8)(l > 6 && l < (MT9V03X_W - 3));
    uint8 r_ok = (uint8)(r > 6 && r < (MT9V03X_W - 3));
    if(r - l < 20 || r - l > 140) return 0;
    if(l_ok && r_ok) return 1;
    return (uint8)((l_ok && l < (MT9V03X_W * 3 / 4)) ||
                   (r_ok && r > (MT9V03X_W / 4)));
}

void S3010_Control(void)
{
    static uint16 start_ticks = 0;
    static int32 lane_target = 60;
    static uint32 last_vision_frame = 0;
#if CONE_ACT_ENABLE
    static uint8 latched = 0;   /* 1 = 正在绕行（锁存中） */
    static double od_start = 0; /* 变道起点的绝对里程(cm) */
    static uint16 hold    = 0;  /* 兜底计时（5ms 一次） */
    static uint8 done    = 0;   /* 这一组锥桶已经处理过 */
    static uint16 clear_cnt = 0;/* 连续没看到锥桶的帧数 */
    static uint8 confirm = 0;
    static uint32 confirm_frame = 0;
    static int8  dir     = 0;   /* +1 = 往右变道   -1 = 往左变道 */
#endif
    uint8 cone_steering = 0;
    uint8 look_row = 22;
    if(Line_Count > 22 && Line_Count <= 52) look_row = (uint8)(Line_Count + 2);
    /* Paint and cones can corrupt one scan row while nearer road edges remain
       visible. Require two adjacent usable rows before taking a fallback. */
    if(Line_Count >= 0 && Line_Count <= 52 && !Mai_Lane_Row_Usable(look_row))
    {
        uint8 row;
        for(row = (uint8)(look_row + 1); row <= 54; row ++)
        {
            if(Mai_Lane_Row_Usable(row) && Mai_Lane_Row_Usable((uint8)(row + 1)))
            {
                look_row = row;
                break;
            }
        }
    }
    steer_look_row = look_row;
    cone_turn_percent = 0;
    steer_normal_target = -1;
    if(vision_processed_frames != last_vision_frame)
    {
        last_vision_frame = vision_processed_frames;
        vision_age_ticks = 0;
    }
    else if(vision_age_ticks < 60000) vision_age_ticks ++;
    Point_Error = ABS(Point-Point_Mid);   //用于赛道判断
    /* [2026-10-20] START GATE: before the car is released the camera has no
       valid image, Point stays 0, the PID sees an error of 60 and slams the
       servo to its left limit (785..915) for the whole countdown.
       Keep the wheels straight until the start signal sets Stop_1.
       Steering stays active after the start even while stopped at the
       zebra line (Flag_stop==1) so the lane is still held. */
    /* [2026-10-20] only Stop_1 matters.  With a ToF fitted, System.c leaves
       Flag_stop = 1 at power-up, so the old condition never fired.
       Steering still works after the start (zebra stop keeps Flag_stop=1). */
    if(Stop_1 == 0)
    {
        start_ticks = 0;
        track_bad_ticks = 0;
        run_distance_cm = 0;
        lane_target = Point_Mid;
        steer_settle = 1;
        steer_mode = 0;
        Point_PID.LastError = 0;
        slalom_dir = -1;   /* [2026-10-26] 每次发车前把交错方向复位成"先往左" */
        cone_group_cnt = 0;   /* 每次发车前清零，允许重新绕两次 */
#if CONE_ACT_ENABLE
        latched = done = confirm = 0;
        cone_active = 0;   /* [2026-10-31] 发车前清掉 */
        hold = clear_cnt = 0;
        od_start = Run_Odo;
        dir = 0;
        confirm_frame = vision_processed_frames;
#endif
        end_stopped = 0;      /* 终点停车标志复位 */
        /* [2026-10-26] 上一遍绕行把选道切走了就复位回右道；
           屏幕上调的 20~95 不动，免得覆盖手工标定。 */
        if(lane_k == 0 || lane_k == 100) lane_k = 75;
        /* [2026-10-22] 原来这里直接给 SteerMiddle，没带 Steer_Trim；
           微调只在发车后生效，测车时看起来"调了没用"。 */
        /* ★未发车时也算一下目标点（只显示 TP，不驱动舵机），公式和跑车时完全一致 */
        {
            uint8 l_ok2 = (Left_Line[22] > 6)  && (Left_Line[14] > 6);
            uint8 r_ok2 = (Right_Line[22] > 6) && (Right_Line[14] > 6) &&
                          (Right_Line[22] < (MT9V03X_W - 3)) && (Right_Line[14] < (MT9V03X_W - 3));
            int32 w2 = (int32)Right_Line[22] - (int32)Left_Line[22];

            if(l_ok2 && r_ok2 && w2 > 20)
            {
                steer_target = (int32)Left_Line[22] + w2 / 2;
            }
            else if(l_ok2)  steer_target = (int32)Left_Line[22]  +
                                            (STEER_LANE_W_DEF / 2);
            else if(r_ok2)  steer_target = (int32)Right_Line[22] -
                                            (STEER_LANE_W_DEF / 2);
            else            steer_target = Point_Mid;
        }
        steer_raw_target = steer_target;
        SD5_Duty = range_protect(Steer_Mid + Steer_Trim, Pwm_Min, Pwm_Max);
    }
    /* [2026-10-28] mk_state==3（低速过横道）也进这个分支：斑马线白漆会把左右边线
       全扫塌（实测 W 只剩 29~37，真车道 76），再循迹就是满舵乱打。这里直接走直。 */
    else if(end_stopped || (MK_ACT_ENABLE && mk_state == 2) ||
            (MK_ACT_ENABLE && mk_state == 3) ||
            (MK_ACT_ENABLE && mk_dia_seen) ||   /* [2026-10-31] 压菱形时白漆把边线扫塌，强制走直 */
            /* [2026-10-31] 白漆兜底：两条边线都在画面内、但宽度塌到 30px 以下
               （正常约 76），说明正压在菱形/横道白漆上，此时循迹不可信。 */
            (MK_ACT_ENABLE && !cone_active && Left_Line[22] > 6 && Right_Line[22] < (MT9V03X_W - 3) &&
             Width_Real[22] > 0 && Width_Real[22] < 30) ||
            start_ticks < STEER_START_TICKS)
    {
        if(!end_stopped && !(MK_ACT_ENABLE && (mk_state == 2 || mk_state == 3))) start_ticks ++;
        track_bad_ticks = 0;
        steer_mode = end_stopped ? 5 :
                     ((MK_ACT_ENABLE && mk_state == 3) ? 7 :
                      ((MK_ACT_ENABLE && mk_state == 2) ? 6 : 1));
        steer_settle = (uint8)(steer_mode == 1);
        steer_guard_bits = end_stopped ? 0 : 0x0001;
        steer_raw_target = steer_target = Point_Mid;
        lane_target = Point_Mid;
        Point_PID.LastError = 0;
        /* [2026-10-28] 过横道时中位差一点就会长距离跑偏，单独补一个量 */
        SD5_Duty = range_protect(Steer_Mid + Steer_Trim +
                               ((MK_ACT_ENABLE && mk_state == 3) ? MK_CROSS_TRIM : 0),
                               Pwm_Min, Pwm_Max);

        /* [2026-10-31] 过横道不再死打死中位：远处（MK_XING_ROW 行）如果还有一对
           可信边线，就用它的中点当目标点，让 PID 把车拉回车道中心。
           白漆只把近处扫塌，终点后那条白线是干净的。
           门限不满足就还是上面的固定中位 + MK_CROSS_TRIM。 */
        if(MK_ACT_ENABLE && mk_state == 3 && !end_stopped)
        {
            int32 l14 = (int32)Left_Line[MK_XING_ROW];
            int32 r14 = (int32)Right_Line[MK_XING_ROW];
            int32 w14 = r14 - l14;
            int32 pt2 = -1;

            /* ① 远处那条白线可信 -> 用它的中点 */
            if(l14 > 6 && r14 < (MT9V03X_W - 3) &&
               w14 >= MK_XING_W_MIN && w14 <= MK_XING_W_MAX)
                pt2 = l14 + w14 / 2;

            /* ② ToF 转向：必须要有新鲜读数，而且要比 MK_TOF_MIN_MM 远。
               现在这种照地面的 120~150mm 会被 MK_TOF_MIN_MM 挡掉，
               否则车会一直以为前方有东西，一路偏下去。 */
            if(MK_TOF_TURN_EN)
            {
                uint16 td = Tof_Get_Distance();
                if(Tof_Get_Age_Ms() <= MK_TOF_FRESH_MS &&
                   td >= MK_TOF_MIN_MM && td < MK_TOF_TURN_MM)
                {
                    int32 off, cap;
                    if(pt2 < 0) pt2 = Point_Mid;
                    /* 按距离线性加力：越近打得越多；MK_TOF_TURN_PX 的符号决定往哪边 */
                    cap = (MK_TOF_TURN_PX >= 0) ? MK_TOF_TURN_PX : -MK_TOF_TURN_PX;
                    off = (int32)(MK_TOF_TURN_MM - td) * MK_TOF_PX_GAIN / 100;
                    if(off > cap) off = cap;
                    if(MK_TOF_TURN_PX < 0) off = -off;
                    pt2 += off;
                }
            }

            if(pt2 >= 0)
            {
                int32 duty, off2;
                if(pt2 < 5)               pt2 = 5;
                if(pt2 > (MT9V03X_W - 5)) pt2 = MT9V03X_W - 5;
                steer_raw_target = steer_target = pt2;
                lane_target = pt2;

                /* [2026-10-31] 不走 PID，直接按偏差给舵量：
                   pt2 偏右 -> off2 为负 -> duty 减小 -> 往右打；
                   pt2 一回到中心，off2 立刻归零 —— 转完马上能回正，
                   不会像 PID 那样拖着走一段。 */
                off2 = ((int32)Point_Mid - pt2) * MK_XING_GAIN / 10;
                if(off2 >  MK_XING_DUTY) off2 =  MK_XING_DUTY;
                if(off2 < -MK_XING_DUTY) off2 = -MK_XING_DUTY;
                SD5_Duty = range_protect(Steer_Mid + Steer_Trim + MK_CROSS_TRIM + off2,
                                         Pwm_Min, Pwm_Max);
            }
        }
    }
#if STEER_LANE_FOLLOW
    /* [2026-10-20] lane following wins over the competition state machine */
    else
    {
        /* [2026-10-20] Line_Count is the first row Image_Handle managed to
           process.  Small values mean the fit only has a few rows and is
           numerically unreliable; the old test (>=58) let those through and
           the PID then steered on garbage.  Require a solid row band. */
        /* [2026-10-22] Line_Count 是 break 时的行号、不是有效行数（最坏停在 59），
           整帧没线时会放行残缺中线。改用本帧最远可信行 Result_Far_Row。 */
        /* [2026-10-22] 三种情况一律回中位（不送 PID）：
           1) 起跑后 1 秒稳定期内，图像还不可信
           2) 中点贴到图像两侧（起跑过曝时会算出 P=2）
           3) 第 50 行赛道宽度不是真实车道宽度（过暗时 L=1 R=100 W=99，
              是扫描碰边界的假值，P 会被拉到 39 这种固定偏左值） */
        steer_settle = 0;
        steer_lane_left = Left_Line[look_row];
        steer_lane_right = Right_Line[look_row];
        steer_guard_bits = 0;
        if(steer_settle)                                  steer_guard_bits |= 0x0001;  // 刚上电在等图
        if(0 == Point)                                    steer_guard_bits |= 0x0002;  // 中线为 0
        if(Point <= 6)                                    steer_guard_bits |= 0x0004;  // 中线贴左边
        if(Point >= (MT9V03X_W - 6))                      steer_guard_bits |= 0x0008;  // 中线贴右边
        if((int32)Right_Line[look_row] - (int32)Left_Line[look_row] < 20)                           steer_guard_bits |= 0x0010;  // 22行过窄
        if((int32)Right_Line[look_row] - (int32)Left_Line[look_row] > 140)                          steer_guard_bits |= 0x0020;  // 22行过宽
        if(Width_Real[14] < 20)                           steer_guard_bits |= 0x0040;  // 14行过窄
        if(Width_Real[14] > 140)                          steer_guard_bits |= 0x0080;  // 14行过宽
        if(Left_Line[look_row] <= 2)                            steer_guard_bits |= 0x0100;  // 22行左边撞墙
        if(Right_Line[look_row] >= (MT9V03X_W - 3))             steer_guard_bits |= 0x0200;  // 22行右边撞墙
        if(Left_Line[14] <= 2)                            steer_guard_bits |= 0x0400;  // 14行左边撞墙
        if(Right_Line[14] >= (MT9V03X_W - 3))             steer_guard_bits |= 0x0800;  // 14行右边撞墙
        if(Line_Count < 0 || Line_Count > 52)             steer_guard_bits |= 0x1000;
        if(!vision_processed_frames || vision_age_ticks >= VISION_STALE_TICKS)
            steer_guard_bits |= 0x2000;
        /* ★宽度归一化 + 单边补线（2026-10-24）
           [2026-10-24] l_ok/r_ok 必须同时查上下界：整帧没线时 L=0/R=0，
           原来 r_ok 只查上界，(0 < 117) 成立 → 走单边分支算出 TP=-21，舵机左打死。
           w <= 70：扫到的是一条道  → 取这条道的中心（车当前所在的道）
           w >  70：扫到了整条路    → lane_k<=50 取左道中心，>50 取右道中心
           只有单边线时，用学到的单道宽往缺失那侧推半个道宽。 */
        {
            uint8 l_ok = (Left_Line[look_row] > 6) && (Left_Line[look_row] < (MT9V03X_W - 3));
            uint8 r_ok = (Right_Line[look_row] > 6) && (Right_Line[look_row] < (MT9V03X_W - 3));
            int32 w    = (int32)Right_Line[look_row] - (int32)Left_Line[look_row];
            int32 pt   = -1;
            static int32 lane_w_ema = 0;      /* ★只学"一条道"的宽度，不会被整条路污染 */

            /* The nearest bounding pair is the current corridor. Width alone cannot
               distinguish one lane from two lanes in a clipped perspective image. */
            /* [2026-10-28] 宽度合理性：一条道实测约 76px（lane_w_ema 学出来也是这个量级）。
               左边线出画时（L=1），扫描会配到一对假边界，宽度只剩 47 这类值，
               算出的中心偏 20px 以上，而 PID 增益 0.2x|e| -> 直接满舵甩出去
               （V38 日志 T=88：L=1 R=48 W=47 -> TP=30 -> d=987）。
               宽度不到已知道宽的 2/3 就整帧不信，沿用上一个可信目标点。 */
            if(l_ok && r_ok && w > 20 &&
               !(lane_w_ema > 20 && w * 3 < lane_w_ema * 2))
            {
                pt = (int32)Left_Line[look_row] + w / 2;
                /* 只让「和已有估计差不远」的宽度进 EMA，别被假配对带偏 */
                if(!(steer_guard_bits & (0x1000 | 0x2000)) && w >= 30 && w <= 110 &&
                   (lane_w_ema <= 20 || (w * 3 >= lane_w_ema * 2 && w * 4 <= lane_w_ema * 5)))
                    lane_w_ema = (0 == lane_w_ema) ? w : ((lane_w_ema * 7 + w) / 8);
            }
            else
            {
                int32 lw = (lane_w_ema > 20) ? lane_w_ema : STEER_LANE_W_DEF;

                /* ★2026-10-24 位置合理性：横道/斑马线时边界会贴到画面边缘，
                   例如 L=100 还在"左边界"，补出半个道宽就跑到画面外了（实测 TP=127）。
                   左边线必须在画面左 3/4 内、右边线必须在右 3/4 内才算数。 */
                /* One visible edge does not identify the outer edge of both lanes.
                   Reconstruct only half of one lane; never request an extra lane change. */
                if(l_ok && Left_Line[look_row] < (MT9V03X_W * 3 / 4))
                    pt = (int32)Left_Line[look_row]  + (lw / 2);
                else if(r_ok && Right_Line[look_row] > (MT9V03X_W / 4))
                    pt = (int32)Right_Line[look_row] - (lw / 2);

                /* [2026-10-24] 限幅：实测出现过 TP=134（画面只有 0~119）。
                   lw*3/2 在 L 靠右的时候会推到画面外，舵机从满左直接甩到满右。 */
                /* [2026-10-30] 单边线分支也要查宽度：左右边界都塌到一起时
                   （10-30 日志 T=40 DC=876: L=1 R=52 W=51，真实约 90），
                   这一帧的边界是假的；此时 pt 置 -1 判不可信，沿用上一帧目标，
                   否则会按假的右边界算目标点 -> TP=38 -> 满舵左打 d=987。 */
                if(pt >= 0 && lane_w_ema > 20 && w * 3 < lane_w_ema * 2)
                    pt = -1;

                if(pt >= 0)
                {
                    /* ★锥桶区里收得更紧（±25）：锥桶把边线挡断时单边补线会算出极端值，
                       实测出现过 TP 38 <-> 105 来回甩，车在锥桶前左右横甩然后撞上去。
                       有锥桶时只允许小幅修正，把大方向交给绕行锁存。 */
                    int32 lim = (cone_cnt >= 1) ? 25 : 45;

                    if(pt > (int32)Point_Mid + lim) pt = (int32)Point_Mid + lim;
                    if(pt < (int32)Point_Mid - lim) pt = (int32)Point_Mid - lim;
                }
            }

            steer_normal_target = pt;
#if CONE_ACT_ENABLE
            if(mk_state < 2)
            /* ★锥形标绕行 —— 锁存式（2026-10-24）
               规则：锥形标分两组，第一组“变道”、第二组“回线”。
               所以看到锥桶不能“避一下就回正”，要变过去并保持。

               旧版的错误：车一打方向锥桶就移出视野 → CN=0 →
               50ms 后退出绕行 → 回正 → 又看见锥桶 → 再打过去 → 来回摆。
               实测只转了 3 帧（0.75s）就退出，而一组锥桶有 2~4 米长。

               现在：
                 看到锥桶 → 定方向、锁存、把目标点定到另一条道的中心
                 锁存期间每次看到都续期，看不到也不立刻退出
                 走够 CONE_LANE_CM（按里程）才结束 → 回到循迹
               锁存期间、方向不变，避免锥桶从一侧换到另一侧时反复打方向。 */
            {
                /* [2026-10-24] 一组锥桶只变一次道。
                   原来用"1.5 秒冷却"，但车变道过去以后还看得见锥桶，
                   冷却一结束就又变一次，实测连续左满舵 8 次、越转越左。
                   现在要求：锥桶必须连续 CONE_CLEAR_TICKS 帧看不见，
                   才算这一组过完，之后才允许下一次变道（也就是第二组"回线"）。 */
                if(cone_ok) clear_cnt = 0;
                else if(clear_cnt < 60000) clear_cnt ++;

                if(done && clear_cnt >= CONE_CLEAR_TICKS) done = 0;

                /* Count completed vision results, not repeated 5 ms reads of one result. */
                if(steer_guard_bits & 0x2000) confirm = 0;
                if(confirm_frame != vision_processed_frames)
                {
                    confirm_frame = vision_processed_frames;
                    if(run_distance_cm >= CONE_ARM_CM && !(steer_guard_bits & 0x2000) &&
                       cone_ok && cone_cnt >= 2 && cone_near >= CONE_TRIG_ROW &&
                       cone_x >= 6 && cone_x <= (MT9V03X_W - 6))
                    {
                        if(confirm < CONE_CONFIRM_FRAMES) confirm ++;
                    }
                    else confirm = 0;
                }
                if(!latched && !done && run_distance_cm >= CONE_ARM_CM &&
                   !(steer_guard_bits & 0x2000) && confirm >= CONE_CONFIRM_FRAMES && cone_ok && cone_cnt >= 2 &&
                   cone_x >= 6 && cone_x <= (MT9V03X_W - 6) &&      /* 贴边的 blob 是噪点 */
                   cone_near >= CONE_TRIG_ROW &&
                   cone_group_cnt < CONE_MAX_GROUPS &&            /* 最多绕两组 */
                   (CONE_MAX_CM == 0 || run_distance_cm <= (double)CONE_MAX_CM))   /* 只在前段触发 */
                {
                    /* [2026-10-26] 方向完全按交错顺序走，不再看锥桶在画面偏左还是偏右。
                       原来「锥桶偏右就往左」那条规则会覆盖交错：实测第一排往左、
                       第二排又往左（偏右一点点就被它改掉了），结果正撞上第二排。
                       这条赛道是固定的 —— 右道锥桶在前、左道锥桶在后，车从右道出发，
                       所以必然是 左、右、左、右 …，顺序由 slalom_dir 记着。 */
                    dir = slalom_dir;

                    done = 1;                    /* 这一组已经处理，别再触发 */
                    if(dir != 0)
                    {
                        confirm = 0;
                        latched  = 1;
                        od_start = Run_Odo;   /* 记下起点里程 */
                        hold     = CONE_HOLD_MAXTICKS;
                        /* [2026-10-26] 只打方向不够：循迹按 lane_k 选道，
                           绕完会立刻把车拉回原车道（实测就是"躲开了但没进左道"）。
                           所以这里把选道也一起切过去，绕完车会真的留在另一条道。 */
                        lane_k = (int16)((dir < 0) ? 0 : 100);
                        slalom_dir = (int8)(-dir);   /* 下一次反过来 */
                        cone_group_cnt ++;             /* 这一组算掉了 */
                    }
                    /* dir==0：这组跳过，等锥桶出画面后 done 自动复位 */
                }
                /* [2026-10-24] 这里原来"每次看到锥桶就续期"，结果锥桶一组有 2~4 米，
                   车一直看得见就一直续 → 满舵左打 2 秒多 → 冲出赛道。
                   改成【不再续期】：从第一次看到开始，固定做一次变道动作。 */

                if(latched)
                {
                    /* [2026-10-26] 走够 CONE_LANE_CM 才算变道完成；hold 只做超时兜底 */
                    if((Run_Odo - od_start) >= (double)CONE_LANE_CM) latched = 0;
                    else if(hold == 0)                               latched = 0;
                    else                                             hold --;

                    if(latched)
                    {
                        int32 half = CONE_LANE_HALF;
                        int32 margin, t;
                        double used = Run_Odo - od_start;
                        /* The second group is the return to the home lane. Do not
                           keep full lateral demand after the lane change is underway. */
                        if(cone_group_cnt == 2 && used > CONE_RETURN_TAPER_FROM)
                            half = (int32)(CONE_LANE_HALF * (CONE_LANE_CM - used) /
                                           (CONE_LANE_CM - CONE_RETURN_TAPER_FROM));
                        if(half < 0) half = 0;
                        margin = half < 6 ? half : 6;
                        cone_turn_percent = (uint8)(half * 100 / CONE_LANE_HALF);
                        t = (int32)Point_Mid + (int32)dir * half;

                        /* [2026-10-26] 目标点不许越过第 22 行的边线（再留 4px），
                           否则舵机会把车直接顶着墙走。
                           但限幅也不能把方向反过去，所以最后再夹一次。 */
                        if(dir < 0 && Left_Line[22] > 2)
                        {
                            if(t < (int32)Left_Line[22] + 4) t = (int32)Left_Line[22] + 4;
                        }
                        if(dir > 0 && Right_Line[22] < (MT9V03X_W - 3))
                        {
                            if(t > (int32)Right_Line[22] - 4) t = (int32)Right_Line[22] - 4;
                        }
                        if(dir < 0 && t > (int32)Point_Mid - margin) t = (int32)Point_Mid - margin;
                        if(dir > 0 && t < (int32)Point_Mid + margin) t = (int32)Point_Mid + margin;

                        pt = t;
                        cone_steering = 1;
                    }
                    else
                    {
                        lane_target = Point_Mid;
                        Point_PID.LastError = 0;
                    }
                }
                cone_active = (uint8)latched;   /* [2026-10-31] 给 S3010_Control 看 */
            }
            #endif

            /* [2026-10-28] 坏帧处理改回 V23 的「保持上一个可信目标点」。
               之前任何一帧不可信就直接把 lane_target 拽回 60：
               分道线断口这种正常现象会让车不停回正 -> 弯道里转不够 ->
               冲出跑道；同时 track_bad_ticks 200ms 就停车 -> 跑一路停一路。
               现在：坏帧只是「这一帧不更新目标」，保持上一次的可信值；
               只有连续 TRACK_LOSS_TICKS 都不可信才回中位（并停车）。 */
            steer_raw_target = pt;
                /* [2026-10-28] 横道白条会把左边界顶歪，实测 TP 会往右偏十几像素。既然已经确认看到横道（或眼前就是白条），就直接按中线走，别在斑马线上打方向。 */
                if(MK_ACT_ENABLE && mk_state < 2 && !cone_steering && (mk_cw_seen || mk_near_white) && pt >= 0)
                    pt = Point_Mid;
            if(!cone_steering)
            {
                uint8 bad = (uint8)(pt < 0 ||
                    (steer_guard_bits & (0x1000 | 0x2000 | 0x0010 | 0x0020)));
                if(bad)
                {
                    if(track_bad_ticks < 60000) track_bad_ticks ++;
                    if(track_bad_ticks >= TRACK_LOSS_TICKS)
                        lane_target = Point_Mid;   /* 真丢了才回中 */
                    steer_mode = 4;
                }
                else
                {
                    track_bad_ticks = 0;
                    pt = range_protect(pt, (int32)Point_Mid - STEER_STRAIGHT_RANGE,
                                           (int32)Point_Mid + STEER_STRAIGHT_RANGE);
                    if(lane_target < pt) lane_target += STEER_TARGET_STEP;
                    else if(lane_target > pt) lane_target -= STEER_TARGET_STEP;
                    steer_mode = 2;
                }
                pt = lane_target;
            }
            else
            {
                lane_target = Point_Mid;
                track_bad_ticks = 0;
                steer_mode = 3;
            }

            steer_target = pt;
            if(pt >= 0)
            {
                int32 duty;
                if(pt < 5)               pt = 5;
                if(pt > (MT9V03X_W - 5)) pt = MT9V03X_W - 5;
                duty  = range_protect(PlacePID_Control(&Point_PID, Point_Mid, pt), Pwm_Min, Pwm_Max);
                duty += Steer_Trim;
                if(duty < Pwm_Min) duty = Pwm_Min;
                if(duty > Pwm_Max) duty = Pwm_Max;
                SD5_Duty = duty;
            }
            else
            {
                SD5_Duty = Steer_Mid + Steer_Trim;   /* 两边都没线：回中 */
            }
        }
    }
#else
    /* [2026-10-20] these used to be hard-coded duty values (908/794/854 ...),
       so they ignored SteerMiddle and pulled the car right whenever the
       image logic set Flag_turn=3.  They are offsets from the trim now. */
    else if(Flag_turn==1)          SD5_Duty = SteerMiddle + 51;   /* left        */
    else if(Flag_turn==2)     SD5_Duty = SteerMiddle - 63;   /* right       */
    else if(Flag_turn==3)     SD5_Duty = SteerMiddle;        /* straight    */
    else if(Flag_turn==4)     SD5_Duty = SteerMiddle + 63;   /* hard left   */
    else if(Flag_turn==5)     SD5_Duty = SteerMiddle - 1;    /* slight right*/
    else if(Flag_turn==6)     SD5_Duty = SteerMiddle - 70;   /* more right  */
    else if(Flag_turn==8)     SD5_Duty = SteerMiddle + 51;   /* left        */
    else
    {
        // ★保护：没线就回中位，不跑 PID
        if((0 == Point) || (Line_Count >= STEER_NO_LINE_LINECOUNT))
        {
            SD5_Duty = SteerMiddle;
        }
        else
        {
            SD5_Duty = PlacePID_Control(&Point_PID,Point_Mid,Point);   // 舵机PID 得到当前的舵机占空比

//            if(SD5_Duty<=852)
//            {
//                SD5_Duty= 2*SD5_Duty-852;
//            }
            SD5_Duty = range_protect(SD5_Duty, Pwm_Min, Pwm_Max);  //舵机占空比限幅保护
        }
    }
#endif
    SteerSet(SD5_Duty);     //修改舵机占空比
}

void Motor_Control(void)
{
    isr_tick_5ms ++;
//    编码器;
    // ★2024bizhang 改动：实测往前推两个轮子，原来算出来的是负的（R=-6、L=-7）。
    //   现在统一成「往前推 = 正数」。这两个符号是按你现在的电机/编码器安装方向定下来的，
    //   以后如果拆过电机、换过编码器方向，要回来重新确认一次。
    // 右轮：去掉负号；这两个符号是按你现在的电机/编码器安装方向定的，
    // 以后如果拆过电机、换过编码器方向，要回来重新确认一次。
    Rcount =  encoder_get_count(ENCODER1_TIM);  // 右轮（★方向反转后去掉了负号）
    Lcount = -encoder_get_count(ENCODER2_TIM);  // 左轮（★方向反转后加了负号）

    encoder_clear_count(ENCODER2_TIM);
    encoder_clear_count(ENCODER1_TIM);

//     Distance_ON=1;
    if(Distance_ON)                    //开启距离累加
    {
        Distance += (DISTANCE( +(Rcount+Lcount)/2) );
    }
    /* [2026-10-22] 前进才累计。发车后刚放开那几秒车可能还没起步（PID 爬坡），
       或者被人往后退，这些都不算前进里程。 */
    if(Stop_1 && ((int32)Rcount + (int32)Lcount) > 0)
    {
        Run_Odo += DISTANCE(((int32)Rcount + (int32)Lcount) / 2);   // Legacy tuning units.
        run_distance_cm += (((double)Rcount + (double)Lcount) / 2.0) *
                           (Length * Multiple / (All_Line * Ratio));
    }
   // else  Distance = 0;

    
    if(Distance_Test_ON)                    //开启距离累加
    {
        Distance_Test += (DISTANCE(Rcount+Lcount)/2 );
    }
    //else  Distance_Test = 0;
    
    Rrealspeed  = Rcount;
    Lrealspeed  =  Lcount;

//    printf("%d,%d",Rrealspeed,RMotorSpeed);   // ★原在 5ms 中断里，每秒 200 次，占用串口和中断时间


    /* A latched final stop overrides PID history and every motor test mode. */
    if(!Stop_1 || end_stopped || drive_inhibit)
    {
        RMPID.Sum = RMPID.e0 = RMPID.e1 = RMPID.e2 = 0;
        LMPID.Sum = LMPID.e0 = LMPID.e1 = LMPID.e2 = 0;
        RSpeedPWM = LSpeedPWM = 0;
        gpio_set_level(MOTOR1_A, 0);
        gpio_set_level(MOTOR2_A, 0);
        pwm_set_duty(MOTOR1_B, 0);
        pwm_set_duty(MOTOR2_B, 0);
        return;
    }
    RSpeedPWM = RMotorPid_Col(RMotorSpeed);
    LSpeedPWM = LMotorPid_Col(LMotorSpeed);

    // ★按 MOTOR_TEST_MODE 覆盖输出（默认 0 = 不改变行为）
    #if (MOTOR_TEST_MODE == 1)
        /* ★★全脚拉低 = 万能保险（新增）★★
           原来只把 RSpeedPWM/LSpeedPWM 设成 0，但下面输出逻辑仍然会把 DIR 脚拉高
           （gpio_set_level(MOTORx_A, 1)）—— DIR 恒高 = 恒 3.3V。
           如果驱动板的 PWM 输入接在这根线上，它看到的就是 100%，车直接飞。

           现在改成：四个脚全部拉低后直接 return。
           => 无论驱动板的 PWM / DIR 接在哪个脚上，它看到的都是 0%（低电平），车不会动。
           这是软件能做到的极限保险。 */
        RSpeedPWM = 0;                      // 变量也清零，免得表达式窗口还显示 PID 算出的 8000
        LSpeedPWM = 0;
        gpio_set_level(MOTOR1_A, 0);
        gpio_set_level(MOTOR2_A, 0);
        pwm_set_duty(MOTOR1_B, 0);
        pwm_set_duty(MOTOR2_B, 0);
        return;                             // 直接退出，不走后面的输出
    #elif (MOTOR_TEST_MODE == 2)
        RSpeedPWM = MOTOR_TEST_PWM;         // 开环：给固定小 PWM
        LSpeedPWM = MOTOR_TEST_PWM;
    #elif (MOTOR_TEST_MODE == 4)
        /* ★模式 4：DIR 抖动测试
           每 100ms 把两个 DIR 脚翻转一次，PWM 保持 8%。
           如果 DIR 线真的接在驱动板的 DIR 输入上 -> 轮子会每 100ms 换向一次，
           表现为抖动 / 嗡嗡响 / 正反转交替。
           如果 DIR 线没接上/接错 -> 轮子平稳地朝一个方向转，毫无变化。 */
        {
            static uint8  dir_flip = 0;
            static uint16 dir_cnt  = 0;
            if(++ dir_cnt >= 20) { dir_cnt = 0; dir_flip = !dir_flip; }   // 20 x 5ms = 100ms
            gpio_set_level(MOTOR1_A, dir_flip);
            gpio_set_level(MOTOR2_A, dir_flip);
            pwm_set_duty(MOTOR1_B, 800);
            pwm_set_duty(MOTOR2_B, 800);
        }
        return;
    #endif
//    LSpeedPWM = -2000;
//    RSpeedPWM=-2000;

    // ★★硬限幅（新增）：最后一道保险，任何情况下输出都不会超过 MOTOR_PWM_MAX
    if(RSpeedPWM >   MOTOR_PWM_MAX) RSpeedPWM =  MOTOR_PWM_MAX;
    if(RSpeedPWM <  -MOTOR_PWM_MAX) RSpeedPWM = -MOTOR_PWM_MAX;
    if(LSpeedPWM >   MOTOR_PWM_MAX) LSpeedPWM =  MOTOR_PWM_MAX;
    if(LSpeedPWM <  -MOTOR_PWM_MAX) LSpeedPWM = -MOTOR_PWM_MAX;

    if(LSpeedPWM >= 0)                    // ★方向反转：正转时 DIR 拉低
    {
        gpio_set_level(MOTOR2_A,0);
        pwm_set_duty(MOTOR2_B, LSpeedPWM);
    }
    else
    {
        gpio_set_level(MOTOR2_A,1);
        pwm_set_duty(MOTOR2_B, - LSpeedPWM);
    }

    if(RSpeedPWM >= 0)                         //新方案双电机控制  ★方向反转：正转时 DIR 拉低
    {
        gpio_set_level(MOTOR1_A,0);
        pwm_set_duty(MOTOR1_B, RSpeedPWM);
    }
    else
    {
        gpio_set_level(MOTOR1_A,1);
        pwm_set_duty(MOTOR1_B, - RSpeedPWM);
    }
}

/*
*   计算终点位置
*
*/
/* [2026-10-22] 把 2026 标线常量放到函数外，避免在函数中间写 #define 污染后续代码。 */
#define MK_DIA_SPEED_PCT    (70)      // [2026-10-28] 50->70: 两个菱形之间减速太狠，规则只要求「减速」
#define MK_DIA_CONFIRM       (8)       /* Distinct completed camera frames. */
#define MK_DIA_CM           (850)     /* [2026-10-31] 菱形视觉始终没检测到（DI 恒为 0），用车程兜底保一次减速；0 = 关闭。 */
#define MK_CW_CONFIRM        (15)      /* Crosswalk pattern persists across completed frames. */
#define MK_CW_NEAR_CONFIRM    (3)      /* Near position also needs independent confirmation. */
#define MK_STOP_ROW                 (40)     // [2026-10-28] 45->40: 实测 near 最大只有 44, 45 永远够不到 -> 横道不停
#define MK_STOP_TICKS       (1000)    // 停车 5 秒（5ms * 1000）
#define MK_CROSS_CM         (1120)   /* [2026-10-28] 改回视觉为主，这个只做兜底：条纹判据万一还是没认出来，走到这里强制停，别冲过终点（终点墙实测 1174）。0 = 关闭。 */
#define MK_CW_MIN_CM        (800)    /* [2026-10-28] 横道只可能在赛道后段，前段不认 */
#define MK_CROSS_SPEED      (22)      /* [2026-10-28] 10->18: 停 5 秒后重新起步太慢 */
#define MK_CLEAR_CM         (50)      /* Camera-to-rear ~30 cm + requested margin ~20 cm. */
#define MK_CROSS_LIMIT_CM   (150)     /* Bound travel if stripe tracking never clears. */
#define MK_STILL_COUNTS     (2)
#define MK_PARK_DUTY        (0)       // 停车时电机输出（0 = 自然滑停）

void Point_Control(void)
{
    float res;
    uint8 point_raw;   // [2026-10-20] point_med removed
    /* [2026-10-20] remember the raw Point before the median for the loss guard */
//    if(Deal_Flag > 20 && !Dis_Flag)
//    {
//        Line_Point = 45;
//    }
//    else
        Line_Point = 20;   /* [2026-10-20] back to 20 (12 behaved worse) */

    if(Line_Count < 0 || Line_Count > 54)
    {
        Mid_Ka = 0;
        Mid_Kb = Point_Mid;
        Point = 0;     /* Publish invalid instead of converting NaN to an image coordinate. */
        return;
    }
    Least_Sqaure_Method(&Mid_Ka,&Mid_Kb,58, Line_Count+2); // 拟合中线斜率
    //Least_Sqaure_Method(&Mid_Ka,&Mid_Kb,58, Line_Count+2); // 拟合中线斜率
    Fictitious_Beeline_Num = Fictitious_Beeline(Mid_Ka,Mid_Kb);  // 拟合中线  返回相似度
    res = Line_Point * Mid_Ka + Mid_Kb;
    point_raw = range_protect((int32)res, 2, 118);
    /* [2026-10-20] median/centre-snap removed: with AGL_POINT_MEDIAN_EN=0
       it was dead code, and re-enabling it later would jerk the steering. */
    Point = point_raw;
    if(STEER_FORCE_CENTER) Point = Point_Mid;   // [2026-10-20] self-test

}


void Control_Speed(void)
{
    static uint16 ramp_cnt = 0, mk_stop_cnt = 0;
    static int32 out_L = 0, out_R = 0;
    static uint8 mk_dia_run = 0, mk_cw_run = 0, mk_cw_near_run = 0, cross_near_seen = 0;
    static uint8 mk_dia_cnt = 0, mk_dia_latch = 0;   /* [2026-10-28] 第几个菱形 */
    static uint32 cross_frame = 0, marker_frame = 0;
    static double cross_start_cm = 0, last_white_cm = 0;
    uint8 vision_fault;

    LMotorSpeed = RMotorSpeed = 210;
    if(Flag_stop == 1) LMotorSpeed = RMotorSpeed = 0;
    else if(Flag_stop == 6) LMotorSpeed = RMotorSpeed = 175;
    else if(Flag_stop == 8) LMotorSpeed = RMotorSpeed = 85;
    else if(Flag_stop >= 2 && Flag_stop <= 7) LMotorSpeed = RMotorSpeed = 135;
    LMotorSpeed = LMotorSpeed * SPEED_PERCENT / 100;
    RMotorSpeed = RMotorSpeed * SPEED_PERCENT / 100;

    if(!Stop_1)
    {
        mk_state = 0;
        mk_stop_cnt = 0;
        mk_dia_run = mk_cw_run = mk_cw_near_run = cross_near_seen = 0;
        mk_dia_cnt = mk_dia_latch = 0;
        marker_frame = vision_processed_frames;
        mk_stop_elapsed_ms = mk_finish_reason = 0;
        mk_clear_remaining_mm = MK_CLEAR_CM * 10;
    }
    else
    {
        if(marker_frame != vision_processed_frames)
        {
            marker_frame = vision_processed_frames;
            if(mk_dia_seen) { if(mk_dia_run < 255) mk_dia_run ++; } else mk_dia_run = 0;
            if(mk_cw_seen) { if(mk_cw_run < 255) mk_cw_run ++; }
            else if(mk_cw_run) mk_cw_run --;   /* [2026-10-28] 丢一帧不清零, 否则检测稍微抖一下永远到不了 15 */
            if(mk_cw_seen && mk_cw_near >= MK_STOP_ROW)
            { if(mk_cw_near_run < 255) mk_cw_near_run ++; }
            else if(mk_cw_near_run) mk_cw_near_run --;   /* [2026-10-28] 同上 */
        }
        if(!vision_processed_frames || vision_age_ticks >= VISION_STALE_TICKS)
            mk_dia_run = mk_cw_run = mk_cw_near_run = 0;

        /* [2026-10-28] 数菱形：第 1 个进减速，第 2 个之后恢复原速。
           锁存保证一个菱形只计一次（它从画面消失时 mk_dia_run 归零）。 */
        if(!mk_dia_seen) mk_dia_latch = 0;
        else if(mk_dia_run >= MK_DIA_CONFIRM && !mk_dia_latch)
        {
            mk_dia_latch = 1;
            if(mk_dia_cnt < 9) mk_dia_cnt ++;
        }

        if(mk_state == 0 || mk_state == 1)
        {
            if(mk_cw_run >= MK_CW_CONFIRM && mk_cw_near_run >= MK_CW_NEAR_CONFIRM &&
               mk_cw_seen && mk_cw_near >= MK_STOP_ROW && run_distance_cm >= MK_CW_MIN_CM)
            {
                mk_state = 2;
                mk_stop_cnt = 0;
                mk_stop_elapsed_ms = 0;
            }
            /* [2026-10-28] 里程兜底：横道识别不出来时，按固定赛道里程停。 */
            else if(MK_CROSS_CM > 0 && run_distance_cm >= (double)MK_CROSS_CM)
            {
                mk_state = 2;
                mk_stop_cnt = 0;
                mk_stop_elapsed_ms = 0;
            }
            /* [2026-10-28] 第二个菱形过了 -> 退出减速，恢复原速。
               放在「进减速」之前：mk_dia_cnt 已经 >=2 时就不再进 state 1。 */
            else if(mk_state == 1 && mk_dia_cnt >= 2)
            {
                mk_state = 0;
                Distance = 0;
                Distance_ON = 0;
            }
            else if(mk_state == 0 && mk_dia_cnt == 1 && mk_dia_run >= MK_DIA_CONFIRM)
            {
                mk_state = 1;
                Distance = 0;
                Distance_ON = 1;
            }
            /* [2026-10-31] 菱形视觉全程没检出，用里程兜底进一次减速（只进一次）。 */
            else if(mk_state == 0 && MK_DIA_CM > 0 && run_distance_cm >= (double)MK_DIA_CM)
            {
                mk_state = 1;
                Distance = 0;
                Distance_ON = 1;
            }
            /* Keep the first confirmed slowdown through both diamonds and the
               unmarked gap. Only the crosswalk advances this state. */
        }
        else if(mk_state == 2)
        {
            /* The five seconds start when encoder feedback says the vehicle is stationary. */
            if(ABS(Lrealspeed) <= MK_STILL_COUNTS && ABS(Rrealspeed) <= MK_STILL_COUNTS)
            {
                if(mk_stop_cnt < MK_STOP_TICKS) mk_stop_cnt ++;
            }
            else mk_stop_cnt = 0;
            mk_stop_elapsed_ms = mk_stop_cnt * 5;
            if(mk_stop_cnt >= MK_STOP_TICKS)
            {
                mk_state = 3;
                cross_start_cm = last_white_cm = run_distance_cm;
                cross_near_seen = 0;
                cross_frame = vision_processed_frames;
                Distance = 0;
                Distance_ON = 0;
            }
        }
        else if(mk_state == 3)
        {
            if(cross_frame != vision_processed_frames)
            {
                cross_frame = vision_processed_frames;
                if(mk_near_white)
                {
                    cross_near_seen = 1;
                    last_white_cm = run_distance_cm;
                }
            }
            if(run_distance_cm - last_white_cm < MK_CLEAR_CM)
                mk_clear_remaining_mm = (uint16)((MK_CLEAR_CM - (run_distance_cm - last_white_cm)) * 10);
            else mk_clear_remaining_mm = 0;
            if(cross_near_seen && run_distance_cm - last_white_cm >= MK_CLEAR_CM)
            {
                mk_state = 4;
                mk_finish_reason = 1;
            }
            else if(run_distance_cm - cross_start_cm >= MK_CROSS_LIMIT_CM)
            {
                mk_state = 4;
                mk_finish_reason = 2;  /* Distance safeguard, not verified visual clearance. */
            }
        }
    }

#if MK_ACT_ENABLE
    if(mk_state == 1)
    {
        LMotorSpeed = LMotorSpeed * MK_DIA_SPEED_PCT / 100;
        RMotorSpeed = RMotorSpeed * MK_DIA_SPEED_PCT / 100;
    }
    else if(mk_state == 2) LMotorSpeed = RMotorSpeed = MK_PARK_DUTY;
    else if(mk_state == 3)
    {
        if(LMotorSpeed > MK_CROSS_SPEED) LMotorSpeed = MK_CROSS_SPEED;
        if(RMotorSpeed > MK_CROSS_SPEED) RMotorSpeed = MK_CROSS_SPEED;
    }
#if MK_FINAL_STOP
    if(mk_state == 4) end_stopped = 1;
#endif
#endif
    if(Stop_1 && end_stop_cm > 0 && Run_Odo >= (double)end_stop_cm) end_stopped = 1;

    /* Missing camera data is always a stop condition. Lane paint may hide the
       boundaries while crossing, so only that known low-speed phase allows it. */
    vision_fault = (uint8)(!vision_processed_frames || vision_age_ticks >= VISION_STALE_TICKS ||
                          (mk_state != 3 && track_bad_ticks >= TRACK_LOSS_TICKS));
    /* IR telemetry: waiting / final park / explicit stop / camera / lane / crosswalk. */
    drive_inhibit_reason = 0;
    if(!Stop_1) drive_inhibit_reason |= 0x01;
    if(end_stopped) drive_inhibit_reason |= 0x02;
    if(Flag_stop == 1) drive_inhibit_reason |= 0x04;
    if(!vision_processed_frames || vision_age_ticks >= VISION_STALE_TICKS)
        drive_inhibit_reason |= 0x08;
    if(mk_state != 3 && track_bad_ticks >= TRACK_LOSS_TICKS)
        drive_inhibit_reason |= 0x10;
    if(MK_ACT_ENABLE && mk_state == 2) drive_inhibit_reason |= 0x20;
    drive_inhibit = (uint8)(!Stop_1 || end_stopped || Flag_stop == 1 || vision_fault ||
                           (MK_ACT_ENABLE && mk_state == 2));
#if MOTOR_TEST_MODE == 0
    if(drive_inhibit)
    {
        out_L = out_R = 0;
        ramp_cnt = 0;
    }
        /* [2026-10-28] 过横道那段软启动快 5 倍：0->18 从 3.6 秒缩到 0.7 秒 */
        else if(++ ramp_cnt >= ((MK_ACT_ENABLE && mk_state == 3) ? (MOTOR_RAMP_TICKS / 5)
                                                  : MOTOR_RAMP_TICKS))
    {
        ramp_cnt = 0;
        if(out_L < LMotorSpeed) out_L ++; else if(out_L > LMotorSpeed) out_L --;
        if(out_R < RMotorSpeed) out_R ++; else if(out_R > RMotorSpeed) out_R --;
    }
    LMotorSpeed = out_L;
    RMotorSpeed = out_R;
#elif MOTOR_TEST_MODE == 3
    if(drive_inhibit) { out_L = out_R = 0; ramp_cnt = 0; }
    else if(++ ramp_cnt >= MOTOR_RAMP_TICKS)
    {
        ramp_cnt = 0;
        if(out_L < MOTOR_TEST_TARGET_SPEED) out_L ++;
        out_R = out_L;
    }
    LMotorSpeed = out_L;
    RMotorSpeed = out_R;
#endif
    if(drive_inhibit) LMotorSpeed = RMotorSpeed = 0;
}

uint32 Tof_Get_Age_Ms(void)
{
    uint32 ticks;
    if(!tof_has_sample) return 0xFFFFFFFFUL;
    ticks = isr_tick_5ms - tof_sample_tick;
    if(ticks > 858993459UL) return 0xFFFFFFFFUL;
    return ticks * 5UL;
}

uint16 Tof_Get_Distance(void)
{
    if(!tof_has_sample || (uint32)(isr_tick_5ms - tof_sample_tick) > TOF_FRESH_TICKS ||
       tof_sample_mm < 10 || tof_sample_mm > 4000) return 8192;
    return tof_sample_mm;
}

//最小二乘法求散点拟合函数y=a+bx的参数a和b
void Least_Sqaure_Method(float *Ka, float *Kb,uint8 start, uint8 end)
{
    uint8 i;
    double Sum1=0,Sum2=0;
    double Mean_i=0,Mean_y=0;
    if(start >= 61 || end >= start || (start - end) < 2)
    {
        *Ka = 0;
        *Kb = Point_Mid;
        return;
    }
    //计算平均值
    for(i=start;i>end;i--)
    {
        Sum1 += i;
        Sum2 += Mid_Line[i];
    }
    Mean_i = 1.0*Sum1/(start-end);
    Mean_y = 1.0*Sum2/(start-end);
    //计算Ka
    Sum1=Sum2=0;
    for(i=start;i>end;i--)
    {
        Sum1+=1.0*(i-Mean_i)*(Mid_Line[i]-Mean_y);
        Sum2+=1.0*(i-Mean_i)*(i-Mean_i);
    }
    if(Sum2 <= 0)
    {
        *Ka = 0;
        *Kb = Point_Mid;
        return;
    }
    *Ka=Sum1/Sum2;
    //计算Kb
    *Kb=Mean_y - (*Ka)*Mean_i;
}

/*
*   虚拟直线
*
*   说明：构建虚拟直线，赛道中线与之比较，返回相似度
*/
int16 Fictitious_Beeline(float Ka, float Kb)
{
    uint8 i;
    int16 Sum=0;
    int16 Num;
    float res;
    for(i=59;i>5;i-=1)
    {
        res = i * Ka + Kb;
        Fictitious_Midline[i] = range_protect((int32)res, 1, COL-1);
        if (Mid_Line[i])
        {
            Num = Mid_Line[i]- Fictitious_Midline[i];
            Num = ABS(Num);
            Sum+=Num;
        }
    }
    return Sum;
}

/* ★转向灯 / 刹车灯 / 大灯（2026-10-24 重写）
   接线（实测确认，不是按位置顺序排的，别搞反）：
       P02_4 = 左前转向      P02_5 = 右前转向
       P02_6 = 左后转向      P02_7 = 右后转向
       P20_11 = 左后刹灯     P20_12 = 右后刹灯
       P20_13 = 左前大灯     P20_14 = 右前大灯   ← 大灯还没接，引脚待确认
   规则要求「同侧前后转向灯同频率闪亮」，所以按【左右】成对控制：
       左 = P02_4 + P02_6      右 = P02_5 + P02_7

   旧版本靠 Light_Flag 驱动，但 Light_Flag 的赋值全在已经关闭的 2024 状态机里，
   实际从来没被改过 → 灯一直是灭的。这里改成直接看全局状态，不依赖 Light_Flag。 */
#define LIGHT_BLINK_TICKS   (13)   /* 10ms × 13 = 130ms 半周期 → 约 3.8Hz */
#define CONE_PASS_PX        (30)   /* 从锥桶旁边 30px（约 23cm，半个道宽）通过 */
#define LIGHT_TURN_DB       (12)   /* 目标点偏离中心超过 12px 才算在转弯 */

void Light(void)
{
    static uint8 blink_cnt = 0;
    uint8 blink_on, brake, left, right;

    /* ① 没发车：全亮（规则要求开机后所有车灯亮起超过 3 秒） */
    if(!Stop_1)
    {
        gpio_set_level(P02_4, 1);
        gpio_set_level(P02_5, 1);
        gpio_set_level(P02_6, 1);
        gpio_set_level(P02_7, 1);
        gpio_set_level(P20_11, 1);
        gpio_set_level(P20_12, 1);
        gpio_set_level(P15_7, 1);
        gpio_set_level(P20_9, 1);
        blink_cnt = 0;
        return;
    }

    /* ② 闪烁节拍 */
    if(++ blink_cnt >= (LIGHT_BLINK_TICKS * 2)) blink_cnt = 0;
    blink_on = (blink_cnt < LIGHT_BLINK_TICKS);

    /* ③ 刹车灯：减速(mk_state==1) 或 停车(mk_state==2 / Flag_stop==1) */
    brake = (uint8)((1 == mk_state) || (2 == mk_state) || (1 == Flag_stop));

    /* ④ 转向灯：目标点相对画面中心偏哪边
           steer_target 大 → 目标在画面右侧 → 车往右打 → 右转向灯
           steer_target 小 → 左 */
    left = right = 0;
    if(steer_target >= 0)
    {
        if(steer_target > (int32)Point_Mid + LIGHT_TURN_DB)      right = 1;
        else if(steer_target < (int32)Point_Mid - LIGHT_TURN_DB) left  = 1;
    }

    /* ⑤ 输出 */
    gpio_set_level(P02_4,  (left  && blink_on) ? 1 : 0);   /* 左前转向 */
    gpio_set_level(P02_6,  (left  && blink_on) ? 1 : 0);   /* 左后转向 */
    gpio_set_level(P02_5,  (right && blink_on) ? 1 : 0);   /* 右前转向 */
    gpio_set_level(P02_7,  (right && blink_on) ? 1 : 0);   /* 右后转向 */

    /* 刹灯按实际接线：[2026-10-24] 左后刹 = P20_11，右后刹 = P20_12 */
    gpio_set_level(P20_11, brake ? 1 : 0);                 /* 左后刹灯 */
    gpio_set_level(P20_12, brake ? 1 : 0);                 /* 右后刹灯 */

    /* [2026-10-30] 前照灯 P15_7/P20_9：平时常亮（3V3 直驱） */
    {
        uint8 head_on = 1;
        gpio_set_level(P15_7, head_on);                    /* 左前大灯 */
        gpio_set_level(P20_9, head_on);                    /* 右前大灯 */
    }

}
