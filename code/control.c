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
#define SPEED_PERCENT               (12)   /* [2026-10-21] 10 -> 12：目标速度 21 -> 25，先快一点点 */
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
int16    Steer_Trim = 4;      // ★转向微调：正值往左，负值往右
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
void S3010_Control(void)
{
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
        /* [2026-10-22] 原来这里直接给 SteerMiddle，没带 Steer_Trim；
           微调只在发车后生效，测车时看起来"调了没用"。 */
        SD5_Duty = SteerMiddle + Steer_Trim;
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
        if((0 == Point) || (Result_Far_Row < STEER_MIN_VALID_ROWS))
            SD5_Duty = SteerMiddle + Steer_Trim;                 // ★没线时也带微调
        else
        {
            int32 duty = range_protect(PlacePID_Control(&Point_PID, Point_Mid, Point), Pwm_Min, Pwm_Max);
            duty += Steer_Trim;                                  // ★直行补偿
            if(duty < Pwm_Min) duty = Pwm_Min;
            if(duty > Pwm_Max) duty = Pwm_Max;
            SD5_Duty = duty;
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
        Run_Odo += DISTANCE(((int32)Rcount + (int32)Lcount) / 2);   // 绝对里程(cm)
   // else  Distance = 0;

    
    if(Distance_Test_ON)                    //开启距离累加
    {
        Distance_Test += (DISTANCE(Rcount+Lcount)/2 );
    }
    //else  Distance_Test = 0;
    
    Rrealspeed  = Rcount;
    Lrealspeed  =  Lcount;

//    printf("%d,%d",Rrealspeed,RMotorSpeed);   // ★原在 5ms 中断里，每秒 200 次，占用串口和中断时间


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
#define MK_DIA_SPEED_PCT    (50)      // 菱形处速度 = 原速的 50%
#define MK_GAP_CM           (60)      // 菱形消失后再走多少 cm 恢复原速
#define MK_STOP_TICKS       (1000)    // 停车 5 秒（5ms * 1000）
#define MK_PARK_DUTY        (0)       // 停车时电机输出（0 = 自然滑停）

void Point_Control(void)
{
    float res;
    int32 Sum = 0;
    uint8 point_raw;   // [2026-10-20] point_med removed
    /* [2026-10-20] remember the raw Point before the median for the loss guard */
//    if(Deal_Flag > 20 && !Dis_Flag)
//    {
//        Line_Point = 45;
//    }
//    else
        Line_Point = 20;   /* [2026-10-20] back to 20 (12 behaved worse) */

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
    LMotorSpeed = RMotorSpeed= 210;//360
    if(Flag_stop==1)//停车
    {
        LMotorSpeed = RMotorSpeed = 0;
    }
    else if(Flag_stop==2)//左转
    {
        LMotorSpeed = 135;
        RMotorSpeed = 135;
    }
    else if(Flag_stop==3)//右转
    {
         RMotorSpeed =135;
         LMotorSpeed =135;
    }
    else if(Flag_stop==4)//加速
    {
        LMotorSpeed = RMotorSpeed =135;
    }
    else if(Flag_stop==5)//低速
    {
        LMotorSpeed = RMotorSpeed =135;
    }
    else if(Flag_stop==6)//（中速）
    {
        LMotorSpeed = RMotorSpeed =175;
    }
    else if(Flag_stop==7)//（转弯前减速）
        {
            LMotorSpeed = RMotorSpeed =135;
        }
    else if(Flag_stop==8)//（转弯前减速）
            {
                LMotorSpeed = RMotorSpeed =85;
            }

    // ★统一按 SPEED_PERCENT 缩放（新增）
    LMotorSpeed = LMotorSpeed * SPEED_PERCENT / 100;
    RMotorSpeed = RMotorSpeed * SPEED_PERCENT / 100;

    // ★模式 0：软启动
    //   加速：每 MOTOR_RAMP_TICKS 个 5ms 才 +1（40 x 5ms = 200ms 一格，0->31 约 6 秒）
    //   减速 / 停车：立即生效，不做缓降（保证能随时停下）
    #if (MOTOR_TEST_MODE == 0)
        {
            static uint16 ramp_cnt2 = 0;
            static int32  out_L = 0, out_R = 0;

            /* [2026-10-20] BUG FIX: the ramp only ever went UP.
               Each 5 ms tick adds +1 to out_L/out_R whenever they are
               below the target, but nothing pulls them down when the
               target drops (Flag_stop 135 -> 85 or 0).  The car therefore
               ratcheted up to the highest target it ever saw and stayed
               there - the "sudden acceleration" and the high speed.
               Now the ramp walks towards the target in BOTH directions. */
            if(++ ramp_cnt2 >= MOTOR_RAMP_TICKS)
            {
                ramp_cnt2 = 0;
                if(out_L < LMotorSpeed)      out_L ++;
                else if(out_L > LMotorSpeed) out_L --;
                if(out_R < RMotorSpeed)      out_R ++;
                else if(out_R > RMotorSpeed) out_R --;
            }

            LMotorSpeed = out_L;
            RMotorSpeed = out_R;
        }
    #endif

    // ★模式 3：闭环测试 + 软启动，目标速度从 0 慢慢爬到设定值
    #if (MOTOR_TEST_MODE == 3)
        {
            static uint16 ramp_cnt   = 0;
            static int32  ramp_speed = 0;
            if(++ ramp_cnt >= MOTOR_RAMP_TICKS)
            {
                ramp_cnt = 0;
                if(ramp_speed < MOTOR_TEST_TARGET_SPEED) ramp_speed ++;
            }
            LMotorSpeed = ramp_speed;
            RMotorSpeed = ramp_speed;
        }
    #endif

    /* ================= [2026-10-22] 2026 标线：减速菱形 + 人行横道停车 =================
       状态：0 空闲 -> 1 慢行(看到菱形) -> 2 停车计时(到人行横道) -> 3 已通过
       只在该车放行后（Stop_1==1）才动作，避免上电摆车位置误触发。
       本函数在 5ms 中断里跑，只能做简单计数。 */
    if(Stop_1)
    {
        static uint16 mk_stop_cnt = 0;

        if(mk_state == 0 && mk_dia_seen)                 // 见到菱形 -> 慢行
        {
            mk_state = 1;
            Distance = 0;
            Distance_ON = 1;
        }
        else if(mk_state == 1)                           // 慢行中
        {
            if(mk_cw_seen)                               // 先看到人行横道 -> 停车
            {
                mk_state = 2;
                mk_stop_cnt = 0;
            }
            else if(!mk_dia_seen && Distance > MK_GAP_CM)// 菱形已过 -> 恢复原速
            {
                Distance = 0;
                Distance_ON = 0;
                mk_state = 0;
            }
        }
        else if(mk_state == 2)                           // 停车计时 5 秒
        {
            if(++ mk_stop_cnt >= MK_STOP_TICKS)
            {
                mk_state = 3;
                mk_stop_cnt = 0;
                Distance = 0;
                Distance_ON = 0;
            }
        }
        else if(mk_state == 3)                           // 已通过，横道消失后回空闲
        {
            if(!mk_cw_seen) mk_state = 0;
        }

#if MK_ACT_ENABLE
        if(mk_state == 1)
        {
            LMotorSpeed = LMotorSpeed * MK_DIA_SPEED_PCT / 100;
            RMotorSpeed = RMotorSpeed * MK_DIA_SPEED_PCT / 100;
        }
        else if(mk_state == 2)
        {
            LMotorSpeed = RMotorSpeed = MK_PARK_DUTY;
        }
#else
        (void)mk_stop_cnt;   /* 只记录模式：计时期照走，用于日志 */
#endif
    }
}

//最小二乘法求散点拟合函数y=a+bx的参数a和b
void Least_Sqaure_Method(float *Ka, float *Kb,uint8 start, uint8 end)
{
    uint8 i;
    double Sum1=0,Sum2=0;
    double Mean_i=0,Mean_y=0;
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

void Light(void)
{
    switch(Light_Flag)
    {
        case 0://刹车灯熄
        {
            gpio_set_level(P02_4,0);//左前转向
            gpio_set_level(P02_5,0);//左后转向
            gpio_set_level(P02_6,0);//右前转向
            gpio_set_level(P02_7,0);//右后转向
            gpio_set_level(P20_11,0);//左前大灯
            gpio_set_level(P20_12,0);//右前大灯
            gpio_set_level(P20_13,0);//左后刹灯
            gpio_set_level(P20_14,0);//右后刹灯
            break;
        }
        case 1://刹车灯亮
        {
            gpio_set_level(P20_13,1);//左后刹灯
                gpio_set_level(P20_14,1);//右后刹灯
            break;
        }
        case 2://左转向灯
        {
            if(Distance<40)
            {
                gpio_set_level(P02_4,1);//左前转向
                   gpio_set_level(P02_5,1);//左后转向
            }
            if(Distance>39&&Distance<80)
            {
                gpio_set_level(P02_4,0);//左前转向
                   gpio_set_level(P02_5,0);//左后转向
            }
            if(Distance>79&&Distance<120)
            {
                gpio_set_level(P02_4,1);//左前转向
                   gpio_set_level(P02_5,1);//左后转向
            }
            if(Distance>120)
            {
                gpio_set_level(P02_4,0);//左前转向
                   gpio_set_level(P02_5,0);//左后转向
            }
            break;
         }
        case 3://右转向灯
        {
            if(Distance<40)
            {
                gpio_set_level(P02_6,1);//右前转向
                    gpio_set_level(P02_7,1);//右后转向
            }
            if(Distance>39&&Distance<80)
            {
                gpio_set_level(P02_6,0);//右前转向
                    gpio_set_level(P02_7,0);//右后转向
            }
            if(Distance>79&&Distance<120)
            {
                gpio_set_level(P02_6,1);//右前转向
                    gpio_set_level(P02_7,1);//右后转向
            }
            if(Distance>119)
            {
                gpio_set_level(P02_6,0);//右前转向
                    gpio_set_level(P02_7,0);//右后转向
            }
            break;
        }
        case 4://会车灯
        {
            if(Distance_Test >50 && Distance_Test < 100)
            {
                gpio_set_level(P20_11,1);//左前大灯
                    gpio_set_level(P20_12,1);//右前大灯
            }
            else if(Distance_Test >100 && Distance_Test < 120)
            {
                gpio_set_level(P20_11,0);//左前大灯
                    gpio_set_level(P20_12,0);//右前大灯
            }
            if(Distance_Test >120 && Distance_Test < 170)
            {
                gpio_set_level(P20_11,1);//左前大灯
                    gpio_set_level(P20_12,1);//右前大灯
            }
            else if(Distance_Test >170 && Distance_Test < 200)
            {
                gpio_set_level(P20_11,0);//左前大灯
                    gpio_set_level(P20_12,0);//右前大灯
            }
            break;
        }

        case 10://刹车灯熄
        {
            gpio_set_level(P20_13,0);//左后刹灯
            gpio_set_level(P20_14,0);//右后刹灯
            break;
        }
        default:break;
    }
//    if(Light_Flag==3)//刹车灯亮
//    {
//        gpio_set_level(P20_14,1);
//        gpio_set_level(P20_13,1);
//    }
//    if(Light_Flag==4)//刹车灯灭
//    {
//        gpio_set_level(P20_14,0);
//        gpio_set_level(P20_13,0);
//    }
//    if(Flag_stop==2 || Flag_stop==6)
//    {
//        if(Light_Flag==1)//左转
//        {
//            if(Distance<10)
//            {
//                gpio_set_level(P02_5,1);
////                gpio_set_level(P02_7,1);
//            }
//            if(Distance>11&&Distance<17)
//            {
//                gpio_set_level(P02_5,0);
//             //   gpio_set_level(P02_7,0);
//            }
//            if(Distance>18&&Distance<25)
//            {
//                gpio_set_level(P02_5,1);
//               // gpio_set_level(P02_7,1);
//            }
//            if(Distance>26)
//            {
//                gpio_set_level(P02_5,0);
//               // gpio_set_level(P02_7,0);
//            }
//        }
//    }
//    if(Light_Flag==5)
//    {
//        gpio_set_level(P02_5,0);
//        gpio_set_level(P02_7,0);
//    }
//    if(Flag_stop==3)
//    {
//        if(Light_Flag==2)//右转
//        {
//            if(Distance<10)
//            {
//                gpio_set_level(P02_7,1);
//               // gpio_set_level(P02_7,1);
//            }
//            if(Distance>11&&Distance<18)
//            {
//                gpio_set_level(P02_7,0);
//               // gpio_set_level(P02_6,0);
//            }
//            if(Distance>20&&Distance<27)
//            {
//                gpio_set_level(P02_7,1);
//               // gpio_set_level(P02_4,1);
//            }
//            if(Distance>28)
//            {
//                gpio_set_level(P02_7,0);
//                //gpio_set_level(P02_6,0);
//            }
//        }
//    }
//    if(Light_Flag==6)
//    {
//        gpio_set_level(P02_4,0);
//        gpio_set_level(P02_6,0);
//    }
//    if(Light_Flag == 7)
//    {
//        if(Distance_Test >50 && Distance_Test < 100)
//        {
//            gpio_set_level(P20_12, 1);           //前右白色
//            gpio_set_level(P20_11, 1);           //前左白色
//        }
//        else if(Distance_Test >100 && Distance_Test < 120)
//        {
//            gpio_set_level(P20_12, 0);           //前右白色
//            gpio_set_level(P20_11, 0);           //前左白色
//        }
//        if(Distance_Test >120 && Distance_Test < 170)
//        {
//            gpio_set_level(P20_12, 1);           //前右白色
//            gpio_set_level(P20_11, 1);           //前左白色
//        }
//        else if(Distance_Test >170 && Distance_Test < 200)
//        {
//            gpio_set_level(P20_12, 0);           //前右白色
//            gpio_set_level(P20_11, 0);           //前左白色
//        }
//    }
}
