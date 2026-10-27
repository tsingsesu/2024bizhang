#include "zf_common_headfile.h"
#include "Mai_cone_geometry.h"

/*
*   结果变量 用于在屏幕显示
*/
uint8 result_screen;
uint8 Rhombus_Out_Flag;
uint8 aaa=0,Rhombus_Out;

uint32 time_dl1b;
uint32 time_dl1b_s;
uint8 time_dl1b_on;
uint8 mode_2024,tiaoguang;

/* [2026-10-22] 发车布防：上电到 Stop_1 变 1 之间，先连续 1 秒确认前方 400mm 内没有遮挡，
   才允许通过 time_dl1b_on 计时放行。防止 ToF 朝下/前方有杂物时，一上电就自己发车。 */
volatile uint8 start_armed = 0;

/* [2026-10-22] 发车触发：必须「连续」遮挡 ToF 2 秒。旧逻辑是累计 5 秒 + 累计 2 秒，
   手来回伸几次时间会被加起来，触发时机看起来随机。松手立刻清零。 */
volatile uint8 start_trigger = 0;
/* [2026-10-22] 非接触发车诊断：基线距离(mm, 0=空旷)、连续命中计数、是否已建立基线 */
volatile uint16 start_base = 0, start_hold = 0;
volatile uint8  start_ready = 0;
/* [2026-10-24] ToF 新样本传递：CPU1 每测到一个新距离就置位一次，
   CPU0 中断负责消费，保证每个测量样本只统计一次（不然 8192 会把样本冲掉） */
volatile uint16 tof_sample_mm  = 0;
volatile uint8  tof_new_sample = 0;

/*************************图像赛道数据****************************/
uint16 Left_Line[61], Right_Line[61], Mid_Line[61]; // 原始左右边界数据
uint16 Left_Add_Line[61], Right_Add_Line[61];       // 左右边界补线数据
uint8 Left_Add_Flag[61], Right_Add_Flag[61];        // 左右边界补线标志位
uint16 Width_Real[61];                                  // 实际赛道宽度
int16 Width_Add[61];                                    // 补线赛道宽度
int16 Width_Min;                                        // 最小赛道宽度

uint8 Left_Add_Start, Right_Add_Start;  // 左右补线起始行坐标
uint8 Left_Add_Stop, Right_Add_Stop;    // 左右补线结束行坐标

uint8 Left_Line_Start, Right_Line_Start;  //  左右 边界线 实线起始点
uint8 Left_Line_Break, Right_Line_Break;   //  左右 边界线 断点

int8 Out_Side = 0;  // 丢线控制
int8 Line_Count;        // 记录成功识别到的赛道行数
uint8 Result_Far_Row = 59;   // [2026-10-22] 本帧最远可信行（整帧没线时贴近 5）

int Count=0;
uint8 Last_Point;
uint8 turnleft;
/***********************赛道斜率************************/
float Left_Ka = 0, Right_Ka = 0;
float Left_Kb = 1, Right_Kb = (COL - 1);    //  最小二乘法参数
float Mid_qulv,Right_qulv,Left_qulv;

/************环岛标志 *****************/
uint8 LDrop_Line_Num , RDrop_Line_Num;  // 左右 连续 丢边 数量 2020.5.26
uint8 Left_Linear_Flag, Right_Linear_Flag;     //  左右 边界线 是否 是 直线 的标志 1 为 直线

uint8 Left_Ring_Flag,Right_Ring_Flag;    //  进行环岛识别的标志
uint8 Ring_Flag= 0;
uint8 Flag1=0;          //0直道  1 環島底部  2 環島第二入口  3 環島中部
uint8 Dirc;           //1左  2右
uint8 Ring_Enter = 0;
uint8 Ring_Enter1 = 0;
uint8 Cycle_Flag;
/************停车标志 *****************/
uint8 Flag_stop;
uint8 Stop_Out;;
uint8 stop,Stop_1;
uint8 Start_Line_Flag,Stop_Flag;
uint8 white_count,n,k,j,a,b,c,i,a1,b1;
uint8 Zuo_Flag,Zhuan_Flag,Left_Flag1,Right_Flag1,Stop1,Right_Flag2;
uint8 Flag_turn;
uint8 Deal_Flag,Time_banmaxian,Time_banmaxian_Flag,Dis_Flag,Measurement;
uint16 Distances = 120;
uint8 Guaidian;
uint8 Rhombus_Flag;
uint8 White_Flag;
uint8 Right_Max,Left_Min,Right_Flag,Left_Flag,Right_Flag1;
uint8 zuobianjie_Flag,youbianjie_Flag;
uint8 Cone_bucket,Lin_Flag,rhombus_Flag,rhombus_Flag1;
uint32 distance_0,distance_1,distance_2;                //转换出来的距离  单位为厘米
uint8 Distance_Left_Flag,Distance_Right_Flag;
uint8 Stadiometry_Flag1,Stadiometry_Flag2,Stdiometry_Flag3;
uint8 Light_Flag,Light_Flag1;
/* ===== [2026-10-20] reflection (glare) suppression, see Img_analyse.h ===== */
uint8 agl_rejected_rows = 0;   // diag: rows rejected because white runs looked like glare
uint8 agl_blob_pair     = 0;   // diag: rows that contained two glare-sized white runs

/* ================= [2026-10-22] 2026 道路标线识别 ================= */
/* 赛道是双车道 45cm，车在一条车道里跑，摄像头 120x70。
   只看图像中央一条竖带（避开两侧边线和中线）：
     - 人行横道：50cm 白条横跨车道，一行里白点很多，且有间隔地出现多行
     - 减速菱形：车道中央的细白斜线，一行里白点很少，但连续出现很多行
   行号 0 在最上面（远），59 在下面（近车头）。 */
#define MARKER_EN               (1)
#define MK_BAND_L               (36)    // 中央带左列
#define MK_BAND_R               (84)    // 中央带右列
#define MK_ROW_MIN              (8)     // 扫描最远行
#define MK_ROW_MAX              (58)    // 扫描最近行
#define MK_CW_RUN_MIN           (22)    // 连续白带 >= 此宽度 = 人行横道的一行（灰底，必须有间断的黑色缝）
#define MK_CW_ROW_MIN           (22)    // 同一行白点数下限
#define MK_CW_MIN_ROWS          (4)     // 至少几行白带才判人行横道（停车线只有 1~2 行）
#define MK_THIN_MIN             (2)     // 细白线最少宽度(px)
#define MK_THIN_MAX             (10)    // 细白线最大宽度(px)
#define MK_DIA_MIN_PAIRS        (2)     // 同一行至少几条细白线（菱形两条斜边，中线只有一条）
#define MK_DIA_ROW_MAX          (50)    // 菱形行白点数上限（再大就是横道/反光）
#define MK_DIA_MIN_ROWS         (6)     // 至少几行才判菱形
#define MK_DIA_MAX_ROWS         (40)    // 超过这个行数不算菱形（防止整屏误判）
#define MK_CW_STOPLINE_ROW      (52)    // 近车头 52 行以后出现单条白带 = 停车线
#define MK_CW_SPAN_PCT          (60)   // [2026-10-26] 连续白带要占到本行车道宽的百分之多少才算人行横道；误杀就调小，误判就调大
#define MK_CW_MIN_LANE_W        (30)   // [2026-10-26] 本行车道宽小于它就跳过这一行（边界不可信，判了也是错的）

volatile uint8 mk_cw_seen = 0,  mk_cw_near  = 0, mk_cw_score  = 0;   // 人行横道
volatile uint8 mk_dia_seen = 0, mk_dia_near = 0, mk_dia_score = 0;   // 减速菱形
volatile uint8 mk_state = 0;                                        // 0空闲 1慢行 2停车计时 3已通过

void Marker_Scan(uint8 *data)
{
    uint8 i, j, j0, w;
    uint8 cw_rows = 0, dia_rows = 0;
    uint8 cw_near = 0, dia_near = 0;
    uint8 wc, run_max, pairs;

    /* [2026-10-22] 整幅过曝(>190)或过暗(<10)时二值图没有意义，
       起跑瞬间 iavg=205 会让人行横道误报。 */
    if(diedai_img_avg > 190 || diedai_img_avg < 10)
    {
        mk_cw_seen = 0;  mk_cw_near = 0;  mk_cw_score = 0;
        mk_dia_seen = 0; mk_dia_near = 0; mk_dia_score = 0;
        return;
    }

    for(i = MK_ROW_MIN; i <= MK_ROW_MAX; i ++)
    {
        /* [2026-10-26] 原来固定扫 MK_BAND_L~MK_BAND_R（36~84）这 49 列，
           不管车道在哪，也不管这一行到底有多宽。结果自制锥桶近看时
           一条 20+px 的连续白带就满足 run_max>=22 和 wc>=22，
           直接被人行横道判据吃下 -> mk_state=2 -> 停车 5 秒，车死在障碍前面。

           改成【相对本行车道】来判断：
             人行横道 = 一条白带横跨整条车道（>= MK_CW_SPAN_PCT% 车道宽，且盖住车道中心）
             锥桶     = 一条窄白带，只占车道的一小段，盖不住中心
           本行车道宽不可信（边界贴边/丢线）就跳过这一行，宁可不判也不误判。 */
        uint16 lx = Left_Line[i];
        uint16 rx = Right_Line[i];
        uint8  c0, c1, lane_w;
        uint8  run_s = 0, run_e = 0;
        uint8  thin_l = 255, thin_r = 0;
        uint8  has_c = 0;

        wc = 0; run_max = 0; pairs = 0; j0 = 0; w = 0;

        if(lx < 2 || rx > (COL - 3) || rx <= lx + MK_CW_MIN_LANE_W)
            continue;                       /* 本行边界不可信 */

        c0     = (uint8)(lx + 1);
        c1     = (uint8)(rx - 1);
        lane_w = (uint8)(c1 - c0 + 1);

        for(j = c0; j <= c1; j ++)
        {
            if(data[i * COL + j])
            {
                wc ++;
                w ++;
                if(w == 1) j0 = j;
            }
            else if(w)
            {
                if(w >= MK_THIN_MIN && w <= MK_THIN_MAX)
                {
                    pairs ++;
                    if(j0 < thin_l)   thin_l = j0;
                    if(j  > thin_r)   thin_r = (uint8)j;
                }
                if(w > run_max) { run_max = w; run_s = j0; run_e = (uint8)(j - 1); }
                w = 0;
            }
        }
        if(w)                                   /* 行尾还有一段白 */
        {
            if(w >= MK_THIN_MIN && w <= MK_THIN_MAX)
            {
                pairs ++;
                if(j0 < thin_l) thin_l = j0;
                if(c1 > thin_r) thin_r = c1;
            }
            if(w > run_max) { run_max = w; run_s = j0; run_e = c1; }
        }

        /* 最长白带有没有盖住车道中心列 */
        if(run_max)
        {
            uint8 cx = (uint8)((c0 + c1) / 2);
            if(cx >= run_s && cx <= run_e) has_c = 1;
        }

        /* 人行横道：连续白带横跨本车道，且盖住车道中心 */
        if(run_max >= (uint8)(((uint16)lane_w * MK_CW_SPAN_PCT) / 100) && has_c)
        {
            cw_rows ++;
            cw_near = i;
        }
        /* 菱形：同一行至少两条细白线，且左右拉得够开（横跨半个车道以上）。
           一排锥桶的细白段是挤在一起的，跨不了这么宽。 */
        else if(pairs >= MK_DIA_MIN_PAIRS && wc <= MK_DIA_ROW_MAX &&
                thin_r > thin_l && (uint16)(thin_r - thin_l) >= (uint16)(lane_w / 2))
        {
            dia_rows ++;
            dia_near = i;
        }
    }

    
    mk_cw_score  = cw_rows;
    mk_dia_score = dia_rows;
    /* [2026-10-26] 这里原来有个「只看到 1~3 行就当成停车线、直接升格成横道」的捷径。
       实测 O=3351 那次 cw_rows 只有 3 行（日志 CW=0/0/3）就被升格 ->
       mk_state=2 -> 停 5 秒，车死在障碍前面。
       横道本身有 MK_CW_MIN_ROWS 行以上，靠近时 cw_near 也一定会涨过 MK_STOP_ROW，
       不需要这个捷径。 */
    mk_cw_seen   = (cw_rows >= MK_CW_MIN_ROWS);
    mk_dia_seen  = (dia_rows >= MK_DIA_MIN_ROWS && dia_rows <= MK_DIA_MAX_ROWS);
    mk_cw_near   = mk_cw_seen  ? cw_near  : 0;
    mk_dia_near  = mk_dia_seen ? dia_near : 0;
}

/* ================= [2026-10-26] 终点线探测（只记录，不动作）=============
   和停用的 Find_Stop() 用同一个窗口（第 30~65 行、第 10~60 列数白点），
   但这里每帧都算，而且【只置标志、不改车速】。
   目的：先跑一整圈看 stop_latch 会不会在终点以外的地方变 1，
   确认判据靠得住，再把真正的停车接上去。
   stop_latch 一旦置 1 就不再复位的 —— 这正是要观察的东西：
   如果它在中途就变 1，说明真开着的话车会在半路停死。 */
volatile uint16 stop_aaa   = 0;   /* 窗口里的白点数（越大越不像终点）*/
volatile uint16 stop_run   = 0;   /* 连续多少帧 A<2（中途最长只有 3 帧，终点处应该会很长）*/
volatile uint8  stop_now   = 0;   /* 本帧 aaa<2 */
volatile uint8  stop_latch = 0;   /* 本遍里是否出现过 aaa<2（不复位）*/

void Stop_Probe(uint8 *data)
{
    uint16 i, j, n = 0;

    for(i = 30; i <= 65 && i < ROW; i ++)
        for(j = 10; j <= 60 && j < COL; j ++)
            if(data[i * COL + j]) n ++;

    stop_aaa = n;
    stop_now = (n < 2);
    if(stop_now) { if(stop_run < 60000) stop_run ++; }
    else           stop_run = 0;
    if(stop_now) stop_latch = 1;
}

/* ================= [2026-10-24] 锥形标识别 v3 =================
   锥桶是红色，灰度 ≈76 > 二值阈值 → 二值图里是【实心白块】，和车道线一样白，
   所以只能靠【横向宽度】区分：
       车道线 = 2cm ≈ 2~3px（细）
       锥桶   = 6~9cm，远处 5px、贴着车会到 30~40px（粗）

   v3 修了两个 bug：
     1) v2 的计数是"上一行没有宽白段才算新锥桶"，但每行都有几十个宽白段，
        于是 CN 永远卡在 1~2。改成真正的二维 blob 跟踪：同列、相邻行才合并。
     2) 只在【车道内部】扫（左右边界之间），避免抓赛道外的白色区域
        （v2 的 cone_x 一直卡在 108，而右边界才 100）。

   cone_wh[] 宽度直方图：[0]<=4  [1]5~10  [2]11~18  [3]19~30  [4]>30
   CONE_ACT_ENABLE = 0：只统计 + 画点，不动舵机。 */
#define CONE_ROW_MIN     (14)
#define CONE_ROW_MAX     (56)
#define CONE_COL_MIN     (6)
#define CONE_COL_MAX     (111)
#define CONE_W_MIN       (5)
#define CONE_W_MAX       (40)
#define CONE_LINK_X      (10)    /* 相邻行横向偏差 <=10px 算同一个锥桶 */
#define CONE_NEAR_ROW    (36)    /* 这一行以后算"近端"锥桶，绕行方向用它决定 */
#define CONE_MIN_ROWS    (1)     /* blob 至少占 1 行：锥桶白箍只有 1cm 高，二值化后就是 1 行，要求 2 行等于全扔 */
#define CONE_MAX_BLOBS   (12)
#define CONE_MAX_PTS     (48)

volatile uint8  cone_cnt  = 0;
volatile uint8  cone_near = 0;
volatile int16  cone_x    = -1;
volatile uint8  cone_ok   = 0;
volatile uint8 cone_group_ok = 0;
volatile uint8 cone_group_support = 0, cone_group_span_x = 0, cone_group_span_y = 0, cone_group_error = 0;
volatile uint16 cone_wh[5] = {0, 0, 0, 0, 0};

uint8 cone_px[CONE_MAX_PTS];
uint8 cone_py[CONE_MAX_PTS];
volatile uint8 cone_pts = 0;

void Cone_Scan(uint8 *data)
{
    uint8 cbx[CONE_MAX_BLOBS];      /* blob 最近一行的中心 x */
    uint8 cbr[CONE_MAX_BLOBS];      /* blob 最近命中的行 */
    uint8 cbn[CONE_MAX_BLOBS];      /* blob 累计命中行数   */
    uint8 nb = 0, i, j, w, cx, b;
    uint8 n = 0, best_row = 0;
    int16 best_x = -1;
    Mai_Cone_Group group;

    cone_cnt = 0; cone_near = 0; cone_x = -1; cone_pts = 0;
    cone_ok = cone_group_ok = 0;
    cone_group_support = cone_group_span_x = cone_group_span_y = cone_group_error = 0;
    cone_wh[0] = cone_wh[1] = cone_wh[2] = cone_wh[3] = cone_wh[4] = 0;

    /* [2026-10-24] 这里原来抄了 Marker_Scan 的 "iavg<10 就放弃"，
       但这车正常画面 iavg 就是 4 左右（大面积黑赛道），
       结果 79% 的帧被误杀，锥桶识别率只有 13%。去掉了。
       过曝保护保留：整片白的时候白段会连成一大坨，判不准。 */
    if(diedai_img_avg > 190)
    {
        cone_ok = 0;
        return;
    }

    for(i = CONE_ROW_MIN; i <= CONE_ROW_MAX; i ++)
    {
        /* 只在车道内部扫，避免把赛道外的白色地板当成锥桶。
           但边界本身可能是假的：实测出现过 R=31（车道只剩 23px），
           这时如果照着裁，扫描窗口被压成一条缝，CWD 会全是 0。
           所以只在边界"看起来像真的"时才裁：
             左边界必须落在画面左半边，右边界必须落在右半边，且宽度 > 25px。 */
        uint8 c0 = CONE_COL_MIN;
        uint8 c1 = CONE_COL_MAX;

        if(Left_Line[i] > 2 && Left_Line[i] < (COL / 2))
            c0 = (uint8)(Left_Line[i] + 2);
        if(Right_Line[i] < COL - 3 && Right_Line[i] > (COL / 2))
            c1 = (uint8)(Right_Line[i] - 2);

        if(c0 < CONE_COL_MIN) c0 = CONE_COL_MIN;
        if(c1 > CONE_COL_MAX) c1 = CONE_COL_MAX;
        /* [2026-10-26] 门槛 25 -> 50。
           刚发车时车道读数还不稳（实测 T=41 那帧 L=29 R=71 W=42，真实是 76），
           窗口被裁成 [31,69]，而锥桶在 cone_x=87 —— 在窗口外，根本扫不到。
           这就是「第一次障碍识别比后面差、有时会撞」的原因：
           触发前 40 帧里 CN>=2 只有 5 帧，后面几次都是 10~16 帧。
           真实车道宽约 76，裁完还有 72，不会被这个门槛误退。 */
        if(c1 < c0 + 50)                /* 窗口太窄说明边界不可信 → 退回全宽 */
        {
            c0 = CONE_COL_MIN;
            c1 = CONE_COL_MAX;
        }

        j = c0;
        while(j < c1)
        {
            if(data[i * COL + j])
            {
                uint8 x0 = j;
                w = 0;
                while(j < c1 && data[i * COL + j])
                {
                    j ++;
                    w ++;
                }

                if(w <= 4)          cone_wh[0] ++;
                else if(w <= 10)    cone_wh[1] ++;
                else if(w <= 18)    cone_wh[2] ++;
                else if(w <= 30)    cone_wh[3] ++;
                else                cone_wh[4] ++;

                if(w >= CONE_W_MIN && w <= CONE_W_MAX)
                {
                    cx = (uint8)(x0 + w / 2);

                    /* 找上一行、横向附近的 blob 合并 */
                    for(b = 0; b < nb; b ++)
                    {
                        int16 dx = (int16)cx - (int16)cbx[b];
                        if(dx < 0) dx = -dx;
                        if(cbr[b] + 1 == i && dx <= CONE_LINK_X) break;
                    }

                    if(b < nb)
                    {
                        cbx[b] = cx;
                        cbr[b] = i;
                        if(cbn[b] < 200) cbn[b] ++;
                    }
                    else if(nb < CONE_MAX_BLOBS)
                    {
                        cbx[nb] = cx;
                        cbr[nb] = i;
                        cbn[nb] = 1;
                        nb ++;
                    }

                    /* 不再逐点记录：逐点会有几百个，SPI 画不过来。
                       只在最后把"有效 blob"的中心存下来（最多 12 个）。 */
                }
            }
            else
            {
                j ++;
            }
        }
    }

    /* 够高的 blob 才算锥桶。
       [2026-10-24] cone_x 原来取"最靠下那个 blob"的 x，但白箍是碎片化的(1 行一个)，
       "最近的那个"每帧都在跳：实测相邻帧 61 -> 102 -> 79 -> 37。
       绕行的方向判断完全靠它，于是一会儿左满舵一会儿右满舵。
       改成【所有有效 blob 的平均 x】= 整组锥桶的中心，稳定得多。 */
    {
        int32 sum_all = 0, sum_near = 0;
        uint8 cnt_all = 0, cnt_near = 0;

        for(b = 0; b < nb; b ++)
        {
            if(cbn[b] >= CONE_MIN_ROWS)
            {
                n ++;
                sum_all += (int32)cbx[b];
                cnt_all ++;

                /* 靠下（离车近）的那一段单独统计 */
                if(cbr[b] >= CONE_NEAR_ROW)
                {
                    sum_near += (int32)cbx[b];
                    cnt_near ++;
                }

                if(cbr[b] >= best_row) best_row = cbr[b];

                if(cone_pts < CONE_MAX_PTS)
                {
                    cone_px[cone_pts] = cbx[b];
                    cone_py[cone_pts] = cbr[b];
                    cone_pts ++;
                }
            }
        }

        /* [2026-10-24] 优先用"近端"的锥桶位置。
           锥桶是一组斜跨两条道的斜线，车要跟着斜线走，
           所以方向应该由【马上要经过的那几个】决定，而不是整组的平均值。
           近端找不到再退回整组平均。 */
        /* [2026-10-26] 锥桶是【分两排】摆的：一排在某条车道靠前、另一排靠后。
           原来把"所有近端 blob"取平均，两排同时在视野里就被平均成中间值，
           方向判据拿到的是一个谁都不像的 x —— 实测相邻帧在左右之间翻转 57 次。
           现在只取【离车最近的那一排】（最靠下的那几行）的平均。 */
        if(nb)
        {
            int32 sum_deep = 0;
            uint8 cnt_deep = 0;

            for(b = 0; b < nb; b ++)
            {
                if(cbn[b] >= CONE_MIN_ROWS && (cbr[b] + 4) >= best_row)
                {
                    sum_deep += (int32)cbx[b];
                    cnt_deep ++;
                }
            }

            if(cnt_deep >= 2)
            {
                best_x = (int16)(sum_deep / (int32)cnt_deep);
                cone_near = best_row;
            }
            else if(cnt_near) best_x = (int16)(sum_near / (int32)cnt_near);
            else if(cnt_all)  best_x = (int16)(sum_all  / (int32)cnt_all);
        }
    }

    cone_cnt  = n;
    cone_near = best_row;
    cone_x    = best_x;
    /* V25: two white fragments are insufficient evidence of a cone group.
       Preserve raw CN/points, but only confirm a distributed diagonal group. */
    cone_group_ok = Mai_Cone_Group_Valid(cone_px, cone_py, cone_pts, &group);
    cone_group_support = group.support;
    cone_group_span_x = group.span_x;
    cone_group_span_y = group.span_y;
    cone_group_error = group.error;
    cone_ok = cone_group_ok;
}

static uint8 agl_run_limit(uint8 i)
{
    return (i >= AGL_NEAR_ROW) ? AGL_RUN_NEAR : AGL_RUN_FAR;
}

static uint8 agl_jump_limit(uint8 i)
{
    return (i >= AGL_NEAR_ROW) ? AGL_EDGE_JUMP_NEAR : AGL_EDGE_JUMP_FAR;
}

/* Width of the white run that starts at column j (0 if data[j] is black). */
static uint8 agl_run_width(uint8 *data, uint8 i, uint8 j)
{
    uint8 w = 0;
    if(j >= COL || !data[i*COL + j]) return 0;
    while((uint16)(j + w) < COL && data[i*COL + j + w]) w ++;
    return w;
}

/* Count white px strictly between the two lane edges. */
uint8 LinkCnt_Valid_Rows(uint8 *data, uint8 i)
{
    uint8 j, n = 0;
    for(j = Left_Line[i] + 1; j + 1 < Right_Line[i]; j ++)
    {
        if(data[i*COL + j]) n ++;
    }
    return n;
}

/* Count white runs wider than the glare limit inside [c0,c1]. */
static uint8 agl_blob_count(uint8 *data, uint8 i, uint8 c0, uint8 c1)
{
    uint8 j, n = 0, in = 0;
    if(c0 >= COL) c0 = COL - 1;
    if(c1 >= COL) c1 = COL - 1;
    for(j = c0; j <= c1; j ++)
    {
        if(!in)
        {
            if(data[i*COL + j])
            {
                in = 1;
                if(agl_run_width(data, i, j) > agl_run_limit(i)) n ++;
            }
        }
        else if(!data[i*COL + j]) in = 0;
    }
    return n;
}

/* Replace an implausible edge by prediction from the row below. */
static uint8 agl_sanitize_edge(uint16 *Line, uint8 i, uint8 near_side)
{
    int16 pred, diff;
    (void)near_side;
    if(i + 1 > 58) return 0;                     // 59/60 are uninitialised defaults
    pred = (int16)Line[i+1] + ((int16)Line[i+1] - (int16)Line[(i + 2 > 58) ? 58 : (i + 2)]);
    if(pred < 1)  pred = 1;
    if(pred > COL-2) pred = COL-2;
    diff = (int16)Line[i] - pred;
    if(diff < 0) diff = -diff;
    if(diff > (int16)agl_jump_limit(i))
    {
        Line[i] = (uint16)pred;
        return 1;
    }
    return 0;
}

/* Row-level glare pass, runs after Image_Handle() has built Left/Right_Line.
   Only rows Image_Handle actually produced (Line_Count .. 59) are touched. */
/* [2026-10-20] snapshot of Left/Right/Mid as Image_Handle produced them,
   captured before Glare_Suppress touches anything.  Used to draw both
   versions on page1 (red/blue = before, green/yellow = after). */
uint16 agl_L_before[61];
uint16 agl_R_before[61];
uint16 agl_M_before[61];

void agl_capture_before(void)
{
    uint8 k;
    for(k = 0; k < 61; k ++)
    {
        agl_L_before[k] = Left_Line[k];
        agl_R_before[k] = Right_Line[k];
        agl_M_before[k] = Mid_Line[k];
    }
}

void Glare_Suppress(uint8 *data)
{
    uint8 i, changed;
    agl_capture_before();
    agl_rejected_rows = 0;
    agl_blob_pair = 0;
#if AGL_EN
    for(i = 57; i >= 1; i --)              // near -> far; row 58 handled by the blob test only
    {
        if(i < Line_Count) continue;       // rows above Line_Count were not produced
        changed = 0;
        if((uint16)Right_Line[i] <= (uint16)(Left_Line[i] + 4))
        {
            changed = 1;
        }
#if AGL_CROSS_GUARD
        /* [2026-10-20] reflection in the image centre makes both scans
           stop at the same blob, giving L == R (or L > R).  Only the
           absolute geometry is trusted: the lane centre must be below
           the image centre in a row, and the two edges must stay apart. */
        else if((uint16)Right_Line[i] <= (uint16)(Left_Line[i] + 12))
        {
            changed = 1;
        }
#endif
        else
        {
            // [2026-10-20] order matters: a wide white blob makes the first
            // transition a SPURIOUS edge, so run the blob test before the
            // jump test.  The jump test is only a fallback for narrow blobs.
            {
                uint8 interior = LinkCnt_Valid_Rows(data, i);
                (void)interior;
                if(interior > AGL_MAX_INTERIOR) changed = 1;       // floor turned white
            }
            if(!changed)
            {
                changed |= agl_sanitize_edge(Left_Line,  i, 1);
                changed |= agl_sanitize_edge(Right_Line, i, 0);
            }
        }
        if(changed)
        {
            agl_rejected_rows ++;
            if(i + 1 <= 60)
            {
                Left_Line[i]       = Left_Line[i+1];
                Right_Line[i]      = Right_Line[i+1];
                Left_Add_Line[i]   = Left_Line[i];
                Right_Add_Line[i]  = Right_Line[i];
                Left_Add_Flag[i]   = 0;
                Right_Add_Flag[i]  = 0;
            }
        }
        else if(agl_blob_count(data, i, Left_Line[i] + 1, Right_Line[i] - 1) >= 2)
        {
            agl_blob_pair ++;
        }
        Width_Real[i] = Right_Line[i] - Left_Line[i];
        Width_Add[i]  = Width_Real[i];
        Mid_Line[i]   = (Right_Line[i] + Left_Line[i]) / 2;
    }
#else
    (void)data;
#endif
}

uint8 Num_Turn_L,Num_Turn_R;
int32 Time_Cal;
/*
 *
 *
*   图像算法参数初始化
*
*   说明：仅影响第一行特殊处理
*/
void Image_Para_Init(void)
{
    Mid_Line[60] = 60;
    Left_Line[60] = 2;
    Right_Line[60] = (COL - 2);
    Left_Add_Line[60] = 2;
    Right_Add_Line[60] = (COL - 2);
    Width_Real[60] = (COL - 2);
    Width_Add[60] = (COL - 2);
}
/*
*   图像处理算法
*
*   说明：处理普通图像，包括十字、障碍
*/
void Image_Handle(uint8 *data)
{       //变量设置
    uint8 i;    // 控制行
    uint8 j;
    uint8 res;
    Line_Count = 0; // 赛道行数复位
    Width_Min = COL-1;
    //        Count++;     //  不知道用来干嘛  我用来 时间计数
    //=====================================================
    Stop1=1;
    Left_Add_Start = 0;  // 左复位补线起始行坐标
    Right_Add_Start = 0; // 右复位补线起始行坐标
    Left_Add_Stop = 0;   // 左补线终止行坐标
    Right_Add_Stop = 0;  // 右补线终止行坐标

    Left_Line_Start = 0;   //左右 边界线 实线起始点
    Right_Line_Start = 0;
    Left_Line_Break = 0;   //  左右 边界线 断点
    Right_Line_Break = 0;

    LDrop_Line_Num = 0;    // 左右 连续 丢边 数量
    RDrop_Line_Num = 0;
    //Out_Side = 0;         // 丢线控制
    Line_Count = 59;      // 记录成功识别到的赛道行数
    /***************************** 第一行特殊处理 *****************************/
    /* [2026-10-22] Right_Flag 一旦被置 1 就再也没人清零（zuobianjie/youbianjie
       里置 1，但没有任何函数调用它们），所以从第一帧起 First_Line_Handle 就
       不再执行，res 永远是未初始化值。原来这句是 `if (res == 0)`，等于在读
       栈上的垃圾：值碰巧是 0 的这一帧直接 return，什么都不更新，Point 一直
       停在旧值——表现就是偶尔丢帧后舵机卡住不跟线。
       改成「只有真的判断了第一行、且判断失败」才 return。 */
    if(!Right_Flag)
    {
        res = First_Line_Handle(data);
    }
    else
    {
        res = 1;
    }
    if (res == 0)
    {
        //            if (g_fDirectionSum[0] + g_fDirectionSum[1] < 50 )
        {
            //                        Out_Side = 1;
        }  // 丢线 停车#

        return ;
    }
    /*************************** 第一行特殊处理结束 ***************************/
    for (i = 59;i >= 5;)
    {
        i -= 1;
        if (data[(i-2)*COL + Mid_Line[i+1]] && data[(i-1)*COL + Mid_Line[i+1]]) //  前行中点在本行为黑点，可能是赛道结束，也可能是环路
        {
            if(i <= 50)     //  防止扫的线行数太少
                break;
        }
        else  // 使用前2行中点向两边扫描边界
        {
            Traversal_Mid_Line(i, data, Mid_Line[i+1], 1, (COL-3), Left_Line, Right_Line, Left_Add_Line, Right_Add_Line);
        }
        /**************************** 补线检测开始 ****************************/
        if (Width_Real[i] > Width_Min+10)//&&(Left_Add_Flag[i]||Right_Add_Flag[i]))   // 赛道宽度变宽，可能是十字
        {
            if (Left_Add_Line[i] <  Left_Add_Line[i+1])
            {
                if (!Left_Add_Flag[i])
                {
                    Left_Add_Flag[i] = 1;       // 强制认定为需要补线
                }
            }
            if (Right_Add_Line[i] > Right_Add_Line[i+1])
            {
                if (!Right_Add_Flag[i])
                {
                    Right_Add_Flag[i] = 1;       // 强制认定为需要补线
                }
            }
        }

        /******************** 第二轮 补线检测开始 ******************************/
        if (Left_Add_Line[i] > Left_Add_Line[i+1]+10 && !Left_Add_Flag[i+1])  // 如果这次的点 距离上一个点位置太远 且上一个点不要补线
        {
            if (!Left_Add_Flag[i])
            {
                Left_Add_Flag[i] = 1;       // 强制认定为需要补线
            }
        }
        if (Right_Add_Line[i] < Right_Add_Line[i+1]-10 && !Right_Add_Flag[i+1])  // 如果这次的点 距离上一个点位置太远 且上一个点不要补线
        {
            if (!Right_Add_Flag[i])
            {
                Right_Add_Flag[i] = 1;       // 强制认定为需要补线
            }
        }

        /*************************** 第一轮补线开始 ***************************/
        if (Left_Add_Flag[i])   // 左侧需要补线
        {
            if (i > 10 && i < 53)
            {
                LDrop_Line_Num++;
                // 左丢线数目
            }
            if (i >= 55)    // 前5行补线不用计算
            {
                if (!Left_Add_Start)
                {
                    Left_Ka = 0;
                    Left_Kb = Left_Add_Line[i];
                }
                Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb);
            }
            else
            {
                if (!Left_Add_Start)    // 之前没有补线
                {
                    Left_Add_Start = i+1; // 记录左侧补线开始行
                }
                if (Left_Line_Break && (Left_Line_Start -Left_Line_Break) >= 5)    // 如果 左断点存在 用起始点斜率补线
                {
                    Curve2_Fitting(&Left_Ka, &Left_Kb,Left_Line_Start,Left_Line_Break,Left_Add_Line,1);
                }
                else
                    Curve_Fitting(&Left_Ka, &Left_Kb, &i, Left_Add_Line, Left_Add_Flag, 1);    //  补线
                Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb);  // 补线完成
            }
        }
        else   //  不需要补线  记下补线结束行  用来最后进行两点补线
        {
            if (!Left_Line_Start  && i > 50 && i < 56)    // 记下 左实际边界线的起点
            {
                Left_Line_Start = i;
            }
            if (Left_Add_Start) // 已经开始补线
            {
                if (!Left_Add_Stop && !Left_Add_Flag[i+1]) // && !Left_Add_Flag[i+2])
                {
                    if ((Left_Add_Line[i] > Left_Add_Line[i+1])) // && (Left_Add_Line[i+1] > Left_Add_Line[i+2]))
                    {
                        Left_Add_Stop = i+1;        // 记录左侧补线结束行
                    }
                }
            }
        }

        if (Right_Add_Flag[i])  // 右侧需要补线
        {
            if (i > 10 && i<53)
            {
                RDrop_Line_Num++;
            }
            if(Right_Flag) break;
            if (i >= 55)    // 前三行补线不算
            {
                if (!Right_Add_Start)
                {
                    Right_Ka = 0;
                    Right_Kb = Right_Add_Line[i];
                }

                Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);   // 使用前一帧图像右边界斜率补线
            }
            else
            {
                if (!Right_Add_Start)       // 之前没有补线
                {
                    Right_Add_Start = i+1;  // 记录右侧补线开始行
                }
                if (Right_Line_Break && (Right_Line_Start - Right_Line_Break) >= 5)    // 如果 左断点存在 用起始点斜率补线
                {
                    Curve2_Fitting(&Right_Ka, &Right_Kb,Right_Line_Start,Right_Line_Break,Right_Add_Line,2);
                }
                else
                    Curve_Fitting(&Right_Ka, &Right_Kb, &i, Right_Add_Line, Right_Add_Flag, 2);   // 使用两点法拟合直线

                Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);// 补线完成
            }
        }
        else     //  不需要补线  记下补线结束行  用来最后进行两点补线
        {
            if (!Right_Line_Start)    // 记下 左实际边界线的起点
            {
                Right_Line_Start = i;
            }
            if (Right_Add_Start)    // 已经开始补线
            {
                if (!Right_Add_Stop && !Right_Add_Flag[i+1]) // && !Right_Add_Flag[i+2])
                {
                    if ((Right_Add_Line[i] < Right_Add_Line[i+1]))// && (Right_Add_Line[i+1]<Right_Add_Line[i+2]))
                    {
                        Right_Add_Stop = i+1; // 记录右侧补线结束行
                    }
                }
            }
        }
        /*************************** 第一轮补线结束 ***************************/
        Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];        // 重新计算赛道宽度
        Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;   // 计算中线

        if (Width_Add[i] < Width_Min) //&& !Left_Add_Flag[i] && !Right_Add_Flag[i])
        {
            Width_Min = Width_Add[i];                               // 更新最小赛道宽度
        }
        Line_Count = i;                                            //     记录成功识别到的赛道行数
        if(Left_Add_Stop && Right_Add_Stop)                      //补线结束后又要补线这不补
        {
            if(Left_Add_Flag[i]||Right_Add_Flag[i])
            {
                Line_Count = i+2;
                break;
            }
        }
        //             if(Right_Add_Line[i] < 5)
        //                 Right_Add_Line[i] = Right_Add_Line[i+1];
    }//for結束



    /*************************** 第二轮补线修复开始 ***************************/
    if (Left_Add_Stop)      // 左边界需要补线
    {
        Line_Repair(Left_Add_Start, Left_Add_Stop, data, Left_Line, Left_Add_Line, Left_Add_Flag, 1);
    }
    if (Right_Add_Stop) // 右边界需要补线
    {
        Line_Repair(Right_Add_Start, Right_Add_Stop, data, Right_Line, Right_Add_Line, Right_Add_Flag, 2);
    }
    /*************************** 第二轮补线修复结束 ***************************/
    /* [2026-10-22] 「整帧没找到线」也要拦住。扫描循环遇到连续黑行就 break，
       Line_Count 会停在最后成功的那一行，最坏情况停在 59；舵机那边只拦
       Line_Count < 40，于是会拿一帧残缺的中线去算舵机（日志里 P=2、LC=59、
       duty 直接打到 915 就是这种情况）。这里记下本帧还看得见的最远行。 */
    {
        uint8 k;
        Result_Far_Row = 59;
        for(k = 59; k > 5; k --)
        {
            if(Width_Real[k] >= 3 && Width_Real[k] < (COL - 6))
            {
                Result_Far_Row = k;
                break;
            }
        }
    }
    for (i = 59; i >= Line_Count;)   // 重新计算 中值
    {
        i =i-1;
        Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2 +Differ;

    }


    /************************************************  运动处理函数2024   ******************************************************/
    /************************************************  运动处理函数2024   ******************************************************/
    /************************************************  运动处理函数2024   ******************************************************/
    /************************************************  运动处理函数2024   ******************************************************/
//    if(Deal_Flag==0&&Stop_1==1 && dl1b_distance_mm > 1 && dl1b_distance_mm < 200)//无接触发车
//    {
//        Flag_turn=0;
//        Flag_stop = 0;
//        Deal_Flag=1;
//    }
    /* [2026-10-22] 2024 赛道的运动状态机：菱形、斑马线、Turn_Left/Turn_Right。
       它现在一上电就被 ToF 发车逻辑自动拉起来（Flag_stop=7, Flag_turn=3），
       而这套菱形/单车道逻辑不是 2026 双车道赛道，会把基础循迹搅乱。
       先整体关掉，纯跟线；以后重做标线时改 MOTION_2024_ENABLE=1 或写新的。 */
#define MOTION_2024_ENABLE    (0)
#if MOTION_2024_ENABLE
    if(tiaoguang==1)
        {
            Tiao_Light(data);
        }
    if(mode_2024==1)
        {
        if(Deal_Flag==0)
                {
                    Flag_turn=3;
                    Flag_stop=0;
                    Find_rhombus_Out(data);
                    if(Rhombus_Out_Flag==1)
                    {
                        Flag_turn=0;
                        Deal_Flag=1;
                        Rhombus_Out_Flag=0;

                    }
                }
            if(Flag_turn==0&&Deal_Flag==1)
            {
                Find_rhombus(data);
            }
            if(Flag_turn==0&&Deal_Flag==2)
            {
                Find_rhombus(data);
            }
            if(Flag_turn==0&&Deal_Flag==3)
            {
                Find_Decelerate(data);
            }
            if(Flag_turn==0&&Deal_Flag==4)
            {
                Find_Decelerate(data);
            }
            if(Flag_turn==0&&Deal_Flag==5)
            {
                Find_Zebra(data);
            }
            if(Flag_turn==0&&Deal_Flag==6)
            {
                Find_Rhombus(data);
            }
        }


        if(mode_2024==2)
        {
            if(Deal_Flag==0)
                    {
                        Flag_turn=3;
                        Flag_stop=0;
                        Find_rhombus_Out(data);
                        if(Rhombus_Out_Flag==1)
                        {
                            Flag_turn=0;
                            Deal_Flag=1;
                            Rhombus_Out_Flag=0;

                        }
                    }
            if(Deal_Flag!=3&&Deal_Flag!=0&&Flag_turn==0)//识别到障碍物
            {
                Find_Decelerate(data);
            }
            if(Deal_Flag==3&&Flag_turn==0)//识别到障碍物
            {
                Find_Zebra(data);
            }

        }



    //////////////////////////////////////////////////////执行区/////////////////////////////////////////////////

        if(Rhombus_Flag==1&&mode_2024==1)
        {
                    switch(Deal_Flag)
                    {
                    case 1:
                        {
                            Turn_Left();//6
                            break;
                        }
                    case 2:
                        {
                            Turn_Right();//0
                            break;
                        }
                    case 3:
                        {
                            Decelerate(data);//6
                            break;
                        }
                    case 4:
                        {
                            Decelerate_2(data);//6
                            break;
                        }
                    case 5:
                        {
                            Zebra_Crossing(data);//6
                            break;
                        }
                    case 6:
                        {
                            Flag_stop=5;
                            Flag_turn=5;
                            Distance_ON=1;
                            Deal_Flag++;
                            break;
                        }
                    case 7:
                        {
                            Find_Stop(data);
                            if(Distance>20&&Stop_Out==1)
                            {
                            Flag_stop=1;
                            gpio_set_level(P33_10,1);
                            Light_Flag=1;
                            }
                            break;
                        }
                    default:break;
                    }
        }

        if(Rhombus_Flag==1&&mode_2024==2)
        {
                switch(Deal_Flag)
                {
                case 1:
                    {
                        Decelerate(data);//6
                        break;
                    }
                case 2:
                    {
                        Decelerate_2(data);//0
                        break;
                    }
                case 3:
                    {
                        Zebra_Crossing(data);//6
                        break;
                    }
                case 4:
                    {

                        Flag_stop=5;
                        Flag_turn=5;
                        //Rhombus_Flag=0;
                        Distance_ON=1;
                        Deal_Flag++;
                        break;
                    }
                case 5:
                    {
                        if(Distance>20 )
                        {
                        Light_Flag=0;
                        Flag_stop=1;
                        gpio_set_level(P33_10,1);
                        }
                        break;
                    }
                default:break;
                }
        }
#endif

//8.6






  // 7.31
//    if(Deal_Flag==1&&Rhombus_Flag==1)
//    {
//        Turn_Left();
//    }
//    if(Deal_Flag==2&&Rhombus_Flag==1)
//    {
//        Turn_Right();
//    }
//    if(Deal_Flag==3&&Rhombus_Flag==1)
//    {
//        Decelerate(data);
//    }
//    if(Deal_Flag==4&&Rhombus_Flag==1)
//    {
//        Decelerate(data);
//    }
//    if(Deal_Flag==6&&Rhombus_Flag==1)
//    {
//        Flag_stop=0;
//        Flag_turn=3;
//        Rhombus_Flag=0;
//
//      //  Distance_Test_ON=0;
//        Distance_ON=1;
//        Deal_Flag++;
//    }
//    if(Deal_Flag==7&&Distance>50)
//    {
//        Flag_stop=1;
//    }
//    if(Flag_turn == 1 && Distance > 124 )
//    {
//        gpio_set_level(P33_10,0);//蜂鸣器关闭
//        Flag_turn = 2;//右转矫正
//        Distance_ON = 0;
//        Distance = 0;
//        Flag_stop = 5;//右转矫正速度
//        Distance_Test_ON = 1;
//
//    }
//    if(Flag_turn == 2 && Distance_Test > 62 )
//    {
//        Flag_turn = 0;
//        Distance_Test = 0;
//        Distance_Test_ON = 0;
//        Flag_stop = 0;
//        gpio_set_level(P21_3, 0);
//    }
//    if(1)
//    {
//        Distance_Test_ON = 1;
//        Light_Flag = 7;
//        if(Distance_Test > 100)
//        {
//            gpio_set_level(P33_10,1);
//            Distance_ON = 1;
//            Flag_turn = 2;//右转
//            Flag_stop = 3;
//            Distance_Test_ON = 0;
//            Distance_Test = 0;
//            Light_Flag=2;
//        }
//    }


//    if(Flag_turn == 2 && Distance > 200  )
//    {
//        gpio_set_level(P33_10,0);
//        Light_Flag = 6;
//        Flag_turn = 0;
//        Distance_ON = 0;
//        Distance = 0;
//        Flag_stop = 0;
//        Distance_Test_ON = 0;//本来是1
//        gpio_set_level(P21_5, 0);
//    }
//


    //如果侧边的TOF距离变化，就不执行菱形，直接拐弯，直接识别斑马线，停车
//    if(Turn_Flag == 2 && Distance_Test > Distances)//寻找菱形
//    {
//        Measurement = 1;
//        Find_rhombus(data);
//    }
//    if(Turn_Flag == 2 && Rhombus_Flag == 1)//找到第一个菱形
//    {
//        Flag_turn = 3;
//        Distance_Test_ON = 0;
//        Distance_Test = 0;
//        gpio_set_level(P33_10,1);
//        Flag_stop = 0;
//        Turn_Flag = 3;//第一个菱形
//    }
//    if(Turn_Flag == 3 && !Dis_Flag)
//    {
//        Distance_ON=1;
//        Flag_turn = 3;
//        if(Distance>45)
//        {
//            gpio_set_level(P33_10,0);
//            Flag_turn = 0;
//            Rhombus_Flag = 0;
//            Flag_stop = 0;
//            Turn_Flag = 4;//第一个菱形结束
//            Distance_ON = 0;
//            Distance = 0;
//        }
//    }
//    if(Turn_Flag == 4 && !Dis_Flag)//寻找菱形
//    {
//        Flag_stop = 0;
//        Distance_Test_ON = 1;
//        if(Distance_Test > 20)
//        {
//            Find_rhombus(data);
//            Distance_Test_ON = 0;
//            Distance_Test = 0;
//        }
//    }
//    if(Turn_Flag == 4 && Rhombus_Flag == 1 && !Dis_Flag)
//    {
//        gpio_set_level(P33_10,1);
//        Flag_stop = 0;
//        Flag_turn = 3;
//        Turn_Flag = 5;//第二个菱形
//    }
//    if(Turn_Flag == 5 && !Dis_Flag)
//    {
//        Flag_turn = 3;
//        Distance_ON=1;
//        if(Distance>45)
//        {
//            gpio_set_level(P33_10,0);
//            Flag_turn = 0;
//            Rhombus_Flag = 0;
//            Flag_stop = 0;
//            Turn_Flag = 6;//第二个菱形结束
//            Distance_ON = 0;
//            Distance = 0;
//            Distance_Test_ON = 1;
//        }
//    }
//    if(Measurement == 1 && !Dis_Flag)
//    {
//        gpio_toggle_level(P20_8);
//        if(dl1a_distance_mm > 100 && dl1a_distance_mm < 700 && dl1b_distance_mm > 8000)
//        {
//            if(Turn_Flag > 2)//菱形之后
//            {
//                gpio_toggle_level(P20_9);
//                Dis_Flag = 1;
//                Distance_ON = 0;
//                Distance = 0;
//                Distance_Test_ON = 0;
//                Distance_Test = 0;
//            }
//            else
//            {
//                if(Point_Error < 10)
//                {
//                    gpio_toggle_level(P20_9);
//                    Dis_Flag = 1;
//                    Distance_ON = 0;
//                    Distance = 0;
//                    Distance_Test_ON = 0;
//                    Distance_Test = 0;
//                }
//            }
//
//        }
//    }
//    if(Dis_Flag == 1)//识别到有车
//    {
//        Dis_Flag = 2;
//    }
//    if(Dis_Flag == 2)
//    {
//        gpio_set_level(P33_10,1);
//        Distance_ON = 1;
//        Flag_turn = 1;//左转
//        Flag_stop = 6;
//    }
//    if(Flag_turn == 1 && Distance > 62 && Dis_Flag == 2)
//    {
//        Dis_Flag = 3;
//        gpio_set_level(P33_10,0);
//        Flag_turn = 2;
//        Distance_ON = 0;
//        Distance = 0;
//        Flag_stop = 6;
//        Distance_Test_ON = 1;
//    }
//    if(Flag_turn == 2 && Distance_Test > 5 && Dis_Flag == 3)
//    {
//        Light_Flag = 5;
//        Flag_turn = 0;
//        Distance_Test = 0;
//        Distance_Test_ON = 0;
//        Flag_stop = 4;
//        Dis_Flag = 4;
//    }



  //时振瑞
//    if(Dis_Flag == 4 && !Start_Line_Flag)
//    {
//        Flag_stop = 4;//加速
//        Flag_turn = 0;
//        Start_Line_Judg(data);//寻找斑马线
//    }
//    if(Dis_Flag == 4 && Start_Line_Flag == 1)
//    {
//        Flag_stop = 4;
//        gpio_set_level(P33_10,1);
//        Dis_Flag = 5;//找到斑马线
//    }
//    if(Dis_Flag == 5)
//    {
//        Flag_stop = 4;
//        Flag_turn = 3;
//        gpio_set_level(P33_10,0);
//        Distance_ON = 1;
//        if(Distance > 25)//原本200
//        {
//            Flag_stop = 1;//原本为0
//            Dis_Flag = 6;
//            Distance_ON = 0;
//            Distance = 0;
//            Time_banmaxian=1;
//            Turn_Flag=10;
//        }
//    }


    //时振瑞
//    if(Time_banmaxian_Flag == 1)
//     {
//        Flag_stop=5;
//        gpio_set_level(P33_10,1);
//        if(dl1b_distance_mm<150)
//        {
//            gpio_set_level(P33_10,0);
//            Flag_stop=1;
//            Turn_Flag=0;
//        }
//
//     }



//    if(Dis_Flag == 6)
//    {
//        Flag_stop = 4;
//        gpio_set_level(P33_10,1);
//        Flag_turn = 0;
//        Dis_Flag = 7;//要结束        Time_banmaxian_Flag
//        Light_Flag = 3;
//    }
//    if(Dis_Flag == 7)
//    {
//        Flag_turn = 0;
//        Flag_stop = 4;
//        Stop_stop(data);
//        if(Stop_Flag == 1)
//        {
//            Flag_turn = 3;
//            Distance_Test_ON = 1;
//            if(Distance_Test > 20)//原本60
//            {
//                Flag_turn = 3;
//                Dis_Flag = 10;
//                Flag_stop = 1;//停车
//            }
//        }
//    }
//    if(Dis_Flag == 10)
//    {
//        Light_Flag = 4;
//        Flag_stop = 1;
//        Flag_turn = 3;
//    }
//    Num_Turn_L = Find_J_Black(data,5,58,20);
//    Num_Turn_R = Find_J_Black(data,115,58,20);
}     //此处为image结束

            //以下为子函数
uint8 Start_Line_Judg(uint8 *data)   // 鏂戦┈绾垮垽鍒�
{
    uint8 j;
    uint8 i,white_count,k,a;
    for(i = 25;i>15;i--)
    {
        k= 0,a=0 ;
        for(j = 20;j<100;j++)
        {
            if(data[i*COL + j])//鐧�
            {
                white_count=1;
            }
            else
            {
                white_count=0;
            }

            if(white_count!=n)
            {
                n = white_count;
                k++;
            }
            if(k>11)
            {
                a++;
                if(a>3) break;
            }
        }
        if(a>3) break;
    }
    if(a>3)
    {
        Start_Line_Flag=1;
    }
}
void Stop_stop(uint8 *data)
{
    uint8 j;
    uint8 i,a;
    for(i = 45;i>40;i--)
    {
        for(j = 55;j<65;j++)
        {
            if(data[i*COL+j])
            {
                Stop_Flag=1;
            }
        }
    }

}
void Find_rhombus(uint8 *data)
{
    uint8 i,j,a=0;
    for(i=10;i<=30;i++)
        {
            for(j=54;j<67;j++)
            {
                if(data[i*COL+j])
                {
                    a++;
                }
            }
            if(a>=2||dl1b_distance_mm<1200)
            {
                Rhombus_Flag = 1;
                break;
            }
        }
        a=0;
        for(i=1;i<=10;i++)
        {
            for(j=58;j<63;j++)
            {
                if(data[i*COL+j])
                {
                    a++;
                }
            }
            if(a>=2||dl1b_distance_mm<1200)
            {
                Rhombus_Flag = 1;
                break;
            }
        }
}
void Find_Rhombus(uint8 *data)
{
    uint8 i,j,a=0;
    for(i=20;i<=50;i++)
    {
        for(j=55;j<65;j++)
        {
            if(data[i*COL+j])
            {
                a++;
            }
        }
        // [2026-10-20] skip rows that hold TWO glare-sized white runs
        if(agl_blob_count(data, i, 1, COL-2) >= 2)
        {
            a = 0;
        }
        if(a>2)
        {
            Rhombus_Flag = 1;
            break;
        }
    }
}
void Find_Zebra(uint8 *data)
{
    uint8 i,j,a=0;
    for(i=25;i<=60;i++)
    {
        for(j=56;j<64;j++)
        {
            if(data[i*COL+j])
            {
                a++;
            }
        }
        if(agl_blob_count(data, i, 1, COL-2) >= 2) a = 0;   // [2026-10-20] glare row
        if(a>2)
        {
            gpio_set_level(P33_10,1);
            Flag_stop=1;
            Rhombus_Flag = 1;
            break;
        }
    }
}
void Find_Decelerate(uint8 *data)
{
    uint8 i,j,a=0;
    for(i=55;i>30;i--)
    {
        for(j=58;j<62;j++)
        {
            if(data[i*COL+j])
            {
                a++;
            }
        }
        if(agl_blob_count(data, i, 1, COL-2) >= 2) a = 0;   // [2026-10-20] glare row
        if(a>=2)
        {
            Rhombus_Flag = 1;
            break;
        }
    }
}
void Find_rhombus_Out(uint8 *data)
{
    uint8 i,j,a=0;
    for(i=60;i>29;i--)
    {
        for(j=58;j<62;j++)
        {
            if(data[i*COL+j])
            {
                a++;

            }
        }
    }
    if(a<3)
    {
        Rhombus_Out_Flag =1;
    }
}
/*********鑷富杈圭晫*************************/
void New_boundary(uint8 *data, float *Ka, float *Kb)
{
    float ka,kb;
    uint8 res;
    Zhuan_Flag=1;
    if(Zhuan_Flag)
    {
    for(i=15;i<18;i++)
    {
        for(j=35;j>30;j--)
        {
            if(data[i*COL+j]&&!data[(i-1)*COL+j]&&!data[(i+1)*COL+j])
            {
                a=i;
                b=j;
            }
        }
    }
    a1=60;
    b1=98;
        ka = 1.0*(b1-b) / (a1-a);
        kb = 1.0*b - (ka*a);
        res = a * ka + kb;
        res = range_protect((int32)res, 1, (COL-2)); // 闄愬箙
        *Ka = ka;
        *Kb = kb;
    Right_Ka=*Ka;
    Right_Kb=*Kb;
    for(i=60;i>5;i--)
    {
        Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);
        Left_Add_Line[i]=4;
        Mid_Line[i] = (Right_Add_Line[i]+Left_Add_Line[i]) / 2;
    }
    }
}
/*********杈呭姪鍒ゅ埆閿ユ《*************************/
uint8 Find_Cone_bucket(uint8 *data)
{
    uint8 i,k;
    for(j=0;j<10;j++)
    {
        for(i=60;i<64;i++)
        {
            if(data[i*COL+j])
            {
                k++;
                if(k>10)
                {
                Cone_bucket=1;
                zyuzhi = 38;
                }
            }
        }
    }
}
//  浠庢煇涓�鍒� 浠庡簳鍚戜笂鎼滃缁堢偣  杩斿洖 琛屾暟
uint8 Find_J_Black(uint8 *data, uint8 j,uint8 start,uint8 end)
{
    uint8 i;
    uint8 result = 0;

      for(i=start;i>=end;)
      {
        i -= 1;
        if(data[(i+Gray_Num)*COL + j]!=data[i*COL + j])
        {
             result = i;
             return result;
        }
      }
     return i;
}


/*
*   涓ょ偣娉曟眰鐩寸嚎鏂滅巼
*
*   璇存槑锛氭嫙鍚堢洿绾� y = Ka * x + Kb  鐨勬枩鐜�
*/
void Curve_Fitting(float *Ka, float *Kb, uint8 *Start, uint16 *Line, uint8 *Add_Flag, uint8 Mode)
{
    uint8 start=0,start2,start3;
    uint8 stop=0;
    float ka,kb;
    float res;
    for (uint8 i = *Start+2; i < 57; i++) // 浠庤琛ョ嚎鐨勭偣 寰�涓嬫壘鍒颁笉闇�瑕佽ˉ绾跨殑鐐� 璁颁负鏂滅巼璁＄畻璧风偣
    {
        if (!Add_Flag[i])
        {
            start = i;
            break;
        }
    }
    if (start)  // 鏂滅巼璁＄畻 璧风偣鎵惧埌
    {
        for (uint8 i = start+2; i < start+10; i++) // 浠庢枩鐜囪绠楄捣鐐� 寰�涓嬫壘 涓嶉渶瑕佽ˉ绾跨殑鐐� 璁颁负鏂滅巼璁＄畻缁堢偣
        {
            if (!Add_Flag[i])
            {
                stop = i+1;
                break;
            }
            if (i > 58)
            {
                break;
            }
        }
    if (stop)        // 缁堢偣鎵惧埌
        {
            ka = 1.0*(Line[stop] - Line[start]) / (stop - start);
            kb = 1.0*Line[start] - (ka*start);
        }
    }
    else // 娌℃壘鍒� 鏂滅巼璁＄畻璧风偣
    {
        ka = 0;
        kb = Line[*Start+1];
    }
    res = (*Start) * ka + kb;
    res = range_protect((int32)res, 1, (COL-2)); // 闄愬箙
    if (Mode ==1 )
    {
        if ((res - Line[*Start+1]) < -3) // 濡傛灉璁＄畻鍑烘潵鐨勮宸笌涓婁竴鐐圭殑璇樊澶ぇ 浣跨敤涓婁竴鐐圭殑鍧愭爣
        {
            Line[*Start] =  Line[*Start+1];
            *Ka = 0;
            *Kb = Line[*Start+1];
        }
        else
        {
            *Ka = ka;
            *Kb = kb;
            Line[*Start] = res;
        }
    }
    else if (Mode == 2)
    {
        if ((res - Line[*Start+1]) > 3) // 濡傛灉璁＄畻鍑烘潵鐨勮宸笌涓婁竴鐐圭殑璇樊澶ぇ 浣跨敤涓婁竴鐐圭殑鍧愭爣
        {
            Line[*Start] =  Line[*Start+1];
            *Ka = 0;
            *Kb = Line[*Start+1];
        }
        else
        {
            *Ka = ka;
            *Kb = kb;
            Line[*Start] = res;
        }
    }
}
/*
*   涓ょ偣娉曟眰鐩寸嚎
*
*   璇存槑锛氭嫙鍚堢洿绾� y = Ka * x + Kb   Mode == 1浠ｈ〃宸﹁竟鐣岋紝Mode == 2浠ｈ〃鍙宠竟鐣�
*/
void Curve2_Fitting(float *Ka, float *Kb, uint8 Start,uint8 End, uint16 *Line, uint8 Mode)
{
        if (Mode==1)
        {
             *Ka = 1.0*((Line[Start]) - Line[End]) / (Start-End);
             *Kb = 1.0*Line[End] - (*Ka * End);
        }
        else
        {
              *Ka = 1.0*((Line[Start]) - Line[End]) / (Start-End);
              *Kb = 1.0*Line[End] - (*Ka * End);
        }

}

/*
*   璁＄畻琛ョ嚎鍧愭爣
*
*   璇存槑锛氫娇鐢ㄤ袱鐐规硶璁＄畻鎷熷悎鍑虹殑琛ョ嚎鍧愭爣
*/
uint8 Calculate_Add(uint8 i, float Ka, float Kb)
{
    float res;
    uint8 Result;

    res = i * Ka + Kb;
    Result = range_protect((int32)res, 1, (COL-2));

    return Result;
}

//  棣栬澶勭悊
uint8 First_Line_Handle(uint8 *data)
{
    uint8 i = 59;   // 鎺у埗琛�
    uint8 Weight_Left, Weight_Right;    // 宸﹀彸璧涢亾瀹藉害
    uint8 Mid_Left, Mid_Right;
    uint8 black_count = 0;
    Left_Add_Flag[59] = 0;
    Right_Add_Flag[59] = 0;     //    鍒濆鍖栬ˉ绾挎爣蹇�
    Image_Para_Init();
    if (data[i*COL + Mid_Line[i+1]])// 60 琛岀粓鐐瑰湪 59 琛屼负 榛戣壊  鍙兘璺戝嚭璧涢亾 涔熷彲鑳戒笉鍦ㄨ禌閬撴渶涓績浣嶇疆
    {
        for (uint8 j = 1; j < COL-2; j++)
        {
            if (data[i*COL + j] )
            {
                black_count++;
            }
            if (black_count > 100)
            {
                return 0;
            }
        }

        Weight_Left = Traversal_Left(i, data, &Mid_Left, 1, (COL-2));       // 浠庡乏渚ф悳绱㈣竟鐣�  0
        Weight_Right = Traversal_Right(i, data, &Mid_Right, 1, (COL-2));// 浠庡彸渚ф悳绱㈣竟鐣�  0

        if (Weight_Left >= Weight_Right && Weight_Left)                 // 宸﹁禌閬撳搴﹀ぇ浜庡彸璧涢亾瀹藉害涓斾笉涓�0
        {
              Traversal_Left_Line(i, data, Left_Line, Right_Line);      // 浣跨敤宸﹂亶鍘嗚幏鍙栬禌閬撹竟鐣�
        }
        else if (Weight_Left < Weight_Right && Weight_Right)
        {
              Traversal_Right_Line(i, data, Left_Line, Right_Line);     // 浣跨敤鍙抽亶鍘嗚幏鍙栬禌閬撹竟鐣�
        }
        else    // 璇存槑娌℃煡鍒�
        {
              return 0;
        }
    }
    else
    {
          Traversal_Mid_Line(i, data, Mid_Line[i+1], 1, (COL-2), Left_Line, Right_Line, Left_Add_Line, Right_Add_Line); // 浠庡墠涓�琛屼腑鐐瑰悜涓よ竟鎵弿
    }

    Left_Line[60] = Left_Line[59];
    Right_Line[60] = Right_Line[59];
    Left_Add_Line[60] = Left_Add_Line[59];
    Right_Add_Line[60] = Right_Add_Line[59];

    if (Left_Add_Flag[59] && Right_Add_Flag[59])
    {
        Mid_Line[59] = Mid_Line[60];
    }
    else
    {
        Mid_Line[59] = (Right_Line[59] + Left_Line[59]) / 2;
        Mid_Line[60] = Mid_Line[59];                           // 鏇存柊绗�60琛岃櫄鎷熶腑鐐癸紝渚夸簬涓嬩竴甯у浘鍍忎娇鐢�
    }
    Width_Real[60] = Width_Real[59];
    Width_Add[60] = Width_Add[59];
    Width_Min = Width_Add[59];
    if (Mid_Line[59] < 5 || Mid_Line[59] > 115)  // 濡傛灉 鏈�浣庤鐨� 涓偣鍧愭爣 澶皬鎴栧お澶� 鑷繁琛ヤ竴涓�
    {
        Mid_Line[59] = 60;
        Mid_Line[60] = Mid_Line[59];
    }
    return 1;
}

/*
*   浠庡乏渚у紑濮嬫悳绱㈣竟鐣岋紝杩斿洖璧涢亾瀹藉害
*
*   璇存槑锛氭湰鍑芥暟浠呬粎浣滀负鎺㈡祴璧涢亾浣跨敤锛屼粎杩斿洖璧涢亾瀹藉害锛屼笉淇濆瓨杈圭晫鏁版嵁
*/
uint8 Traversal_Left(uint8 i, uint8 *data, uint8 *Mid, uint8 Left_Min, uint8 Right_Max)  //  Traversal_Left(i, data, &Mid_Left, 1, (COL-1)); i = 89
{
    uint8 j, White_Flag = 0;
    uint8 Left_Line = Left_Min, Right_Line = Right_Max;

    for (j = Left_Min+Gray_Num; j <= Right_Max-Gray_Num; j+=1)  // 杈圭晫鍧愭爣 1鍒� 119
    {
          if (!White_Flag)  // 鍏堟煡鎵惧乏杈圭晫
          {
              // 宸﹁竟鍙兘涓㈢嚎 濡傛灉鐩存帴涓㈢嚎 閭ｄ箞璁颁笅宸︾晫
              if(data[i*COL + Left_Min+Gray_Num])
              {
                    Left_Line = Left_Min+Gray_Num;// 璁板綍褰撳墠j鍊间负鏈宸﹁竟鐣�
                    White_Flag = 1;// 宸﹁竟鐣屽凡鎵惧埌锛屽繀鏈夊彸杈圭晫
                    j += Gray_Num;
                    continue;   // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
              }
              if (data[i*COL + j+Gray_Num]!=data[i*COL + j])   // 妫�娴嬭禌閬撹竟鐣�
              {
                  Left_Line = j;// 璁板綍褰撳墠j鍊间负鏈宸﹁竟鐣�
                  White_Flag = 1;// 宸﹁竟鐣屽凡鎵惧埌锛屽繀鏈夊彸杈圭晫
                  j += Gray_Num;
                  continue; // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
              }
          }
          else
          {
              if (data[i*COL + j]!=data[i*COL + j+Gray_Num])// 妫�娴嬭禌閬撹竟鐣�
              {
                  Right_Line = j;//璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
                  break;    // 宸﹀彸杈圭晫閮芥壘鍒帮紝缁撴潫寰幆
              }
          }
    }
    if (!White_Flag)    // 鏈壘鍒板乏鍙宠竟鐣�
    {
        return 0;
    }
    else
    {
        *Mid = (Right_Line + Left_Line) / 2;
        return (Right_Line - Left_Line);
    }
}

/*
*   浠庡彸渚у紑濮嬫悳绱㈣竟鐣岋紝杩斿洖璧涢亾瀹藉害
*
*   璇存槑锛氭湰鍑芥暟浠呬粎浣滀负鎺㈡祴璧涢亾浣跨敤锛屼粎杩斿洖璧涢亾瀹藉害锛屼笉淇濆瓨杈圭晫鏁版嵁
*/
uint8 Traversal_Right(uint8 i, uint8 *data, uint8 *Mid, uint8 Left_Min, uint8 Right_Max)
{
    uint8 j, White_Flag = 0;
    uint8 Left_Line = Left_Min, Right_Line = Right_Max;

    for (j = Right_Max-Gray_Num; j >= Left_Min+Gray_Num; j-=1)  // 杈圭晫鍧愭爣 235 ~ 4
    {
        if (!White_Flag)        // 鍏堟煡鎵惧彸杈圭晫
        {
            // 鍙宠竟鍙兘涓㈢嚎 濡傛灉鐩存帴涓㈢嚎 閭ｄ箞璁颁笅鍙宠竟鐣�
            if(data[i*COL +  Right_Max-Gray_Num] )
             {
                    Right_Line = Right_Max-Gray_Num;// // 璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
                    White_Flag = 1;// // 鍙宠竟鐣屽凡鎵惧埌锛屽繀鏈夊乏杈圭晫
                    j -= Gray_Num;
                    continue;   // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
            }
            if (data[i*COL + j-Gray_Num]!=data[i*COL + j])     // 妫�娴嬪埌鐧界偣
            {
                Right_Line = j; // 璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
                White_Flag = 1;     // 鍙宠竟鐣屽凡鎵惧埌锛屽繀鏈夊乏杈圭晫
                j -= Gray_Num;
                continue;// 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
            }
        }
        else
        {
            if (data[i*COL + j]!=data[i*COL + j - Gray_Num])   //妫�娴嬮粦鐐�
            {
                Left_Line = j;  //璁板綍褰撳墠j鍊间负鏈宸﹁竟鐣�
                break;      // 宸﹀彸杈圭晫閮芥壘鍒帮紝缁撴潫寰幆
            }
        }
    }
    if (!White_Flag)    // 鏈壘鍒板乏鍙宠竟鐣�
    {
        return 0;
    }
    else
    {
        *Mid = (Right_Line + Left_Line) / 2;
        return (Right_Line - Left_Line);
    }
}


/*
*   浠庡乏渚у紑濮嬫悳绱㈣竟鐣岋紝淇濆瓨璧涢亾杈圭晫锛岃繑鍥�1鎴愬姛 0澶辫触
*
*   璇存槑锛氭湰鍑芥暟浣跨敤鍚庡皢淇濆瓨杈圭晫鏁版嵁
*/
uint8 Traversal_Left_Line(uint8 i, uint8 *data, uint16 *Left_Line, uint16 *Right_Line)   // Traversal_Left_Line(i, data, Left_Line, Right_Line);
{
    uint8 j, White_Flag = 0;

    Left_Line[i] = 1;
    Right_Line[i] = (COL-1);

    for (j = 1+1; j < COL-1-Gray_Num; j += 1)       // 杈圭晫鍧愭爣 1鍒癈OL
    {
        if (!White_Flag)    // 鍏堟煡鎵惧乏杈圭晫
        {
            // 宸﹁竟鍙兘涓㈢嚎 濡傛灉鐩存帴涓㈢嚎 閭ｄ箞璁颁笅宸﹁竟灞�
            if(data[i*COL + 1+1] )
            {
                    Left_Line[i] = 1+1;// 璁板綍褰撳墠j鍊间负鏈宸﹁竟鐣�
                    White_Flag = 1;// 宸﹁竟鐣屽凡鎵惧埌锛屽繀鏈夊彸杈圭晫
                    j += Gray_Num;
                    continue;   // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
            }
            if (data[i*COL + j+Gray_Num]!=data[i*COL + j]) // 妫�娴嬪埌鐧界偣
            {
                    Left_Line[i] = j;        //褰撳墠j鍊间负鏈宸﹁竟鐣�
                    White_Flag = 1;     // 宸﹁竟鐣屽凡鎵惧埌锛屽繀鏈夊彸杈圭晫
                    j += Gray_Num;
                    continue;   // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
            }
        }
        else
        {
            if (data[i*COL + j]!=data[i*COL + j+Gray_Num]) //妫�娴嬮粦鐐�
            {
                  Right_Line[i] = j-1;    //璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
                  break;    // 宸﹀彸杈圭晫閮芥壘鍒帮紝缁撴潫寰幆
            }
        }
    }
    if (White_Flag  )
    {
        Left_Add_Line[i] = Left_Line[i];
        Right_Add_Line[i] = Right_Line[i];
        Width_Real[i] = Right_Line[i] - Left_Line[i];
        Width_Add[i] = Width_Real[i];
    }

    return White_Flag;  // 杩斿洖鎼滅储缁撴灉
}
/*
*   浠庡彸渚у紑濮嬫悳绱㈣竟鐣岋紝淇濆瓨璧涢亾杈圭晫锛岃繑鍥�1鎴愬姛 0澶辫触
*
*   璇存槑锛氭湰鍑芥暟浣跨敤鍚庡皢淇濆瓨杈圭晫鏁版嵁
*/
uint8 Traversal_Right_Line(uint8 i, uint8 *data, uint16 *Left_Line, uint16 *Right_Line)
{
    uint8 j, White_Flag = 0;
    Left_Line[i] = 1;
    Right_Line[i] = (COL-2);
    for (j = (COL-1-1); j > 0+1+Gray_Num; j -= 1)   // 杈圭晫鍧愭爣 1鍒�79
    {
        if (!White_Flag)    // 鍏堟煡鎵惧彸杈圭晫
        {
              //    鍙宠竟鍙兘涓㈢嚎 濡傛灉鐩存帴涓㈢嚎 閭ｄ箞璁颁笅鍙宠竟鐣�
              if(data[i*COL +  (COL-3)] )
              {
                  Right_Line[i] = (COL-2);// // 璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
                  White_Flag = 1;// // 鍙宠竟鐣屽凡鎵惧埌锛屽繀鏈夊乏杈圭晫
                  j -= Gray_Num;
                  continue; // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
              }
              if (data[i*COL + j-Gray_Num]!=data[i*COL + j])   // 妫�娴嬪埌鐧界偣
              {
                  Right_Line[i] = j;    // 璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
                  White_Flag = 1;           // 鍙宠竟鐣屽凡鎵惧埌锛屽繀鏈夊乏杈圭晫
                  j -= Gray_Num;
                  continue; // 缁撴潫鏈寰幆锛岃繘鍏ヤ笅娆″惊鐜�
              }
        }
        else
        {
              if (data[i*COL + j]!=data[i*COL + j-Gray_Num])       //妫�娴嬮粦鐐�
              {
                  Left_Line[i] = j+1;//璁板綍褰撳墠j鍊间负鏈宸﹁竟鐣�
                  break;        // 宸﹀彸杈圭晫閮芥壘鍒帮紝缁撴潫寰幆
              }
        }
    }
    if (White_Flag )
    {
        Left_Add_Line[i] = Left_Line[i];
        Right_Add_Line[i] = Right_Line[i];
        Width_Real[i] = Right_Line[i] - Left_Line[i];
        Width_Add[i] = Width_Real[i];
    }

    return White_Flag;      // 杩斿洖鎼滅储缁撴灉
}

/*
*   浠庝腑闂村悜涓よ竟鎼滅储杈圭晫
*
*   璇存槑锛氭湰鍑芥暟浣跨敤鍚庡皢淇濆瓨杈圭晫鏁版嵁
*/
void Traversal_Mid_Line(uint8 i, uint8 *data, uint8 Mid, uint8 Left_Min, uint8 Right_Max, uint16 *Left_Line, uint16 *Right_Line, uint16 *Left_Add_Line, uint16 *Right_Add_Line)
{
    uint8 j;
    Left_Add_Flag[i] = 1;// 鍒濆鍖栬ˉ绾挎爣蹇椾綅
    Right_Add_Flag[i] = 1;
    Right_Line[i] = Right_Max;    //   COL -1
    Left_Line[i] = Left_Min;    // 缁欏畾杈圭晫鍒濆鍊�   1

    for (j = Mid; j >= Left_Min+1+Gray_Num; j-=1)   // 浠ュ墠涓�琛屼腑鐐逛负璧风偣鍚戝乏鏌ユ壘杈圭晫
    {
        if (data[i*COL + j]!=data[i*COL + j - Gray_Num])   //妫�娴嬪埌榛戣浆鐧界偣
        {
            Left_Add_Flag[i] = 0;   //宸﹁竟鐣屼笉闇�瑕佽ˉ绾匡紝娓呴櫎鏍囧織浣�
            Left_Line[i] = j;       //璁板綍褰撳墠j鍊间负鏈瀹為檯宸﹁竟鐣�
            Left_Add_Line[i] = j;    // 璁板綍瀹為檯宸﹁竟鐣屼负琛ョ嚎宸﹁竟鐣�
            break;
        }
    }
    for (j = Mid; j <= Right_Max-1*Gray_Num; j+=1)  // 浠ュ墠涓�琛屼腑鐐逛负璧风偣鍚戝彸鏌ユ壘鍙宠竟鐣�
    {
        if (data[i*COL + j]!=data[i*COL + j + Gray_Num])   //妫�娴嬪埌榛戣浆鐧界偣
        {
            Right_Add_Flag[i] = 0;      //鍙宠竟鐣屼笉闇�瑕佽ˉ绾匡紝娓呴櫎鏍囧織浣�
            Right_Line[i] = j;      //璁板綍褰撳墠j鍊间负鏈鍙宠竟鐣�
            Right_Add_Line[i] = j;  // 璁板綍瀹為檯鍙宠竟鐣屼负琛ョ嚎鍙宠竟鐣�
            break;
       }
    }
    if (Left_Add_Flag[i])       // 宸﹁竟鐣岄渶瑕佽ˉ绾�
    {
        if (i >= 57)    // 鍓�3琛�
        {
            Left_Add_Line[i] = Left_Line[59];   // 浣跨敤搴曡鏁版嵁
        }
        else                     ///
        {
            Left_Add_Line[i] = Left_Add_Line[i+1];  // 浣跨敤鍓�1琛屽乏杈圭晫浣滀负鏈宸﹁竟鐣�
        }
    }
    if (Right_Add_Flag[i])  // 鍙宠竟鐣岄渶瑕佽ˉ绾�
    {
        if (i >= 57)    // 鍓�3琛�
        {
            Right_Add_Line[i] = Right_Line[59];// 浣跨敤搴曡鏁版嵁
        }
        else
        {
            Right_Add_Line[i] = Right_Add_Line[i+1];    // 浣跨敤鍓�1琛屽彸杈圭晫浣滀负鏈鍙宠竟鐣�
        }
    }

    Width_Real[i] = Right_Line[i] - Left_Line[i];               // 璁＄畻瀹為檯璧涢亾瀹藉害
    Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];    // 璁＄畻琛ョ嚎璧涢亾瀹藉害

}


/*
*   琛ョ嚎淇
*
*   璇存槑锛氭湁濮嬫湁缁堟墠浣跨敤锛岀洿鎺ヤ娇鐢ㄤ袱鐐规枩鐜囪繘琛岃ˉ绾�
*
*/ //Line_Repair(Left_Add_Start, Left_Add_Stop, data, Left_Line, Left_Add_Line, Left_Add_Flag, 1);
void Line_Repair(uint8 Start, uint8 Stop, uint8 *data, uint16 *Line, uint16 *Line_Add, uint8 *Add_Flag, uint8 Mode)
{
    float res;
    uint8 i;    // 鎺у埗琛�
    float Ka, Kb;

        if (Stop)   // 鏈夊鏈夌粓
        {
                if (Start <= 57)
                {
                    Start +=2;
                }
                Ka = 1.0*(Line_Add[Start] - Line_Add[Stop]) / (Start - Stop);
                Kb = 1.0*Line_Add[Start] - (Ka * Start);
                for (i = Stop; i <= Start; )
                {
                    i += 1;
                    res = i * Ka + Kb;
                    Line_Add[i] = range_protect((int32)res, 2, (COL-2));
                }
            // }
        }
    // }
}

/*
*   琛ョ嚎淇
*
*   璇存槑锛氫换鎰忎袱鐐硅ˉ涓�鏉＄洿绾�
*
*/   // Start Stop 寮�濮嬬粨鏉� 琛ョ嚎鐐圭殑 琛屾暟  position_start position_stop   琛ョ嚎鐐圭殑鍒楁暟    Line_Add 瑕佽ˉ鐨勭嚎
void Point_Repair_Line(uint8 Start, uint8 Stop,uint16 position_start, uint16 position_stop ,uint16 *Line_Add)
{
    float res;
    uint8 i;    // 鎺у埗琛�
    float ka, kb;
    ka = 1.0*(position_start - position_stop) / (Start - Stop);
    kb = 1.0*position_start - (ka * Start);

    for (i = Stop; i <= Start; )
    {
        i += 1;
        res = i * ka + kb;
        Line_Add[i] = range_protect((int32)res, 2, (COL-2));
    }
}




/******** 闄愬箙淇濇姢 *********/
int32 range_protect(int32 duty, int32 min, int32 max)//闄愬箙淇濇姢
{
    if (duty >= max)
    {
        return max;
    }
    else if (duty <= min)
    {
        return min;
    }
    else
    {
        return duty;
    }
}
/******** 浠庡乏鎵乏杈圭晫 *********/
uint8 zuobianjie( uint8 *data,uint8 Left_Min,uint8 Right_Max)
{
    uint8 i,j;
    for(i=60;i>5;i--)
    {
        for (j =2; j <= 30; j+=1)
        {
            if(data[i*COL + j])
            {
                Right_Flag = 1;
                Left_Add_Line[i] = j;
                break;
            }
        }
    }
}
/******** 浠庡彸鎵彸杈圭晫 *********/
uint8 youbianjie( uint8 *data, uint8 Left_Min, uint8 Right_Max)
{
        for(i=60;i>=5;i--)
        {
            for (j = 110; j >= 90; j-=1)  // 杈圭晫鍧愭爣 235 ~ 4
            {
                    if(data[i*COL + j] )
                    {
                        Right_Flag = 1;
                        Right_Add_Line[i]=j;
                        break;
                    }
            }
        }
}

void  Find_Rhombus_Out(uint8 *data)
{
        aaa=0;
        for(int i=30;i<=60;i++)
        {
            for(int j=57;j<=63;j++)
            {
                if(data[i*COL+j])
                {
                    aaa++;
                }
            }
        }
        // [2026-10-20] two glare blobs in one row = reflection, not "diamond gone"
        {
            uint8 gi, pair = 0;
            for(gi = 40; gi <= 50; gi ++)
            {
                if(agl_blob_count(data, gi, 1, COL-2) >= 2) pair = 1;
            }
            if(pair) aaa += 10;
        }
        if(aaa<2)
        {
            gpio_set_level(P33_10,0);
            Rhombus_Out=1;
        }
}
void  Find_Stop(uint8 *data)
{
        aaa=0;
        for(int i=30;i<=65;i++)
        {
            for(int j=10;j<=60;j++)
            {
                if(data[i*COL+j])
                {
                    aaa++;
                }
            }
        }
        if(aaa<2)
        {
            Stop_Out=1;
        }
}

void Turn_Left(void)
{
    if(Flag_turn == 0&&Flag_stop==0)
    {
        Flag_stop = 7;
        Flag_turn = 3;
        Distance_Test_ON = 1;
    }
    if(Flag_stop == 7&&Distance_Test>50)
    {
        Distance_Test = 0;
        Distance_Test_ON = 0;

        Light_Flag=2;

        Flag_turn = 1;
        Flag_stop = 2;

        Distance_ON = 1;
    }
    if(Flag_turn == 1 && Distance > 165)//180
    {
        Distance_ON = 0;
        Distance = 0;
        Light_Flag=0;

        Flag_turn = 6;//鍙宠浆鐭

        Flag_stop = 3;//鍙宠浆鐭閫熷害
        Distance_Test_ON = 1;
    }

    if(Flag_turn == 6 && Distance_Test > 85 )//鍙宠浆澶嶄綅
    {
        gpio_set_level(P33_10,1);//铚傞福鍣ㄥ叧闂�
        Distance_Test = 0;
        Distance_Test_ON = 0;

        Flag_turn = 0;
        Flag_stop = 5;
        Distance_ON = 1;
    }
    if(Flag_turn == 0 && Distance > 450 )//缁欎竴娈佃窛绂诲惊杩瑰浣�
    {
        gpio_set_level(P33_10,0);//铚傞福鍣ㄥ叧闂�
        Distance_ON = 0;
        Distance = 0;

        Flag_turn = 0;
        Flag_stop=0;

        Rhombus_Flag=0;
        Deal_Flag=2;
        Distance_Test_ON = 1;
    }

}
void Turn_Right(void)
{
    if(Flag_turn == 0&&Flag_stop==0)
    {
        Flag_stop = 7;
        Flag_turn = 3;
        Distance_Test_ON = 1;
    }
    if(Flag_stop == 7&&Distance_Test>30)
    {
        Distance_Test = 0;
        Distance_Test_ON = 0;
        Light_Flag=3;//鍙宠浆鍚戠伅

        Flag_turn = 2;//鍙宠浆
        Flag_stop = 3;//鍙宠浆閫熷害

        Distance_ON = 1;//寮�鍚噷绋嬭
    }
    if(Flag_turn == 2 && Distance >165)
    {
        Distance_ON = 0;
        Distance = 0;


        Light_Flag=0;
        gpio_set_level(P33_10,1);//铚傞福鍣�
        Flag_turn = 1;//宸﹁浆鐭

        Flag_stop = 2;//宸﹁浆鐭閫熷害
        Distance_Test_ON = 1;
    }

    if(Flag_turn == 1 && Distance_Test >85)
    {
        gpio_set_level(P33_10,0);//铚傞福鍣�
        Distance_Test = 0;
        Distance_Test_ON = 0;
        gpio_set_level(P20_13,1);
        gpio_set_level(P21_4,1);
        Flag_turn = 10;
        Flag_stop = 5;

        gpio_set_level(P21_5, 0);

        Distance_ON = 1;
     }
    if(Flag_turn == 10 && Distance > 400 )
     {
        Rhombus_Flag=0;

        Deal_Flag=3;
        Flag_turn = 0;
        Flag_stop = 0;
        Distance_ON = 0;
        Distance = 0;
     }

}
void Decelerate(uint8 *data)
{
    Flag_stop=5;
    Flag_turn=3;
    Light_Flag=1;

    Find_rhombus_Out(data);
    if(Rhombus_Out_Flag==1)
    {
       Flag_stop=5;
       Flag_turn=0;
       Deal_Flag++;
       Rhombus_Flag=0;
       Rhombus_Out_Flag=0;

   }
}
void Decelerate_2(uint8 *data)
{
    Flag_stop=5;
    Flag_turn=3;
    Light_Flag=1;

    Find_rhombus_Out(data);
    if(Rhombus_Out_Flag==1)
    {
        Distance_Test_ON=1;
        if(Distance_Test>150)
        {
            Distance_Test_ON=0;
            Distance_Test=0;
            Flag_stop=8;
            Flag_turn=0;
            Deal_Flag++;
            Rhombus_Flag=0;
            Rhombus_Out_Flag=0;
        }
    }
}
void Zebra_Crossing(uint8 *data)
{
    if(Flag_turn==0)
    {
        Flag_turn=5;
        Flag_stop=1;
        gpio_set_level(P33_10,1);
        Flag_turn=5;
        Light_Flag=0;
        time_dl1b_on=1;

    }
    if(Flag_turn==5&&time_dl1b_s > 525)
    {
        LMotorSpeed = RMotorSpeed =150;
        gpio_set_level(P33_10,0);
        time_dl1b_on=0;
        Flag_stop=6;

        Light_Flag=0;
        Distance_ON=1;
            if(Distance>100)
            {

            Find_Rhombus_Out(data);
            }
    }
    if(Rhombus_Out==1)
    {
        Distance_ON=0;
        Distance=0;

        Flag_stop=6;
        Flag_turn=0;

        Pwm_Min=822;
        Pwm_Max=882;

        Rhombus_Out=0;
        Rhombus_Flag=0;
        Deal_Flag++;
   //     Distance_Test_ON=1;
    }
}
void  Tiao_Light(uint8 *data)
{
        uint8 i,j;
        uint32 a;
        a=0;
        for(i=30;i>5;i--)
        {
            for(j=55;j<65;j++)
            {
                if(data[i*COL+j])
                {
                    a++;
                    time_dl1b_on=1;
                }
            }
        }
        if(a>=1)
        {
            Differ_light++;
        }
}

