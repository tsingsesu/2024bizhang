/*
 * Lu_UI.c
 *
 *  Created on: 2023年3月9日
 *      Author: 86185
 */
/*
 * 2023.6.28第 1 版调参UI——lujunjie
 */
#include "Lu_UI.h"
/* [2026-10-20] The display re-thresholds imagebinary itself.  imagebinary
   holds 0/255 now; another zyuzhi comparison is redundant and was the reason
   the picture could go all black.  0 = show the array as-is (grayscale). */
#define LU_IMG_THRESHOLD    0
/* 1 = page1 shows the RAW grayscale frame, 0 = binary image */
#define LU_IMG_RAW_PREVIEW  1   // [2026-10-22] 先看灰度原图，判断亮度和摄像头方向
/* [2026-10-20] lu_ui_dbg_log is forwarded from CPU1 so the print happens on
   CPU0 (the only core owning the debug UART).  Keep it 0 during matches. */
volatile uint8 lu_ui_dbg_log = 0;
uint8 lcd_raw_view = 1;   // ★屏幕显示：1=灰度原图  0=二值图（长按 S4 切）
// [2026-10-21] 与外界通信已删除：这里不再 printf，保留空函数供 cpu0_main 调用
void LuUI_Log_Poll(void)
{
    lu_ui_dbg_log = 0;
}
#include "zf_common_headfile.h"
int8 Page_Choose_flag = 0;
int8 Parameter_Choose_flag = 0;
uint8 Page_Clear_flag = 0;
uint8 Key_Protect[KEY_NUMBER] = {0};
uint16 Show_Y[]={0,20,40,60,80,100,120,140,160,180,200,220,240,260,280,300};
uint16 Show_X[]={0,5,150};
int16 canshu_test[8] = {0};
int8 Differ = 13;    // ★实测调好的阈值偏移（原 0）

// ================= ★摄像头曝光 / 增益（调参界面可改） =================
// 曝光：MT9V03X_EXP_TIME_DEF 默认 500，范围约 50~2000，越大越亮
// 增益：MT9V034 全局增益寄存器 0x35，范围 16~64，越大越亮（噪点也越多）
uint16 cam_exposure = 700;       /* [2026-10-26] 2400->700：换到亮场地后实测 ExpTime 700 / Gain 16，屏幕 iavg=186、imax=255 */
uint8  cam_gain     = 16;        /* [2026-10-26] 64->16：亮场地不需要增益，跟屏幕一致 */

uint8 steer_mid_cal = 0;   // [2026-10-22] 1 = 舵机中位校准模式（长按 S2 切换）      // ★实测调好的增益（原 63）          // ★默认增益（原 63 偏高，噪点大）

/* [2026-10-22] 保存参数写入结果：0=成功 1=失败。
   图像一直很暗时先看这两个值——如果是 1，说明 SCCB 写不进去，
   调曝光增益根本没用，问题在摄像头通信/供电，不在参数。 */
uint8 cam_set_exp_ret  = 0xFF;
uint8 cam_set_gain_ret = 0xFF;

void cam_param_apply(void)
{
    cam_set_exp_ret  = mt9v03x_set_exposure_time(cam_exposure);   // 命令 0xF0
    cam_set_gain_ret = mt9v03x_set_reg(0x35, cam_gain);           // MT9V034 Global Gain
}

/*************************************************************************
 *
 *  功能说明：屏幕显示页
 *
 *************************************************************************/
/* [2026-10-24] 锥形标识别结果（必须在 page1 之前声明）*/
extern uint8 cone_px[], cone_py[];
extern volatile uint8 cone_pts, cone_cnt, cone_near, cone_ok;
extern volatile int16 cone_x;

void LuUI_Show_page1(void)
{
    if(mt9v03x_finish_flag)       //总钻风图像
        {
            /* [2026-10-20] DIAG: show the RAW camera image in grayscale.
               Set to 0 to go back to the binary (imagebinary) view. */
            if(lcd_raw_view)
                ips200_show_gray_image(0, 0,*mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 140, 0);
            else
                ips200_show_gray_image(0, 0,*imagebinary, MT9V03X_W, MT9V03X_H, 240, 140, LU_IMG_THRESHOLD);   /* [2026-10-20] was zyuzhi */     // 显示二值化图像
            /* [2026-10-20] before/after the glare pass:
               red/blue   = edge lines as Image_Handle produced them
               green/yellow = after Glare_Suppress (what the car uses) */
            DisplayLine(agl_L_before,   RGB565_RED);
            DisplayLine(agl_R_before,   RGB565_BLUE);
            DisplayLine(Left_Add_Line,  RGB565_GREEN);
            DisplayLine(Right_Add_Line, RGB565_YELLOW);
            for(uint8 i = 59;i>5;i--)
            {
               ips200_drawpoint1(2*Fictitious_Midline[i], 2*i,RGB565_YELLOW);
            }
            for(uint8 i=2;i<238;i++)
            {
                ips200_draw_point(i,5*2,RGB565_PURPLE);
            }
            for(uint8 i=2;i<238;i++)
            {
                ips200_draw_point(i,30*2,RGB565_PURPLE);
            }
            for(uint8 j = 2; j < 138; j++)
            {
                ips200_draw_point(56*2,j,RGB565_PURPLE);
            }
            for(uint8 j = 2; j < 138; j++)
            {
                ips200_draw_point(64*2,j,RGB565_PURPLE);
            }

            /* [2026-10-24] 锥形标识别结果：青色点 = 被判为"锥桶"的粗白块
               看图就能确认识别对不对：点应该正好落在每个锥桶上 */
            /* 用红色 3x3 方块：二值图是黑白底，红色在两种底色上都看得见，
               青色画在白块上几乎看不见（之前就是这个问题）。 */
            for(uint8 k = 0; k < cone_pts; k ++)
            {
                uint16 dx = (uint16)(2 * cone_px[k]);
                uint16 dy = (uint16)(2 * cone_py[k]);
                ips200_drawpoint1(dx,   dy,   RGB565_RED);
                ips200_drawpoint1(dx+1, dy,   RGB565_RED);
                ips200_drawpoint1(dx,   dy+1, RGB565_RED);
                ips200_drawpoint1(dx+1, dy+1, RGB565_RED);
                ips200_drawpoint1(dx+2, dy+1, RGB565_RED);
                ips200_drawpoint1(dx+1, dy+2, RGB565_RED);
            }
            mt9v03x_finish_flag=0;
        }

    ips200_show_int(Show_X[1],Show_Y[7],dl1b_distance_mm,4);

    /* [2026-10-22] page3 新增 STrim 一行后，最下面一行 y 会超屏；
       这里改成 uint8 用 show_int 画，3 位 = 150+26=176，留足余量。
       （ips200_show_int 会画 位数*2+2 个字符，x 超 232 就触发 zf_assert） */
    ips200_show_int(Show_X[2],Show_Y[8],Flag_stop,3);
    ips200_show_int(Show_X[1],Show_Y[8],Flag_turn,3);

    ips200_show_int(Show_X[1],Show_Y[9],Rhombus_Flag,4);
    ips200_show_int(Show_X[2],Show_Y[9],Deal_Flag,4);

    ips200_show_int(Show_X[1],Show_Y[10],Distance_Test,4);
    ips200_show_int(Show_X[2],Show_Y[10],Distance,4);

    ips200_show_int(Show_X[1],Show_Y[11],Rhombus_Out_Flag,4);
    ips200_show_int(Show_X[2],Show_Y[11],Rhombus_Out,4);

    ips200_show_int(Show_X[1],Show_Y[12],time_dl1b_s,4);
    ips200_show_int(Show_X[2],Show_Y[12],time_dl1b_s,4);

    ips200_show_int(Show_X[1],Show_Y[13],Distance,4);
    ips200_show_int(Show_X[2],Show_Y[13],SD5_Duty,4);
    /* [2026-10-20] live camera diagnostics */
    ips200_show_int(Show_X[1],Show_Y[14],diedai_threshold,4);
    ips200_show_int(Show_X[2],Show_Y[14],zyuzhi + Differ_light,4);
    /* [2026-10-20] Point vs centre: Point > Point_Mid(60) means "steer right".*/
    ips200_show_int(Show_X[1],Show_Y[15],Point,4);
    ips200_show_int(Show_X[2],Show_Y[15],SD5_Duty,4);
//
//    ips200_show_float(Show_X[1],Show_Y[14],KR,4,4);
//    ips200_show_float(Show_X[2],Show_Y[14],KL,4,4);
//
//
 //   ips200_show_int(Show_X[1],Show_Y[15],Stop_Flag,4);
//    ips200_show_int(Show_X[2],Show_Y[15],Turnoff_Flag,4);
}
void LuUI_Show_page2(void)
{
        ips200_show_int(Show_X[1],Show_Y[7],Lcount,4);
        ips200_show_int(Show_X[2],Show_Y[7],Rcount,4);

        ips200_show_int(Show_X[1],Show_Y[9],LMotorSpeed,4);
        ips200_show_int(Show_X[2],Show_Y[9],RMotorSpeed,4);
//
        ips200_show_int(Show_X[1],Show_Y[11],Lrealspeed,4);
        ips200_show_int(Show_X[2],Show_Y[11],Rrealspeed,4);
//
        ips200_show_int(Show_X[1],Show_Y[14],LSpeedPWM,4);
        ips200_show_int(Show_X[2],Show_Y[14],RSpeedPWM,4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[4],"InductorValue[3]",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[4],InductorValue_Read[3],4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[5],"Inductor_Error_1_4",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[5],Inductor_Error_1_4,4);
//
//    ips200_show_int(Show_X[1],Show_Y[6],ad_max[3],4);
//    ips200_show_int(Show_X[2],Show_Y[6],ad_max[0],4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[7],"Use_Inductor_Flag",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[7],Use_Inductor_Flag,4);
//    //
//    //    ips200_show_string_color(Show_X[1],Show_Y[7],"Current_Speed_L",RGB565_BLACK,RGB565_YELLOW);
//    //    ips200_show_int(Show_X[2],Show_Y[7],Current_Speed_L,4);
//    ips200_show_string_color(Show_X[1],Show_Y[11],"Roadblock_Flag",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[11],Roadblock_Flag,4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[12],"Roadblock_Count",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[12],Roadblock_Count,4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[13],"Ramp_Flag",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[13],Ramp_Flag,4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[14],"Flag1",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[14],Flag1,4);
//
//    ips200_show_string_color(Show_X[1],Show_Y[15],"Ring_Count",RGB565_BLACK,RGB565_YELLOW);
//    ips200_show_int(Show_X[2],Show_Y[15],Ring_Count,4);
}
extern uint8 wifi_init_step;
extern char  wifi_spi_version[12];   // ★模块固件版本（能读出=SPI通）
extern int16 Steer_Trim;
extern int16 lane_k;
extern int32 steer_target;
/* [2026-10-24] ToF 实时诊断：发车判定就看这三个数 */
extern volatile uint16 start_base, start_hold;
extern volatile uint8  start_ready;
extern uint8 cam_send_raw;
extern uint8 cam_init_ret;
extern vuint8 mt9v03x_finish_flag;
extern uint8 wifi_raw_cmd;
extern uint8 wifi_raw_dat[3];
extern uint8 wifi_dbg_cmd;
extern uint8 wifi_dbg_int;
extern uint8 wifi_init_ret;   // ★WiFi 模块初始化返回值

void LuUI_Show_page3(void)
{
    uint8 i;

    /* ===== 6 个可调参数：' > ' 指向当前选中项 ===== */
    ips200_show_string(16, Show_Y[1], "ZYuzhi");
    ips200_show_int(150, Show_Y[1], zyuzhi, 4);

    ips200_show_string(16, Show_Y[2], "Differ");
    ips200_show_int(150, Show_Y[2], Differ, 4);

    ips200_show_string(16, Show_Y[3], "ExpTime");
    ips200_show_int(150, Show_Y[3], cam_exposure, 5);

    ips200_show_string(16, Show_Y[4], "Gain");
    ips200_show_int(150, Show_Y[4], cam_gain, 4);

    ips200_show_string(16, Show_Y[5], "STrim");
    ips200_show_int(150, Show_Y[5], Steer_Trim, 4);

    ips200_show_string(16, Show_Y[6], "LaneK");
    ips200_show_int(150, Show_Y[6], lane_k, 4);

    for(i = 1; i <= 6; i ++)
    {
        if(i == (Parameter_Choose_flag + 1)) ips200_show_char(0, Show_Y[i], '>');
        else                                  ips200_show_char(0, Show_Y[i], ' ');
    }

    /* ===== 识别状态 ===== */
    ips200_show_string(16,  Show_Y[7], "T0");
    ips200_show_int(60,     Show_Y[7], diedai_threshold, 3);
    ips200_show_string(110, Show_Y[7], "W");
    ips200_show_int(140,    Show_Y[7], Width_Real[22], 3);

    ips200_show_string(16,  Show_Y[8], "L");
    ips200_show_int(60,     Show_Y[8], Left_Line[22], 3);
    ips200_show_string(110, Show_Y[8], "R");
    ips200_show_int(140,    Show_Y[8], Right_Line[22], 3);

    ips200_show_string(16,  Show_Y[9], "TP");
    ips200_show_int(60,     Show_Y[9], steer_target, 3);
    ips200_show_string(110, Show_Y[9], "P");
    ips200_show_int(140,    Show_Y[9], Point, 3);

    ips200_show_string(16,  Show_Y[10], "imin");
    ips200_show_int(60,     Show_Y[10], diedai_img_min, 3);
    ips200_show_string(110, Show_Y[10], "imax");
    ips200_show_int(164,    Show_Y[10], diedai_img_max, 3);

    ips200_show_string(16,  Show_Y[11], "iavg");
    ips200_show_int(60,     Show_Y[11], diedai_img_avg, 3);
    ips200_show_string(110, Show_Y[11], "LC");
    ips200_show_int(140,    Show_Y[11], Line_Count, 3);

    ips200_show_string(16,  Show_Y[12], "Mid");
    ips200_show_int(60,     Show_Y[12], Steer_Mid, 4);
    ips200_show_string(140, Show_Y[12], "Trm");
    ips200_show_int(180,    Show_Y[12], Steer_Trim, 3);
    if(steer_mid_cal) ips200_show_string(110, Show_Y[12], "CAL");

    ips200_show_string(16,  Show_Y[13], "Duty");
    ips200_show_int(70,     Show_Y[13], SD5_Duty, 4);
    ips200_show_string(150, Show_Y[13], "WiFi");
    ips200_show_int(200,    Show_Y[13], wifi_init_step, 2);

    ips200_show_string(16,  Show_Y[14], "CW");
    ips200_show_int(50,     Show_Y[14], mk_cw_seen, 1);
    ips200_show_int(70,     Show_Y[14], mk_cw_score, 2);
    ips200_show_string(110, Show_Y[14], "DI");
    ips200_show_int(140,    Show_Y[14], mk_dia_seen, 1);
    ips200_show_int(160,    Show_Y[14], mk_dia_score, 2);
    ips200_show_string(196, Show_Y[14], "MK");
    ips200_show_int(220,    Show_Y[14], mk_state, 1);

    /* ★ToF 实时诊断（2026-10-24）：挥手时看这三个数怎么变
       To = 当前读数（8192 = 前方空旷，测不到东西）
       A  = 是否已布防（1 = 看够了 1 秒空旷，可以挥手）
       H  = 最近 12 个样本里被判为"近距"的个数，到 8 就发车 */
    ips200_show_string(16,  Show_Y[15], "To");
    ips200_show_int(50,     Show_Y[15], dl1b_distance_mm, 5);
    ips200_show_string(105, Show_Y[15], "A");
    ips200_show_int(130,    Show_Y[15], start_ready, 1);
    ips200_show_string(175, Show_Y[15], "H");
    ips200_show_int(200,    Show_Y[15], start_hold, 2);
}

void LuUI_Show(void)
{
    if(Page_Clear_flag){ ips200_clear(); Page_Clear_flag = 0; }
    switch(Page_Choose_flag)
        {
            case 0:{ LuUI_Show_page1(); break; }
            case 1:{ LuUI_Show_page2(); break; }
            case 2:{ LuUI_Show_page3(); break; }
            default:break;
        };
}
void Computer_Show(void)
{


}
void LuUI_Key(void)
{
    key_scanner();

    // ---------- S2 : 切页 ----------
    if(key_get_state(KEY_UP)==KEY_SHORT_PRESS && Key_Protect[KEY_UP]==0)
    {
        Page_Choose_flag ++;
        Page_Choose_flag = (Page_Choose_flag < PAGES) ? Page_Choose_flag : 0;
        Page_Clear_flag = 1;
        Key_Protect[KEY_UP] = 1;
    }
    else if(key_get_state(KEY_UP)== KEY_RELEASE)
    {
        Key_Protect[KEY_UP] = 0;
    }

    // ---------- ★长按 S2 : 进入/退出舵机中位校准 ----------
    /* [2026-10-22] 拉杆/舵机臂重新装过之后，代码里的 857 不再是中位。
       进入校准后：L = S3/S4（每下 2），R = S5（每下 1），长按 S5 保存到 flash。
       先进入 page3 再长按 S2。 */
    {
        static uint8 cal_done = 0;
        if(key_get_state(KEY_UP) == KEY_LONG_PRESS)
        {
            if(0 == cal_done)
            {
                steer_mid_cal = steer_mid_cal ? 0 : 1;
                Page_Clear_flag = 1;
                cal_done = 1;
            }
        }
        else
        {
            cal_done = 0;
        }
    }

    // ---------- S5 : 切换要调的变量 ----------
    if(key_get_state(KEY_RIGHT)==KEY_SHORT_PRESS && Key_Protect[KEY_RIGHT]==0)
    {
        if(steer_mid_cal)
        {
            Steer_Set_Mid(Steer_Mid + 1);   /* S5 精调 +1 */
            Page_Clear_flag = 1;
        }
        else if(Page_Choose_flag == 2)
        {
            Parameter_Choose_flag ++;
            if(Parameter_Choose_flag > 5) Parameter_Choose_flag = 0;   // 0=zyuzhi 1=Differ 2=曝光 3=增益 4=转向
            Page_Clear_flag = 1;
        }
        Key_Protect[KEY_RIGHT] = 1;
    }
    else if(key_get_state(KEY_RIGHT)== KEY_RELEASE)
    {
        Key_Protect[KEY_RIGHT] = 0;
    }

    // ---------- S3 : 减小 ----------
    if(key_get_state(KEY_DOWN)==KEY_SHORT_PRESS && Key_Protect[KEY_DOWN]==0)
    {
        if(steer_mid_cal)
        {
            Steer_Set_Mid(Steer_Mid - 5);
        }
        else if(Page_Choose_flag == 2)
        {
            switch(Parameter_Choose_flag)
            {
                case 0: if(zyuzhi > 20) zyuzhi -= 4; break;
                case 1: if(Differ > -50) Differ -= 1; break;   // [2026-10-22] 与 flash 校验范围一致
                case 2: if(cam_exposure > 60) { cam_exposure -= 100; cam_param_apply(); } break;
                case 3: if(cam_gain > 16)   { cam_gain     -= 4;  cam_param_apply(); } break;
                case 4: if(Steer_Trim > -60) Steer_Trim -= 2; break;
                default: if(lane_k > 20)     lane_k     -= 2; break;
            }
        }
        Key_Protect[KEY_DOWN] = 1;
    }
    else if(key_get_state(KEY_DOWN)== KEY_RELEASE)
    {
        Key_Protect[KEY_DOWN] = 0;
    }

    // ---------- S4 : 增大 ----------
    if(key_get_state(KEY_LEFT)==KEY_SHORT_PRESS && Key_Protect[KEY_LEFT]==0)
    {
        if(steer_mid_cal)
        {
            Steer_Set_Mid(Steer_Mid + 5);
        }
        else if(Page_Choose_flag == 2)
        {
            switch(Parameter_Choose_flag)
            {
                case 0: if(zyuzhi < 255) zyuzhi += 4; break;
                case 1: if(Differ < 50) Differ += 1; break;
                case 2: if(cam_exposure < 10000) { cam_exposure += 100; cam_param_apply(); } break;
                case 3: if(cam_gain < 124)     { cam_gain     += 4;  cam_param_apply(); } break;  // [2026-10-24] 64=1.0x增益，放开到124(0x7C)=1.94x；0x80是符号位不能用
                case 4: if(Steer_Trim < 60)  Steer_Trim += 2; break;
                default: if(lane_k < 95)      lane_k     += 2; break;
            }
        }
        Key_Protect[KEY_LEFT] = 1;
    }
    else if(key_get_state(KEY_LEFT)== KEY_RELEASE)
    {
        Key_Protect[KEY_LEFT] = 0;
    }

    // ---------- ★长按 S5 : 保存参数到 flash ----------
    {
        static uint8 save_done = 0;
        if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)
        {
            if(0 == save_done)
            {
                Parameter_Save();
                save_done = 1;
            }
        }
        else
        {
            save_done = 0;
        }
    }

    // ---------- ★长按 S4 : 图传内容 灰度/二值 切换 ----------
    {
        static uint8 view_done = 0;
        if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS)
        {
            if(0 == view_done)
            {
                lcd_raw_view = (lcd_raw_view) ? 0 : 1;
                view_done = 1;
            }
        }
        else
        {
            view_done = 0;
        }
    }

    // ---------- ★长按 S3 : 恢复默认（仅第3页） ----------
    if(Page_Choose_flag == 2)
    {
        static uint8 reset_done = 0;
        if(key_get_state(KEY_DOWN) == KEY_LONG_PRESS)
        {
            if(0 == reset_done)
            {
                zyuzhi       = 95;   // [2026-10-22] 跟默认值一致
                Differ       = 13;
                Steer_Trim   = 16;
                lane_k       = 75;    // [2026-10-22] 恢复默认时转向也要回 0
                cam_exposure = 700;    // [2026-10-26] 长按 DOWN 恢复默认时也用 700
                cam_gain     = 16;     // [2026-10-26] 同上
                cam_param_apply();
                reset_done   = 1;
            }
        }
        else
        {
            reset_done = 0;
        }
    }
}
void Parameter_Save(void)
{
    // ★保存标记：读回时用它判断是否存过
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    flash_buffer_clear();
    flash_union_buffer[0].float_type = (float)zyuzhi;
    flash_union_buffer[1].float_type = (float)Differ;
    flash_union_buffer[2].float_type = (float)cam_exposure;
    flash_union_buffer[3].float_type = (float)cam_gain;
    flash_union_buffer[5].float_type = (float)Steer_Trim;
    flash_union_buffer[6].float_type = (float)Steer_Mid;    // [2026-10-22] 舵机中位
    flash_union_buffer[7].float_type = (float)lane_k;
    flash_union_buffer[4].float_type = 12345.0f;          // ★有效标记
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    /* [2026-10-22] 同时存一份到第 17 页：现场调好的值先冻结住，
       以后把 PARAM_TUNING 改回 0 并核对无误，再从这里恢复正常读回。 */
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_BAK) && EEPROM_PAGE_NUM > FLASH_PAGE_BAK)
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_BAK);
    flash_buffer_clear();
    flash_union_buffer[0].float_type = (float)zyuzhi;
    flash_union_buffer[1].float_type = (float)Differ;
    flash_union_buffer[2].float_type = (float)cam_exposure;
    flash_union_buffer[3].float_type = (float)cam_gain;
    flash_union_buffer[5].float_type = (float)Steer_Trim;
    flash_union_buffer[6].float_type = (float)Steer_Mid;
    flash_union_buffer[7].float_type = (float)lane_k;
    flash_union_buffer[4].float_type = 12345.0f;
    if(EEPROM_PAGE_NUM > FLASH_PAGE_BAK)
        flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_BAK);
}
void Parameter_Read(void)
{
    /* [2026-10-22] PARAM_TUNING=1（见 Lu_UI.h）：本阶段不读 flash，
       始终用代码里的默认值。比赛定稿后改回 0。 */
#if PARAM_TUNING
    /* [2026-10-22] 调参阶段其它参数用代码默认值，但舵机中位是现场校准出来的，
       必须从 flash 恢复，否则每次上电都被打回 857。 */
    #if FLASH_RESTORE_STEER_MID   /* [2026-10-26] 调参阶段不用 flash 里的旧中位 */
    if(EEPROM_PAGE_NUM > FLASH_PAGE_BAK)      /* [2026-10-22] 页号越界会触发 zf_assert */
    {
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_BAK);
    if(flash_union_buffer[4].float_type > 12300.0f && flash_union_buffer[4].float_type < 12400.0f)
    {
        float m = flash_union_buffer[6].float_type;
        if(m >= 700.0f && m <= 1000.0f) Steer_Mid = (int16)m;
    }
    }
    #endif
    return;
#endif
    float z, d, e, g, mark;

    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    mark = flash_union_buffer[4].float_type;
    if(mark < 12300.0f || mark > 12400.0f)
    {
        return;                                            // ★没存过 -> 保留代码里的默认值
    }

    z = flash_union_buffer[0].float_type;
    d = flash_union_buffer[1].float_type;
    e = flash_union_buffer[2].float_type;
    g = flash_union_buffer[3].float_type;

    // ★范围校验，防止上次读出 0 导致全黑
    if(z >= 20.0f  && z <= 255.0f)   zyuzhi       = (uint8)z;
    if(d >= -50.0f && d <= 50.0f)    Differ       = (int8)d;
    if(e >= 60.0f  && e <= 10000.0f) cam_exposure = (uint16)e;
    if(g >= 16.0f  && g <= 124.0f)   cam_gain     = (uint8)g;   // [2026-10-24] 随增益上限一起放到 124
    if(flash_union_buffer[5].float_type >= -60.0f && flash_union_buffer[5].float_type <= 60.0f)
        Steer_Trim = (int16)flash_union_buffer[5].float_type;
    if(flash_union_buffer[7].float_type >= 20.0f && flash_union_buffer[7].float_type <= 95.0f)
        lane_k = (int16)flash_union_buffer[7].float_type;
}

/*
 *  @brief      显示线的函数
 */
void DisplayLine(uint16 *p, uint16 color)
{
    uint8 i;
    uint16 x;
        for( i = 60; i >= Line_Count; i-=1)                         //显示线
        {
             x = p[i];
             ips200_drawpoint1(2*x, 2*i,color);
        }
}
