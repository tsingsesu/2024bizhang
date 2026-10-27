/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

/* [2026-10-22] 日志版本号放在本文件：每次要改版本就等于改本文件，
   保证这份源码一定会被重新编译。放头文件里会出现 .o 没重编、
   日志仍打印旧版本号的情况（V8 实测打印成 V6）。 */
#define CAR_LOG_VERSION           (27)   // V27: choose a fresh near-field lane row instead of rejecting the whole frame
/* [2026-10-22] 发车方式
   0 = 不自动发车，只能人工放行（调试用；板上只有 S2~S5，没有 S1）
   1 = ToF 遮挡发车（当前，比赛要求非接触发车） */
#define START_MODE                (2)      // 1=ToF非接触发车  2=调试自动发车      // ★ 1=ToF发车(比赛)  2=调试自动发车
#define START_DEBUG_DELAY_MS      (20000)  // ★ 调试模式：上电多少毫秒后自动走

// ================= ★WiFi(SPI) 图传 → 逐飞助手 =================
// 1 = 开（模块未到或没配网时请改 0，否则上电会多等几秒）
#define CAMERA_SEND_ENABLE      (0)   // [2026-10-26] 1->0：图看完了，关掉省得和日志抢连接
                                      //   要看图：改回 1，并把 wifi_spi.h 的 AUTO_CONNECT 改回 1(逐飞助手TCP)

// ★图传内容：1 = 原始灰度图（调曝光/增益用）  0 = 二值图（调阈值/跑车用）
// ★图传内容：1 = 灰度原图  0 = 二值图（长按 S4 就地切换）
uint8 cam_send_raw = 1;

// ★图传降频：每 N 帧新图发一条，主循环更跟手（1 = 每帧都发）
#define CAM_SEND_DIV            (2)

// ================= ★UDP 日志（2026-10-22 新增）=================
// 1 = 通过 WiFi 向电脑发日志（电脑端跑 tools/car_log.py 接收）
// 0 = 关掉，比赛时可以关
#define CAR_LOG_ENABLE          (1)   // [2026-10-26] 0->1：恢复 UDP 日志，tools/run_log_save.bat 接收
#define CAR_LOG_PERIOD_LOOPS    (10)      // 每 N 圈主循环发一条（一次发送约 250ms）
static uint16 car_log_cnt  = 0;
static uint32 car_log_tick = 0;

extern uint8 end_stopped;
extern volatile uint8 mk_near_white;
extern uint16 steer_guard_bits;
extern int32  steer_target;
extern volatile uint32 isr_tick_5ms;
/* [2026-10-24] 锥形标识别结果 */
extern volatile uint16 stop_aaa;        /* [2026-10-26] 终点探测窗口白点数 */
extern volatile uint16 stop_run;
extern volatile uint8  stop_now, stop_latch;
extern volatile uint8 cone_cnt, cone_near, cone_ok;
extern volatile int16 cone_x;
extern volatile uint8 cone_group_ok, cone_group_support, cone_group_span_x, cone_group_span_y, cone_group_error;
extern volatile uint16 cone_wh[5];
extern uint8 wifi_init_step;
static uint8 cam_send_cnt = 0;   // ★图传降频计数
uint8 wifi_init_ret = 0xFF;      // ★wifi_spi_init() 的返回值：0=成功，其他=失败
#if CAMERA_SEND_ENABLE
static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static seekfree_assistant_camera_struct camera_information;
#endif
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

volatile uint32 main_loop_tick = 0;
volatile uint32 vision_processed_frames = 0;
volatile uint8 steer_settle = 1;   // [2026-10-22] 1 = 起跑后仍在中位保护期

int core0_main(void)
{
   clock_init();                   // 获取时钟频率<务必保留>
   /* [2026-10-22] 时钟一好就输出舵机中位，再去做延时/串口/WiFi。
      原来 1.5 秒延时在舵机初始化之前，这段时间信号线悬空，舵机没有保持力会被拽到一边。 */
   pwm_init(S_MOTOR_PIN, 50, Steer_Mid);
   gpio_init(MOTOR1_A, GPO, GPIO_LOW, GPO_PUSH_PULL);
   gpio_init(MOTOR2_A, GPO, GPIO_LOW, GPO_PUSH_PULL);
   pwm_init(MOTOR1_B, 17000, 0);
   pwm_init(MOTOR2_B, 17000, 0);
   system_delay_ms(1500);
   debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    System();
    Parameter_Read();      // ★从 flash 读上次保存的参数（没存过就用默认）
    /* [2026-10-22] 读回校准过的舵机中位后，同步 PWM 和左右限幅 */
    Steer_Set_Mid(Steer_Mid);
#define CAM_APPLY_AT_BOOT  (1)   // [2026-10-26] 0->1：上电就把 cam_exposure/cam_gain 写进相机，省得每次开机在屏幕上重调
#if CAM_APPLY_AT_BOOT
    cam_param_apply();     // ★让默认/读回的曝光增益真正写进相机
#endif
    // 此处编写用户代码 例如外设初始化代码等
    /* [2026-10-23] WiFi 初始化挪到 System() 之后：屏幕 2 秒就亮，
       WiFi 连上再发日志。原来放在前面会把屏幕拖到 30 秒才亮。 */
    // ★WiFi 模块初始化（不受图传开关影响，方便看 WiFiRet / WiFiStep）
    wifi_init_ret = wifi_spi_init(WIFI_SSID, WIFI_PASS);
    if(0 != wifi_init_ret) wifi_spi_raw_probe();   // ★诊断：绕开 INT 直接读
#if CAR_LOG_ENABLE
    { char log_buf[48];
      sprintf(log_buf, "V%d BOOT wifi_step=%d ret=%d ver=%s\r\n",
          (int)CAR_LOG_VERSION, (int)wifi_init_step, (int)wifi_init_ret, wifi_spi_version);
      wifi_spi_send_string(log_buf); }
#endif
#if CAMERA_SEND_ENABLE
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_config(&camera_information, SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                     MT9V03X_W, MT9V03X_H, image_copy[0]);
#endif

    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while(TRUE)
    {
        main_loop_tick ++;
#if CAMERA_SEND_ENABLE
        // WiFi 图传：30MHz SPI + WiFi，一帧约 7ms，不会卡主循环
        if(mt9v03x_finish_flag)        // ★只有新帧才发
        {
        if(cam_send_raw)
            memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);   // 灰度原图
        else
            memcpy(image_copy[0], imagebinary[0], MT9V03X_IMAGE_SIZE);     // 二值图
        cam_send_cnt ++;
        if(cam_send_cnt >= CAM_SEND_DIV)
        {
            cam_send_cnt = 0;
            seekfree_assistant_camera_send(&camera_information);
        }
        }
#endif
        LuUI_Log_Poll();   // [2026-10-20] CPU1-requested serial telemetry (off by default)
#if CAR_LOG_ENABLE
        // ★心跳：每 CAR_LOG_PERIOD_LOOPS 圈发一行关键量，电脑端 car_log.py 里看
        if(++ car_log_cnt >= CAR_LOG_PERIOD_LOOPS)
        {
            static char log_buf[1536];   // CPU0 stack is 2 KB; keep telemetry storage off the stack.
            car_log_cnt = 0;
            car_log_tick ++;
            /* [2026-10-22] 调参日志：FR = 本帧最远可信行；L/R/W = 第 22 行左右边线与宽度；
               T0 = Diedai 实际二值阈值；zy = 阈值系数+偏置；dl = Differ_light；
               Rm = Rhombus_Flag；To = DL1B 距离 */
            sprintf(log_buf, "V%d T=%lu P=%d LC=%d FR=%d L=%d R=%d W=%d d=%ld st=%d/%d Rm=%d T0=%d zy=%d dl=%d To=%d exp=%d g=%d imin=%d imax=%d iavg=%d se=%d sg=%d DI=%d/%d/%d CW=%d/%d/%d MK=%d MKs=%d SB=%d/%d/%d O=%d GB=0x%04X TP=%ld FC=%lu H128=%lu H192=%lu CN=%d/%d/%d CWD=%d/%d/%d/%d/%d SO=%d A=%d SR=%d SM=%d RT=%ld LK=%d MS=%lu PC=%lu VS=%ld/%ld VE=%d/%d VP=%ld/%ld ES=%d FS=%d CA=%d CG=%d GP=%d/%d/%d/%d DH=%d VL=%d/%d DC=%ld WT=%d CL=%d CE=%d NW=%d TD=%d TA=%lu LR=%d LN=%d/%d\r\n",
                (int)CAR_LOG_VERSION,
                (unsigned long)car_log_tick, (int)Point, (int)Line_Count, (int)Result_Far_Row,
                (int)Left_Line[22], (int)Right_Line[22], (int)Width_Real[22],
                (long)SD5_Duty, (int)Flag_stop, (int)Flag_turn, (int)Rhombus_Flag,
                (int)diedai_threshold, (int)zyuzhi + (int)Differ_light, (int)Differ_light,
                (int)dl1b_distance_mm, (int)cam_exposure, (int)cam_gain,
                (int)diedai_img_min, (int)diedai_img_max, (int)diedai_img_avg,
                (int)cam_set_exp_ret, (int)cam_set_gain_ret,
                (int)mk_dia_seen, (int)mk_dia_near, (int)mk_dia_score,
                (int)mk_cw_seen, (int)mk_cw_near, (int)mk_cw_score, (int)mk_state, (int)MK_ACT_ENABLE, (int)start_ready, (int)start_base, (int)start_hold,
                (int)Run_Odo, (unsigned)steer_guard_bits, (long)steer_target, (unsigned long)mt9v03x_frame_cnt,
                (unsigned long)diedai_img_hi128, (unsigned long)diedai_img_hi192,
                (int)cone_cnt, (int)cone_near, (int)cone_x,
                (int)cone_wh[0], (int)cone_wh[1], (int)cone_wh[2],
                (int)cone_wh[3], (int)cone_wh[4],
                 (int)stop_latch, (int)stop_aaa, (int)stop_run,
                 (int)steer_mode, (long)steer_raw_target, (int)lane_k,
                 (unsigned long)(isr_tick_5ms * 5UL), (unsigned long)vision_processed_frames,
                 (long)LMotorSpeed, (long)RMotorSpeed,
                 (int)Lrealspeed, (int)Rrealspeed,
                 (long)LSpeedPWM, (long)RSpeedPWM,
                 (int)end_stopped, (int)MK_FINAL_STOP, (int)CONE_ACT_ENABLE,
                 (int)cone_group_ok, (int)cone_group_support,
                 (int)cone_group_span_x, (int)cone_group_span_y, (int)cone_group_error,
                 (int)drive_inhibit, (int)vision_age_ticks, (int)track_bad_ticks,
                 (long)run_distance_cm, (int)mk_stop_elapsed_ms, (int)mk_clear_remaining_mm,
                 (int)mk_finish_reason, (int)mk_near_white,
                 (int)Tof_Get_Distance(), (unsigned long)Tof_Get_Age_Ms(),
                 (int)steer_look_row, (int)steer_lane_left, (int)steer_lane_right);
            wifi_spi_send_string(log_buf);
        }
#endif
        LuUI_Key();
        /* Startup centering is timed in the 5 ms steering ISR. */
        LuUI_Show();    // ★屏幕已确认可用（官方例程能亮）；坐标已核对不会触发 zf_assert
                        //   LuUI_Key() 必须保留（按键扫描在里面）
        //////感应发车/////////////

        if(Stop_1==0)
        {
            /* [2026-10-24] 发车只看 start_trigger（isr 每 5ms 更新）：
                 快速通道：连续 5 个有效样本都比基线近 15mm → 立刻发车（约 0.25s）
                 常规通道：最近 10 个有效样本里 7 个被遮挡 → 发车
               8192 是传感器没测完，已经被过滤，不参与统计。*/
            gpio_set_level(P02_4,1);//左前转向
            gpio_set_level(P02_5,1);//左后转向
            gpio_set_level(P02_6,1);//右前转向
            gpio_set_level(P02_7,1);//右后转向
            gpio_set_level(P20_11,1);//左前大灯
            gpio_set_level(P20_12,1);//右前大灯
            gpio_set_level(P20_13,1);//左后刹灯
            gpio_set_level(P20_14,1);//右后刹灯

            /* ★调试模式：不看 ToF，上电 N 秒后直接发车 */
            if(((1 == START_MODE) && start_trigger) ||
               ((2 == START_MODE) && (isr_tick_5ms >= (START_DEBUG_DELAY_MS / 5))))
            {
                gpio_set_level(P02_4,0);//左前转向
                gpio_set_level(P02_5,0);//左后转向
                gpio_set_level(P02_6,0);//右前转向
                gpio_set_level(P02_7,0);//右后转向
                gpio_set_level(P20_11,0);//左前大灯
                gpio_set_level(P20_12,0);//右前大灯
                gpio_set_level(P20_13,0);//左后刹灯
                gpio_set_level(P20_14,0);//右后刹灯
                Flag_turn=3;
                Flag_stop=0;
                Deal_Flag=0;
                mode_2024=1;
                time_dl1b_s=0;
                time_dl1b=0;
                time_dl1b_on=0;
                Stop_1=1;
            }
        }


    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：LED 不闪烁
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
// 问题2：SWITCH1 / SWITCH2 更改组合流水灯频率无变化
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
//      万用表检查对应 SWITCH1 / SWITCH2 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错
// 问题2：KEY1 / KEY2 / KEY3 / KEY4 接GND或者按键按下无变化
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
//      万用表检查对应 KEY1 / KEY2 / KEY3 / KEY4 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错

