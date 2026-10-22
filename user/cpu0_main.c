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
#define CAR_LOG_VERSION           (15)
/* [2026-10-22] 发车方式
   0 = 仅长按 S1（当前，推荐）：ToF 装在低位、一直读地面 44~130mm，
       8192 还是通信超时的无效值，用它发车会一上电就自己跑
   1 = ToF 连续遮挡 2 秒（只有在确认 ToF 能稳定读到 >400mm 空旷时才用） */
#define START_MODE                (1)

// ================= ★WiFi(SPI) 图传 → 逐飞助手 =================
// 1 = 开（模块未到或没配网时请改 0，否则上电会多等几秒）
#define CAMERA_SEND_ENABLE      (0)   // [2026-10-22] 图传暂关：它和 UDP 日志共用一条连接会串数据
                                      //   要看图：改回 1，并把 wifi_spi.h 的 AUTO_CONNECT 改回 1(逐飞助手TCP)

// ★图传内容：1 = 原始灰度图（调曝光/增益用）  0 = 二值图（调阈值/跑车用）
// ★图传内容：1 = 灰度原图  0 = 二值图（长按 S4 就地切换）
uint8 cam_send_raw = 1;

// ★图传降频：每 N 帧新图发一条，主循环更跟手（1 = 每帧都发）
#define CAM_SEND_DIV            (2)

// ================= ★UDP 日志（2026-10-22 新增）=================
// 1 = 通过 WiFi 向电脑发日志（电脑端跑 tools/car_log.py 接收）
// 0 = 关掉，比赛时可以关
#define CAR_LOG_ENABLE          (1)
#define CAR_LOG_PERIOD_LOOPS    (10)      // 每 N 圈主循环发一条（一次发送约 250ms）
static uint16 car_log_cnt  = 0;
static uint32 car_log_tick = 0;

extern uint8 wifi_init_step;
static uint8 cam_send_cnt = 0;   // ★图传降频计数
uint8 wifi_init_ret = 0xFF;      // ★wifi_spi_init() 的返回值：0=成功，其他=失败
#if CAMERA_SEND_ENABLE
static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static seekfree_assistant_camera_struct camera_information;
#endif
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

volatile uint32 main_loop_tick = 0;
volatile uint8 steer_settle = 0;   // [2026-10-22] 1 = 起跑后仍在中位保护期

int core0_main(void)
{
   clock_init();                   // 获取时钟频率<务必保留>
   /* [2026-10-22] 时钟一好就输出舵机中位，再去做延时/串口/WiFi。
      原来 1.5 秒延时在舵机初始化之前，这段时间信号线悬空，舵机没有保持力会被拽到一边。 */
   pwm_init(S_MOTOR_PIN, 50, SteerMiddle);
   gpio_init(MOTOR1_A, GPO, GPIO_LOW, GPO_PUSH_PULL);
   gpio_init(MOTOR2_A, GPO, GPIO_LOW, GPO_PUSH_PULL);
   pwm_init(MOTOR1_B, 17000, 0);
   pwm_init(MOTOR2_B, 17000, 0);
   system_delay_ms(1500);
   debug_init();                   // 初始化默认调试串口
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
    // 此处编写用户代码 例如外设初始化代码等
    System();
    Parameter_Read();      // ★从 flash 读上次保存的参数（没存过就用默认）
    cam_param_apply();     // ★让默认/读回的曝光增益真正写进相机
    // 此处编写用户代码 例如外设初始化代码等
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
            char log_buf[320];   // 30 个字段，留足余量，防止 sprintf 溢出
            car_log_cnt = 0;
            car_log_tick ++;
            /* [2026-10-22] 调参日志：FR = 本帧最远可信行；L/R/W = 第 50 行左右边线与宽度；
               T0 = Diedai 实际二值阈值；zy = 阈值系数+偏置；dl = Differ_light；
               Rm = Rhombus_Flag；To = DL1B 距离 */
            sprintf(log_buf, "V%d T=%lu P=%d LC=%d FR=%d L=%d R=%d W=%d d=%ld st=%d/%d Rm=%d T0=%d zy=%d dl=%d To=%d exp=%d g=%d imin=%d imax=%d iavg=%d se=%d sg=%d DI=%d/%d/%d CW=%d/%d/%d MK=%d MKs=%d SB=%d/%d/%d O=%d\r\n",
                (int)CAR_LOG_VERSION,
                (unsigned long)car_log_tick, (int)Point, (int)Line_Count, (int)Result_Far_Row,
                (int)Left_Line[50], (int)Right_Line[50], (int)Width_Real[50],
                (long)SD5_Duty, (int)Flag_stop, (int)Flag_turn, (int)Rhombus_Flag,
                (int)diedai_threshold, (int)zyuzhi + (int)Differ_light, (int)Differ_light,
                (int)dl1b_distance_mm, (int)cam_exposure, (int)cam_gain,
                (int)diedai_img_min, (int)diedai_img_max, (int)diedai_img_avg,
                (int)cam_set_exp_ret, (int)cam_set_gain_ret,
                (int)mk_dia_seen, (int)mk_dia_near, (int)mk_dia_score,
                (int)mk_cw_seen, (int)mk_cw_near, (int)mk_cw_score, (int)mk_state, (int)MK_ACT_ENABLE, (int)start_ready, (int)start_base, (int)start_hold,
                (int)Run_Odo);
            wifi_spi_send_string(log_buf);
        }
#endif
        LuUI_Key();
        /* [2026-10-22] 起跑后的 1 秒内舵机保持中位，等摄像头曝光稳定，
           避免用过曝/全黑的垃圾图像把舵机顶到极限。 */
        {
            static uint16 start_settle = 0;
            if(Stop_1 && start_settle < 50) start_settle ++;   // 50 x 20ms ≈ 1s
            steer_settle = (start_settle < 50);
        }
        /* [2026-10-22] 调试后门：长按 S1(KEY_MID) 强制发车，不依赖 ToF。
           S1 在 LuUI_Key 里没有映射，不会和翻页/调参冲突。 */
        if(key_get_state(KEY_MID) == KEY_LONG_PRESS) Stop_1 = 1;
        LuUI_Show();    // ★屏幕已确认可用（官方例程能亮）；坐标已核对不会触发 zf_assert
                        //   LuUI_Key() 必须保留（按键扫描在里面）
        //////感应发车/////////////

        if(Stop_1==0)
        {
            /* [2026-10-22] 发车只用 start_trigger：isr 里连续遮挡 ToF 2 秒置 1，
               松手立刻清零。旧的累计计时（time_dl1b_s>500 且 time_dl1b>200）
               已被替换——它会把几次伸手的时间加在一起，触发时机不可控。 */
            gpio_set_level(P02_4,1);//左前转向
            gpio_set_level(P02_5,1);//左后转向
            gpio_set_level(P02_6,1);//右前转向
            gpio_set_level(P02_7,1);//右后转向
            gpio_set_level(P20_11,1);//左前大灯
            gpio_set_level(P20_12,1);//右前大灯
            gpio_set_level(P20_13,1);//左后刹灯
            gpio_set_level(P20_14,1);//右后刹灯

            if((1 == START_MODE) && start_trigger)
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

