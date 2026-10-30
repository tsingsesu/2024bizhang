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
* 文件名称          cpu1_main
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
extern void Cone_Scan(uint8 *data);
extern void Stop_Probe(uint8 *data);        /* [2026-10-26] 终点探测（只记录）*/
extern volatile uint16 stop_aaa;
extern volatile uint16 stop_run;
extern volatile uint8  stop_now, stop_latch;
extern volatile uint16 tof_sample_mm;
extern volatile uint32 isr_tick_5ms;
extern volatile uint8  tof_new_sample;

/* [2026-10-31] ToF 安装调试统计（必须放在 #pragma cpu1_dsram 之前，CPU0 要读）*/
volatile uint16 tof_try_cnt = 0;    /* 调了几次 dl1b_get_distance */
volatile uint16 tof_ok_cnt  = 0;    /* 其中返回有效距离的次数 */
volatile uint16 tof_win_min = 0xFFFF;
volatile uint16 tof_win_max = 0;
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************
void core1_main(void)
{
    // ★ 原来这里还有一次 System()：System() 被调了 3 次(CPU0 两次 + CPU1 一次)
    //   两个核并发跑 dl1b_init()/mt9v03x_init()，抢同一条软件 IIC，初始化必挂。已删除。
    disable_Watchdog();                     // 关闭看门狗 ★提到最前：避免两个核的 WDT 操作撞在同一时间点
    system_delay_ms(1000);
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等



    // 此处编写用户代码 例如外设初始化代码等

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        // ★ToF 没装就不调用：否则每次都要等软件 I2C 的 ACK 超时，白白拖慢 CPU1
        if(dl1b_init_flag)
        {
            if(tof_try_cnt < 60000) tof_try_cnt ++;
            dl1b_get_distance();
            /* ★只有这一轮真的测到新距离时才置位（finsh_flag 只成功时=1），
               然后立刻清掉，防止被下一个 8192 空轮冲掉 */
            if(dl1b_finsh_flag)
            {
                dl1b_finsh_flag = 0;
                tof_sample_mm   = dl1b_distance_mm;
                tof_sample_tick = isr_tick_5ms;
                tof_has_sample = 1;
                tof_new_sample  = 1;
                if(tof_ok_cnt < 60000) tof_ok_cnt ++;
                if(dl1b_distance_mm < tof_win_min) tof_win_min = dl1b_distance_mm;
                if(dl1b_distance_mm > tof_win_max) tof_win_max = dl1b_distance_mm;
            }
        }
        if(mt9v03x_finish_flag)
        {
            binaryzation(Diedai(mt9v03x_image[0],COL,ROW));
            Image_Handle(*imagebinary);     // 图像处理
            /* [2026-10-22] 标线扫描放在 Image_Handle 之后：
               横道/菱形是白色，先扫会被边缘识别当成"赛道变宽"去补线。 */
            Marker_Scan(*imagebinary);      // 2026 标线：人行横道 / 减速菱形
            Cone_Scan(*imagebinary);        // [2026-10-24] 锥形标识别（只统计）
            Stop_Probe(*imagebinary);       /* [2026-10-26] 终点探测：只统计，不动作 */
            Glare_Suppress(*imagebinary);   // [2026-10-20] reject reflection blobs
            Point_Control();      // 中点计算
            vision_processed_frames ++;  /* Publish a completed perception result. */
            // ★原来清这个标志的是 LuUI_Show_page1() 里的那句 mt9v03x_finish_flag=0
            //   现在屏幕关了，必须在这里清 —— 否则标志一直是 1，同一帧会被反复处理
#if (AGL_LOG_EVERY_N > 0)
            /* [2026-10-20] serial telemetry: one line per N frames. */
            {
                static uint16 agl_log_cnt = 0;
                if(++ agl_log_cnt >= AGL_LOG_EVERY_N)
                {
                    agl_log_cnt = 0;
                    lu_ui_dbg_log = 1;
                }
            }
#endif
            mt9v03x_finish_flag = 0;
        }

    }
}
#pragma section all restore
