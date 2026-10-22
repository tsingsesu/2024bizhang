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
* 文件名称          zf_device_mt9v03x
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  TXD                查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX 宏定义
*                  RXD                查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX 宏定义
*                  PCLK               查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN 宏定义
*                  VSY                查看 zf_device_mt9v03x.h 中 MT9V03X_VSYNC_PIN 宏定义
*                  D0-D7              查看 zf_device_mt9v03x.h 中 MT9V03X_DATA_PIN 宏定义 从该定义开始的连续八个引脚
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*                  ------------------------------------
********************************************************************************************************************/

#include "zf_common_interrupt.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_device_camera.h"
#include "zf_device_config.h"
#include "zf_device_mt9v03x.h"

vuint8  mt9v03x_finish_flag = 0;
volatile uint32 mt9v03x_frame_cnt = 0;   // ★诊断：帧计数器                            // 一场图像采集完成标志位
IFX_ALIGN(4) uint8  mt9v03x_image[MT9V03X_H][MT9V03X_W];    // 必须4字节对齐

static  m9v03x_type_enum mt9v03x_type;                      // 定义摄像头类型
static  uint16    mt9v03x_version = 0x00;                   // 定义摄像头版本号


int16   timeout = MT9V03X_INIT_TIMEOUT;                     // 定义超时溢出时长

uint8   mt9v03x_lost_flag = 1;                              // 图像丢失标志位
uint8   mt9v03x_dma_int_num;                                // 当前DMA中断次数
uint8   mt9v03x_dma_init_flag;                              // 是否需要重新初始化
uint8   mt9v03x_link_list_num;

/* ★诊断变量（新增）—— 用来在 AURIX 表达式窗口里看 mt9v03x_init() 卡在哪一步。
   加 cam_diag_step 和 cam_diag_sccb_ok 这两个到 Expressions 标签页即可。 */
uint8   cam_diag_step    = 0;   // 0=还没跑  1=开始(先试SCCB)  2=SCCB失败改试串口
                                // 4=串口 set_config 失败   5=串口 get_config 失败
                                // 9=走到 camera_init（即配置成功）
uint8   cam_diag_sccb_ok = 0;   // 1 = SCCB 通信成功过 -> 说明摄像头确实有应答！
uint8   imagebinary[MT9V03X_H][MT9V03X_W];
uint8   imagen[ROW][COL];

/* [2026-10-20] 100 = use the real Otsu threshold.
   With the OLD broken Diedai() the effective threshold was ~89 because 0/0
   gave T0=255 and then x0.35.  Now Otsu works, so 35 would mean 0.35*Otsu
   (~35) and the whole floor turns white.  100 restores the old brightness. */
uint8 zyuzhi = 109,Differ_light;   // ★实测调好的阈值系数（原 53）
uint8 diedai_threshold = 0;     // [2026-10-20] diag: final threshold of Diedai()
// 需要配置到摄像头的数据 不允许在这修改参数
static int16 mt9v03x_set_confing_buffer[MT9V03X_CONFIG_FINISH][2]=
{
    {MT9V03X_INIT,              0},                                             // 摄像头开始初始化
    {MT9V03X_AUTO_EXP,          MT9V03X_AUTO_EXP_DEF},                          // 自动曝光设置   范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    {MT9V03X_EXP_TIME,          MT9V03X_EXP_TIME_DEF},                          // 曝光时间      摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
    {MT9V03X_FPS,               MT9V03X_FPS_DEF},                               // 图像帧率      摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
    {MT9V03X_SET_COL,           MT9V03X_W},                                     // 图像列数量    范围1-752
    {MT9V03X_SET_ROW,           MT9V03X_H},                                     // 图像行数量    范围1-480
    {MT9V03X_LR_OFFSET,         MT9V03X_LR_OFFSET_DEF},                         // 图像左右偏移量  正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_UD_OFFSET,         MT9V03X_UD_OFFSET_DEF},                         // 图像上下偏移量  正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_GAIN,              MT9V03X_GAIN_DEF},                              // 图像增益      范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
    {MT9V03X_PCLK_MODE,         MT9V03X_PCLK_MODE_DEF},                         // 像素时钟模式   仅总钻风MT9V034 V2.0以及以上版本支持该命令
};

// 从摄像头内部获取到的配置数据 不允许在这修改参数
static int16 mt9v03x_get_confing_buffer[MT9V03X_CONFIG_FINISH - 1][2]=
{
    {MT9V03X_AUTO_EXP,          0},                                             // 自动曝光设置
    {MT9V03X_EXP_TIME,          0},                                             // 曝光时间
    {MT9V03X_FPS,               0},                                             // 图像帧率
    {MT9V03X_SET_COL,           0},                                             // 图像列数量
    {MT9V03X_SET_ROW,           0},                                             // 图像行数量
    {MT9V03X_LR_OFFSET,         0},                                             // 图像左右偏移量
    {MT9V03X_UD_OFFSET,         0},                                             // 图像上下偏移量
    {MT9V03X_GAIN,              0},                                             // 图像增益
    {MT9V03X_PCLK_MODE,         0},                                             // 像素时钟模式命令 PCLK模式 < 仅总钻风 MT9V034 V1.5 以及以上版本支持该命令 >
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     配置摄像头内部配置信息
// 参数说明     buff            发送配置信息的地址
// 返回参数     uint8           1-失败 0-成功
// 使用示例     mt9v03x_set_config(mt9v03x_set_confing_buffer);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 mt9v03x_set_config (int16 buff[MT9V03X_CONFIG_FINISH][2])
{
    uint8 return_state = 1;
    uint8  uart_buffer[4];
    uint16 temp = 0;
    uint16 timeout_count = 0;
    uint32 loop_count = 0;
    uint32 uart_buffer_index = 0;

    switch(mt9v03x_version)
    {
        case 0x0230:    loop_count = MT9V03X_PCLK_MODE;  break;
        default:        loop_count = MT9V03X_GAIN;       break;
    }
    // 设置参数  具体请参看问题锦集手册
    // 开始配置摄像头并重新初始化
    for(; MT9V03X_SET_DATA > loop_count; loop_count --)
    {
        uart_buffer[0] = 0xA5;
        uart_buffer[1] = (uint8)buff[loop_count][0];
        temp = buff[loop_count][1];
        uart_buffer[2] = temp >> 8;
        uart_buffer[3] = (uint8)temp;
        uart_write_buffer(MT9V03X_COF_UART, uart_buffer, 4);

        system_delay_ms(2);
    }

    do
    {
        if(3 <= fifo_used(&camera_receiver_fifo))
        {
            uart_buffer_index = 3;
            fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
            if((0xff == uart_buffer[1]) || (0xff == uart_buffer[2]))
            {
                return_state = 0;
                break;
            }
        }
        system_delay_ms(1);
    }while(MT9V03X_INIT_TIMEOUT > timeout_count ++);
    // 以上部分对摄像头配置的数据全部都会保存在摄像头上51单片机的eeprom中
    // 利用set_exposure_time函数单独配置的曝光数据不存储在eeprom中
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取摄像头内部配置信息
// 参数说明     buff            接收配置信息的地址
// 返回参数     uint8           1-失败 0-成功
// 使用示例     mt9v03x_get_config(mt9v03x_get_confing_buffer);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 mt9v03x_get_config (int16 buff[MT9V03X_CONFIG_FINISH - 1][2])
{
    uint8 return_state = 0;
    uint8  uart_buffer[4];
    uint16 temp = 0;
    uint16 timeout_count = 0;
    uint32 loop_count = 0;
    uint32 uart_buffer_index = 0;

    switch(mt9v03x_version)
    {
        case 0x0230:    loop_count = MT9V03X_PCLK_MODE;  break;
        default:        loop_count = MT9V03X_GAIN;       break;
    }

    for(loop_count = loop_count - 1; 1 <= loop_count; loop_count --)
    {
        if((0x0230 > mt9v03x_version) && (MT9V03X_PCLK_MODE == buff[loop_count][0]))
        {
            continue;
        }
        uart_buffer[0] = 0xA5;
        uart_buffer[1] = MT9V03X_GET_STATUS;
        temp = buff[loop_count][0];
        uart_buffer[2] = temp >> 8;
        uart_buffer[3] = (uint8)temp;
        uart_write_buffer(MT9V03X_COF_UART, uart_buffer, 4);

        timeout_count = 0;    
        do
        {
            if(3 <= fifo_used(&camera_receiver_fifo))
            {
                uart_buffer_index = 3;
                fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
                buff[loop_count][1] = uart_buffer[1] << 8 | uart_buffer[2];
                break;
            }
            system_delay_ms(1);
        }while(MT9V03X_INIT_TIMEOUT > timeout_count ++);
        if(MT9V03X_INIT_TIMEOUT < timeout_count)                                // 超时
        {
            return_state = 1;
            break;
        }
    }
    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头串口通信回调
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_uart_callback();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_uart_handler (void)
{
    uint8 data = 0;
    uart_query_byte(MT9V03X_COF_UART, &data);
    if(0xA5 == data)
    {
        fifo_clear(&camera_receiver_fifo);
    }
    fifo_write_element(&camera_receiver_fifo, data);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头场中断
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_vsync_handler();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_vsync_handler(void)
{
    exti_flag_clear(MT9V03X_VSYNC_PIN);
    mt9v03x_dma_int_num = 0;
    if(mt9v03x_dma_init_flag )
    {
        mt9v03x_dma_init_flag = 0;
        IfxDma_resetChannel(&MODULE_DMA, MT9V03X_DMA_CH);
        mt9v03x_link_list_num = dma_init(MT9V03X_DMA_CH,
                                         MT9V03X_DATA_ADD,
                                         mt9v03x_image[0],
                                         MT9V03X_PCLK_PIN,
                                         EXTI_TRIGGER_RISING,
                                         MT9V03X_IMAGE_SIZE);           // 如果超频到300M 倒数第二个参数请设置为FALLING
        dma_enable(MT9V03X_DMA_CH);
    }
    else
    {
        if(1 == mt9v03x_link_list_num)
        {
            dma_set_destination(MT9V03X_DMA_CH, mt9v03x_image[0]);      // 没有采用链接传输模式 重新设置目的地址
        }
        dma_enable(MT9V03X_DMA_CH);
    }
    mt9v03x_lost_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头DMA完成中断
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_dma_handler();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_dma_handler(void)
{
    clear_dma_flag(MT9V03X_DMA_CH);

    if(IfxDma_getChannelTransactionRequestLost(&MODULE_DMA, MT9V03X_DMA_CH)) // 图像错位判断
    {
        mt9v03x_finish_flag = 0;
        dma_disable(MT9V03X_DMA_CH);
        IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, MT9V03X_DMA_CH);
        mt9v03x_dma_init_flag = 1;
    }
    else
    {
        mt9v03x_dma_int_num++;
        if(mt9v03x_dma_int_num >= mt9v03x_link_list_num)
        {
            // 采集完成
            // 一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
            mt9v03x_dma_int_num = 0;
            mt9v03x_lost_flag   = 0;
            mt9v03x_frame_cnt ++;   // ★诊断
            mt9v03x_finish_flag = 1;
            dma_disable(MT9V03X_DMA_CH);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取摄像头固件版本
// 参数说明     void
// 返回参数     uint16          0-获取错误 N-版本号
// 使用示例     mt9v03x_get_version();                          // 调用该函数前请先初始化串口
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint16 mt9v03x_get_version (void)
{
    uint16 temp;
    uint8  uart_buffer[4];
    uint16 timeout_count = 0;
    uint16 return_value = 0;
    uint32 uart_buffer_index = 0;

    uart_buffer[0] = 0xA5;
    uart_buffer[1] = MT9V03X_GET_STATUS;
    temp = MT9V03X_GET_VERSION;
    uart_buffer[2] = temp >> 8;
    uart_buffer[3] = (uint8)temp;
    uart_write_buffer(MT9V03X_COF_UART, uart_buffer, 4);

    do
    {
        if(3 <= fifo_used(&camera_receiver_fifo))
        {
            uart_buffer_index = 3;
            fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
            return_value = uart_buffer[1] << 8 | uart_buffer[2];
            break;
        }
        system_delay_ms(1);
    }while(MT9V03X_INIT_TIMEOUT > timeout_count ++);
    return return_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单独设置摄像头曝光时间
// 参数说明     light           设定曝光时间
// 返回参数     uint8           1-失败 0-成功
// 使用示例     mt9v03x_set_exposure_time(100);                 // 调用该函数前请先初始化串口
// 备注信息     设置曝光时间越大图像越亮
//              摄像头收到后会根据分辨率及FPS计算最大曝光时间如果设置的数据过大
//              那么摄像头将会设置这个最大值
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_set_exposure_time (uint16 light)
{
    uint8 return_state = 0;
    if(MT9V03X_UART == mt9v03x_type)
    {
        set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, mt9v03x_uart_handler);
        uint8  uart_buffer[4];
        uint16 temp = 0;
        uint16 timeout_count = 0;
        uint32 uart_buffer_index = 0;

        uart_buffer[0] = 0xA5;
        uart_buffer[1] = MT9V03X_SET_EXP_TIME;
        temp = light;
        uart_buffer[2] = temp >> 8;
        uart_buffer[3] = (uint8)temp;
        uart_write_buffer(MT9V03X_COF_UART, uart_buffer, 4);

        do
        {
            if(3 <= fifo_used(&camera_receiver_fifo))
            {
                uart_buffer_index = 3;
                fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
                temp = uart_buffer[1] << 8 | uart_buffer[2];
                break;
            }
            system_delay_ms(1);
        }while(MT9V03X_INIT_TIMEOUT > timeout_count ++);
        if((temp != light) || (MT9V03X_INIT_TIMEOUT <= timeout_count))
        {
            return_state = 1;
        }
        set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, NULL);
    }
    else
    {
        return_state = mt9v03x_set_exposure_time_sccb(light);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对摄像头内部寄存器进行写操作
// 参数说明     addr            摄像头内部寄存器地址
// 参数说明     data            需要写入的数据
// 返回参数     uint8           1-失败 0-成功
// 使用示例     mt9v03x_set_reg(addr, data);                    // 调用该函数前请先初始化串口
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_set_reg (uint8 addr, uint16 data)
{
    uint8 return_state = 0;
    if(MT9V03X_UART == mt9v03x_type)
    {
        set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, mt9v03x_uart_handler);
        uint8  uart_buffer[4];
        uint16 temp = 0;
        uint16 timeout_count = 0;
        uint32 uart_buffer_index = 0;

        uart_buffer[0] = 0xA5;
        uart_buffer[1] = MT9V03X_SET_ADDR;
        temp = addr;
        uart_buffer[2] = temp >> 8;
        uart_buffer[3] = (uint8)temp;
        uart_write_buffer(MT9V03X_COF_UART, uart_buffer, 4);

        system_delay_ms(10);
        uart_buffer[0] = 0xA5;
        uart_buffer[1] = MT9V03X_SET_DATA;
        temp = data;
        uart_buffer[2] = temp >> 8;
        uart_buffer[3] = (uint8)temp;
        uart_write_buffer(MT9V03X_COF_UART, uart_buffer, 4);

        do
        {
            if(3 <= fifo_used(&camera_receiver_fifo))
            {
                uart_buffer_index = 3;
                fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
                temp = uart_buffer[1] << 8 | uart_buffer[2];
                break;
            }
            system_delay_ms(1);
        }while(MT9V03X_INIT_TIMEOUT > timeout_count ++);
        if((temp != data) || (MT9V03X_INIT_TIMEOUT <= timeout_count))
        {
            return_state = 1;
        }
        set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, NULL);
    }
    else
    {
        return_state = mt9v03x_set_reg_sccb(addr, data);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MT9V03X 摄像头初始化
// 参数说明     void
// 返回参数     uint8           1-失败 0-成功
// 使用示例     zf_log(mt9v03x_init(), "mt9v03x init error");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_init (void)
{
    uint8 return_state = 0;
    soft_iic_info_struct mt9v03x_iic_struct;
    do
    {
        system_delay_ms(200);
        // 首先尝试SCCB通讯
        cam_diag_sccb_ok = 0;          // ★每次重试都复位
        cam_diag_step    = 1;          // ★开始：先试 SCCB
        mt9v03x_type = MT9V03X_SCCB;
        set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, NULL);
        soft_iic_init(&mt9v03x_iic_struct, 0, MT9V03X_COF_IIC_DELAY, MT9V03X_COF_IIC_SCL, MT9V03X_COF_IIC_SDA);
        if(mt9v03x_set_config_sccb(&mt9v03x_iic_struct, mt9v03x_set_confing_buffer))
        {
            // SCCB通讯失败，尝试串口通讯
            cam_diag_step = 2;         // ★SCCB 失败，改试串口
            mt9v03x_type = MT9V03X_UART;
            set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, mt9v03x_uart_handler);
            camera_fifo_init();

            // 初始换串口 配置摄像头
            uart_init(MT9V03X_COF_UART, MT9V03X_COF_BAUR, MT9V03X_COF_UART_RX, MT9V03X_COF_UART_TX);
            uart_rx_interrupt(MT9V03X_COF_UART, 1);
            fifo_clear(&camera_receiver_fifo);
            // 等待摄像头上电初始化成功 方式有两种：延时或者通过获取配置的方式 二选一
            // system_delay_ms(1000);                                               // 延时方式

            // if(mt9v03x_get_config(mt9v03x_get_confing_buffer))
            // {
            //     // 如果程序在输出了断言信息 并且提示出错位置在这里
            //     // 那么就是串口通信出错并超时退出了
            //     // 检查一下接线有没有问题 如果没问题可能就是坏了
            //     zf_log(0, "MT9V03X get config error.");
            //     set_camera_type(NO_CAMERE, NULL, NULL, NULL);
            //     return_state = 1;
            //     break;
            // }
            mt9v03x_version = mt9v03x_get_version();                                // 获取配置的方式

            if(mt9v03x_set_config(mt9v03x_set_confing_buffer))
            {
                // 如果程序在输出了断言信息 并且提示出错位置在这里
                // 那么就是串口通信出错并超时退出了
                // 检查一下接线有没有问题 如果没问题可能就是坏了
                cam_diag_step = 4;     // ★串口 set_config 失败
                zf_log(0, "MT9V03X set config error.");
                uart_rx_interrupt(MT9V03X_COF_UART, 0);
                set_camera_type(NO_CAMERE, NULL, NULL, NULL);
                return_state = 1;
                break;
            }

            // 获取配置便于查看配置是否正确
            if(mt9v03x_get_config(mt9v03x_get_confing_buffer))
            {
                // 如果程序在输出了断言信息 并且提示出错位置在这里
                // 那么就是串口通信出错并超时退出了
                // 检查一下接线有没有问题 如果没问题可能就是坏了
                cam_diag_step = 5;     // ★串口 get_config 失败
                zf_log(0, "MT9V03X get config error.");
                uart_rx_interrupt(MT9V03X_COF_UART, 0);
                set_camera_type(NO_CAMERE, NULL, NULL, NULL);
                return_state = 1;
                break;
            }
        }
        if(cam_diag_step < 2) cam_diag_sccb_ok = 1;   // ★step 还停在 1 -> 是 SCCB 直接成功的
        cam_diag_step = 9;                            // ★配置成功
        mt9v03x_link_list_num = camera_init(MT9V03X_DATA_ADD, mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    }while(0);
    return return_state;
}

// 冒泡排序升序
uint8 Bubble_Sort(uint8 n1, uint8 n2, uint8 n3, uint8 n4, uint8 n5,uint8 n6, uint8 n7, uint8 n8, uint8 n9)
{
    uint8 arr[9];
    uint8 temp,k,j;
    arr[0] = n1;    arr[1] = n2;    arr[2] = n3;    arr[3] = n4;    arr[4] = n5;
    arr[5] = n6;    arr[6] = n7;    arr[7] = n8;    arr[8] = n9;
        for(j=0;j<9;j++)
        {
           for(k=0;k<9-j;k++)
           {
              if(arr[k] > arr[k+1])        //前面的比后面的大  则进行交换
              {
                 temp = arr[k+1];
                 arr[k+1] = arr[k];
                 arr[k] = temp;
              }
           }
        }
     return arr[4];//返回中值
}


//
void Laplace(void)         // 拉普拉斯变换  图像锐化   有 噪点
{
    int32 res;
    for (uint8 i = 1; i < MT9V03X_H-8; i++)
    {
        for (uint8 j = 1; j < MT9V03X_W-1; j++)
        {
            res = imagebinary[i][j] + 8* imagebinary[i][j] - imagebinary[i-1][j] - imagebinary[i-1][j+1] - imagebinary[i-1][j-1] - imagebinary[i][j-1] - imagebinary[i][j+1] -imagebinary[i+1][j] -imagebinary[i+1][j+1] - imagebinary[i+1][j-1];
            if(res > 255)
               imagen[i][j] = 255;
            else if(res < 0)
               imagen[i][j] = 0;
            else
            imagen[i][j] = res;
            imagen[i][MT9V03X_W-1] = imagebinary[i][MT9V03X_W-1];
            imagen[i][0] = imagebinary[i][0];
        }
    }
    for (uint8 j = 1; j < MT9V03X_W; j++)
        imagen[0][j] = imagebinary[0][j];
}



void Gaussian_Blur()     //   梯度处理 直接得到边线 但是不好用
{
  uint8 i,j;
  for(i=1;i<MT9V03X_H;i++)
   {
          for(j=1;j<MT9V03X_W-1;j++)
          {
             imagebinary[i][j] += ABS((mt9v03x_image[i+1][j] + mt9v03x_image[i+1][j+1]+mt9v03x_image[i+1][j-1]) - (mt9v03x_image[i-1][j] + mt9v03x_image[i-1][j+1]+mt9v03x_image[i-1][j-1]));
          }
          imagebinary[i][MT9V03X_W-1] = mt9v03x_image[i][MT9V03X_W-1];
           imagebinary[i][0] = mt9v03x_image[i][0];
   }
   for(j=0;j<MT9V03X_W;j++)
   {
      imagebinary[0][j] = mt9v03x_image[0][j];
   }
}

//void Sunshine(void)  //阳光算法  2022 1 20
//{
//    int8 num=0,sunflagi=0,sunflagj=0;
//    int32 res1;
//   //int32 flag;
//     for (uint8 i = 1; i < MT9V03X_H; i++)//MT9V03X_H-60
//      {
//          for (uint8 j = 1; j < MT9V03X_W-1; j++)
//          {
//              res1 = (-1)* imagebinary[i-1][j-1] +(-2)* imagebinary[i][j-1] +(-1)* imagebinary[i+1][j-1]  + imagebinary[i-1][j+1]  +(2)* imagebinary[i][j+1]  +  imagebinary[i+1][j+1];
//             if(res1<-80)
//             {
//                 num++;
//                 sunflagi=i;
//                 sunflagj=j;
//             }
//          }
//      }           //根据Sobel算子算出梯度（分界线）
//
//      if(num>25)  flag=1;
//
//       if(flag==1)
//          {
//           for (uint8 i =  sunflagi; i > 0; i--)    // for (uint8 i = sunflagi+8; i > 0; i--)
//              {
//                for (uint8 j =  sunflagj; j > 0; j--)  //for (uint8 j = sunflagj+8; j > 0; j--)
//                {
//                    res1 = imagebinary[i][j] + 8* imagebinary[i][j] - imagebinary[i-1][j] - imagebinary[i-1][j+1] - imagebinary[i-1][j-1] - imagebinary[i][j-1] - imagebinary[i][j+1] -imagebinary[i+1][j] -imagebinary[i+1][j+1] - imagebinary[i+1][j-1];
//                    if(res1 > FORM)
//                       imagen[i][j] = FORM;                  //和拉普拉斯不一样，直接去掉可疑的
//                    else if(res1 < 0)
//                       imagen[i][j] = 0;
//                    else
//                    imagen[i][j] = res1;
//                }
//              }                                         //用拉普拉斯补充图像完成
//
//           for (uint8 i =  sunflagi; i < MT9V03X_H; i++)    // for (uint8 i = sunflagi+8; i > 0; i--)  sunflagi  MT9V03X_H
//                         {
//                           for (uint8 j =  sunflagj; j <  MT9V03X_W-1; j++)  //for (uint8 j = sunflagj+8; j > 0; j--)  MT9V03X_W-1  sunflagj
//                           {
//                               res1 = imagebinary[i][j] + 8* imagebinary[i][j] - imagebinary[i-1][j] - imagebinary[i-1][j+1] - imagebinary[i-1][j-1] - imagebinary[i][j-1] - imagebinary[i][j+1] -imagebinary[i+1][j] -imagebinary[i+1][j+1] - imagebinary[i+1][j-1];
//                               if(res1 > 255)
//                                  imagen[i][j] = 255;
//                               else if(res1 < 0)
//                                  imagen[i][j] = 0;
//                               else
//                               imagen[i][j] = res1;
//                           }
//                         }
//    //   freshpicture();                                //更新图像
//       flag=0;
//        }
//       else    Laplace();            // Laplace 图像滤波
//}

// 二值化
void binaryzation(uint8 val)
{
    uint8 temp = 0;
    uint8 i = 0,j = 0;

      temp = val;

      for(i=0;i<MT9V03X_H;i++)
      {

          for(j=0;j<MT9V03X_W;j++)
          {
              if( mt9v03x_image[i][j] > temp)
              {
                imagebinary[i][j] = 0xFF;   // [2026-10-20] white=255 (LuUI thresholds this array again)
              }
              else
              {
                imagebinary[i][j] = 0x0000;

              }
          }
      }
}

uint8 Diedai(uint8 *image, uint16 col, uint16 row)//    碌眉麓煤路篓     //
{
  // [2026-10-20] rewritten ----------------------------------------------
    /* The old loop started at (row-90); with ROW=70 that is -20, so the
       histogram was never filled and 0/0 was evaluated later.  The actual
       threshold was whatever the hardware did with the divide-by-zero. */
    static uint16 pixelCount[256];      // static: 512 B, keeps CPU1 stack small
    int32  i, j, y;
    int32  X, lP1, lP2, lS1, lS2;
    uint8  A1, A2, T0;
    uint8 *data = image;
    int32  min = 255, max = 0;

    for (i = 0; i < 256; i ++)
    {
        pixelCount[i] = 0;
    }
    for (i = 0; i < (int32)row; i ++)                       // whole frame
    {
        for (j = 0; j < (int32)col; j ++)
        {
            pixelCount[data[i * col + j]] ++;
            if(data[i * col + j] > max)  max = data[i * col + j];
            if(data[i * col + j] < min)  min = data[i * col + j];
        }
    }

    T0 = (uint8)((min + max) / 2);
    X = T0;
    A1 = A2 = T0;
    for(y = 0; X != T0 && y < 200;  y ++)                   // iterative split
    {
        X   = T0;
        lP1 = lP2 = lS1 = lS2 = 0;                          // cleared every pass
        for (i = min; i < X; i ++)                          // step 1, not 2
        {
            lP1 += (int32)pixelCount[i] * i;
            lS1 += pixelCount[i];
        }
        A1 = (lS1 > 0) ? (uint8)(lP1 / lS1) : T0;
        for (i = X + 1; i <= max; i ++)
        {
            lP2 += (int32)pixelCount[i] * i;
            lS2 += pixelCount[i];
        }
        A2 = (lS2 > 0) ? (uint8)(lP2 / lS2) : T0;
        T0 = (uint8)((A1 + A2) / 2);
    }

    T0 = (uint8)(T0 * ((zyuzhi + Differ_light) / 100.0));   // keep original tuning factor
    if(T0 < 12)   T0 = 12;                                  // never all-white
    if(T0 > 240)  T0 = 240;                                 // never all-black
    diedai_threshold = T0;
    return T0;
}
