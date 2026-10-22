#include "zf_common_headfile.h"

// ★用于调试：摄像头初始化返回值  0 = 成功，非 0 = 失败
//   在 AURIX 表达式窗口里加 cam_init_ret 就能看到
uint8 cam_init_ret = 0xFF;

void System()
{
    // ★路标日志 1：如果串口链路是通的，这句一定会出现在逐飞助手的接收区
    zf_log(0, "=== System() begin ===");
    // ★关蜂鸣器：原理图上蜂鸣器控制脚 = P11.11（经 Q2 三极管，高电平响）
    gpio_init(P11_11, GPO, GPIO_LOW, GPO_PUSH_PULL);
    // ★屏幕放在最前面：一来保证它一定会亮，
    //   二来后面任何初始化触发 zf_assert / trap，错误信息会直接打在屏幕上
    ips200_init(IPS200_TYPE_SPI);

    gpio_init(MOTOR1_A, GPO, GPIO_HIGH, GPO_PUSH_PULL);// 新方案双电机初始化
    pwm_init(MOTOR1_B, 17000, 0);
    gpio_init(MOTOR2_A, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_B, 17000, 0);

    pwm_init(ATOM1_CH1_P33_9,50,SteerMiddle); // 舵机初始化
    //编码器初始化
    encoder_quad_init(ENCODER1_TIM,ENCOEDER1_CH1,ENCOEDER1_CH2);
    encoder_quad_init(ENCODER2_TIM,ENCOEDER2_CH1,ENCOEDER2_CH2);

    // DL1B 初始化：失败就重试(最多 5 次)，上电未就绪时能自愈。逐飞官方例程同样是循环重试
    // 注意 dl1b_init() 的返回值： 1 = 失败   0 = 成功
    {
        uint8 dl1b_retry = 0;
        while((0 != dl1b_init()) && (++ dl1b_retry < 2))   // ★ToF 没装：只试 2 次，不拖时间
        {
            system_delay_ms(100);
        }
    }
    system_delay_ms(500);                        // 先给摄像头留出上电时间
    {
        uint8 cam_retry = 0;
        do
        {
            cam_init_ret = mt9v03x_init();
            if(0 == cam_init_ret)    break;      // 成功就退出
            system_delay_ms(300);               // 失败就等一下再试，最多 3 次
        }while(++ cam_retry < 3);
    }

    key_init(10);
    gpio_init(P33_13, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P33_10, GPO, GPIO_LOW, GPO_PUSH_PULL);
//    gpio_init(P20_9, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    gpio_init(P20_8, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    if(gpio_get_level(P33_12))    Parameter_Read();//★暂时关掉（见下）
    //   V3.1 上 P33_12 是 RGB 接口的蓝灯脚，被 key_init 拉高 -> 每次上电都会读 flash
    //   而这颗片子 flash 里没写过参数 -> zyuzhi 被冲成 0
    //   zyuzhi=0 会让 Diedai() 最后那句 T0 = T0*((zyuzhi+Differ_light)/100.0) 把阈值算成 0，二值化全废
    //   关掉之后 zyuzhi 用库里的默认值 35（Img_analyse.c:1015 也是按 35~38 调的）
    //   以后想用存储参数：先用按键走一次 Parameter_Save() 把值写进去，再把这行放开

    /*灯  ★V3.1 引脚重排★
      原来的 P32_4/P20_10/P11_3/P11_2/P15_7/P23_1/P11_6/P20_7
      在 V3.1 上分别是 串口Tx / 按键S5 / 按键S4 / 串口Rx / 无线SPI-RST / 无线串口-RST / 按键S3，全部冲突*/
    gpio_init(P02_4,  GPO, GPIO_LOW, GPO_PUSH_PULL);//左前转向 -> P7 有刷电机2座
    gpio_init(P02_5,  GPO, GPIO_LOW, GPO_PUSH_PULL);//左后转向 -> P7 有刷电机2座
    gpio_init(P02_6,  GPO, GPIO_LOW, GPO_PUSH_PULL);//右前转向 -> P7 有刷电机2座
    gpio_init(P02_7,  GPO, GPIO_LOW, GPO_PUSH_PULL);//右后转向 -> P7 有刷电机2座
    gpio_init(P20_11, GPO, GPIO_LOW, GPO_PUSH_PULL);//左前大灯 -> P4 姿态传感器座
    gpio_init(P20_12, GPO, GPIO_LOW, GPO_PUSH_PULL);//右前大灯 -> P4 姿态传感器座
    gpio_init(P20_13, GPO, GPIO_LOW, GPO_PUSH_PULL);//左后刹灯 -> P4 姿态传感器座
    gpio_init(P20_14, GPO, GPIO_LOW, GPO_PUSH_PULL);//右后刹灯 -> P4 姿态传感器座

    // ★★开机自检闪灯（诊断用）★★
    //   4 个转向灯闪 3 下 = 程序跑到了这里（说明 DL1B/摄像头初始化全部走完，马上要开 PIT）
    //   不闪 = 程序根本没跑 / 卡在初始化里。这一条能直接把「程序没跑」和「电机不转」区分开。
    {
        uint8 blink_i;
        for(blink_i = 0; blink_i < 3; blink_i ++)
        {
            gpio_set_level(P02_4,1); gpio_set_level(P02_5,1);
            gpio_set_level(P02_6,1); gpio_set_level(P02_7,1);
            system_delay_ms(120);
            gpio_set_level(P02_4,0); gpio_set_level(P02_5,0);
            gpio_set_level(P02_6,0); gpio_set_level(P02_7,0);
            system_delay_ms(120);
        }
    }

    // ★没装 ToF 时不做发车门禁：否则 Flag_stop 永远是 1，机器人永远不走
    //   ToF 装上以后自动恢复正常的感应发车
    Flag_stop = (dl1b_init_flag) ? 1 : 0;
    // 原为 3。=3 时 S3010_Control() 把 SD5_Duty 写死 854、PID 那行不执行 -> 舵机不跟线
    // 改成 0 才会掉进 else 分支跑 PID。比赛逻辑(Img_analyse.c)后面会自己把它改成 1/2/3
    Flag_turn=0;


    Light_Flag=15;
    system_delay_ms(100);
    ///////////////开局亮灯///////////////////////
//    gpio_set_level(P02_4,1);//左前转向
//    gpio_set_level(P02_5,1);//左后转向
//    gpio_set_level(P02_6,1);//右前转向
//    gpio_set_level(P02_7,1);//右后转向
//    gpio_set_level(P20_11,1);//左前大灯
//    gpio_set_level(P20_12,1);//右前大灯
//    gpio_set_level(P20_13,1);//左后刹灯
//    gpio_set_level(P20_14,1);//右后刹灯

    ////////////感应发车/////////////////////////

    pit_ms_init(CCU60_CH0, 5);
    pit_ms_init(CCU60_CH1, 5);
    pit_ms_init(CCU61_CH0, 10);
    pit_ms_init(CCU61_CH1, 10);

    // ★路标日志 2：初始化全部跑完了
    zf_log(0, "=== System() end ===");
}



