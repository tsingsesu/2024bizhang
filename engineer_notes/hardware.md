# 硬件 / 引脚 / 通信 / 坑

> 最近更新：2026-10-22。引脚以代码为准（`user/System.h`、`user/System.c`、各 `zf_device_*.h`）。

## 主板 V3.1 已用引脚

| 用途 | 引脚 | 说明 |
|---|---|---|
| 舵机 | `ATOM1_CH1_P33_9` | 中位 `SteerMiddle=857`，范围 `785~915` |
| 右轮 PWM / DIR | `ATOM1_CH0_P21_2` / `P21_3` | 板上"有刷电机1"接口 P6；编码器 TIM6（P20_3/P20_0） |
| 左轮 PWM / DIR | `ATOM1_CH2_P21_4` / `P21_5` | 编码器 TIM2（P33_7/P33_6） |
| 摄像头 MT9V03X | 默认组（P02_x/P00_x） | `MT9V03X_SWAP_CFG_PINS=0` |
| DL1B ToF | 软 IIC `P33_4`(SCL) / `P33_5`(SDA) | INT 关闭（`DL1B_INT_ENABLE=0`），不插也不影响出车 |
| WiFi SPI | SPI3：SCK P22_3 / MOSI P22_0 / MISO P22_1 | CS `P22_2`，INT `P15_8`，RST `P23_1` |
| IPS200 屏 | SPI2：SCL P15_3 / SDA P15_5 / RST P15_1 / DC P15_0 / CS P15_2 / BL P15_4 | |
| 蜂鸣器 | `P11_11`（高电平响） | 代码里实际用 `P33_13`/`P33_10` 当日志输出脚 |
| 8 个车灯 | `P02_4~7`、`P20_11~14` | ⚠ `P20_11/P20_14` 与 DL1B2 软件 IIC 冲突，加第二路 ToF 前先换脚 |
| 按键 | `P20_6`(S1)、`P20_7`(S2)、`P11_2`(S3)、`P11_3`(S4)、`P33_12`(S5) | 见 `zf_device_key.h` |

## 无线通信（WIFI6B21-SPI V2.0）

配置全在 `libraries/zf_device/zf_device_wifi_spi.h`：

```c
#define WIFI_SPI_AUTO_CONNECT  (2)              // 2 = UDP（当前）；1 = TCP（逐飞助手图传）；0 = 不连
#define WIFI_SSID              "minoshima"
#define WIFI_PASS              "uuuuvvvv"
#define WIFI_SPI_TARGET_IP     "192.168.43.210" // ← 电脑在热点下的 IPv4（必改）
#define WIFI_SPI_TARGET_PORT   "8086"
#define WIFI_SPI_LOCAL_PORT    "6666"           // 车自己的 UDP 端口
```

**怎么查电脑的 IP**（换电脑、热点重连后都可能是新的）：

```bash
ipconfig    # 或 powershell: Get-NetIPConfiguration
```

找 WLAN 适配器的 IPv4，填进 `WIFI_SPI_TARGET_IP`。
例：电脑是热点，则网关通常是 `192.168.137.1`，电脑本机 IP 可能是 `192.168.137.1` 或另分配的地址，**以 `ipconfig` 为准**。

**这个模块的坑**：
- INT 引脚（P15_8）恒为低，接上拉也一样 → 驱动里 `WIFI_SPI_USE_INT=0`，所有命令改用固定延时
- 后果：**上电到屏幕亮约 15 秒**，期间黑屏正常
- SPI 已从 30 MHz 降到 10 MHz（长排线更稳）

## 调试通道

| 通道 | 当前状态 |
|---|---|
| IPS200 屏 | 正常。`page1`=图像+关键量，`page2`=电机/编码器，`page3`=参数+WiFi 诊断 |
| 调试串口 | `debug_init()` 默认串口。工程里**没开** `DEBUG_UART_USE_INTERRUPT`，`zf_log` 可能不输出——用 UDP 日志代替 |
| WiFi UDP 日志 | ✅ 当前主通道。车端 `CAR_LOG_ENABLE=1`，电脑端 `tools/car_log.py` |

## 三个大坑（来自队友，仍有效）

1. **屏幕坐标有上限**：`ips200_show_int` 实际画 `位数*2+2` 个字符，每字符 8 px → `x + (位数*2+2)*8 ≤ 232`，超了触发 `zf_assert`。
2. **断言会静默关掉一切**：`zf_assert` 失败会执行 `pit_all_close()` + `pwm_all_channel_close()` + `exti_all_close()` → **电机不转、摄像头不出图，但屏幕文字照常刷新**。看到"屏幕还在刷但车不动"先怀疑断言。
3. **烧录后按复位**：AURIX Flasher 烧完可能停在 halt；别用下载器供电。

## 其他已知坑

- `Img_analyse.c` 是 **UTF-8**，其余源码是 **GBK**。改中文注释必须按原编码写回。
- `Zebra_Crossing()` 会把 `Pwm_Min/Pwm_Max` 改成 `822/882`，此后舵机行程变窄——调舵机前留意这一行有没有被触发过。
- `Find_Cone_bucket()` 里 `zyuzhi=38` 是 2024 调的旧值，与当前 109 不匹配；该函数目前**没人调用**，真要用前先改。
- `Tiao_Light()` 每帧给 `Differ_light++`，而 `Differ_light` 只在 `tiaoguang==1` 时回落；`tiaoguang` 目前没人置 1 → 这路逻辑暂时不会生效。
- `Line_Count` 是"扫描 break 时停下的行号"，**不是有效行数**；最坏情况停在 59，不能只拿它判断有没有线。
