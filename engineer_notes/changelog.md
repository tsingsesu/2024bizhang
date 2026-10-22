# 改动记录

> 每轮对话收工时追加一条。版本回退用 `git log` / `git diff`，不再留 `.bak_Vxx`。

## 2026-10-22（与队友代码合并后第一轮）

基线：`a8215a2` 队友交付版（V35b，含 WiFi SPI 图传、page3 五参数、V3.1 引脚重排）。

**本轮目标**：先把无线通信从"逐飞助手 TCP 图传"切到"终端 UDP 日志"，为后续按日志改代码做准备。

| 文件 | 改动 | 原因 |
|---|---|---|
| `libraries/zf_device/zf_device_wifi_spi.h` | `WIFI_SPI_AUTO_CONNECT` 1→**2**（UDP） | 用户要终端收日志，不用逐飞助手 |
| `user/cpu0_main.c` | 新增 `CAR_LOG_ENABLE` / `CAR_LOG_PERIOD_LOOPS`；上电发一行 `BOOT wifi_step=.. ret=.. ver=..`；主循环每 60 圈发 `T=.. P=.. LC=.. duty=.. st=../.. trim=..` | 车一上电就能在电脑端确认 WiFi 通没通、控制量是否合理 |
| `code/Img_analyse.c` | `Image_Handle()` 里 `res` 未初始化就参与 `if (res == 0)` 判断，补 `else res = 1;` | `Right_Flag` 被置 1 后永不清零（`zuobianjie/youbianjie` 没人调用），`First_Line_Handle` 从第一帧起就不再执行，这一帧可能随机 `return`，`Point` 冻结 → 舵机卡住不跟线 |
| `code/Lu_UI.c` | page1 的 `Flag_stop` 从 `show_float(...,4,4)` 改成 `show_int(...,3)`，`Flag_turn` 挪到 Show_Y[7] | page3 加了 STrim 一行后最下面 y 超屏；同时避开 show_float 的 18 字符宽度（150 + 18*8 = 294 > 232 会断言） |
| `code/Lu_UI.c` | page3 STrim 显示 x 150→120；按键范围 ±60；恢复默认时 `Steer_Trim=0` | 与 flash 校验范围一致，负号+3位也不会顶出屏幕 |
| `code/Lu_UI.c` | `Differ` 按键范围限到 ±50 | 与 `Parameter_Read()` 的校验范围一致，否则存进去读回来会被丢掉 |
| `user/cpu0_main.c` | `CAMERA_SEND_ENABLE` 1→**0** | 逐飞助手图传和 UDP 日志共用同一条连接会串数据；调参看电脑端日志优先，要看图再切回 |
| `tools/car_log.py` | 新增电脑端 UDP 日志接收器 | 按 `\n` 切行、自动拼包、显示时间戳和来源 IP |

**新增脚手架**

- `2024bizhang/engineer_notes/`（README / hardware / changelog）
- git 仓库 + 基线提交 + `.gitignore`

**没做 / 待确认**

- 本轮改动还没在车子上编译烧录（本机没有 `cctc`，需要在 ADS 里 Build）
- 图传（`CAMERA_SEND_ENABLE=1`）和 UDP 日志会同时占用 WiFi 模块，目前两者都用 `wifi_spi_send_*`，**同一条连接里混发**；如果图传一开日志就乱，先把 `CAMERA_SEND_ENABLE` 改 0
- 2024 的标线/菱形逻辑和 2026 双车道规则不匹配，尚未动

### 同日第二轮（UDP 日志跑通后）

第一条日志就暴露了两个真问题：

- `st=7/3`：`Flag_stop=7 / Flag_turn=3` 是 `Turn_Left/Turn_Right` 的起始状态，说明 **2024 菱形/斑马线状态机被 ToF 发车逻辑自动拉起来了**。已用 `MOTION_2024_ENABLE (0)` 整体包住（`Img_analyse.c`），先纯跟线。
- `P=2 LC=59`：整帧没找到线时 `Line_Count` 停在 59，旧保护条件放行 → 舵机瞬间打到 915。新增 `Result_Far_Row`（本帧最远可信行），`control.c` 改用它做保护。

| 文件 | 改动 |
|---|---|
| `code/Img_analyse.c` | `#define MOTION_2024_ENABLE (0)` 包住 170 行 2024 状态机；新增 `Result_Far_Row` |
| `code/Img_analyse.h` | `extern uint8 Result_Far_Row;` |
| `code/control.c` | 舵机保护 `Line_Count` → `Result_Far_Row` |
| `libraries/zf_device/zf_device_mt9v03x.c` | `zyuzhi` 默认 109 → **63** |
| `code/Lu_UI.c` | 恢复默认的 `zyuzhi` 同步改 63 |
| `user/cpu0_main.c` | 心跳日志扩到 `FR/L/R/W/T0/zy/dl/Rm/To/exp/g`，周期 60→10 圈 |

日志字段含义：

```
T    心跳序号          P  拟合中点(x)        LC 扫描停下的行号
FR   本帧最远可信行     L/R 第50行左右边线    W 第50行宽度
d    舵机占空比        st Flag_stop/Flag_turn Rm 菱形标志
T0   Diedai 实际阈值   zy 系数+偏置           dl Differ_light
To   DL1B 距离(mm)     exp 曝光               g 增益
```

正常直道预期：`FR` 30~59、`W` 20~50、`T0` 150~220、`zy` 60~70。
`FR<40` 会把舵机拉回中位；`W` 明显偏大（60+）说明二值化太亮，调小 `zy`。

### V3（2026-10-22，深夜）

| 文件 | 改动 |
|---|---|
| `code/Img_analyse.c/.h` | 新增 `Marker_Scan()`：中央竖带 36~84 列，白点多=横道/停车线，白点少的连续行=菱形；近车头单行白线按停车线处理 |
| `user/cpu1_main.c` | `Marker_Scan` 放在 `Image_Handle` **之后**（先扫会被补线当成"赛道变宽"） |
| `code/control.c` | `Control_Speed` 末尾状态机 0→1（菱形慢行 50%）→2（横道停车 1000×5ms=5s）→3；仅 `Stop_1=1` 生效；发车前舵机也叠加 `Steer_Trim` |
| `user/cpu0_main.c` | 日志加 `V<n>` 前缀与 `DI/CW/MK` 字段 |
| `code/Lu_UI.h` | `CAR_LOG_VERSION` 单一来源，当前 3 |

日志新增字段：

```
DI=菱形seen/near行/score   CW=横道seen/near行/score   MK=mk_state(0~3)
```
