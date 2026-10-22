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
| `bizhang_1108\tools\car_log.py` | 新增电脑端 UDP 日志接收器 | 按 `\n` 切行、自动拼包、显示时间戳和来源 IP |

**新增脚手架**

- `2024bizhang/engineer_notes/`（README / hardware / changelog）
- git 仓库 + 基线提交 + `.gitignore`

**没做 / 待确认**

- 本轮改动还没在车子上编译烧录（本机没有 `cctc`，需要在 ADS 里 Build）
- 图传（`CAMERA_SEND_ENABLE=1`）和 UDP 日志会同时占用 WiFi 模块，目前两者都用 `wifi_spi_send_*`，**同一条连接里混发**；如果图传一开日志就乱，先把 `CAMERA_SEND_ENABLE` 改 0
- 2024 的标线/菱形逻辑和 2026 双车道规则不匹配，尚未动
