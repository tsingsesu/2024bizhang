# archive/ —— 不参与编译的归档目录

**这里的文件不会进 ADS 工程**（全是 `.txt` / `.c_before_cleanup`，ADS 只编 `.c`）。
第一次清理时我误把归档写成 `.c`，结果 ADS 把整个目录编进去了，Build 直接炸——已改掉。

| 文件 | 是什么 | 什么时候会用到 |
|---|---|---|
| `legacy_2024_state_machine.c.txt` | 2026-10-30 从 `code/Img_analyse.c` 摘出来的 2024 赛道状态机（`#if MOTION_2024_ENABLE` 那块）+ 11 个零调用函数 | 想恢复单车道菱形/斑马线逻辑时 |
| `Img_analyse.c_before_cleanup` | 清理前的完整 `Img_analyse.c`（2523 行） | 怀疑删错了，整文件回滚 |
| `Img_analyse.h_before_cleanup` | 清理前的完整 `Img_analyse.h` | 同上 |
| `broken_Img_analyse_after_cleanup.c.txt` | 第一版清理**失败**的产物（注释收尾被删，`/*` 与 `*/` 不配平） | 留作反面教材，别用 |

## 2026-10-30 清理了什么

`code/Img_analyse.c`：**2523 → 2033 行**（-490）。`code/Img_analyse.h`：删掉 12 条对应声明。

删掉的函数（全工程零调用，调用图确认过）：

```
Turn_Left  Turn_Right  Decelerate  Decelerate_2
Find_Zebra  Find_Decelerate  Find_rhombus_Out
New_boundary  Point_Repair_Line  zuobianjie  youbianjie
Find_Cone_bucket
```

外加 `#define MOTION_2024_ENABLE (0)` + `#if MOTION_2024_ENABLE … #endif` 整块（第 984~1156 行）。

**没动的东西**：`Flag_turn` / `Flag_stop` / `Deal_Flag` / `Light_Flag` / `Rhombus_Flag` /
`Distance` / `Distance_Test` 这些全局量全部保留 —— `control.c` 和 `Light()` 还在用。

## ⚠️ 第一版为什么失败（别再犯）

第一次按"括号配对"删函数时，`MOTION_2024_ENABLE` 前面那段 4 行注释的**收尾行 `*/`** 也被算进了删除区间，
结果整段代码被吞进注释，编译器报了一屏 E208。教训：

> **删代码前先遮蔽注释和字符串再做括号配对**，删完再检查 `/*` 和 `*/` 数量是否相等。

第二版（`cleanup_v2.py`）按这个思路做的，删完校验：`/*`/`*/` = 88/88，遮蔽后花括号平衡 0，`#if`/`#endif` = 2/2。

## 恢复方法

1. 从 `legacy_2024_state_machine.c.txt` 里粘回对应段落
2. 把 `Img_analyse.h` 里的函数声明加回去（也在那份归档里）
3. `MOTION_2024_ENABLE` 改回 1
