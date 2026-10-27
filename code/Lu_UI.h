/*
 * Lu_UI.h
 *
 *  Created on: 2023年3月9日
 *      Author: 86185
 */

#ifndef CODE_LU_UI_H_
#define CODE_LU_UI_H_

#include "zf_common_headfile.h"
/* [2026-10-20] cross-core debug print request (set by CPU1, printed by CPU0) */
extern volatile uint8 lu_ui_dbg_log;
void LuUI_Log_Poll(void);
/* serial telemetry cadence: 0 = off, N = one line per N frames */
#define AGL_LOG_EVERY_N   0


#define PAGES   3
#define PARAMETERS   15

#define FLASH_SECTION_INDEX       (0)
#define FLASH_PAGE_INDEX          (11)
#define FLASH_PAGE_BAK            (10)   // [2026-10-22] 调参备份页，见 Parameter_Save
                                          // [2026-10-22] 驱动只支持 0~11 页（EEPROM_PAGE_NUM=12），原来写 17 会触发 zf_assert
#define PARAM_TUNING              (1)    // [2026-10-22] 1=上电不读 flash，用代码默认值；定稿后改 0
/* [2026-10-26] 0 = 调参阶段连舵机中位也用代码里的 Steer_Mid，不从 flash 取。
   原来 PARAM_TUNING=1 时还在从 flash 备份页恢复 Steer_Mid，
   flash 里如果留着早期标定的 857，开机就被打成接近右打死（857 vs 917 差 60）。
   定稿、把中位存进 flash 之后再改回 1。 */
#define FLASH_RESTORE_STEER_MID   (0)
/* [2026-10-22] 标线动作开关，日志里用 MK%s 显示；
   0 = 只记录（首次实测用）  1 = 真正减速/停车 */
#define MK_ACT_ENABLE             (1)    /* [2026-10-26] 锥桶被误判成横道导致停车，已改 Marker_Scan 判据（白带须横跨 >=60% 车道宽且盖住车道中心）。若跑起来又在障碍处停车，先把这里改回 0。 */
/* [2026-10-24] 锥形标绕行：1=检测到锥桶就转向避让  0=只识别不动舵机 */
#define CONE_ACT_ENABLE           (1)
/* 1 = stop 5 seconds, cross, then park; 0 = keep driving after crossing. */
#define MK_FINAL_STOP             (1)
extern int8 RMotorSpeed1;
extern int16 canshu_test[8];
extern int8 FLAG1111;
extern int8 Differ;
void LuUI_Key(void);
void LuUI_Show_page1(void);
void LuUI_Show_page2(void);
void LuUI_Show_page3(void);
void LuUI_Show(void);
void Computer_Show(void);
void Parameter_Save(void);
void cam_param_apply(void);
extern uint16 cam_exposure;
extern uint8 steer_mid_cal;   // [2026-10-22] 舵机中位校准模式
extern uint8 cam_set_exp_ret, cam_set_gain_ret;   // [2026-10-22] SCCB 写入结果
extern uint8  cam_gain;
void Parameter_Read(void);
void DisplayLine(uint16 *p, uint16 color);
#endif /* CODE_LU_UI_H_ */
