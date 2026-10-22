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
#define FLASH_PAGE_BAK            (17)   // [2026-10-22] 调参备份页，见 Parameter_Save
#define PARAM_TUNING              (1)    // [2026-10-22] 1=上电不读 flash，用代码默认值；定稿后改 0
/* [2026-10-22] 标线动作开关，日志里用 MK%s 显示；
   0 = 只记录（首次实测用）  1 = 真正减速/停车 */
#define MK_ACT_ENABLE             (0)
#define CAR_LOG_VERSION           (7)    // [2026-10-22] 每次改代码 +1；日志每行开头 V3，防止拿旧日志对错版本
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
extern uint8 cam_set_exp_ret, cam_set_gain_ret;   // [2026-10-22] SCCB 写入结果
extern uint8  cam_gain;
void Parameter_Read(void);
void DisplayLine(uint16 *p, uint16 color);
#endif /* CODE_LU_UI_H_ */
