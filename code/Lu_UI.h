/*
 * Lu_UI.h
 *
 *  Created on: 2023Äê3ÔÂ9ÈÕ
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
extern uint8  cam_gain;
void Parameter_Read(void);
void DisplayLine(uint16 *p, uint16 color);
#endif /* CODE_LU_UI_H_ */
