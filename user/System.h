/*
 * System.h
 *
 *  Created on: 2023年9月1日
 *      Author: ASUS
 */
#include "zf_common_headfile.h"
#ifndef USER_SYSTEM_H_
#define USER_SYSTEM_H_

extern void System();



#define S_MOTOR_PIN   ATOM1_CH1_P33_9       //定义舵机引脚

#define SteerMiddle       857  // 850     // 舵机中值百分比（保持 1108 原来的标定）
#define SteerLeft         915       //
#define SteerRight        785     // 600               // 舵机最大右打角百分比
#define SteerSet(x)    pwm_set_duty(S_MOTOR_PIN,x);  // 修改舵机占空比

// 电机 宏定义
// ★两个轮子都用「已验证能转」的 PWM 通道：
//   左轮 PWM = ATOM1_CH5_P02_5  （实测能慢慢转）
//   右轮 PWM = ATOM1_CH6_P02_6  （2024 年跑通过的原装脚）
//   禁用：ATOM1_CH7_P02_7（实测疯转/不转，通道不可用）、ATOM1_CH4_P02_4（未验证）
// ★换到板上的「有刷电机1」接口 P6（21.2~21.5）。P7 那一组的 DIR 反复不生效，换组脚排查。
//   P6 丝印： 21.5 GND 21.4  /  21.3 GND 21.2
//   右轮：PWM=21.2  DIR=21.3     左轮：PWM=21.4  DIR=21.5
#define MOTOR1_A   P21_3                 // 右轮 DIR
#define MOTOR1_B   ATOM1_CH0_P21_2       // 右轮 PWM
#define MOTOR2_A   P21_5                 // 左轮 DIR
#define MOTOR2_B   ATOM1_CH2_P21_4       // 左轮 PWM


//#define ENCODER2_TIM    TIM2_ENCODER    //R
//#define ENCOEDER2_CH1   TIM2_ENCODER_CH1_P33_7
//#define ENCOEDER2_CH2   TIM2_ENCODER_CH2_P33_6



//
//
// ★2024bizhang 改动：原来用 TIM5（P10_3/P10_1，板上"编码器2"座）。
//   该座接触不良，插任何编码器都读不出（已验证编码器本身是好的）。
//   改到板上"编码器4"座（P15）= 20.3/20.0，用 TIM6。
//   备选：板上"编码器1"座（P13）= 02.8/00.9，对应 TIM4。
#define ENCODER1_TIM    TIM6_ENCODER    //R
#define ENCOEDER1_CH1   TIM6_ENCODER_CH1_P20_3
#define ENCOEDER1_CH2   TIM6_ENCODER_CH2_P20_0
//编码器宏定义
#define ENCODER2_TIM    TIM2_ENCODER    //R
#define ENCOEDER2_CH1   TIM2_ENCODER_CH1_P33_7
#define ENCOEDER2_CH2   TIM2_ENCODER_CH2_P33_6
#define MotorMax    3000                              //★和 control.c 的 MOTOR_PWM_MAX 保持一致，
                                                      //  否则 PID 内部积到 8000 但只输出 3000，
                                                      //  多出来的 5000 是空转行程，会造成 ±3000 方波振荡
#define MotorMid    0
#define MotorMin    -3000



//编码器宏定义
#define All_Line       1024.0             //编码器线数
#define Ratio          2.267                 //齿轮比（大齿轮比小齿轮）
#define Length         0.201                 //轮胎长度20.1cm
#define Time           0.005                 //定时周期5ms
#define Multiple       100                   //m/s转换成cm/s
#define SPEED(X)       (int32)(X/All_Line/Ratio*Length/Time*Multiple)
#define DISTANCE(X)    (double)((X)*0.0288)//0.0288

#endif
