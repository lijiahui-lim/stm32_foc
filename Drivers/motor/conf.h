#pragma once
#include "global_def.h"
// 电机物理参数：
#define POLE_PAIRS 7 // 极对数

// 电路参数：
#define U_BUS 12.0f            // 实际母线电压，伏；24V供电时改为24
#define U_BUS_TUNE 12.0f       // 电流环增益的整定母线电压，伏
#define U_BUS_SCALE (U_BUS_TUNE / U_BUS) // 电压环增益缩放：占空比标幺输出乘以母线电压才是相电压
#define R_SHUNT 0.02           // 电流采样电阻，欧姆
#define OP_GAIN 50             // 运放放大倍数
#define MAX_CURRENT 1.5f       // 电流环标幺基准，A；1.65V偏置和1V/A增益仅能测到约±1.65A
#define MOTOR_ALIGN_CURRENT_A 0.30f // 校零目标电流，A
#define MOTOR_RUN_CURRENT_A 0.50f   // 连续运行q轴目标上限，A
#define MOTOR_SUSTAINED_CURRENT_A 0.60f // 0.5A指令的持续过流保护阈值，留出调节纹波余量
#define ADC_REFERENCE_VOLT 3.3 // 电流采样adc参考电压，伏
#define ADC_BITS 12            // ADC精度，bit

// 单片机配置参数：
#define motor_pwm_freq 40000       // 中心对齐PWM实际频率，Hz
#define motor_control_freq 8000    // Q15电流环和FOC频率
#define motor_outer_control_freq 1000 // 速度环和位置环频率
#define motor_speed_calc_freq 5000 // 电机速度计算频率，Hz

// 软件参数：

// 电机软件上的多圈周期，位置模式下能控制的范围，等于正半周期+负半周期，可以任意修改(比如改为1234*pi)
#define position_cycle (6 * PI)
