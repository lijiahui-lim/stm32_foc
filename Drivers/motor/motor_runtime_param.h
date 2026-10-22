#pragma once
#include <stdint.h>
#include "conf.h"
#define rotor_phy_angle (encoder_angle - rotor_zero_angle) // 转子物理角度（机械角度）
#define rotor_logic_angle (rotor_phy_angle * POLE_PAIRS)   // 转子逻辑角度（电角度）
extern volatile float motor_i_u;
extern volatile float motor_i_v;
extern volatile float motor_i_d;
extern volatile float motor_i_q;

// 单位均是弧度
extern volatile float motor_speed;        // 电机转速，弧度/秒；TIM3中断更新
extern volatile float motor_logic_angle;  // 电机多圈角度；SPI中断更新
extern volatile float encoder_angle;      // 编码器直接读出的角度；SPI中断更新
extern volatile uint16_t encoder_raw;      // MT6701原始14位机械角度
extern float encoder_init_angle; // 刚上电时编码器角度
extern volatile float rotor_zero_angle;   // 主循环校零，ADC中断使用
extern volatile uint16_t rotor_zero_raw;   // 定点FOC使用的机械零位

#define MOTOR_FAULT_NONE        0u
#define MOTOR_FAULT_ENCODER     1u
#define MOTOR_FAULT_OVERCURRENT 2u
#define MOTOR_FAULT_STALL       3u
#define MOTOR_FAULT_CONTROL     4u
#define MOTOR_FAULT_CURRENT_SENSOR 5u
#define MOTOR_FAULT_SUSTAINED_CURRENT 6u
#define MOTOR_FAULT_DRIVER      7u
#define MOTOR_FAULT_ALIGN       8u
extern volatile uint32_t motor_fault;
