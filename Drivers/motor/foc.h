#pragma once
#include <stdint.h>

typedef enum
{
    control_type_null,                  // 不进行控制
    control_type_position,              // 位置控制
    control_type_speed,                 // 速度控制
    control_type_torque,                // 力矩控制
    control_type_speed_torque,          // 速度-力矩控制
    control_type_position_speed_torque, // 位置-速度-力矩控制
    control_type_align,                 // 固定定子方向的限流校零
} motor_control_type;

typedef struct
{
    motor_control_type type;
    float position;        // 目标角度，单位rad
    float speed;           // 目标速度，单位rad/s
    float torque_norm_d;   // 目标d轴强度，0~1
    float torque_norm_q;   // 目标q轴强度，0~1
    float max_speed;       // 串级控制时的最大速度，单位rad/s
    float max_torque_norm; // 串级控制时的最大q轴力矩，0~1
} motor_control_context_t;

extern volatile motor_control_context_t motor_control_context;

float cycle_diff(float diff, float cycle);
void motor_control_fixed(int16_t i_alpha, int16_t i_beta,
                         int16_t i_d, int16_t i_q,
                         int16_t sin_value, int16_t cos_value);
// 以下两个函数由板级代码实现（main.c）：写三相PWM比较寄存器
void set_pwm_duty(float d_u, float d_v, float d_w);
void set_pwm_duty_q15(uint16_t d_u, uint16_t d_v, uint16_t d_w);

void set_motor_pid(
    // Ki单位为每秒，Kd单位为秒；函数会按各控制环实际频率离散化。
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d
		);
