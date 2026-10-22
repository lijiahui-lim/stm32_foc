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
void foc_forward(float d, float q, float rotor_rad);
void foc_update_rotor_trig(float rotor_rad, float *sin_value, float *cos_value);
void motor_control_fixed(int16_t i_alpha, int16_t i_beta,
                         int16_t i_d, int16_t i_q,
                         int16_t sin_value, int16_t cos_value);
void set_pwm_duty(float d_u, float d_v, float d_w);
void set_pwm_duty_q15(uint16_t d_u, uint16_t d_v, uint16_t d_w);

void lib_position_control(float rad);                                                           // 位置（角度）控制
void lib_speed_control(float speed);                                                            // 速度控制
void lib_torque_control(float torque_norm_d, float torque_norm_q);                              // 力矩（电流）控制
void lib_speed_torque_control(float speed, float max_torque_norm);                              // 速度-力矩控制
void lib_position_speed_torque_control(float position, float max_speed, float max_torque_norm); // 位置-速度-力矩控制
void lib_align_control(float current_norm, float i_alpha, float i_beta);                       // 限流校零

void set_motor_pid(
    // Ki单位为每秒，Kd单位为秒；函数会按各控制环实际频率离散化。
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d
		);
