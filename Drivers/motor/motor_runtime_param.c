#include "motor_runtime_param.h"

volatile float motor_i_u;
volatile float motor_i_v;
volatile float motor_i_d;
volatile float motor_i_q;
volatile float motor_speed;
volatile float motor_logic_angle;
volatile float encoder_angle;
volatile uint16_t encoder_raw;
float encoder_init_angle;
volatile float rotor_zero_angle;
volatile uint16_t rotor_zero_raw;
volatile uint32_t motor_fault;
