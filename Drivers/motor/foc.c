#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include <stdbool.h>
#include <math.h>
#define SQRT3_OVER_2 0.8660254f
#define INV_SQRT3 0.57735027f

arm_pid_instance_f32 pid_position;
arm_pid_instance_f32 pid_speed;
arm_pid_instance_f32 pid_torque_d;
arm_pid_instance_f32 pid_torque_q;
static arm_pid_instance_f32 pid_align_alpha;
static arm_pid_instance_f32 pid_align_beta;
volatile motor_control_context_t motor_control_context;
static float cached_rotor_rad;
static float cached_sin;
static float cached_cos;
static bool cached_trig_valid;

#define PID_GAIN_SHIFT 12
#define Q15_ONE 32767
#define Q15_HALF 16384
#define Q15_DUTY_MIN 3277
#define Q15_DUTY_MAX 29490
#define Q15_DUTY_RANGE 13107
#define Q15_SQRT3_OVER_2 28378
#define Q15_INV_SQRT3 18919
#if (motor_control_freq % motor_outer_control_freq) != 0
#error "motor_control_freq must be divisible by motor_outer_control_freq"
#endif
#define OUTER_LOOP_DIVIDER (motor_control_freq / motor_outer_control_freq)

typedef struct
{
    int32_t a0;
    int32_t a1;
    int32_t a2;
    int16_t error_1;
    int16_t error_2;
    int32_t output;
} fixed_pid_t;

static fixed_pid_t pid_torque_d_fixed;
static fixed_pid_t pid_torque_q_fixed;
static fixed_pid_t pid_align_alpha_fixed;
static fixed_pid_t pid_align_beta_fixed;
static int16_t current_command_limit_q15;

static int16_t saturate_q15(int32_t value)
{
    if (value > Q15_ONE)
        return Q15_ONE;
    if (value < -Q15_ONE)
        return -Q15_ONE;
    return (int16_t)value;
}

static int16_t float_to_q15(float value)
{
    if (value >= 1.0f)
        return Q15_ONE;
    if (value <= -1.0f)
        return -Q15_ONE;
    return (int16_t)lroundf(value * 32768.0f);
}

static int16_t limit_current_target(int16_t target)
{
    if (target > current_command_limit_q15)
        return current_command_limit_q15;
    if (target < -current_command_limit_q15)
        return -current_command_limit_q15;
    return target;
}

static void fixed_pid_init(fixed_pid_t *pid, float kp, float ki, float kd)
{
    pid->a0 = (int32_t)lroundf((kp + ki + kd) * (1 << PID_GAIN_SHIFT));
    pid->a1 = (int32_t)lroundf((-kp - 2.0f * kd) * (1 << PID_GAIN_SHIFT));
    pid->a2 = (int32_t)lroundf(kd * (1 << PID_GAIN_SHIFT));
    pid->error_1 = 0;
    pid->error_2 = 0;
    pid->output = 0;
}

static int16_t fixed_pid_step(fixed_pid_t *pid, int16_t error, int16_t limit)
{
    int64_t delta = (int64_t)pid->a0 * error +
                    (int64_t)pid->a1 * pid->error_1 +
                    (int64_t)pid->a2 * pid->error_2;
    int32_t output = pid->output + (int32_t)(delta >> PID_GAIN_SHIFT);
    if (output > limit)
        output = limit;
    else if (output < -limit)
        output = -limit;
    pid->error_2 = pid->error_1;
    pid->error_1 = error;
    pid->output = output;
    return (int16_t)output;
}

static void set_voltage_alpha_beta_q15(int32_t alpha, int32_t beta)
{
    int32_t phase_u = alpha;
    int32_t phase_v = -alpha / 2 + (beta * Q15_SQRT3_OVER_2 >> 15);
    int32_t phase_w = -alpha / 2 - (beta * Q15_SQRT3_OVER_2 >> 15);
    int32_t highest = phase_u > phase_v ? phase_u : phase_v;
    highest = highest > phase_w ? highest : phase_w;
    int32_t lowest = phase_u < phase_v ? phase_u : phase_v;
    lowest = lowest < phase_w ? lowest : phase_w;
    int32_t common = (highest + lowest) / 2;

    int32_t duty_u = Q15_HALF + ((phase_u - common) * Q15_INV_SQRT3 >> 15);
    int32_t duty_v = Q15_HALF + ((phase_v - common) * Q15_INV_SQRT3 >> 15);
    int32_t duty_w = Q15_HALF + ((phase_w - common) * Q15_INV_SQRT3 >> 15);
    highest = duty_u > duty_v ? duty_u : duty_v;
    highest = highest > duty_w ? highest : duty_w;
    lowest = duty_u < duty_v ? duty_u : duty_v;
    lowest = lowest < duty_w ? lowest : duty_w;

    int32_t scale = Q15_ONE;
    if (highest > Q15_DUTY_MAX)
        scale = (Q15_DUTY_RANGE << 15) / (highest - Q15_HALF);
    if (lowest < Q15_DUTY_MIN)
    {
        int32_t low_scale = (Q15_DUTY_RANGE << 15) / (Q15_HALF - lowest);
        if (low_scale < scale)
            scale = low_scale;
    }
    duty_u = Q15_HALF + ((duty_u - Q15_HALF) * scale >> 15);
    duty_v = Q15_HALF + ((duty_v - Q15_HALF) * scale >> 15);
    duty_w = Q15_HALF + ((duty_w - Q15_HALF) * scale >> 15);
    duty_u = duty_u < Q15_DUTY_MIN ? Q15_DUTY_MIN :
             (duty_u > Q15_DUTY_MAX ? Q15_DUTY_MAX : duty_u);
    duty_v = duty_v < Q15_DUTY_MIN ? Q15_DUTY_MIN :
             (duty_v > Q15_DUTY_MAX ? Q15_DUTY_MAX : duty_v);
    duty_w = duty_w < Q15_DUTY_MIN ? Q15_DUTY_MIN :
             (duty_w > Q15_DUTY_MAX ? Q15_DUTY_MAX : duty_w);
    set_pwm_duty_q15((uint16_t)duty_u, (uint16_t)duty_v, (uint16_t)duty_w);
}

static void foc_forward_q15(int16_t d, int16_t q, int16_t sin_value, int16_t cos_value)
{
    int32_t alpha = ((int32_t)d * cos_value - (int32_t)q * sin_value) >> 15;
    int32_t beta = ((int32_t)d * sin_value + (int32_t)q * cos_value) >> 15;
    set_voltage_alpha_beta_q15(alpha, beta);
}

static float limited_pid(arm_pid_instance_f32 *pid, float error, float limit)
{
    float out = arm_pid_f32(pid, error);
    out = fmaxf(-limit, fminf(out, limit));
    pid->state[2] = out; // CMSIS增量式PID的输出状态，防止积分饱和
    return out;
}

void foc_update_rotor_trig(float rotor_rad, float *sin_value, float *cos_value)
{
    cached_rotor_rad = rotor_rad;
    cached_sin = arm_sin_f32(rotor_rad);
    cached_cos = arm_cos_f32(rotor_rad);
    cached_trig_valid = true;
    *sin_value = cached_sin;
    *cos_value = cached_cos;
}

static void svpwm(float sin_phi, float cos_phi, float d, float q,
                  float *d_u, float *d_v, float *d_w)
{
    float alpha = 0.0f;
    float beta = 0.0f;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    // 等价的零序注入SVPWM，省去扇区判断、平方根和四次三角函数。
    float phase_u = alpha;
    float phase_v = -0.5f * alpha + SQRT3_OVER_2 * beta;
    float phase_w = -0.5f * alpha - SQRT3_OVER_2 * beta;
    float highest = fmaxf(phase_u, fmaxf(phase_v, phase_w));
    float lowest = fminf(phase_u, fminf(phase_v, phase_w));
    float common = 0.5f * (highest + lowest);

    *d_u = 0.5f + (phase_u - common) * INV_SQRT3;
    *d_v = 0.5f + (phase_v - common) * INV_SQRT3;
    *d_w = 0.5f + (phase_w - common) * INV_SQRT3;
}

void set_pwm_duty(float d_u, float d_v, float d_w) __attribute__((weak));
void set_pwm_duty(float d_u, float d_v, float d_w)
// __attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
{
    while (1)
        ;
}

void set_pwm_duty_q15(uint16_t d_u, uint16_t d_v, uint16_t d_w) __attribute__((weak));
void set_pwm_duty_q15(uint16_t d_u, uint16_t d_v, uint16_t d_w)
{
    while (1)
        ;
}

void foc_forward(float d, float q, float rotor_rad)
{
    float sin_phi;
    float cos_phi;
    if (cached_trig_valid && rotor_rad == cached_rotor_rad)
    {
        sin_phi = cached_sin;
        cos_phi = cached_cos;
    }
    else
    {
        sin_phi = arm_sin_f32(rotor_rad);
        cos_phi = arm_cos_f32(rotor_rad);
    }
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(sin_phi, cos_phi, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

static float position_loop(float rad)
{
    float diff = cycle_diff(rad - motor_logic_angle, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

static float speed_loop(float speed_rad, float out_limit)
{
    float diff = speed_rad - motor_speed;
    return limited_pid(&pid_speed, diff, fmaxf(0.0f, fminf(out_limit, 1.0f)));
}

static float torque_d_loop(float d)
{
    float diff = d - motor_i_d / MAX_CURRENT;
    return limited_pid(&pid_torque_d, diff, 1.0f);
}

static float torque_q_loop(float q)
{
    float diff = q - motor_i_q / MAX_CURRENT;
    return limited_pid(&pid_torque_q, diff, 1.0f);
}

void lib_position_control(float rad)
{
    float d = 0;
    float q = position_loop(rad);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_control(float speed)
{
    float d = 0;
    float q = speed_loop(speed, 1.0f);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_torque_control(float torque_norm_d, float torque_norm_q)
{
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad, max_torque_norm);
    lib_torque_control(0, torque_norm);
}

void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = fmaxf(-max_speed_rad, fminf(speed_rad, max_speed_rad));
    lib_speed_torque_control(speed_rad, max_torque_norm);
}

void lib_align_control(float current_norm, float i_alpha, float i_beta)
{
    // 尚未测得转子零位时，在静止的定子坐标系闭环施加alpha轴电流。
    float target = fmaxf(0.0f, fminf(current_norm, 1.0f));
    float alpha_voltage = limited_pid(&pid_align_alpha, target - i_alpha / MAX_CURRENT, 0.5f);
    float beta_voltage = limited_pid(&pid_align_beta, -i_beta / MAX_CURRENT, 0.5f);
    foc_forward(alpha_voltage, beta_voltage, 0.0f);
}

void motor_control_fixed(int16_t i_alpha, int16_t i_beta,
                         int16_t i_d, int16_t i_q,
                         int16_t sin_value, int16_t cos_value)
{
    int16_t target_d = 0;
    int16_t target_q = 0;
    int16_t voltage_d = 0;
    int16_t voltage_q = 0;
    static motor_control_type last_type = control_type_null;
    static uint16_t outer_loop_counter;
    static int16_t held_outer_target_q;

    if (motor_control_context.type != last_type)
    {
        last_type = motor_control_context.type;
        outer_loop_counter = OUTER_LOOP_DIVIDER - 1;
        held_outer_target_q = 0;
    }
    bool update_outer_loop = ++outer_loop_counter >= OUTER_LOOP_DIVIDER;
    if (update_outer_loop)
        outer_loop_counter = 0;

    switch (motor_control_context.type)
    {
    case control_type_align:
    {
        int16_t target_alpha = float_to_q15(motor_control_context.torque_norm_d);
        int16_t voltage_alpha = fixed_pid_step(
            &pid_align_alpha_fixed,
            saturate_q15((int32_t)target_alpha - i_alpha), Q15_HALF);
        int16_t voltage_beta = fixed_pid_step(
            &pid_align_beta_fixed, saturate_q15(-(int32_t)i_beta), Q15_HALF);
        set_voltage_alpha_beta_q15(voltage_alpha, voltage_beta);
        return;
    }
    case control_type_position:
        if (update_outer_loop)
            held_outer_target_q = limit_current_target(
                float_to_q15(position_loop(motor_control_context.position)));
        target_q = held_outer_target_q;
        break;
    case control_type_speed:
        if (update_outer_loop)
            held_outer_target_q = limit_current_target(float_to_q15(speed_loop(
                motor_control_context.speed, MOTOR_RUN_CURRENT_A / MAX_CURRENT)));
        target_q = held_outer_target_q;
        break;
    case control_type_torque:
        target_d = limit_current_target(
            float_to_q15(motor_control_context.torque_norm_d));
        target_q = limit_current_target(
            float_to_q15(motor_control_context.torque_norm_q));
        break;
    case control_type_speed_torque:
        if (update_outer_loop)
            held_outer_target_q = limit_current_target(float_to_q15(speed_loop(
                motor_control_context.speed, motor_control_context.max_torque_norm)));
        target_q = held_outer_target_q;
        break;
    case control_type_position_speed_torque:
    {
        if (update_outer_loop)
        {
            float target_speed = position_loop(motor_control_context.position);
            target_speed = fmaxf(-motor_control_context.max_speed,
                                 fminf(target_speed, motor_control_context.max_speed));
            held_outer_target_q = limit_current_target(float_to_q15(speed_loop(
                target_speed, motor_control_context.max_torque_norm)));
        }
        target_q = held_outer_target_q;
        break;
    }
    default:
        set_pwm_duty_q15(0, 0, 0);
        return;
    }

    voltage_d = fixed_pid_step(
        &pid_torque_d_fixed,
        saturate_q15((int32_t)target_d - i_d), Q15_ONE);
    voltage_q = fixed_pid_step(
        &pid_torque_q_fixed,
        saturate_q15((int32_t)target_q - i_q), Q15_ONE);
    foc_forward_q15(voltage_d, voltage_q, sin_value, cos_value);
}

void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d)
{
    // 调参接口采用连续时间增益；CMSIS增量式PID需要离散化后的每样本增益。
    float position_i_discrete = position_i / motor_outer_control_freq;
    float position_d_discrete = position_d * motor_outer_control_freq;
    float speed_i_discrete = speed_i / motor_outer_control_freq;
    float speed_d_discrete = speed_d * motor_outer_control_freq;
    float torque_d_i_discrete = torque_d_i / motor_control_freq;
    float torque_d_d_discrete = torque_d_d * motor_control_freq;
    float torque_q_i_discrete = torque_q_i / motor_control_freq;
    float torque_q_d_discrete = torque_q_d * motor_control_freq;

    pid_position.Kp = position_p;
    pid_position.Ki = position_i_discrete;
    pid_position.Kd = position_d_discrete;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i_discrete;
    pid_speed.Kd = speed_d_discrete;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i_discrete;
    pid_torque_d.Kd = torque_d_d_discrete;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i_discrete;
    pid_torque_q.Kd = torque_q_d_discrete;

    pid_align_alpha.Kp = torque_d_p;
    pid_align_alpha.Ki = torque_d_i_discrete;
    pid_align_alpha.Kd = torque_d_d_discrete;
    pid_align_beta.Kp = torque_q_p;
    pid_align_beta.Ki = torque_q_i_discrete;
    pid_align_beta.Kd = torque_q_d_discrete;

    arm_pid_init_f32(&pid_position, true);
    arm_pid_init_f32(&pid_speed, true);
    arm_pid_init_f32(&pid_torque_d, true);
    arm_pid_init_f32(&pid_torque_q, true);
    arm_pid_init_f32(&pid_align_alpha, true);
    arm_pid_init_f32(&pid_align_beta, true);

    fixed_pid_init(&pid_torque_d_fixed, torque_d_p,
                   torque_d_i_discrete, torque_d_d_discrete);
    fixed_pid_init(&pid_torque_q_fixed, torque_q_p,
                   torque_q_i_discrete, torque_q_d_discrete);
    fixed_pid_init(&pid_align_alpha_fixed, torque_d_p,
                   torque_d_i_discrete, torque_d_d_discrete);
    fixed_pid_init(&pid_align_beta_fixed, torque_q_p,
                   torque_q_i_discrete, torque_q_d_discrete);
    current_command_limit_q15 = float_to_q15(MOTOR_RUN_CURRENT_A / MAX_CURRENT);
}

float cycle_diff(float diff, float cycle)
{
    if (diff > (cycle / 2))
        diff -= cycle;
    else if (diff < (-cycle / 2))
        diff += cycle;
    return diff;
}
