// 电机控制的全部中断回调集中在此文件：
// 1. ADC注入转换完成（8kHz电流环/FOC主路径）
// 2. TIM3周期（5kHz编码器读取与速度估算）
// 3. EXTI（DRV8313 nFAULT硬件故障）
// 与CubeMX生成的外设初始化文件解耦，重新生成.ioc配置时不会误伤控制逻辑。
#include "adc.h"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "algorithm/filter.h"
#include "global_def.h"
#include "arm_math.h"
#include <math.h>

#define CURRENT_OFFSET_SAMPLES 256u
#define Q15_ONE 32767
#define Q15_FILTER_ALPHA 3277
#define Q14_INV_SQRT3 9459
#define Q14_TWO_INV_SQRT3 18919
// 编码器帧滞后补偿上限：0.5ms对应的CPU周期数
#define ANGLE_AGE_MAX_CYCLES 36000u
// 单帧角度差分允许的最大速度，超出视为编码器异常毛刺
#define MAX_PLAUSIBLE_SPEED_RAD_S 400.0f

volatile uint8_t motor_current_offset_ready = 0;
static int32_t adc_offset_u = 2048;
static int32_t adc_offset_v = 2048;
static int32_t current_scale_q8;
static int32_t sustained_limit_q15;

static int16_t saturate_current_q15(int32_t value)
{
  if (value > Q15_ONE)
    return Q15_ONE;
  if (value < -Q15_ONE)
    return -Q15_ONE;
  return (int16_t)value;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    if (motor_fault != MOTOR_FAULT_NONE)
      return;
    if (!mt6701_angle_is_fresh())
    {
      motor_stop(MOTOR_FAULT_ENCODER);
      return;
    }

    uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    if (!motor_current_offset_ready)
    {
      static uint32_t sum_u = 0;
      static uint32_t sum_v = 0;
      static uint32_t sample_count = 0;
      sum_u += raw_u;
      sum_v += raw_v;
      if (++sample_count == CURRENT_OFFSET_SAMPLES)
      {
        adc_offset_u = (int32_t)((sum_u + CURRENT_OFFSET_SAMPLES / 2u) /
                                 CURRENT_OFFSET_SAMPLES);
        adc_offset_v = (int32_t)((sum_v + CURRENT_OFFSET_SAMPLES / 2u) /
                                 CURRENT_OFFSET_SAMPLES);
        // 板上INA199由1.65V提供参考；偏置接近电源轨说明采样链故障。
        int32_t adc_full_scale = (1u << ADC_BITS) - 1u;
        if (adc_offset_u < 4 * adc_full_scale / 10 || adc_offset_u > 6 * adc_full_scale / 10 ||
            adc_offset_v < 4 * adc_full_scale / 10 || adc_offset_v > 6 * adc_full_scale / 10)
        {
          motor_stop(MOTOR_FAULT_CURRENT_SENSOR);
          return;
        }
        // 每ADC count换算为标幺Q15；保留8个小数位避免整数比例量化。
        current_scale_q8 = (int32_t)lroundf(
            ADC_REFERENCE_VOLT * 32768.0f * 256.0f /
            (adc_full_scale * R_SHUNT * OP_GAIN * MAX_CURRENT));
        sustained_limit_q15 = (int32_t)lroundf(
            MOTOR_SUSTAINED_CURRENT_A * 32768.0f / MAX_CURRENT);
        motor_current_offset_ready = 1;
      }
      set_pwm_duty_q15(0, 0, 0);
      return;
    }

    int32_t i_u_wide = ((int32_t)raw_u - adc_offset_u) * current_scale_q8 >> 8;
    int32_t i_v_wide = ((int32_t)raw_v - adc_offset_v) * current_scale_q8 >> 8;
    int32_t i_w_wide = -(i_u_wide + i_v_wide);
    static uint8_t overcurrent_samples = 0;
    if (i_u_wide > Q15_ONE || i_u_wide < -Q15_ONE ||
        i_v_wide > Q15_ONE || i_v_wide < -Q15_ONE ||
        i_w_wide > Q15_ONE || i_w_wide < -Q15_ONE)
    {
      if (++overcurrent_samples >= 3u)
      {
        motor_stop(MOTOR_FAULT_OVERCURRENT);
        return;
      }
    }
    else
    {
      overcurrent_samples = 0;
    }

    int16_t i_u = saturate_current_q15(i_u_wide);
    int16_t i_v = saturate_current_q15(i_v_wide);
    int16_t i_alpha = i_u;
    int16_t i_beta = saturate_current_q15(
        ((int32_t)i_u * Q14_INV_SQRT3 +
         (int32_t)i_v * Q14_TWO_INV_SQRT3) >> 14);

    static int64_t current_square_average;
    int64_t current_square = (int32_t)i_alpha * i_alpha +
                             (int32_t)i_beta * i_beta;
    int32_t rms_alpha_q15 = (5 * 32768) / motor_control_freq;
    current_square_average +=
        ((current_square - current_square_average) * rms_alpha_q15) >> 15;
    if (current_square_average >
        (int64_t)sustained_limit_q15 * sustained_limit_q15)
    {
      motor_stop(MOTOR_FAULT_SUSTAINED_CURRENT);
      return;
    }

    // 编码器帧记录的是采样瞬间的位置，最新一帧最多滞后一个SPI帧周期。
    // 按当前速度把电角度推进到本次电流采样时刻，消除高速下的相位滞后。
    uint16_t mechanical_phase = (encoder_raw - rotor_zero_raw) & 0x3fffu;
    uint32_t angle_age = DWT->CYCCNT - mt6701_last_valid_cycle;
    if (angle_age > ANGLE_AGE_MAX_CYCLES)
      angle_age = ANGLE_AGE_MAX_CYCLES;
    int32_t lead_counts = (int32_t)(motor_speed * POLE_PAIRS *
                                    (angle_age * (1.0f / 72000000.0f)) *
                                    (32768.0f / (2.0f * PI)));
    int16_t electrical_phase = (int16_t)(
        (((uint32_t)mechanical_phase * POLE_PAIRS * 2u) +
         (uint32_t)lead_counts) &
        0x7fffu);
    int16_t sin_value = arm_sin_q15(electrical_phase);
    int16_t cos_value = arm_cos_q15(electrical_phase);
    int16_t i_d_raw = saturate_current_q15((int32_t)(
        ((int64_t)i_alpha * cos_value + (int64_t)i_beta * sin_value) >> 15));
    int16_t i_q_raw = saturate_current_q15((int32_t)(
        (-(int64_t)i_alpha * sin_value + (int64_t)i_beta * cos_value) >> 15));
    // 控制环直接使用原始i_d/i_q：PWM中心同步采样已经足够干净，
    // 环内一阶低通(140Hz)会显著压缩相位裕度。滤波值只用于诊断输出。
    static int32_t i_d_filtered;
    static int32_t i_q_filtered;
    i_d_filtered += ((int32_t)(i_d_raw - i_d_filtered) * Q15_FILTER_ALPHA) >> 15;
    i_q_filtered += ((int32_t)(i_q_raw - i_q_filtered) * Q15_FILTER_ALPHA) >> 15;

    // 浮点变量只用于低速串口诊断，每16次控制更新一次。
    static uint8_t diagnostic_divider;
    if (++diagnostic_divider >= 16u)
    {
      diagnostic_divider = 0;
      const float current_from_q15 = MAX_CURRENT / 32768.0f;
      motor_i_u = i_u * current_from_q15;
      motor_i_v = i_v * current_from_q15;
      motor_i_d = i_d_filtered * current_from_q15;
      motor_i_q = i_q_filtered * current_from_q15;
    }

    motor_control_fixed(i_alpha, i_beta, i_d_raw, i_q_raw,
                        sin_value, cos_value);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // 与SPI DMA中断同优先级，读到的是上一帧已经校验过的角度。
    mt6701_start_read();

    static uint32_t last_frame = 0;
    static uint32_t last_cycle = 0;
    static float last_angle = 0.0f;
    uint32_t frame = mt6701_valid_frame_count;
    if (frame == last_frame)
      return; // CRC错误或DMA丢帧时不能把旧角度当作新的零速样本

    uint32_t cycle = mt6701_last_valid_cycle;
    float angle = encoder_angle;
    uint32_t elapsed = cycle - last_cycle;
    if (last_frame != 0u && elapsed != 0u && elapsed <= MT6701_MAX_AGE_CYCLES)
    {
      float diff_angle = cycle_diff(angle - last_angle, 2 * PI);
      float elapsed_seconds = (float)elapsed * (1.0f / 72000000.0f);
      float raw_speed = diff_angle / elapsed_seconds;
      // CRC只能防位错误；钳掉物理上不可能的毛刺再进低通。
      raw_speed = fmaxf(-MAX_PLAUSIBLE_SPEED_RAD_S,
                        fminf(raw_speed, MAX_PLAUSIBLE_SPEED_RAD_S));
      // 约10Hz截止频率；按真实采样间隔计算，避免中断延迟造成转速虚高。
      float alpha = fminf(62.83f * elapsed_seconds, 1.0f);
      motor_speed = low_pass_filter(raw_speed, motor_speed, alpha);
    }
    else if (last_frame != 0u)
    {
      motor_speed = 0.0f;
    }
    last_frame = frame;
    last_cycle = cycle;
    last_angle = angle;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TIM1_Break_Pin)
    motor_stop(MOTOR_FAULT_DRIVER);
}
