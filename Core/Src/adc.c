/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  // 软复位(无nRESET接线)可能残留ADON位；HAL的双模配置要求ADC1/ADC2都处于关闭态，
  // 否则返回HAL_ERROR导致随机启动失败，这里先强制清掉
  ADC1->CR2 &= ~ADC_CR2_ADON;
  ADC2->CR2 &= ~ADC_CR2_ADON;
  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
  else if (adcHandle->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA1     ------> ADC2_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC2_MspInit 1 */

    /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{

  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* ADC1 interrupt Deinit */
    /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
    /* USER CODE END ADC1:ADC1_2_IRQn disable */

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if (adcHandle->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA1     ------> ADC2_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* ADC2 interrupt Deinit */
    /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
    /* USER CODE END ADC2:ADC1_2_IRQn disable */

    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "global_def.h"
#include "arm_math.h"
#include "spi.h"
#include <math.h>

#define CURRENT_OFFSET_SAMPLES 256u
#define Q15_ONE 32767
#define Q15_FILTER_ALPHA 3277
#define Q14_INV_SQRT3 9459
#define Q14_TWO_INV_SQRT3 18919
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

    uint16_t mechanical_phase = (encoder_raw - rotor_zero_raw) & 0x3fffu;
    int16_t electrical_phase = (int16_t)(
        ((uint32_t)mechanical_phase * POLE_PAIRS * 2u) & 0x7fffu);
    int16_t sin_value = arm_sin_q15(electrical_phase);
    int16_t cos_value = arm_cos_q15(electrical_phase);
    int16_t i_d_raw = saturate_current_q15((int32_t)(
        ((int64_t)i_alpha * cos_value + (int64_t)i_beta * sin_value) >> 15));
    int16_t i_q_raw = saturate_current_q15((int32_t)(
        (-(int64_t)i_alpha * sin_value + (int64_t)i_beta * cos_value) >> 15));
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

    motor_control_fixed(i_alpha, i_beta,
                        (int16_t)i_d_filtered, (int16_t)i_q_filtered,
                        sin_value, cos_value);
  }
}

/* USER CODE END 1 */
