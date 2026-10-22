/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
// #if defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6000000)
// #error "不支持使用 Keil ARM Compiler V5编译器 请使用 Keil ARM Compiler V6编译"
// #endif
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "global_def.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_SPEED_RPM 1200.0f
#define TARGET_SPEED_RAD_S (TARGET_SPEED_RPM * 2.0f * PI / 60.0f)
#define SPEED_RAMP_MS 1570u // 维持原来约80 rad/s²的加速斜率
#define STALL_TIMEOUT_MS 2000u
#define STARTUP_STILL_MS 100u
#define STARTUP_STILL_TIMEOUT_MS 2000u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static inline void iwdg_feed(void)
{
  IWDG->KR = 0xAAAAu;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_pwm_duty(float d_u, float d_v, float d_w)
{
  if (!isfinite(d_u) || !isfinite(d_v) || !isfinite(d_w))
  {
    motor_stop(MOTOR_FAULT_CONTROL);
    return;
  }
  if (d_u != 0.0f || d_v != 0.0f || d_w != 0.0f)
  {
    // 保持三相电压矢量方向，同时在上下计数端都留10%的采样窗口。
    float highest = fmaxf(d_u, fmaxf(d_v, d_w));
    float lowest = fminf(d_u, fminf(d_v, d_w));
    float scale = 1.0f;
    if (highest > 0.9f)
      scale = fminf(scale, 0.4f / (highest - 0.5f));
    if (lowest < 0.1f)
      scale = fminf(scale, 0.4f / (0.5f - lowest));
    d_u = fmaxf(0.1f, fminf(0.5f + scale * (d_u - 0.5f), 0.9f));
    d_v = fmaxf(0.1f, fminf(0.5f + scale * (d_v - 0.5f), 0.9f));
    d_w = fmaxf(0.1f, fminf(0.5f + scale * (d_w - 0.5f), 0.9f));
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
  __set_PRIMASK(primask);
}

void set_pwm_duty_q15(uint16_t d_u, uint16_t d_v, uint16_t d_w)
{
  if (d_u > 32767u || d_v > 32767u || d_w > 32767u)
  {
    motor_stop(MOTOR_FAULT_CONTROL);
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  TIM1->CCR1 = ((uint32_t)d_u * TIM1->ARR) >> 15;
  TIM1->CCR2 = ((uint32_t)d_v * TIM1->ARR) >> 15;
  TIM1->CCR3 = ((uint32_t)d_w * TIM1->ARR) >> 15;
  __set_PRIMASK(primask);
}

void motor_stop(uint32_t reason)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (motor_fault == MOTOR_FAULT_NONE)
    motor_fault = reason;
  motor_control_context.type = control_type_null;
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0u)
  {
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->BDTR &= ~TIM_BDTR_MOE; // 锁住PWM输出，必须重启才能恢复
  }
  __set_PRIMASK(primask);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  // DRV8313上电时nFAULT会短暂为低；PWM尚未启动，等待其连续稳定为高。
  uint32_t driver_wait_start = HAL_GetTick();
  uint32_t driver_ready_since = driver_wait_start;
  while (HAL_GetTick() - driver_ready_since < 10u)
  {
    if (HAL_GPIO_ReadPin(TIM1_Break_GPIO_Port, TIM1_Break_Pin) == GPIO_PIN_RESET)
      driver_ready_since = HAL_GetTick();
    if (HAL_GetTick() - driver_wait_start > 500u)
      Error_Handler();
  }
  motor_fault = MOTOR_FAULT_NONE;
  set_motor_pid(
      3.5f, 0.0f, 0.007f,
      0.01f, 0.15f, 0.0f, // 连续时间P,I,D；初始化时按1kHz外环离散化
      1.2f, 160.0f, 0.0f, // 连续时间电流PI；按8kHz离散化后Ki仍为0.02
      1.2f, 160.0f, 0.0f);

  mt6701_start_read();
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    Error_Handler();
  uint32_t wait_start = HAL_GetTick();
  while (mt6701_valid_frame_count == 0u)
  {
    if (HAL_GetTick() - wait_start > 100u)
      Error_Handler();
  }

  // 软复位时转子可能仍在转；必须静止后才能把电流采样值当作零点。
  uint32_t quiet_since = HAL_GetTick();
  wait_start = quiet_since;
  while (HAL_GetTick() - quiet_since < STARTUP_STILL_MS)
  {
    if (motor_fault != MOTOR_FAULT_NONE || !mt6701_angle_is_fresh() ||
        HAL_GetTick() - wait_start > STARTUP_STILL_TIMEOUT_MS)
      Error_Handler();
    if (fabsf(motor_speed) > 1.0f)
      quiet_since = HAL_GetTick();
  }

  if (motor_fault != MOTOR_FAULT_NONE ||
      HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK ||
      HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
      HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
      HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    Error_Handler();

  wait_start = HAL_GetTick();
  while (!motor_current_offset_ready)
  {
    if (motor_fault != MOTOR_FAULT_NONE || HAL_GetTick() - wait_start > 250u)
      Error_Handler();
  }

  encoder_init_angle = encoder_angle; // 记录一下上电时编码器角度
  motor_control_context.torque_norm_d = MOTOR_ALIGN_CURRENT_A / MAX_CURRENT;
  motor_control_context.type = control_type_align;
  // 定子alpha轴限流校零：至少400ms让转子机械到位，且电流须稳定在
  // 目标附近50ms以上才记零位；1.5s不收敛说明电流环根本没工作。
  uint32_t align_start = HAL_GetTick();
  uint32_t align_settled_since = align_start;
  bool align_ok = false;
  while (HAL_GetTick() - align_start < 1500u)
  {
    if (motor_fault != MOTOR_FAULT_NONE)
      Error_Handler();
    if (fabsf(motor_i_u - MOTOR_ALIGN_CURRENT_A) < 0.05f)
    {
      if (HAL_GetTick() - align_settled_since >= 50u &&
          HAL_GetTick() - align_start >= 400u)
      {
        align_ok = true;
        break;
      }
    }
    else
    {
      align_settled_since = HAL_GetTick();
    }
    HAL_Delay(5);
  }
  if (!align_ok)
    motor_stop(MOTOR_FAULT_ALIGN);
  if (motor_fault != MOTOR_FAULT_NONE)
    Error_Handler();
  rotor_zero_angle = encoder_angle;
  rotor_zero_raw = encoder_raw;
  motor_control_context.type = control_type_null;
  set_pwm_duty(0.0f, 0.0f, 0.0f);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 直接进入限流速度环，目标从0缓升；不再先用固定力矩盲转1秒。
  motor_control_context.speed = 0.0f;
  motor_control_context.max_torque_norm = MOTOR_RUN_CURRENT_A / MAX_CURRENT;
  motor_control_context.type = control_type_speed_torque;
  // 独立看门狗：主循环每~200ms喂一次，固件跑飞约1s后复位并切断PWM输出。
  // 调试器halt内核时冻结计数，避免SWD监控触发复位。IWDG启动后由硬件自动开LSI。
  __HAL_DBGMCU_FREEZE_IWDG();
  // 必须先启动看门狗：F1的LSI由IWDG启动时硬件强制开启，先写PR/RLR再等待
  // 会因LSI未起振使SR同步位永远置位而卡死。
  IWDG->KR = 0xCCCCu;
  IWDG->KR = 0x5555u; // 解锁PR/RLR写访问
  IWDG->PR = 4u;      // LSI约40kHz / 64 ≈ 625Hz计数
  IWDG->RLR = 625u;   // 约1s超时
  uint32_t iwdg_sync_start = HAL_GetTick();
  while ((IWDG->SR & 3u) != 0u && HAL_GetTick() - iwdg_sync_start < 50u)
    ; // 等待分频与重载值同步生效；超时则放弃看门狗也不卡死
  iwdg_feed();
  uint32_t ramp_start = HAL_GetTick();
  while (motor_fault == MOTOR_FAULT_NONE)
  {
    iwdg_feed();
    uint32_t elapsed = HAL_GetTick() - ramp_start;
    if (elapsed >= SPEED_RAMP_MS)
    {
      motor_control_context.speed = TARGET_SPEED_RAD_S;
      break;
    }
    motor_control_context.speed = TARGET_SPEED_RAD_S * ((float)elapsed / SPEED_RAMP_MS);
    HAL_Delay(10);
  }
  uint32_t low_speed_since = HAL_GetTick();

  // 理论讲解以及FOC代码逐步实现讲解请前往查看：https://blog.csdn.net/qq570437459/category_12672491.html
  while (1)
  {
    iwdg_feed();
    if (motor_fault == MOTOR_FAULT_NONE)
    {
      uint32_t now = HAL_GetTick();
      float target = motor_control_context.speed;
      float speed_in_target_direction = motor_speed * (target >= 0.0f ? 1.0f : -1.0f);
      if (speed_in_target_direction < 0.5f * fabsf(target))
      {
        if (now - low_speed_since >= STALL_TIMEOUT_MS)
          motor_stop(MOTOR_FAULT_STALL);
      }
      else
      {
        low_speed_since = now;
      }
    }
    // 使用整数避免newlib-nano默认未开启浮点printf时丢失诊断值。
    printf("speed_mrad_s=%ld iq_mA=%ld id_mA=%ld fault=%lu frames=%lu\n",
           (long)(motor_speed * 1000.0f), (long)(motor_i_q * 1000.0f),
           (long)(motor_i_d * 1000.0f), (unsigned long)motor_fault,
           (unsigned long)mt6701_valid_frame_count);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0u)
    TIM1->BDTR &= ~TIM_BDTR_MOE;
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
