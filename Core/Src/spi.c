/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA1_Channel2;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx);

    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{

  if (spiHandle->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(SPI1_IRQn);

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <stdio.h>
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "arm_math.h"
#include <stdint.h>
uint8_t calculate_crc(uint32_t data)
{
  uint8_t crc = 0;
  uint32_t polynomial = 0x43; // (X^6 + X + 1)的二进制值

  // (D[13:0] 和 Mg[3:0])
  for (int i = 17; i >= 0; i--)
  {
    uint8_t bit = (data >> i) & 1;
    crc <<= 1;
    if ((crc >> 6) ^ bit)
      crc ^= polynomial;
    crc &= 0x3F;
  }
  return crc;
}

uint8_t mt6701_rx_data[3];                              // 编码器返回的角度帧（RX）
const uint8_t mt6701_tx_data[3] = {0xFF, 0xFF, 0xFF};   // 固定“读角度”命令（TX），不再复用RX自喂
volatile uint32_t mt6701_frame_count = 0;               // 每完成一帧+1（可选：调试/心跳）
volatile uint32_t mt6701_valid_frame_count = 0;
volatile uint32_t mt6701_sample_tick = 0;
volatile uint32_t mt6701_last_valid_tick = 0;
volatile uint32_t mt6701_last_valid_cycle = 0;
static uint32_t mt6701_active_tick = 0;
static uint32_t mt6701_active_cycle = 0;

uint8_t mt6701_angle_is_fresh(void)
{
  return mt6701_valid_frame_count != 0u &&
         (DWT->CYCCNT - mt6701_last_valid_cycle) <= MT6701_MAX_AGE_CYCLES;
}

// 启动“一帧”读取（由定时器确定性驱动，不靠回调自续）
void mt6701_start_read(void)
{
  mt6701_sample_tick++;
  if (hspi1.State != HAL_SPI_STATE_READY)
    return; // 上一帧还没结束就跳过，避免重入
  mt6701_active_tick = mt6701_sample_tick;
  mt6701_active_cycle = DWT->CYCCNT;
  HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, mt6701_tx_data, mt6701_rx_data, 3) != HAL_OK)
    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
    mt6701_frame_count++;
    int angle_raw = (mt6701_rx_data[1] >> 2) | (mt6701_rx_data[0] << 6);

    // crc校验，不通过则本次角度数据作废（下一帧由定时器再触发）
    uint8_t crc_raw = mt6701_rx_data[2] & ((1 << 6) - 1);
    uint32_t crc_data = (mt6701_rx_data[0] << 16 | mt6701_rx_data[1] << 8 | mt6701_rx_data[2]) >> 6;
    if (calculate_crc(crc_data) == crc_raw)
    {
      encoder_raw = (uint16_t)angle_raw;
      encoder_angle = 2 * PI * angle_raw / (1 << 14);

      static float encoder_angle_last = 0;
      /****encoder_angle_last默认值是0，不能用于计算，要先赋值一次****/
      static int once = 1;
      if (once)
      {
        once = !once;
        encoder_angle_last = encoder_angle;
      }
      /*************/
      float _encoder_angle = encoder_angle;
      // 角度差值，用于累计多圈逻辑角度
      float diff_angle = cycle_diff(_encoder_angle - encoder_angle_last, 2 * PI);
      encoder_angle_last = _encoder_angle;
      // 实现周期操作，将motor_logic_angle转到周期内
      motor_logic_angle = cycle_diff(motor_logic_angle + diff_angle, position_cycle);
      mt6701_last_valid_tick = mt6701_active_tick;
      mt6701_last_valid_cycle = mt6701_active_cycle;
      mt6701_valid_frame_count++;
    }
    return;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
}

/* USER CODE END 1 */
