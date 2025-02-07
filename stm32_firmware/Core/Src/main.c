/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdbool.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

#define UART2RxBufferSize 200
uint8_t UART2RxBuffer[UART2RxBufferSize];

// Position data frame specific variables
uint8_t DataFramaBuffer[UART2RxBufferSize];
volatile bool DataFrameReceived = false;
volatile uint16_t DataFrameLen = 0;

uint8_t DataFrameStartBit = 0x80;
uint8_t DataFrameEndBit = 0x81;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    // HAL_UART_RxEventTypeTypeDef RxEventType = HAL_UARTEx_GetRxEventType(huart);

    // Check if the data frame is received
    if (UART2RxBuffer[0] == DataFrameStartBit && UART2RxBuffer[Size - 1] == DataFrameEndBit)
    {
      memcpy(DataFramaBuffer, UART2RxBuffer, Size);
      DataFrameLen = Size;
      DataFrameReceived = true;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2RxBuffer, UART2RxBufferSize);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // Disable Half Transfer interrupt
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// This magic CRC32 function is necessary to match the CRC32 calculation of ZLIB
// from https://techoverflow.net/2024/08/13/how-to-configure-stm32h7-hardware-crc-to-match-zlib-crc32-in-python/
uint32_t CalculateCRC32(uint8_t *data, size_t length)
{
  uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t *)data, length);
  return crc_result ^ 0xFFFFFFFF;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  // Start UART DMA communication
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2RxBuffer, UART2RxBufferSize);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // Disable Half Transfer interrupt

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (DataFrameReceived)
    {
      DataFrameReceived = false;

      // Check start and end bits
      if (DataFramaBuffer[0] != DataFrameStartBit || DataFramaBuffer[DataFrameLen - 1] != DataFrameEndBit)
      {
        // Invalid data frame
        continue;
      }

      // Process the payload
      uint16_t payload_len = (uint16_t)(DataFramaBuffer[1]) << 8 | DataFramaBuffer[2];

      // Check the payload length
      // payload length = data frame length - 8 (start bit, end bit, 2 bytes for payload length, 4 bytes for CRC)
      if ((payload_len != DataFrameLen - 8) || (payload_len == 0))
      {
        // Invalid data frame
        continue;
      }

      const uint8_t *payload = &DataFramaBuffer[3];

      // Check CRC32 of payload
      uint32_t crc = (uint32_t)(DataFramaBuffer[DataFrameLen - 5]) << 24 | (uint32_t)(DataFramaBuffer[DataFrameLen - 4]) << 16 | (uint32_t)(DataFramaBuffer[DataFrameLen - 3]) << 8 | (uint32_t)(DataFramaBuffer[DataFrameLen - 2]);
      uint32_t crc_calculated = CalculateCRC32(payload, payload_len);

      if (crc != crc_calculated)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CRC Error\n", strlen("CRC Error\n"), 1000);
        // Invalid data frame
        continue;
      }

      int field_count = 0;
      size_t i = 0;
      while (i < payload_len)
      {
        if (i + 1 > payload_len)
          break; // exit if there is no enough data
        uint8_t name_len = payload[i];
        i += 1;
        // check for next data (name_len + 4 bytes for double)
        if (i + name_len + 4 > payload_len)
          break;
        i += name_len;
        i += 4;
        field_count++; // incerase the field count if everything is ok with previous data
      }

      // Check the field count
      if (field_count == 0)
      {
        // Invalid data frame
        continue;
      }

      char **names = malloc(field_count * sizeof(char *));
      float *values = malloc(field_count * sizeof(float));

      i = 0;
      int index = 0;
      while (i < payload_len)
      {
        uint8_t name_len = payload[i];
        i += 1;

        char *name = malloc((name_len + 1) * sizeof(char));
        memcpy(name, &payload[i], name_len);
        name[name_len] = '\0';

        i += name_len;

        uint32_t value = (uint32_t)(payload[i]) << 24 | (uint32_t)(payload[i + 1]) << 16 | (uint32_t)(payload[i + 2]) << 8 | (uint32_t)(payload[i + 3]);

        i += 4;

        names[index] = name;
        values[index] = *(float *)(&value); // Retrieve the float value from uint32_t bits

        index++;
      }

      // Send positions to UART
      for (int i = 0; i < field_count; i++)
      {
        char buffer[100];
        sprintf(buffer, "%s: %f\n", names[i], values[i]);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
      }

      // Free the memory
      for (int i = 0; i < field_count; i++)
      {
        free(names[i]);
      }

      free(names);
      free(values);
    }
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

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
