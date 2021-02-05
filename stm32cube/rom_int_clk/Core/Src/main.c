/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "td4_rom.h"
#include "PCA9624.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t seg7[7];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROM_BASE_ADDRESS    0
#define ROM_DEFAULT_ADDRESS 0

#define I2C_RETRIES         2
#define I2C_TIMEOUT         2

#define LED_PWM_FULL        0xff // PWM = 99.6%
#define LED_PWM_OFF         0    // PWM = 0%
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int ticked = 0;
volatile int led_ok = 0;
uint8_t cmd[32];
seg7 hex2seg7[] = { // 0xff は PWM で 99.6% 出力
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 }, // 0
  { 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00 }, // 1
  { 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff }, // 2
  { 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff }, // 3
  { 0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff }, // 4
  { 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff }, // 5
  { 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff }, // 6
  { 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00 }, // 7
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }, // 8
  { 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff }, // 9
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff }, // a
  { 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff }, // b
  { 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff }, // c
  { 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff }, // d
  { 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff }, // e
  { 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff }, // f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef ret;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* PCA9624 */
  ret = HAL_I2C_IsDeviceReady(&hi2c1, PCA9624_ADDR, I2C_RETRIES, I2C_TIMEOUT);
  if (ret == HAL_OK) // PCA9624 から ACK を受信
  {
    cmd[0] = MODE1; // モードレジスタ
    cmd[1] = 0x0;   // SLEEP = 0 で低消費電力モードを解除して通常動作へ
    ret = HAL_I2C_Master_Transmit(&hi2c1, PCA9624_ADDR, cmd, 2, HAL_MAX_DELAY);
    if (ret == HAL_OK)
    {
      cmd[0] = LEDOUT0 + 0x80; // LEDOUT0 オートインクリメント
      cmd[1] = 0xaa;           // LED3,2,1,0 各々の2ビットに 0b10 で PWM点灯を指定
      cmd[2] = 0xaa;           // LED7,6,5,4 各々の2ビットに 0b10 で PWM点灯を指定
      ret = HAL_I2C_Master_Transmit(&hi2c1, PCA9624_ADDR, cmd, 3, HAL_MAX_DELAY);
      if (ret == HAL_OK)
      {
        led_ok = 1; // PCA9624 の初期化完了
      }
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static int address = ROM_DEFAULT_ADDRESS;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (ticked)
    {
      /* Read demand address for ROM */
      address = ROM_BASE_ADDRESS;
      address += (HAL_GPIO_ReadPin(A0_GPIO_Port, A0_Pin) << 0);
      address += (HAL_GPIO_ReadPin(A1_GPIO_Port, A1_Pin) << 1);
      address += (HAL_GPIO_ReadPin(A2_GPIO_Port, A2_Pin) << 2);
      address += (HAL_GPIO_ReadPin(A3_GPIO_Port, A3_Pin) << 3);
      if (led_ok) // PCA9624
      {
        cmd[0] = PWM0 + 0x80;                  // PWM0 オートインクリメント
        memcpy(&cmd[1], hex2seg7[address], 7); // 7 セグメントの PWM 値を直接設定
        cmd[8] = LED_PWM_FULL;                 // DP は一旦点灯
        ret = HAL_I2C_Master_Transmit(&hi2c1, PCA9624_ADDR, cmd, 9, HAL_MAX_DELAY);
        if (ret != HAL_OK)
        {
          led_ok = 0; // PCA9624 でエラー発生
          // TODO: I2C のリカバリ処理をここで
        }
      }
      ticked = 0;
    }

    /* Update bits of ROM */
    HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, rom[address] & 1 << 0);
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, rom[address] & 1 << 1);
    HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, rom[address] & 1 << 2);
    HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, rom[address] & 1 << 3);
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, rom[address] & 1 << 4);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, rom[address] & 1 << 5);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, rom[address] & 1 << 6);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, rom[address] & 1 << 7);
    if (led_ok) // PCA9624
    {
      HAL_Delay(50);        // ちょっとだけ待ってから
      cmd[0] = PWM7;        // DPを
      cmd[1] = LED_PWM_OFF; // 消灯
      ret = HAL_I2C_Master_Transmit(&hi2c1, PCA9624_ADDR, cmd, 2, HAL_MAX_DELAY);
      if (ret != HAL_OK)
      {
        led_ok = 0; // PCA9624 でエラー発生
        // TODO: I2C のリカバリ処理をここで
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000001;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D7_Pin|D3_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D2_Pin|D1_Pin|D0_Pin|D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D7_Pin D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D7_Pin|D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A3_Pin */
  GPIO_InitStruct.Pin = A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D1_Pin D0_Pin D6_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D1_Pin|D0_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : D5_Pin */
  GPIO_InitStruct.Pin = D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_Pin */
  GPIO_InitStruct.Pin = CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == CLK_Pin)
  {
    ticked = 1;
  }
}

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
