/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "math.h"
#include "bno055.h"
#include "stm32l4xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HAL_USART_MODULE_ENABLED
#define I2C_WA 0x50
#define I2C_RA 0x51
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

// ------------------ SEND CODE ------------------------
  uint8_t writebuf[2] = {0x3D, 0b00001100}; // operation mode to ndof fusion mode
  HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, writebuf, 2, HAL_MAX_DELAY);

  uint8_t calib_check[1] = {0};
  uint8_t calib_write[1] = {0x35};    // read calibration status
  HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &calib_write[0], 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &calib_check[0], 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &calib_write[0], 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &calib_check[0], 1, HAL_MAX_DELAY);
  while (calib_check[0] != 0b11111111) {
      HAL_UART_Transmit(&huart1, (uint8_t*) calib_check, 1, HAL_MAX_DELAY);    // send calib_check register
      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &calib_write[0], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &calib_check[0], 1, HAL_MAX_DELAY);

      HAL_Delay(1000);
  }

  HAL_Delay(10000);

  writebuf[0] = 0x3B;
  writebuf[1] = 0b00000000;    // selected units to m/s^2 and output format in Windows
  HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, writebuf, 2, HAL_MAX_DELAY);

  uint8_t read_buf[6] = {0};
  uint8_t addr[6];

  for (uint8_t i=0; i<6; ++i)
        addr[i] = 0x28 + i;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &addr[0], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &read_buf[0], 1, HAL_MAX_DELAY);

      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &addr[1], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &read_buf[1], 1, HAL_MAX_DELAY);

      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &addr[2], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &read_buf[2], 1, HAL_MAX_DELAY);

      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &addr[3], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &read_buf[3], 1, HAL_MAX_DELAY);

      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &addr[4], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &read_buf[4], 1, HAL_MAX_DELAY);

      HAL_I2C_Master_Transmit(&hi2c1, I2C_WA, &addr[5], 1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive(&hi2c1, I2C_RA, &read_buf[5], 1, HAL_MAX_DELAY);

      if (HAL_UART_Init(&huart1) != HAL_OK) {
          printf("failed");
          return -1;
      }

      HAL_UART_Transmit(&huart1, read_buf, sizeof(read_buf), HAL_MAX_DELAY);    // send x, y, z data (6 1-byte chunks)

      HAL_Delay(500);
  }

  /* USER CODE END 3 */
}


// ------------------ SEND CODE ------------------------


// ------------------RECEIVE CODE -----------------------
//  uint8_t read_buf[6];
//  int16_t x, y, z;
//  int32_t accel;
//  double accel_data[3];
//  char concussion_message[] = "threshold exceeded. check for concussion";
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//
//      if (HAL_UART_Init(&huart1) != HAL_OK) {
//          printf("failed");
//          return -1;
//      }
//
//      HAL_UART_Receive(&huart1, read_buf, sizeof(read_buf), 100);
//
//      x = (read_buf[1]<<8) + read_buf[0];
//      y = (read_buf[3]<<8) + read_buf[2];
//      z = (read_buf[5]<<8) + read_buf[4];    // x, y, z units currently in m/s^2
//
//      accel_data[0] = x;
//      accel_data[1] = y;
//      accel_data[2] = z;
//
//      for (int i = 0; i < 3; i++){
//          accel_data[i] *= accel_data[i] * 0.101972;    // convert m/s^2 to g
//      }
//
//      accel = x*x + y*y + z*z;
//      accel = sqrt(accel);
//
//      if (accel >= 58) {    // check if threshold is met/exceeded; using 58 to account for rounding error
//          HAL_UART_Transmit(&huart1, (uint8_t*) concussion_message, sizeof(concussion_message), HAL_MAX_DELAY);
//          // activate buzzer
//          // make screen turn red
//      }
//
//      HAL_Delay(500);
//  }
//
//  /* USER CODE END 3 */
//}

// ------------------RECEIVE CODE -----------------------


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
