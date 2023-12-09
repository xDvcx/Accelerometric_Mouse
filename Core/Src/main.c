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
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRINTF_TIME_DELAY 100

//Defines for MPU-9250 registers
#define MPU_9250_ADDR 0xD0
#define MPU_9250_WHO_AM_I 0x75
#define MPU_9250_ACCEL_XOUT_H 0x3B
#define MPU_9250_ACCEL_XOUT_L 0x3C
#define MPU_9250_ACCEL_YOUT_H 0x3D
#define MPU_9250_ACCEL_YOUT_L 0x3E
#define MPU_9250_ACCEL_ZOUT_H 0x3F
#define MPU_9250_ACCEL_ZOUT_L 0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

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
int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

uint8_t mpu_9250_read_reg(uint8_t reg)
{
	uint8_t value;
	HAL_I2C_Mem_Read(&hi2c1, MPU_9250_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

	return value;
}

void mpu_9250_write_reg(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU_9250_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //Counter setup
  uint16_t counter = 0;

  //Accelerometer partial values
  uint8_t Mpu_9250_Accel_Xout_H_Value = 0;
  uint8_t Mpu_9250_Accel_Xout_L_Value = 0;
  uint8_t Mpu_9250_Accel_Yout_H_Value = 0;
  uint8_t Mpu_9250_Accel_Yout_L_Value = 0;
  uint8_t Mpu_9250_Accel_Zout_H_Value = 0;
  uint8_t Mpu_9250_Accel_Zout_L_Value = 0;

  //Accelerometer full values
  int16_t Mpu_9250_Accel_Xout_Value = 0;
  int16_t Mpu_9250_Accel_Yout_Value = 0;
  int16_t Mpu_9250_Accel_Zout_Value = 0;

  //Baud rate setup for UART BT401
  HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CT05\r\n", 11, HAL_MAX_DELAY);

  //Startup connection verification
  uint8_t who_am_i = mpu_9250_read_reg(MPU_9250_WHO_AM_I);

  if (who_am_i != 0) {
	  printf("Found: MPU-9250\n");
  }
  else
  {
	  printf("Error: (0x%02X)\n", who_am_i);
  }

  //Reset accelerometer registers
  mpu_9250_write_reg(0x77, 0x00);
  mpu_9250_write_reg(0x78, 0x00);
  mpu_9250_write_reg(0x7A, 0x00);
  mpu_9250_write_reg(0x7B, 0x00);
  mpu_9250_write_reg(0x7D, 0x00);
  mpu_9250_write_reg(0x7E, 0x00);

  //Accelerometer configuration change
  uint8_t Configuration_Data_MPU_9250 = mpu_9250_read_reg(0x1C);
  Configuration_Data_MPU_9250 |= (0 << 3);
  Configuration_Data_MPU_9250 |= (0 << 4);
  Configuration_Data_MPU_9250 |= (1 << 5);
  Configuration_Data_MPU_9250 |= (1 << 6);
  Configuration_Data_MPU_9250 |= (1 << 7);
  mpu_9250_write_reg(0x1C, Configuration_Data_MPU_9250);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Accelerometer data reading
	  Mpu_9250_Accel_Xout_H_Value = mpu_9250_read_reg(MPU_9250_ACCEL_XOUT_H);
	  Mpu_9250_Accel_Xout_L_Value = mpu_9250_read_reg(MPU_9250_ACCEL_XOUT_L);
	  Mpu_9250_Accel_Yout_H_Value = mpu_9250_read_reg(MPU_9250_ACCEL_YOUT_H);
	  Mpu_9250_Accel_Yout_L_Value = mpu_9250_read_reg(MPU_9250_ACCEL_YOUT_L);
	  Mpu_9250_Accel_Zout_H_Value = mpu_9250_read_reg(MPU_9250_ACCEL_ZOUT_H);
	  Mpu_9250_Accel_Zout_L_Value = mpu_9250_read_reg(MPU_9250_ACCEL_ZOUT_L);

	  //Accelerometer full values completing
	  Mpu_9250_Accel_Xout_Value = ((Mpu_9250_Accel_Xout_H_Value << 8)|Mpu_9250_Accel_Xout_L_Value);
	  Mpu_9250_Accel_Yout_Value = ((Mpu_9250_Accel_Yout_H_Value << 8)|Mpu_9250_Accel_Yout_L_Value);
	  Mpu_9250_Accel_Zout_Value = ((Mpu_9250_Accel_Zout_H_Value << 8)|Mpu_9250_Accel_Zout_L_Value);

	  //Accelerometer data sending by BT401
	  printf("%d. MPU-9250 accelerometer reading:\n", counter);
	  printf("	Mpu_9250_Accel_Xout_Value = %d\n", Mpu_9250_Accel_Xout_Value);
	  printf("	Mpu_9250_Accel_Yout_Value = %d\n", Mpu_9250_Accel_Yout_Value);
	  printf("	Mpu_9250_Accel_Zout_Value = %d\n\n", Mpu_9250_Accel_Zout_Value);

	  //Delay and counter update
	  HAL_Delay(PRINTF_TIME_DELAY);
	  counter++;
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
