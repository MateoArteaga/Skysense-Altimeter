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
//#include "font.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver_bmp390.h"
#include <stdarg.h>
#include <math.h>
#include "screen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define PRESSURE_0_FT   101325  // Pressure at 0 feet
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
//uint8_t* buffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

double power(double base, int exponent);
uint8_t bmp390_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t bmp390_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t write_eeprom(uint8_t address, uint8_t value);
uint8_t read_eeprom(uint8_t address, uint8_t* data_buffer);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  screen_init();
  screen_clear();
  screen_display();
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, 0);
  // Create a BMP390 handle and initialize it
  bmp390_handle_t *bmp390_handle = malloc(sizeof(bmp390_handle_t));
  if (bmp390_handle == NULL) {
      // Handle memory allocation error
      Error_Handler();
  }
  	bmp390_handle->iic_spi = BMP390_INTERFACE_IIC;
	bmp390_handle->delay_ms = HAL_Delay;
	bmp390_handle->debug_print = NULL;
	bmp390_handle->iic_addr = 0x76;
	bmp390_handle->iic_read = bmp390_interface_iic_read;
	bmp390_handle->iic_write = bmp390_interface_iic_write;
	bmp390_init(bmp390_handle);
    /*
	screen_clear();
	screen_drawstring(25, 3, "Initializing");
	screen_display();
	HAL_Delay(500);
	screen_clear();
	screen_drawstring(25, 3, "Initializing.");
	screen_display();
	HAL_Delay(500);
	screen_clear();
	screen_drawstring(25, 3, "Initializing..");
	screen_display();
	HAL_Delay(500);
	screen_clear();
	screen_drawstring(25, 3, "Initializing...");
	screen_display();
	HAL_Delay(500);
	*/
	char buffer[17];
	float temp;
	float pressure;
	uint32_t temp_raw;
	uint32_t pressure_raw;
	float altitudeFeet;
	float start_pressure;
	float ground_temp;
	float x;
	uint8_t button_presses;
	uint8_t button_pressed = 0;
	uint8_t button_held = 0;
	uint8_t button_holding = 0;
	char msg_buffer[30];
	read_eeprom(0x00, &button_presses);
	bmp390_read_temperature_pressure(bmp390_handle, &temp_raw, &ground_temp, &pressure_raw, &start_pressure);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);
	  screen_clear();
	  bmp390_read_temperature_pressure(bmp390_handle, &temp_raw, &temp, &pressure_raw, &pressure);

	  if (pressure > start_pressure)
	  {
		  pressure = start_pressure;
	  }
	  x = (pressure / start_pressure);
	  altitudeFeet = (1 - pow(x, (1.0/5.255))) * 44330 * 3.28084;
	  sprintf(buffer, "%d", (int)altitudeFeet);
	  screen_drawbignums(30, 1, buffer);
	  screen_display();

	  //start eeprom
	  if( HAL_GPIO_ReadPin(GPIOA, BUTTON_1_Pin) == 0)
	  {
		  button_pressed = 1;
		  button_holding ++;

	  }
	  if(button_holding >= 200){
		  button_holding = 200;
		  button_held = 1;
		  button_pressed = 0;
	  }
	  if ((HAL_GPIO_ReadPin(GPIOA, BUTTON_1_Pin) == 1) && (button_held == 1))
	  {

		  write_eeprom(0x00, 0);
		  read_eeprom(0x00, &button_presses);
		  sprintf(msg_buffer, "presses: %u", button_presses); //display read data in decimal
		  screen_clear();
		  screen_drawstring(25, 3, msg_buffer);
		  screen_display();
		  HAL_Delay(1000);
		  button_pressed = 0;
		  button_holding = 0;
		  button_held = 0;
	  }
	  if ((HAL_GPIO_ReadPin(GPIOA, BUTTON_1_Pin) == 1) && (button_pressed == 1))
	  {
		  write_eeprom(0x00, button_presses+1);
		  read_eeprom(0x00, &button_presses);
		  sprintf(msg_buffer, "presses: %u", button_presses); //display read data in decimal
		  screen_clear();
		  screen_drawstring(25, 3, msg_buffer);
		  screen_display();
		  HAL_Delay(1000);
		  button_pressed = 0;
		  button_holding = 0;
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x40000A0B;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_EN_Pin|Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_1_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_EN_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = ADC_EN_Pin|Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t write_eeprom(uint8_t address, uint8_t value){
	  uint8_t write_buffer[2] = {address, value}; //prepare write buffer
	  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0xA0, write_buffer, 2, 1000) != HAL_OK) //write data
	  {
		  screen_clear();
		  screen_drawstring(25, 3, "failure i2c transmit");
		  screen_display();
		  return 1;

	  }
	  HAL_Delay(5);
	  return 0;
}
uint8_t read_eeprom(uint8_t address, uint8_t* data_buffer){
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0xA0, &address, 1, 1000) != HAL_OK) //dummy write
    {
        screen_clear();
        screen_drawstring(25, 3, "Failure I2C dummy write");
        screen_display();
        return 1;
    }
    HAL_Delay(5);
	  if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(0xA0 | 1), data_buffer, 1, 1000) != HAL_OK) //read 1 bytes 8 bits
	  {
		  screen_clear();
		  screen_drawstring(25, 3, "failure i2c receive");
		  screen_display();
		  return 1;
	  }
	  return 0;
}

double power(double base, int exponent) {
	double result = 1.0;
	for (int i = 0; i < exponent; i++)
	{
		result *= base;
	}
	return result;
}

uint8_t bmp390_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	uint8_t tx_buffer[1] = {reg};

	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1), tx_buffer,1 , 1000) != HAL_OK)
	{
		return 1; //I2C write failed
	}

	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr << 1 | 1), buf, len, 1000) != HAL_OK)
	{
		return 1; //I2C read failed
	}
    return 0; //success
}
uint8_t bmp390_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	 uint8_t tx_buffer[len + 1];
	 tx_buffer[0] = reg;
	 memcpy(&tx_buffer[1], buf, len);

	 if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1), tx_buffer, len + 1, 1000) != HAL_OK) {
	    return 1; // I2C write failed
	 }
    return 0; //Success
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
