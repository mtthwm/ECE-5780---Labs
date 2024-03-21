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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void config_red () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk);
	GPIOC->ODR &= ~GPIO_ODR_6;
}
void toggle_red () {
	GPIOC->ODR ^= GPIO_ODR_6;
}
void config_blue () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER7_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_Msk);
	GPIOC->ODR &= ~GPIO_ODR_7;
}
void toggle_blue () {
	GPIOC->ODR ^= GPIO_ODR_7;
}
void config_orange () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER8_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk);
	GPIOC->ODR &= ~GPIO_ODR_8;
}
void toggle_orange () {
	GPIOC->ODR ^= GPIO_ODR_8;
}
void config_green () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER9_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_Msk);
	GPIOC->ODR &= ~GPIO_ODR_9;
}
void toggle_green () {
	GPIOC->ODR ^= GPIO_ODR_9;
}

#define GYRO_ADDR 0x69
#define GYRO_WHO_AM_I_REG 0x0F
#define GYRO_CTRL_REG1 0x20
#define GYRO_OUT_X_L_REG 0x28
#define GYRO_OUT_X_H_REG 0x29
#define GYRO_OUT_Y_L_REG 0x2A
#define GYRO_OUT_Y_H_REG 0x2B

int i2c_write (char follower, int num_bytes, char* data) {
	// Prepare a write operation
	I2C2->CR2 |= (follower << (I2C_CR2_SADD_Pos + 1)); // Select follower address
	I2C2->CR2 |= (num_bytes << I2C_CR2_NBYTES_Pos); // Set the number of bytes
	I2C2->CR2 &= ~I2C_CR2_RD_WRN; // Set to write mode
	I2C2->CR2 |= I2C_CR2_START; // Start the transaction
	
	for (int i = 0; i < num_bytes; i++) {
		// Wait until we get either a NACK or a confirmation
		while (1) {
			if (I2C2->ISR & I2C_ISR_TXIS) {
				break;
			}
			if (I2C2->ISR & I2C_ISR_NACKF_Msk) {
				return 0;
			}
			HAL_Delay(1);
		};
	
		// Input data to send
		I2C2->TXDR = data[i];
	}
	
	// Wait until we receive confirmation that the data was sent.
	while (1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}
		HAL_Delay(1);
	}
	
	return 1;
}

uint32_t i2c_read (char follower, int num_bytes) {
	// Prepare a read operation
	I2C2->CR2 |= (follower << (I2C_CR2_SADD_Pos + 1)); // Select follower address
	I2C2->CR2 |= (num_bytes << I2C_CR2_NBYTES_Pos); // Set the number of bytes
	I2C2->CR2 |= I2C_CR2_RD_WRN; // Set to read mode
	I2C2->CR2 |= I2C_CR2_START; // Start the transaction
	
	// Wait until we get either a NACK or a confirmation
	while (1) {
		if (I2C2->ISR & I2C_ISR_RXNE) {
			break;
		}
		if (I2C2->ISR & I2C_ISR_NACKF_Msk) {
			return 0;
		}
		HAL_Delay(1);
	};
	
	return I2C2->RXDR;
}

int config_gyro () {
	// Write the address of the WHO_AM_I register.
	char buff[8];
	buff[0] = GYRO_WHO_AM_I_REG;
	int status = i2c_write(GYRO_ADDR, 1, buff);
	if (!status) {
		return 0;
	}
	
	// Read the value at the address we just sent. It should be 0xD3
	uint32_t result = i2c_read(GYRO_ADDR, 1);
	if (result != 0xD3) {
		return 0;
	}
		
	// Enable the X and Y axes
	buff[0] = GYRO_CTRL_REG1;
	status = i2c_write(GYRO_ADDR, 1, buff);
	if (!status) {
		return 0;
	}
	buff[0] = 0xF;
	status = i2c_write(GYRO_ADDR, 1, buff);
	if (!status) {
		return 0;
	}
		
	return 1;
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
	
	// Enable clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Set GPIO 11 and 13 to use their alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER11_Msk);
	GPIOB->MODER &= ~(GPIO_MODER_MODER13_Msk);
	GPIOB->MODER |= (2 << GPIO_MODER_MODER11_Pos);
	GPIOB->MODER |= (2 << GPIO_MODER_MODER13_Pos);
	
	// Set pins 11 and 13 to open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	
	// Configure GPIO pins for SDA and SCL
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
	GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL11_Pos); // SDA
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL13_Msk;
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL13_Pos); // SCL
	
	// // Configure Pullup Resistors for SDA and SCL
	// GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR11_Msk);
	// GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR13_Msk);
	// GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0;
	// GPIOB->PUPDR |= GPIO_PUPDR_PUPDR13_0;
	
	// Set GPIO PB14 and PC0 to output mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER14_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk);
	GPIOB->MODER |= (1 << GPIO_MODER_MODER14_Pos);
	GPIOC->MODER |= (1 << GPIO_MODER_MODER0_Pos);
	
	// Set pins PB14 and PC0 to push-pull
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;
	
	// Set pins PB14 and PC0 to High
	GPIOB->ODR |= GPIO_ODR_14;
	GPIOC->ODR |= GPIO_ODR_0;
	
	// Set the I2C2 clock speed to 100kHz
	I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);

	// Enable the I2C2 Peripheral
	I2C2->CR1 |= I2C_CR1_PE;
	
	config_blue();
	config_orange();
	config_red();
	config_green();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	int gyro_ok = config_gyro();
	
  while (1)
  {
    /* USER CODE END WHILE */
		
		if (gyro_ok) {
			toggle_green();
		} else {
			toggle_red();
		}
		
		
		HAL_Delay(1000);

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
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
