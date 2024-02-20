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

/*
A handler for our timer3 events, so that the light turns on and off.
*/
void TIM2_IRQHandler () {
	GPIOC->ODR ^= GPIO_ODR_8;
	GPIOC->ODR ^= GPIO_ODR_9;
	TIM2->SR &= ~(TIM_SR_UIF_Msk);
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
	
	// Enable the RCC clock for any peripherals that need it.
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure the modes for the onboard LEDs
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER7_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER8_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER9_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER6_1;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8_Msk);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9_Msk);
		
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_Msk);
	
	// Setting up timer 2 to 4Hz
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	TIM2->DIER |= TIM_DIER_UIE; // Enable Interrupts
	TIM2->CR1 |= TIM_CR1_CEN; // Enable Timer
	
	// Setting up timer 3 to 800Hz
	TIM3->PSC = 100;
	TIM3->ARR = 100;
	// Set channels as output
	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;
	// Configure channel 1 to PWM mode 2
	TIM3->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM3->CCMR1 |= (0x7 << TIM_CCMR1_OC1M_Pos);
	// Configure channel 2 to PWM mode 1
	TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
	TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos);
	// Enable pre-load
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
	// Enable both channels
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	// Set duty-cycle to 20%
	TIM3->CCR1 = 100;
	TIM3->CCR2 = 100;
	
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6_Msk); // Use Alternate function A0 (Channel 1 of TIM3)
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL7_Msk); // Use Alternate function A0 (Channel 2 of TIM3)
	
	TIM3->CR1 |= TIM_CR1_CEN; // Enable Timer
	
	
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// Enable one of the LEDs so we can use a bitwise XOR to toggle the lights later.
	GPIOC->ODR |= GPIO_ODR_9;

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
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(2);
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
