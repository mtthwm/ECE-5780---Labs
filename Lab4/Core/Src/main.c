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

/* USER CODE END 0 */

#define RX_BUFF_SIZE 2

static char rx_chars[RX_BUFF_SIZE];
static int rx_i = 0;

char usart_read_char () {
	int wait = 1;
	while (wait) {
		if ((USART3->ISR & USART_ISR_RXNE_Msk)) {
			wait = 0;
		}
	} // Wait until the register is empty for transmission
	return USART3->RDR;
}

void usart_read_str (char* result, int len) {
	for (int i = 0; i < len; i++) {
		result[i] = usart_read_char();
	} 
}

void usart_transmit_char (char c) {
	int wait = 1;
	while (wait) {
		if ((USART3->ISR & USART_ISR_TXE_Msk)) {
			wait = 0;
		}
	} // Wait until the register is empty for transmission

	USART3->TDR = c;
}

void usart_transmit_str (char* s) {
	int i = 0;
	do {
		usart_transmit_char(s[i]);
		i++;
	} while (s[i] != '\0');
	
	usart_transmit_char('\0');
}

void USART3_4_IRQHandler () {
	char received = USART3->RDR;
	if (rx_i >= RX_BUFF_SIZE || rx_i < 0) {
		return;
	}
	rx_chars[rx_i] = received;
	rx_i++;
}

void handle_light_commands () {
	if (rx_i == RX_BUFF_SIZE) {
			uint8_t has_error = 0;
			uint32_t led_to_modify;
			if (rx_chars[0] == 'r') {
					led_to_modify = GPIO_ODR_6;
			} else if (rx_chars[0] == 'g') {
					led_to_modify = GPIO_ODR_9;
			} else if (rx_chars[0] == 'b') {
					led_to_modify = GPIO_ODR_7;
			} else if (rx_chars[0] == 'o') {
					led_to_modify = GPIO_ODR_8;
			} else {
					has_error = 1;
			}
			
			if (!has_error) {
					if (rx_chars[1] == '0') {
							GPIOC->ODR &= ~led_to_modify;
					} else if (rx_chars[1] == '1') {
							GPIOC->ODR |= led_to_modify;
					} else if (rx_chars[1] == '2') {
							GPIOC->ODR ^= led_to_modify;
					} else {
							has_error = 1;
					}
			}

			if (has_error) {
					usart_transmit_str("INVALID COMMAND\n");
			}
			
			rx_i = 0;
			usart_transmit_str("CMD?\n");
	}
}

void config_red () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk);
}
void config_blue () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER7_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_Msk);
}
void config_orange () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER8_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk);
}
void config_green () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER9_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_Msk);
}

/**
	* @brief Configures USART with PB10=RX, PB11=TX
	*/
void config_usart (uint32_t baudrate) {
	// Set the mode of the GPIO pins to use an alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODER10_Msk);
	GPIOB->MODER &= ~(GPIO_MODER_MODER11_Msk);
	GPIOB->MODER |= (2 << GPIO_MODER_MODER10_Pos);
	GPIOB->MODER |= (2 << GPIO_MODER_MODER11_Pos);
	
	// Set GPIO Pins PB10 and PB11 to use alternate function AF4: USART3
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
	GPIOB->AFR[1] |= (GPIO_AF4_USART3 << GPIO_AFRH_AFSEL10_Pos); // TX
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
	GPIOB->AFR[1] |= (GPIO_AF4_USART3 << GPIO_AFRH_AFSEL11_Pos); // RX
	
	// Enable USART TX and RX
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Set the USART Baud Rate to 115200 bit/sec
	USART3->BRR = (HAL_RCC_GetHCLKFreq()/baudrate);
	
	// Enable RXNE Interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;
	
	// Configure interrupt handler for RXNE
	NVIC_EnableIRQ(USART3_4_IRQn);
	
	// Configure interrupt priorities
	NVIC_SetPriority(USART3_4_IRQn, 1);
	NVIC_SetPriority(SysTick_IRQn, 0);
	
	// Enable USART. Config vars become readonly!
	USART3->CR1 |= USART_CR1_UE;
}

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
	
	// Enable the clock to each peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// LED setup
	config_red();
	config_blue();
	config_orange();
	config_green();
	
	config_usart(115200);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	usart_transmit_str("CMD?\n");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(2);
						
		handle_light_commands();
		
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
