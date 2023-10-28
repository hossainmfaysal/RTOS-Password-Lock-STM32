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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "liquidcrystal_i2c.h"
#include "string.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
GPIO_TypeDef* R1_PORT = GPIOB;
GPIO_TypeDef* R2_PORT = GPIOB;
GPIO_TypeDef* R3_PORT = GPIOA;
GPIO_TypeDef* R4_PORT = GPIOA;
GPIO_TypeDef* C1_PORT = GPIOC;
GPIO_TypeDef* C2_PORT = GPIOA;
GPIO_TypeDef* C3_PORT = GPIOA;
GPIO_TypeDef* C4_PORT = GPIOB;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_PIN GPIO_PIN_8
#define R2_PIN GPIO_PIN_9
#define R3_PIN GPIO_PIN_6
#define R4_PIN GPIO_PIN_7
#define C1_PIN GPIO_PIN_7
#define C2_PIN GPIO_PIN_9
#define C3_PIN GPIO_PIN_8
#define C4_PIN GPIO_PIN_10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId Task1Handle;
osThreadId Task2Handle;
/* USER CODE BEGIN PV */
char key;

char enteredPin[5];

uint8_t entered = 0;

const char setPin[4] = {'1', '2', '3', '4'};

xQueueHandle keypadQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void Task1_Init(void const * argument);
void Task2_Init(void const * argument);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char read_keypad(void)
{
	char keys[4][4] = {{'1', '2', '3', 'A'},
                      {'4', '5', '6', 'B'},
                      {'7', '8', '9', 'C'},
                      {'*', '0', '#', 'D'}};

    for (int i = 0; i < 4; i++)
    {
    	switch (i)
    	{
    		case 0:
                HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);
                break;

            case 1:
                HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);
                break;

            case 2:
                HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);
                break;

            case 3:
                HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_RESET);
                break;
        }

        if (HAL_GPIO_ReadPin(R1_PORT, R1_PIN) == GPIO_PIN_RESET)
            return keys[0][i];

        if (HAL_GPIO_ReadPin(R2_PORT, R2_PIN) == GPIO_PIN_RESET)
            return keys[1][i];

        if (HAL_GPIO_ReadPin(R3_PORT, R3_PIN) == GPIO_PIN_RESET)
            return keys[2][i];

        if (HAL_GPIO_ReadPin(R4_PORT, R4_PIN) == GPIO_PIN_RESET)
            return keys[3][i];
    }

    return '\0';
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HD44780_Init(2);
  HD44780_Clear();
  HD44780_SetCursor(4,0);
  HD44780_PrintStr("WELCOME");
  HAL_Delay(2000);
  HD44780_Clear();

  keypadQueue = xQueueCreate(1, sizeof(char));
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task1 */
  osThreadDef(Task1, Task1_Init, osPriorityNormal, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, Task2_Init, osPriorityNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|C3_Pin|C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 C3_Pin C2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|C3_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C4_Pin */
  GPIO_InitStruct.Pin = C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : C1_Pin */
  GPIO_InitStruct.Pin = C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
enum State
{
	L_HOME,
  	U_HOME,
  	PIN,
  	DIS
};

enum State currentState = L_HOME;

uint8_t pinAtt = 0;
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1_Init */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task1_Init */
void Task1_Init(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint16_t targetCCR = htim2.Instance->CCR1;

	enteredPin[0] = '\0';

	bool isButtonPressed = false;
  /* Infinite loop */
  for(;;)
  {
	key = read_keypad();

	switch (key)
	{
		case 'A':
		{
			currentState = L_HOME;

			break;
		}

		case 'B':
		{
			if (currentState != U_HOME)
			{
				currentState = PIN;
			}

			break;
		}
	}

	while (htim2.Instance->CCR1 != targetCCR)
	{
		if (htim2.Instance->CCR1 < targetCCR)
		{
		  htim2.Instance->CCR1 += 1;
		}
		else
		{
		  htim2.Instance->CCR1 -= 1;
		}

		osDelay(5);
	}

	 switch(currentState)
	 {
		 case L_HOME:
		 {
		  targetCCR = 25;

		  break;
		 }

		 case U_HOME:
		 {
		  targetCCR = 125;
		  pinAtt = 0;

		  break;
		 }

		 case PIN:
		 {
			if (key != '\0')
			{
			 if (!isButtonPressed)
			 {
				if (key >= '0' && key <= '9' && entered < 4)
				{
				 strncat(enteredPin, &key, 1);
				 entered++;
				}
				else if (key == 'C' && entered > 0)
				{
					enteredPin[entered - 1] = '\0';
					entered--;
				}

				if (entered == 4)
				{

				 if (strcmp(enteredPin, setPin) == 0)
				 {
					 currentState = U_HOME;
					 pinAtt = 0;
				 }
				 else
				 {
					 currentState = L_HOME;
					 pinAtt++;
				 }

				 entered = 0;
				 enteredPin[0] = '\0';
				}

				isButtonPressed = true;
			 }
			 else
			 {
				 isButtonPressed = false;
			 }

		  targetCCR = 25;
		  if (pinAtt == 3)
		  {
			currentState = DIS;
		  }

		  break;
		 }

		 case DIS:
		 {
		 	targetCCR = 25;

		 	break;
		 }
		}
	 }
	 osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2_Init */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_Init */
void Task2_Init(void const * argument) // for displaying
{
  /* USER CODE BEGIN Task2_Init */
  /* Infinite loop */
  for(;;)
  {
	switch(currentState)
	{
		case L_HOME:
		{
		  HD44780_Clear();
		  HD44780_SetCursor(0, 0);
		  HD44780_PrintStr("Status: LOCKED");

		  osDelay(100);

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

		  break;
		}

		case U_HOME:
		{
		  HD44780_Clear();
		  HD44780_SetCursor(0, 0);
		  HD44780_PrintStr("Status: UNLOCKED");
		  osDelay(100);

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

		  break;
		}

		case PIN:
		{
			HD44780_Clear();
			HD44780_SetCursor(0,0);
			HD44780_PrintStr("Enter PIN:");

			HD44780_SetCursor(11, 0);
			for (int i = 0; i < entered; i++)
			{
			 HD44780_PrintChar('*');
			}

			if (pinAtt > 0)
			{
				HD44780_SetCursor(0,1);
				HD44780_PrintStr("Attempt:");
				HD44780_SetCursor(9,1);
				char att[2];
				sprintf(att, "%d", pinAtt);
				HD44780_PrintStr(att);
				HD44780_SetCursor(10,1);
				HD44780_PrintStr("/3");
			}
			osDelay(100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

			break;
		}

		case DIS:
		{
			HD44780_Clear();
			HD44780_SetCursor(0,0);
			HD44780_PrintStr("SYSTEM DISABLED");
			osDelay(100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

			break;
	  	}
	 }
  }
  /* USER CODE END Task2_Init */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
