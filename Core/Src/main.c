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
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


#define left_limit_offset = 500;
#define right_limit_offset = 350;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
TIM_HandleTypeDef htim4;
/* USER CODE END PFP */

static void task1_handler(void *parameters);
static void task2_handler(void *parameters);
static void task3_handler(void *parameters);
static void task4_handler(void *parameters);
static void task5_handler(void *parameters);


xSemaphoreHandle xMutex1;


//--[Global variables]
volatile unsigned int stepper_counter    = 0;
volatile unsigned int left_limit_status  = 0;
volatile unsigned int left_limit_value   = 0;
volatile unsigned int right_limit_status = 0;
volatile unsigned int right_limit_value  = 0;
volatile unsigned int stepper_status = 0;
volatile unsigned int process_step = 0;
volatile unsigned int process_counter = 0;
volatile unsigned int pump_status = 0;

int main(void)
{

  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM4_Init();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  TaskHandle_t task1_handle;//calibrations
  TaskHandle_t task2_handle;//process: stepper motor calibration
  TaskHandle_t task3_handle;//process: server motor
  TaskHandle_t task4_handle;//process: stepper motor
  TaskHandle_t task5_handle;//process: pump


    BaseType_t status1;
  //  BaseType_t status2;



   status1= xTaskCreate(task1_handler, "Task-1", configMINIMAL_STACK_SIZE, NULL,4, &task1_handle);
    configASSERT(status1 == pdPASS);
   status1= xTaskCreate(task2_handler, "Task-2", configMINIMAL_STACK_SIZE, NULL,4, &task2_handle);
    configASSERT(status1 == pdPASS);
   status1= xTaskCreate(task3_handler, "Task-3", configMINIMAL_STACK_SIZE, NULL,4, &task3_handle);
    configASSERT(status1 == pdPASS);
   status1= xTaskCreate(task4_handler, "Task-4", configMINIMAL_STACK_SIZE, NULL,4, &task4_handle);
     configASSERT(status1 == pdPASS);
   status1= xTaskCreate(task5_handler, "Task-5", configMINIMAL_STACK_SIZE, NULL,4, &task5_handle);
     configASSERT(status1 == pdPASS);


     xMutex1 = xSemaphoreCreateMutex();

    vTaskStartScheduler();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

//on board green led (LD2)
static void task1_handler(void *parameters){
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}

//stepper motor control: calibration
static void task2_handler(void *parameters){
	while(1)
	{
		//--[Calibration in left direction]
		if((GPIOB->IDR & (1<<15)) && left_limit_status <  1)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			vTaskDelay(pdMS_TO_TICKS(10));

			stepper_counter++;

		}
		else if(!(GPIOB->IDR & (1<<15)) && left_limit_status < 1)
		{
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			vTaskDelay(pdMS_TO_TICKS(10));

			left_limit_value = stepper_counter;
			left_limit_status =1;

		}

		//--[Calibrate in opposite direction]
		else if ((GPIOB->IDR & (1<<14)) && left_limit_status ==  1)
		{

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);//enable stepper motor driver pin to turn right
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);//enable stepper motor driver pin to turn right
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			vTaskDelay(pdMS_TO_TICKS(10));
			stepper_counter++;
		}

		else if(!(GPIOB->IDR & (1<<14)) && left_limit_status == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			vTaskDelay(pdMS_TO_TICKS(10));

			right_limit_value = stepper_counter;
			left_limit_status =2;
			right_limit_status = 1;


		}

		//--[Calibration for end-stops is complete:update status]
		else if(!(GPIOB->IDR & (1<<14)) && left_limit_status == 2 && right_limit_status == 1)
		{
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			vTaskDelay(pdMS_TO_TICKS(10));
			stepper_status = 1; // =1 calibration complete
			process_step = 1;
			vTaskDelete(NULL);
		}

	}
}

//--[Servo motor control]
static void task3_handler(void *parameters){
	while(1)
	{
//		htim4.Instance->CCR1 =25;
//		HAL_Delay(500);
//		htim4.Instance->CCR1 =75;
//				HAL_Delay(500);
//		htim4.Instance->CCR1 =125;
//		HAL_Delay(500);
		if(process_step == 1 )
		{
			xSemaphoreTake(xMutex1, portMAX_DELAY == 1);
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			pump_status = 1;
			for( int i = 0 ; i < 190; i++)
			{
				htim4.Instance->CCR1 =i;
				HAL_Delay(50);
			}
			process_step =2 ;//servo-motor doses product
			xSemaphoreGive(xMutex1);

		}
	//	HAL_Delay(3000);

		if(process_step == 3 )
		{
			xSemaphoreTake(xMutex1, portMAX_DELAY == 1);
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			pump_status = 0;
			for( int i = 190 ; i > 0; i--)
			{
				htim4.Instance->CCR1 =i;
				HAL_Delay(5);
			}
			process_step = 1;
			//HAL_Delay(2000);
			xSemaphoreGive(xMutex1);
		}
	}
}



static void task4_handler(void *parameters){
	while(1)
	{
		//
		//calibration ends at right limit so turn left
		if(stepper_status == 1 && (GPIOB->IDR & (1<<15)) && process_step == 2  )
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //go in left direction
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			vTaskDelay(pdMS_TO_TICKS(10));


			stepper_counter--;
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			pump_status = 0;
			if (stepper_counter <= left_limit_value || !(GPIOB->IDR & (1<<15)))
				{
					stepper_status = 2; //turn right
				}

		}

		//turn right
		else if(stepper_status == 2 && (GPIOB->IDR & (1<<14)))
		{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); //go in right direction
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
					vTaskDelay(pdMS_TO_TICKS(10));

					stepper_counter++;

					if ((stepper_counter >= right_limit_value || !(GPIOB->IDR & (1<<14))))
						{

							process_step = 3;
							stepper_status = 1;
						}


		}

		else if(!(GPIOB->IDR & (1<<14)) || !(GPIOB->IDR & (1<<15)))
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //stop stepper motor when reaching end stops
				}

	}
}

static void task5_handler(void *parameters){
	while(1)
	{
		if(pump_status == 0 )
		{
			HAL_Delay(1500);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		}

		else if (pump_status == 1)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 900-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
