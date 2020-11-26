/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
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

uint32_t temp[2] = {0}, i;
uint8_t ecg[2] = {0};
uint8_t bcg[2] = {0};
uint8_t data[8] = {0};
//TIM_HandleTypeDef htim3;

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
//	HAL_ADC_Start(&hadc1);
	
	MpuInit();
	HAL_Delay(100);
	MpuGetData();
	HAL_Delay(100);
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_Delay(100);  //һ��Ҫ�ӣ������ʼ��DMA��ʧ��
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&temp, 2);
	
  data[0] = 0xAA; 	//���ڴ���У��ͷ
	data[1] = 0x00;
	data[6] = 0x0F; 	//���ڴ���У��β
	data[7] = 0x03;	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		HAL_Delay(1000);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//		ecg[0] = (uint8_t)(temp[0] >> 8);
//		ecg[1] = (uint8_t)(temp[0]);
//	
//		bcg[0] = (uint8_t)(temp[1] >> 8);
//		bcg[1] = (uint8_t)(temp[1]);
	
	/* ʹ�õ��βɼ��ķ��� */
//		uint8_t i=0;
//		
//		for(i=0;i<2;i++)

//		{

//		HAL_ADC_Start(&hadc1);

//		HAL_ADC_PollForConversion(&hadc1,0xffff);

//		temp[i]=HAL_ADC_GetValue(&hadc1);

//		}
//		HAL_ADC_Stop(&hadc1);
//		
	
		/* ���� DMA �����ɼ�  */
		data[0] = 0xAA; 	//���ڴ���У��ͷ
		data[1] = 0x00;
		data[6] = 0x0F; 	//���ڴ���У��β
		data[7] = 0x03;	
		
//	  data[2] = (uint8_t)(temp[0] >> 8);
//		data[3] = (uint8_t)(temp[0]);
//	
//		data[4] = (uint8_t)(temp[1] >> 8);
//		data[5] = (uint8_t)(temp[1]);
		MpuGetData();
		data[2] = (uint8_t)(accelStruct.accelX >> 8);
		data[3] = (uint8_t)(accelStruct.accelX);
	
		data[4] = (uint8_t)(temp[1] >> 8);
		data[5] = (uint8_t)(temp[1]);
	
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_UART_Transmit(&huart1, "\r\nUART Transmit\r\n", 17, 10);

		HAL_UART_Transmit(&huart1, data, 8, 10);
//		HAL_UART_Transmit(&huart1, accelStruct.accelX, 8, 10);

		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&temp, 3);
		
	  
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
