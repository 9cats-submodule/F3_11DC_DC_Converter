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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base.h"
#include "hmi_user_uart.h"
#include "hmi_driver.h"

#include "stdio.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TP_CHECK(x0,y0,x1,y1) tp_dev.x[0] > x0 && tp_dev.y[0] > y0 && tp_dev.x[0] < x1 && tp_dev.y[0] < y1
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
u16 val[3];
u8 str[40] = {0};
u8 mode = 0,next_mode=0;  //0:开环 1:稳压 2:稳压 3:放电 4:预备较准 5:放电判断
u8 autoMode = 0; 		  //自动适应
u8 OK = 0;				  //没有OK
float vcc = 3.29999971f;//3.29999971
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  u8 key;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  delay_init(72);
  TFT_Init();
  PID_Init(30,10,50,100);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  hdac.Instance->DHR12L1 = 1024;
  HAL_ADC_Start_DMA(&hadc1, (u32*)val, 3);
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
  TIM1->CCR4 = 2100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// 10V 占空比最高
	//pid.limit
	//
	delay_ms(10);
	sprintf((char *)str,"v1:%4d,v2:%4d,v3:%4d\r\n",val[0], val[1], val[2]);
	HAL_UART_Transmit_IT(&huart3, str,40);

    delay_ms(50);
    key = KEY_Scan(0);
    if(key == KEY0_PRES)
    {
    	next_mode = 1;
    }
    if(key == KEY1_PRES)
    {
    	next_mode = 2;
    }

	if(next_mode != mode)
	{
		switch(next_mode)
		{
			case 0:
			{
  			  	SetTextValue(0,21,(u8*)"开环");
				TIM1->CCR4 = 2100;
			}
			case 1/*稳流*/:
			{
  			  	SetTextValue(0,21,(u8*)"稳流");
  			  	if(mode = 2)
  			  	{
  			  		PID.Kd = 10;
  			  		PID.Ki = 10;
  			  		PID.Kp = 10;
  			  	}
				TIM1->CCR4 = 2100;
			}break;
			case 2/*稳压*/:
			{
  			  	SetTextValue(0,21,(u8*)"稳压");
  			  	if(mode = 2)
  			  	{
  			  		PID.Kd = 30;
  			  		PID.Ki = 40;
  			  		PID.Kp = 50;
  			  	}
				TIM1->CCR4 = 2100;
			}break;
			case 3/*放电*/:
			{
  			  	SetTextValue(0,21,(u8*)"放电");
				PID_Init(-30,-40,-50,70);
				TIM1->CCR4 = 2100;
			}break;
			case 4/**/:
			{
  			  	SetTextValue(0,21,(u8*)"未接负载");
			}
			case 8/*稳压-充电完毕*/:
			{
  			  	SetTextValue(0,21,(u8*)"充电完毕");
			}break;
		}
		if(next_mode == 8) mode = 2;
		else mode = next_mode;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
