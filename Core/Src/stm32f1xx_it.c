/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hmi_driver.h"
#include "dac.h"
#include "tim.h"

#include "stdio.h"
#include "pid.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern float vcc;
extern u16 val[];
float vt[3]={2.49599981f,2.78299642f,1.96769989f} ;// 2.81699634f                恒压 8.4！！！       2.682853657142857f
u16 VAL_ARR[2000]={0};
u16 i=0,j=1;
extern u8 next_mode;
//2.54899979f;恒流
//2.82399964f;恒压
//1.98000002f;放电？1919
extern u8 mode,next_mode,autoMode,OK;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  static u32 sValue[3][11] = {0};  //0放平均值
  static u16 gValue[3] = {0};
  static u8  warn[3] = {0};
  static u8  reset[2]= {0};
  float vi;
  s16 vi_value,inc;
  u8 str[25] = {0};
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  switch(i%3)
  {
  	  case 0:
  	  {
  		sValue[i%3][0]-= sValue[i%3][j];
  		sValue[i%3][0]+= sValue[i%3][j] = val[0];
  		gValue[0]    = sValue[i%3][0]/10;
  	  }break;
  	  case 1:
  	  {
  		sValue[i%3][0]-= sValue[i%3][j];
  		sValue[i%3][0]+= sValue[i%3][j] = val[1];
  		gValue[1]    = sValue[i%3][0]/10;
  	  }break;
  	  case 2:
  	  {
  		sValue[i%3][0]-= sValue[i%3][j];
  		sValue[i%3][0]+= sValue[i%3][j] = val[2];
  		gValue[2]    = sValue[i%3][0]/10;
  		j = j%10+1;
  	  }break;
  }
  if(mode<=3 && mode)
  {
      //采样值(4096)
      vi_value = val[mode-1];
      //变换为采样值实际(vcc)
      vi = vcc * vi_value/4096;
      //PID输出 占空比
      inc = (s16)PID_Realize(vi, vt[mode-1]);
  }
  //正常模式(1700~3550)
  switch(mode)
  {
  	case 1/*稳流*/:			// 10-8.4Ω-3550为最大    14v-6Ω-1700  最小 2.54899979f
  	{
  		if(autoMode)
  		{
  			//TODO:测试用数据
  			static float v_test = -0.10f;
			//10次超过 改变状态
			if     (val[1] > (vt[1]-v_test)/vcc*4096) warn[1]++;
			else warn[0]++;
			if(warn[1]==8)
			{
				next_mode = 2;
				warn[0] = warn[1] = 0;
				break;
			}
  		}

		//判断断路
		if(fabs(inc) == PID.limit && PID.err-PID.lerr < 0.08) reset[1]++;
		else reset[0]++;
		if(reset[1]==20)
		{
			//TODO:该换地方了
			if(autoMode)next_mode = 4;
			else OK = 0;
			break;
		}
		if(reset[0]==50)
		{
			reset[1] = reset[0] = 0;
		}
		if(warn[0]==20)
		{
			warn[0] = warn[1] = 0;
		}
  		//正常代码
		if(OK)
		{
	  		if(TIM1->CCR4 > 3550)      TIM1->CCR4 = 3550;
	  		else if(TIM1->CCR4 < 1500) TIM1->CCR4 = 1500 ;
	  		else                       TIM1->CCR4+= inc ;
		}
		else
		{
	  		static s32 checkVal0[2] = {0};
			if(i==1)                       TIM1->CCR4 = 2100;
			if(i%3==0  && i >= 9 && i<39)  checkVal0[0]+=val[0];
			if(i==40)					   TIM1->CCR4 = 2300;
			if(i%3==0 && i >= 69&& i<99)   checkVal0[1]+=val[0];
			if(i==100)
			{
				if(fabs(checkVal0[0]-checkVal0[1])>2000)
				{
					PID_Init(10,10,10,20);
					OK = 1;
				}
		  		i = 0;
		  		checkVal0[0] = checkVal0[1] = 0;
			}
		}
  	}break;
  	case 2/*稳压*/:			// 10V-8.4V-3100为最大     14v-8.4v-2200最小   2.81699634f
  	{
  		if(autoMode)
  		{
  			//TODO:测试用数据
  			static float v_test2 = 0.1f,v_over = 0.12366434f;
			if     (val[0] >    (vt[0]+v_test2)/vcc*4096) warn[1]++;
			else if(val[0] <   v_over/vcc*4086) warn[2]++;  //电流
			else warn[0]++;
			if(warn[1]==8)
			{
				//TODO:显示恒流模式
				next_mode = 1;
				warn[0] = warn[1] = warn[2] = 0;
				break;
			}
			if(warn[2]==3)
			{
				//TODO:显示电流过小，电量已经充满
				if(val[0] < 150) next_mode = 4;
				else next_mode = 8;
				warn[0] = warn[1] = warn[2] = 0;
				break;
			}
			if(warn[0]==20)
			{
				warn[0] = warn[1] = warn[2] = 0;
			}
  		}

		//判断断路
		if(fabs(inc) == PID.limit && PID.err-PID.lerr < 0.08) reset[1]++;
		else reset[0]++;
		if(reset[1]==20)
		{
			//TODO:该换地方了
			if(autoMode) next_mode = 4;
			else                OK = 0;
			break;
		}
		if(reset[0]==50)
		{
			reset[1] = reset[0] = 0;
		}
		if(warn[0]==20)
		{
			warn[0] = warn[1] = 0;
		}
  		//正常代码
		if(OK)
		{
	  		if(TIM1->CCR4 > 3400)      TIM1->CCR4 = 3400;
	  		else if(TIM1->CCR4 < 2100) TIM1->CCR4 = 2100;
	  		else                       TIM1->CCR4+= inc ;
		}
		else
		{
	  		static s32 checkVal1[2] = {0};
			if(i==1)                       TIM1->CCR4 = 2100;
			if(i%3==0  && i >= 9 && i<39)  checkVal1[0]+=val[1];
			if(i==40)					   TIM1->CCR4 = 2300;
			if(i%3==0 && i >= 69&& i<99)   checkVal1[1]+=val[1];
			if(i==100)
			{
				if(fabs(checkVal1[0]-checkVal1[1])>2000)
				{
					PID_Init(30,40,50,70);
					OK = 1;
				}
		  		i = 0;
		  		checkVal1[0] = checkVal1[1] = 0;
			}
		}
  	}break;
  	case 3/*放电*/:			// 6v-12v-1770    最小   3600     8.4-12v-2475(也可大些)  最大
  	{
		//判断断路
		if(fabs(inc) == PID.limit && PID.err-PID.lerr < 0.08) reset[1]++;
		else reset[0]++;
		if(reset[1]==20)
		{
			if(autoMode) next_mode = 4;
			else                OK = 0;
			break;
		}
		if(reset[0]==50)
		{
			reset[1] = reset[0] = 0;
		}
		if(warn[0]==20)
		{
			warn[0] = warn[1] = 0;
		}

		if(OK)
		{
			if(TIM1->CCR4 > 3300)     TIM1->CCR4 = 3300;
			else if(TIM1->CCR4 < 1700)TIM1->CCR4 = 1700;
			else                      TIM1->CCR4+= inc ;
		}
		else
		{
	  		static s32 checkVal2[2] = {0};
			if(i==1)                       TIM1->CCR4 = 2100;
			if(i%3==0  && i >= 9 && i<39)  checkVal2[0]+=val[2];
			if(i==40)					   TIM1->CCR4 = 2300;
			if(i%3==0 && i >= 69&& i<99)   checkVal2[1]+=val[2];
			if(i==100)
			{
				if(fabs(checkVal2[0]-checkVal2[1])>2000)
				{
					PID_Init(-30,-40,-50,70);
					OK = 1;
				}
		  		i = 0;
		  		checkVal2[0] = checkVal2[1] = 0;
			}
		}
  	}break;
  	case 4/*准备较准*/:
  	{
  		//首先判断是否放电
  		TIM1->CCR4 = 0;
  		mode = next_mode = 5;
  		i = 0;
  	}break;
  	case 5/*电流判断*/:
  	{
  		//需要多判断

  		/*
  		static s32 samVal0[2];
		if(i==1)                        TIM1->CCR4 = 2100;
		if(i%3==0  && i >= 9 && i<39)	samVal0[0]+=val[2];
		if(i==40)						TIM1->CCR4 = 2200;
		if(i%3==0  && i >= 69&& i<99)   samVal0[1]+=val[2];
		if(i==100)
		{
			if(fabs(samVal0[0]-samVal0[1])<2000)
			{	//vo为断路
				//TODO:显示vo断路
  			  	SetTextValue(0,37,(u8*)"断开");
				next_mode = mode = 7;
			}
			else
			{
				//稳压
  			  	SetTextValue(0,21,(u8*)"放电");
  			  	SetTextValue(0,37,(u8*)"连接");
				PID_Init(-30,-40,-50,70);
				next_mode = mode = 3;
			}
	  		i = 0;
	  		samVal0[0] = samVal0[1] = 0;
	  	}*/
  		static s32 samVal[2][2];
		if(i==1)                        TIM1->CCR4 = 2100;
		if(i%3==0  && i >= 9 && i<39)
		{
			samVal[0][0]+=val[0];
			samVal[1][0]+=val[1];
		}
		if(i==40)						TIM1->CCR4 = 2300;
		if(i%3==0  && i >= 69&& i<99)
		{
			samVal[0][1]+=val[0];
			samVal[1][1]+=val[1];
		}
		if(i==100)
		{
			if(fabs(samVal[0][0]-samVal[0][1])<200 || fabs(samVal[1][0]-samVal[1][1])<1000)
			{	//vo为断路
				//TODO:显示vo断路
  			  	SetTextValue(0,21,(u8*)"未接负载");
			}
			else
			{
				//稳压
				PID_Init(30,40,50,70);
				next_mode = 1;
				OK = 1;
			}
	  		i = 0;
	  		samVal[0][0] = samVal[0][1] = samVal[1][0] = samVal[1][1] = 0;
	  	}
  	}break;
  	case 6/*判断vo是否断路*/:
  	{
  		static s32 samVal1[2];
		if(i==1)                        TIM1->CCR4 = 2100;
		if(i%3==0  && i >= 9 && i<39)	samVal1[0]+=val[1];
		if(i==40)						TIM1->CCR4 = 2200;
		if(i%3==0  && i >= 69&& i<99)   samVal1[1]+=val[1];
		if(i==100)
		{
			if(fabs(samVal1[0]-samVal1[1])<100)
			{	//vo为断路
				//TODO:显示vo断路
  			  	SetTextValue(0,36,(u8*)"断开");
				next_mode = mode = 4;
			}
			else
			{
				//稳压
  			  	SetTextValue(0,21,(u8*)"恒压");
  			  	SetTextValue(0,36,(u8*)"连接");
				PID_Init(30,40,50,70);
				next_mode = mode = 2;
			}
	  		i = 0;
	  		samVal1[0] = samVal1[1] = 0;
		}
  	}break;
	case 7/*判断vi是否断路*/:
	{
  		static s32 samVal2[2];
		if(i==1)                        TIM1->CCR4 = 2100;
		if(i%3==0  && i >= 9 && i<39)	samVal2[0]+=val[0];
		if(i==40)						TIM1->CCR4 = 2200;
		if(i%3==0 && i >= 69&& i<99)   samVal2[1]+=val[0];
		if(i==100)
		{
			if((samVal2[0]-samVal2[1])<100)
			{	//vi为断路
				//TODO:显示vi断路
  			  	SetTextValue(0,35,(u8*)"断开");
				next_mode = mode = 6;
			}
			else
			{
				//稳流
  			  	SetTextValue(0,21,(u8*)"恒流");
  			  	SetTextValue(0,35,(u8*)"连接");
				PID_Init(10,10,10,20);
				next_mode = mode = 1;
			}
	  		i = 0;
	  		samVal2[0] = samVal2[1] = 0;
		}
	}break;
  }

  if(i == 900 && !autoMode)
  {
  	sprintf((char *)str,"%4d      %5.4f",val[0],val[0]*vcc/4096);
  	SetTextValue(0,31,str);
  }
  if(i == 901 && !autoMode)
  {
  	sprintf((char *)str,"%4d      %5.4f",val[1],val[1]*vcc/4096);
  	SetTextValue(0,32,str);
  }
  if(i++ == 902 && !autoMode)
  {
  	sprintf((char *)str,"%4d      %5.4f",val[2],val[2]*vcc/4096);
  	SetTextValue(0,33,str);
  	i=0;
  }
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
