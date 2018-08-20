/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "hcsr04.h"

/** @defgroup HC-SR04_Private_Variables
* @{
*/ 
USENSOR_DrvTypeDef  hcsr04_drv = 
{
  HCSR04_Init,
  HCSR04_Read  
};

TIM_HandleTypeDef    TimHandle;

/*------------------------------------------------------------------------------*/
uint8_t HCSR04_Init(USENSOR_t* HCSR04, GPIO_TypeDef* ECHO_GPIOx, uint16_t ECHO_GPIO_Pin, GPIO_TypeDef* TRIGGER_GPIOx, uint16_t TRIGGER_GPIO_Pin) 
{ 	
  GPIO_InitTypeDef  gpioinitstruct;
  
  /* Save everything */
  HCSR04->ECHO_GPIOx = ECHO_GPIOx;
  HCSR04->ECHO_GPIO_Pin = ECHO_GPIO_Pin;
  HCSR04->TRIGGER_GPIOx = TRIGGER_GPIOx;
  HCSR04->TRIGGER_GPIO_Pin = TRIGGER_GPIO_Pin;
	
  /* Initialize pins */	
  /* Trigger pin */    
  switch((uint32_t)HCSR04->TRIGGER_GPIOx)
  {
    case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
    case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
    case GPIOF_BASE: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
  }  
  gpioinitstruct.Pin = HCSR04->TRIGGER_GPIO_Pin;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_PULLDOWN;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;  
  HAL_GPIO_Init(HCSR04->TRIGGER_GPIOx, &gpioinitstruct);
	
  /* Echo pin */    
  switch((uint32_t)HCSR04->ECHO_GPIOx)
  {
    case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
    case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
    case GPIOF_BASE: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
  }
  gpioinitstruct.Pin = HCSR04->ECHO_GPIO_Pin;
  gpioinitstruct.Mode = GPIO_MODE_INPUT;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;  
  HAL_GPIO_Init(HCSR04->ECHO_GPIOx, &gpioinitstruct);
	
  /* Trigger set to low */  
  HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, GPIO_PIN_RESET);
	
/* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM3CLK = PCLK1
      PCLK1 = HCLK
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */
  uint16_t PrescalerValue = 0;
  
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  /* Compute the prescaler value to have TIMx counter clock equal to 1000000 Hz */
  PrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIM3;

  /* Initialize TIMx peripheral as follows:
       + Period = 1000000 - 1
       + Prescaler = (SystemCoreClock/1000000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 65536 - 1;
  TimHandle.Init.Prescaler         = PrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  { /* Initialization Error */   
  }    
  
  HAL_TIM_Base_Start(&TimHandle);
/*-----------------------------------------------------------------------------------*/  
    
  /* Start measurement, check if sensor is working */
  if(HCSR04_Read(HCSR04) >= 0) 
  {/* Sensor OK */
    return 1;
  }
	
  /* Sensor error */
  return 0;
}

float HCSR04_Read(USENSOR_t* HCSR04) 
{
  uint32_t time, timeout;
  
  /* Timer counter reset */
  __HAL_TIM_SET_COUNTER(&TimHandle, 0);
  
  /* Trigger low */  
  HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, GPIO_PIN_RESET);  
  
  /* Trigger high for 10us */  
  HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, GPIO_PIN_SET);
  /* Delay 10 us */
  while(__HAL_TIM_GET_COUNTER(&TimHandle) < 15);  
  
  /* Trigger low */  
  HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, GPIO_PIN_RESET);
  
  /* Give some time for response */
  timeout = HCSR04_TIMEOUT;  
  while(!HAL_GPIO_ReadPin(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin)) 
  {
    if(timeout-- == 0x00) 
    {
      return -1;
    }
  }
	
  /* Start time */  
  __HAL_TIM_SET_COUNTER(&TimHandle, 0);
  
  /* Wait till signal is low */
  while(HAL_GPIO_ReadPin(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin)) 
  {
  }
  time = __HAL_TIM_GET_COUNTER(&TimHandle);
  
  /* Convert us to cm */
  HCSR04->Distance =  (float)time * HCSR04_NUMBER;
	
  /* Return distance */
  return HCSR04->Distance;
}
