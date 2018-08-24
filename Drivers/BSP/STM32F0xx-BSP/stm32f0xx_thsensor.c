/**
  ******************************************************************************
  * @file    stm32072b_eval_tsensor.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the I2C STLM75 
  *          temperature sensor mounted on STM32072B-EVAL board . 
  *          It implements a high level communication layer for read and write 
  *          from/to this sensor. The needed STM32F0xx hardware resources (I2C and 
  *          GPIO) are defined in stm32072b_eval.h file, and the initialization is 
  *          performed in TSENSOR_IO_Init() function declared in stm32072b_eval.c 
  *          file.
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          TSENSOR_IO_Init() function. 
  *
  *     +-----------------------------------------------------------------+
  *     |                        Pin assignment                           |
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32F0xx I2C Pins                   |   STLM75  |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | STLM75_I2C_SDA_PIN/ SDA               |   SDA     |    1        |
  *     | STLM75_I2C_SCL_PIN/ SCL               |   SCL     |    2        |
  *     | STLM75_I2C_SMBUSALERT_PIN/ SMBUS ALERT|   OS/INT  |    3        |
  *     | .                                     |   GND     |    4  (0V)  |
  *     | .                                    |   GND     |    5  (0V)  |
  *     | .                                     |   GND     |    6  (0V)  |
  *     | .                                     |   GND     |    7  (0V)  |
  *     | .                                    |   VDD     |    8  (3.3V)|
  *     +---------------------------------------+-----------+-------------+
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_thsensor.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup S32F030SENS_Private_Variables Private Variables
  * @{
  */ 
static TSENSOR_DrvTypeDef  *tsensor_drv; 
__IO uint16_t  TSENSORAddress = 0;

/**
  * @}
  */ 

/** @addtogroup S32F030SENS_Exported_Functions
  * @{
  */ 

/**
  * @brief  Initializes peripherals used by the I2C Temperature Sensor driver.
  * @retval TSENSOR status
  */
uint32_t BSP_TSENSOR_Init(void)
{ 
  uint8_t ret = TSENSOR_ERROR;
  TSENSOR_InitTypeDef SHT20_InitStructure;

  /* Temperature Sensor Initialization */
  if(SHT20Drv.IsReady(TSENSOR_I2C_ADDRESS, TSENSOR_MAX_TRIALS) == HAL_OK)
  {
    /* Initialize the temperature sensor driver structure */
    TSENSORAddress = TSENSOR_I2C_ADDRESS;
    tsensor_drv = &SHT20Drv;

    ret = TSENSOR_OK;
  }
  else
  {
    ret = TSENSOR_ERROR;    
  }

  if(ret == TSENSOR_OK)
  {  
    SHT20_InitStructure.ConversionResolution  = SHT20_MEASUREMENT_RESOLUTION_12_14BIT;    
        
    /* TSENSOR Init */   
    tsensor_drv->Init(TSENSORAddress, &SHT20_InitStructure);

    ret = TSENSOR_OK;
  }
  
  return ret;
}

/**
  * @brief  Returns the Temperature Sensor status.
  * @retval The Temperature Sensor status.
  */
uint8_t BSP_TSENSOR_ReadStatus(void)
{
  return (tsensor_drv->ReadStatus(TSENSORAddress));
}

/**
  * @brief  Read Temperature register of SHT20.
  * @retval SHT20 measured temperature value.
  */
uint16_t BSP_TSENSOR_ReadTemp(uint8_t hold)
{ 
  return tsensor_drv->ReadTemp(TSENSORAddress, hold);  
}

/**
  * @brief  Read Humidity register of SHT20.
  * @retval SHT20 measured humidity value.
  */
uint16_t BSP_TSENSOR_ReadHumidity(uint8_t hold)
{ 
  return tsensor_drv->ReadHumidity(TSENSORAddress, hold);  
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
