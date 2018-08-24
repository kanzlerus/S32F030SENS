/**
  ******************************************************************************
  * @file    stlm75.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    24-November-2014
  * @brief   This file provides a set of functions needed to manage the STLM75
  *          Temperature Sensor.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "sht20.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @defgroup SHT20
  * @brief     This file provides a set of functions needed to drive the 
  *            SHT20 Temperature/Hummiduty Sensor.
  * @{
  */

/** @defgroup SHT20_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup SHT20_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup SHT20_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup SHT20_Private_Variables
  * @{
  */ 
TSENSOR_DrvTypeDef SHT20Drv =
{
  SHT20_Init,
  SHT20_IsReady,
  SHT20_ReadStatus,
  SHT20_ReadTemp,
  SHT20_ReadHumidity,
};

/**
  * @}
  */

/** @defgroup STLM75_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup STLM75_Private_Functions
  * @{
  */

/**
  * @brief  Set STLM75 Initialization.
  * @param  DeviceAddr : Device ID address.
  * @param  pInitStruct: pointer to a STLM75_InitTypeDef structure 
  *         that contains the configuration setting for the STLM75.
  * @retval None
  */
void SHT20_Init(uint16_t DeviceAddr, TSENSOR_InitTypeDef *pInitStruct)
{  
  uint8_t  confreg = 0x02;  

  /* Set the Configuration Register */
  confreg = confreg | pInitStruct->ConversionResolution;
  TSENSOR_IO_Write(DeviceAddr, &confreg, SHT20_REG_WRITEUSER, 1);  
}

/**
  * @brief  Read ID address of STLM75
  * @param  DeviceAddr : Device ID address.
  * @param  Trials: Number of trials
  * @retval ID name
  */
uint8_t SHT20_IsReady(uint16_t DeviceAddr, uint32_t Trials)
{
  /* Configure the low level interface ---------------------------------------*/
  TSENSOR_IO_Init();
  
  /* Check is Temperature Sensor is Ready to use */
  return TSENSOR_IO_IsDeviceReady(DeviceAddr, Trials);
}

/**
  * @brief  Read The Temperature Sensor Status
  * @param  DeviceAddr : Device ID address.
  * @retval Status
  */
uint8_t SHT20_ReadStatus(uint16_t DeviceAddr)
{
  uint8_t tmp = 0;

  /* Read Status register */
  TSENSOR_IO_Read(DeviceAddr, &tmp, SHT20_REG_READUSER, 1);

  /* Return Temperature Sensor Status */
  return (uint8_t)(tmp);
}

/**
  * @brief  Read ID address of SHT20, Temperature
  * @param  DeviceAddr : Device ID address.
  * @retval ID name
  */
uint16_t SHT20_ReadTemp(uint16_t DeviceAddr, uint8_t hold)
{
  uint16_t tempreg = 0;
  //uint16_t tmp = 0;

  /* Read Temperature registers */
  switch(hold)
  {
    case HOLD: 
      TSENSOR_IO_Read(DeviceAddr, (uint8_t*)(&tempreg), SHT20_REG_TEMP_HOLD, 2);
    break;
    case NOHOLD:
      TSENSOR_IO_Read(DeviceAddr, (uint8_t*)(&tempreg), SHT20_REG_TEMP_NOHOLD, 2);
    break;
  }
  
  //tmp = ((tempreg & 0x00FF) << 8) | ((tempreg & 0xFF00) >> 8);
  //tempreg = (((tmp & 0x7F80) >> 7) | (tmp & 0x8000));

  /* Return Temperature value */
  return (tempreg);
}

/**
  * @brief  Read ID address of SHT20, Humidity
  * @param  DeviceAddr : Device ID address.
  * @retval ID name
  */
uint16_t SHT20_ReadHumidity(uint16_t DeviceAddr, uint8_t hold)
{
  uint16_t humreg = 0;
  //uint16_t tmp = 0;

  /* Read Humidity registers */
  switch(hold)
  {
    case HOLD: 
      TSENSOR_IO_Read(DeviceAddr, (uint8_t*)(&humreg), SHT20_REG_TEMP_HOLD, 2);
    break;
    case NOHOLD:
      TSENSOR_IO_Read(DeviceAddr, (uint8_t*)(&humreg), SHT20_REG_TEMP_NOHOLD, 2);
    break;
  }
  
  //tmp = ((humreg & 0x00FF) << 8) | ((humreg & 0xFF00) >> 8);
  //humreg = (((tmp & 0x7F80) >> 7) | (tmp & 0x8000));

  /* Return Humidity value */
  return (humreg);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
