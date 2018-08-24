/**
  ******************************************************************************
  * @file    stlm75.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    24-November-2014
  * @brief   This file contains all the functions prototypes for the stlm75.c
  *          temperature sensor driver.
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
  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHT20_H
#define __SHT20_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Common/tsensor.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup STLM75
  * @{
  */
  
/** @defgroup SHT20_Exported_Constants
  * @{
  */
enum
{
  HOLD = 0,
  NOHOLD
};
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/

/***************************** Read Access Only *******************************/
#define SHT20_REG_TEMP_HOLD    0xE3  /*!< Temperature Register Hold master of SHT20 */
#define SHT20_REG_TEMP_NOHOLD  0xF3  /*!< Temperature Register No Hold master of SHT20 */   
#define SHT20_REG_HUM_HOLD     0xE5  /*!< Humidity Register Hold master of SHT20 */
#define SHT20_REG_HUM_NOHOLD   0xF5  /*!< Humidity Register No Hold master of SHT20 */

/***************************** Read/Write Access ******************************/
#define SHT20_REG_READUSER     0xE7  /*!< Read User Register of SHT20 */
#define SHT20_REG_WRITEUSER    0xE6  /*!< Write User Register of SHT20 */
#define SHT20_REG_SRESET       0xFE  /*!< Soft Reset Register of SHT20 */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/
/** @defgroup Conversion_Mode_Selection 
  * @{
  */
#define SHT20_MEASUREMENT_RESOLUTION_12_14BIT   0x00   
#define SHT20_MEASUREMENT_RESOLUTION_8_12BIT    0x01
#define SHT20_MEASUREMENT_RESOLUTION_10_13BIT   0x10
#define SHT20_MEASUREMENT_RESOLUTION_11_11BIT   0x81   
  
   
/**
  * @}
  */

/** @defgroup Operation_Mode 
  * @{
  */
/**
  * @}
  */

/**
  * @}
  */
 
/** @defgroup SHT20_Exported_Functions
  * @{
  */
/* Sensor Configuration Functions */ 
void                      SHT20_Init(uint16_t DeviceAddr, TSENSOR_InitTypeDef *pInitStruct);
uint8_t                   SHT20_IsReady(uint16_t DeviceAddr, uint32_t Trials);
/* Sensor Request Functions */
uint8_t                   SHT20_ReadStatus(uint16_t DeviceAddr);
uint16_t                  SHT20_ReadTemp(uint16_t DeviceAddr, uint8_t hold);
uint16_t                  SHT20_ReadHumidity(uint16_t DeviceAddr, uint8_t hold);

/* Temperature Sensor driver structure */
extern TSENSOR_DrvTypeDef SHT20Drv;

/* Temperature Sensor IO functions */
void                      TSENSOR_IO_Init(void);
void                      TSENSOR_IO_Write(uint16_t DevAddress, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t Length);
void                      TSENSOR_IO_Read(uint16_t DevAddress, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t Length);
uint16_t                  TSENSOR_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
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

#ifdef __cplusplus
 }
#endif
  
#endif /* __STTS751_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
