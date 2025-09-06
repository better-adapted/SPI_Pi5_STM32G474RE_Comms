/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComDMA_Slave/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_nucleo.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_SPI_DBG_D0_Pin GPIO_PIN_11
#define GPIO_SPI_DBG_D0_GPIO_Port GPIOB
#define GPIO_SPI_DBG_D1_Pin GPIO_PIN_12
#define GPIO_SPI_DBG_D1_GPIO_Port GPIOB
#define GPIO_SPI_DBG_D2_Pin GPIO_PIN_13
#define GPIO_SPI_DBG_D2_GPIO_Port GPIOB
#define GPIO_SPI_DBG_D3_Pin GPIO_PIN_14
#define GPIO_SPI_DBG_D3_GPIO_Port GPIOB
#define GPIO_SPI_DBG_D4_Pin GPIO_PIN_15
#define GPIO_SPI_DBG_D4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Size of buffer */
#define SPI_TX_RX_BUFFERSIZE                       16384
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
