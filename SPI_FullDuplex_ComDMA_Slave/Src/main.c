/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComDMA_Slave/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32G4xx SPI HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          DMA transfer.
  *          The communication is done using 2 Boards.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int SPI1_Get_CS();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR,
  TRANSFER_PROCESSED
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
/* Buffer used for transmission */
//uint8_t aTxBuffer[SPI_TX_RX_BUFFERSIZE]   = "tmuwhgbwguygmliawromewzzjebiisyendbbfdluaxqzwmizmkumzoofoblglvnlwjrskthmsjhedgypxamwvgncowmwiurbvdeusykeevcrodvx";
// =  0x613A as CRC-16/UMTS - buffer 61 then 3A as last byte.
// RAW BYTES[112] = 74 6D 75 77 68 67 62 77 67 75 79 67 6D 6C 69 61 77 72 6F 6D 65 77 7A 7A 6A 65 62 69 69 73 79 65 6E 64 62 62 66 64 6C 75 61 78 71 7A 77 6D 69 7A 6D 6B 75 6D 7A 6F 6F 66 6F 62 6C 67 6C 76 6E 6C 77 6A 72 73 6B 74 68 6D 73 6A 68 65 64 67 79 70 78 61 6D 77 76 67 6E 63 6F 77 6D 77 69 75 72 62 76 64 65 75 73 79 6B 65 65 76 63 72 6F 64 76 78 61 3A
// Result	Check	Poly	Init	RefIn	RefOut	XorOut
// 0x613A	0xFEE8	0x8005	0x0000	false	false	0x0000


uint8_t aTxBuffer[SPI_TX_RX_BUFFERSIZE] = "****SPI - Two Boards communication based on DMA **** SPI Message ******** SPI Message ******** SPI Message ****";
// seems to match  CRC-16/UMTS
// https://crccalc.com/?crc=****SPI%20-%20Two%20Boards%20communication%20based%20on%20DMA%20****%20SPI%20Message%20********%20SPI%20Message%20********%20SPI%20Message%20****&method=CRC-16&datatype=ascii&outtype=hex
// ioc settings CRC16 with X0+X2+X15
// RAW BYTES[112] 2A 2A 2A 2A 53 50 49 20 2D 20 54 77 6F 20 42 6F 61 72 64 73 20 63 6F 6D 6D 75 6E 69 63 61 74 69 6F 6E 20 62 61 73 65 64 20 6F 6E 20 44 4D 41 20 2A 2A 2A 2A 20 53 50 49 20 4D 65 73 73 61 67 65 20 2A 2A 2A 2A 2A 2A 2A 2A 20 53 50 49 20 4D 65 73 73 61 67 65 20 2A 2A 2A 2A 2A 2A 2A 2A 20 53 50 49 20 4D 65 73 73 61 67 65 20 2A 2A 2A 2A
// = 73 C2 = 0x73C2
// website says :
// Result	Check	Poly	Init	RefIn	RefOut	XorOut
// 0x73C2	0xFEE8	0x8005	0x0000	false	false	0x0000

//uint8_t aTxBuffer[]   = "well this is just to see what crc does with a shorter packet?"; // 0x7A33

/* Buffer used for reception */
uint8_t aRxBuffer[SPI_TX_RX_BUFFERSIZE];

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	uint32_t HAL_Error_Counter;

	struct
	{
		uint32_t Interrupt_Counter;
		uint32_t States[HAL_SPI_STATE_ABORT+1];
	}CS_End;

	struct
	{
		uint32_t Begin_Counter;
		uint32_t End_Counter;
		uint32_t Length_Max_Error;
		uint32_t Length_DMA_Count_Error;
		uint32_t CRC_Error_Counter;
		uint32_t CRC_OK_Counter;

	}Process;

}SPI_Transfer_Status_t;

SPI_Transfer_Status_t SPI_Transfer_Status={};

int Transfer_Init=1;

#define SPI1_CS_PIN                  GPIO_PIN_4
#define SPI1_CS_PORT                 GPIOA
#define SPI1_CS_EXTI_IRQn            EXTI4_IRQn

#define SPI1_Process_DBG_PIN                  GPIO_PIN_1
#define SPI1_Process_DBG_PORT                 GPIOA

uint16_t cal_crc(const uint8_t *pBuffer,int pSize)
{
	uint16_t crc = 0xFFFF;
	uint8_t temp;
	for(int x=0;x<pSize;x++)
	{
			temp = pBuffer[x];
			crc -= temp;
	}

	return crc;
}

typedef struct
{
	struct
	{
		uint16_t length;
		uint8_t command;
		uint8_t buffer[256];
	} payload;
	uint16_t crc;
}SPI_Transfer_Base_t;

void Process_Buffer()
{
	// all good!
	SPI_Transfer_Status.Process.Begin_Counter++;

	SPI_Transfer_Base_t *packet = (SPI_Transfer_Base_t *)aRxBuffer;

	uint16_t Errors=0;
	int Length_Ok=0;
	int CRC_Ok=0;

	if(packet->payload.length>sizeof(SPI_Transfer_Base_t))
	{
		Errors=0x0001;
		SPI_Transfer_Status.Process.Length_Max_Error++;
	}
	else
	{
		int bytes_rx_in_DMA = SPI_TX_RX_BUFFERSIZE - hspi1.hdmarx->Instance->CNDTR;
		if(packet->payload.length == (bytes_rx_in_DMA)-2)
		{
			Length_Ok=1;
		}
		else
		{
			Errors=0x0002;
			SPI_Transfer_Status.Process.Length_DMA_Count_Error++;
		}
	}


	if(Length_Ok)
	{
		// ok packet length rx'f matches the DMA counter
		uint16_t rx_crc = cal_crc((uint8_t*)&packet->payload,packet->payload.length);

		if(packet->crc == rx_crc)
		{
			CRC_Ok=1;
			SPI_Transfer_Status.Process.CRC_OK_Counter++;
		}
		else
		{
			SPI_Transfer_Status.Process.CRC_Error_Counter++;
		}
	}

	SPI_Transfer_Status.Process.End_Counter++;
}

HAL_StatusTypeDef SPI1_TEST_SEND()
{
	return HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, SPI_TX_RX_BUFFERSIZE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_4)
	{
		SPI_Transfer_Status.CS_End.Interrupt_Counter++;

		HAL_StatusTypeDef state_res = HAL_SPI_GetState(&hspi1);
		if(state_res<=(HAL_StatusTypeDef)HAL_SPI_STATE_ABORT)
		{
			SPI_Transfer_Status.CS_End.States[state_res]++;
		}

		// just make this state - even if no bytes!
		wTransferState = TRANSFER_COMPLETE;
	}
}

int SPI1_Get_CS()
{
	return HAL_GPIO_ReadPin(SPI1_CS_PORT, SPI1_CS_PIN);
}

void SPI1_PA4_EN_Intterrupt()
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = SPI1_CS_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(SPI1_CS_PORT, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(SPI1_CS_EXTI_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(SPI1_CS_EXTI_IRQn);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* STM32G4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED2);
  SPI1_PA4_EN_Intterrupt();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_StatusTypeDef state_res = HAL_SPI_GetState(&hspi1);

	  if(Transfer_Init)
	  {
		  HAL_StatusTypeDef res = SPI1_TEST_SEND();
		  if ( res != HAL_OK)
		  {
		    /* Transfer error in transmission process */
		    Error_Handler();
		  }
		  Transfer_Init=0;
	  }

	  while (wTransferState == TRANSFER_WAIT)
	  {
	  }

	  switch (wTransferState)
	  {
	    case TRANSFER_COMPLETE :
	    	HAL_GPIO_WritePin(SPI_Process_DBG_GPIO_Port, SPI_Process_DBG_Pin, GPIO_PIN_SET);

	    	Process_Buffer();

	    	HAL_SPI_DMAStop(&hspi1);
			HAL_SPI_Abort(&hspi1);

			__HAL_RCC_SPI1_FORCE_RESET();
			__HAL_RCC_SPI1_RELEASE_RESET();

			memset(aRxBuffer,0xFF,sizeof(aRxBuffer));

			HAL_StatusTypeDef res = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, SPI_TX_RX_BUFFERSIZE);


			wTransferState = TRANSFER_PROCESSED;

			HAL_GPIO_WritePin(SPI_Process_DBG_GPIO_Port, SPI_Process_DBG_Pin, GPIO_PIN_RESET);
	      break;

	    case TRANSFER_ERROR:
	    	SPI_Transfer_Status.HAL_Error_Counter++;
			wTransferState = TRANSFER_PROCESSED;

			// https://community.st.com/t5/stm32-mcus-products/restart-spi-dma-transmission/td-p/637909
			HAL_SPI_DMAStop(&hspi1);
			__HAL_RCC_SPI1_FORCE_RESET();
			__HAL_RCC_SPI1_RELEASE_RESET();
			HAL_SPI_DeInit(&hspi1);
			MX_SPI1_Init();
			SPI1_PA4_EN_Intterrupt();

			Transfer_Init=1;
	      break;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_Process_DBG_GPIO_Port, SPI_Process_DBG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_Process_DBG_Pin */
  GPIO_InitStruct.Pin = SPI_Process_DBG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_Process_DBG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  static int callbacks;
  /* Turn LED2 on: Transfer in transmission/reception process is complete */


  callbacks++;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
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
  BSP_LED_Off(LED2);
  BSP_LED_Off(LED2);
  while (1)
  {
    /* Toggle LED2 for error */
	BSP_LED_On(LED2);
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
