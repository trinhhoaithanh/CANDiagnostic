/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "st7789.h"
#include "LCD_display.h"
#include "stdint.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t u16_ADCVal;
float Voltage;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
  if(hadc->Instance == ADC1){
	  u16_ADCVal = HAL_ADC_GetValue(&hadc1);
	  Voltage = (float)u16_ADCVal/4096*3.3;
  }
}

CAN_TxHeaderTypeDef   TxHeader_Node1;
CAN_RxHeaderTypeDef   RxHeader_Node1;

CAN_TxHeaderTypeDef   TxHeader_Node2;
CAN_RxHeaderTypeDef   RxHeader_Node2;

uint32_t TxMailbox;
uint8_t TxData_Node1[8] = {0x22, 0x34,0x10,0,0,0,0,0};
uint8_t RxData_Node1[8];


uint8_t TxData_Node2[8] = {0,0,0,0,0,0,0,0};;
uint8_t RxData_Node2[8];

uint8_t  count=0;

int datacheck = 0;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader_Node1, RxData_Node1) == HAL_OK){
		uint32_t value = RxHeader_Node1.StdId;
		char valueString[11];
		sprintf(valueString, "DataID: 0x%04X", value);
		ST7789_WriteString(140, 10, valueString, Font_7x10, RED, WHITE);

		for(int i=0;i<8;i++){
			uint8_t value = RxData_Node1[i];
		    char valueString[5];
		    sprintf(valueString, "0x%02X", value);
		    ST7789_WriteString(140, (20*i)+30, valueString, Font_7x10, RED, WHITE);
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);

		if(RxData_Node1[0] == 0x7F){
			if(RxData_Node1[2] == 0x11){
				ST7789_WriteString(0, 200, "serviceNotSupportedInActiveSession", Font_7x10, RED, WHITE);
			}
			else if(RxData_Node1[2] == 0x31){
				ST7789_WriteString(0, 200, "requestOutOfRange", Font_7x10, RED, WHITE);
			}
			else{
				ST7789_WriteString(0, 200, "Error", Font_7x10, RED, WHITE);
			}

		}
		else{
			uint16_t combinedValue = (uint16_t)((RxData_Node1[3] << 8) | RxData_Node1[4]);
			float receivedVoltage = (float)combinedValue/4096*3.3;
			char valueString[11];
			sprintf(valueString, "Received Voltage: %f", receivedVoltage);
			ST7789_WriteString(0, 200, valueString, Font_7x10, RED, WHITE);
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader_Node2, RxData_Node2) == HAL_OK){
			uint32_t value = RxHeader_Node2.StdId;
			char valueString[11];
			sprintf(valueString, "DataID: 0x%04X", value);
			ST7789_WriteString(0, 10, valueString, Font_7x10, RED, WHITE);

            for(int i=0;i<8;i++){
            	uint8_t value = RxData_Node2[i];
            	char valueString[5];
            	sprintf(valueString, "0x%02X", value);
            	ST7789_WriteString(0,(20*i)+30, valueString, Font_7x10, RED, WHITE);
            }

            if(RxData_Node2[0] != 0x22){
            	TxData_Node2[0] = 0x7F;
            	TxData_Node2[1] = RxData_Node2[0];
            	TxData_Node2[2] = 0x11;
            }
            else if(RxData_Node2[0] == 0x22 && (RxData_Node2[1] != 0x34 || RxData_Node2[2] != 0x10)){
            	TxData_Node2[0] = 0x7F;
            	TxData_Node2[1] = RxData_Node2[0];
            	TxData_Node2[2] = 0x31;
            }
            else{
            	TxData_Node2[0] = 0x62;
            	TxData_Node2[1] = RxData_Node2[1];
            	TxData_Node2[2] = RxData_Node2[2];
            	HAL_ADC_Start_IT(&hadc1);
            	TxData_Node2[3] = (uint8_t)(u16_ADCVal >> 8);
            	TxData_Node2[4] = (uint8_t)(u16_ADCVal & 0xFF);
            }

			if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader_Node2, TxData_Node2, &TxMailbox) == HAL_OK){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);

			}
		}
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  TxHeader_Node1.DLC = 8;
    TxHeader_Node1.ExtId = 0x02;
    TxHeader_Node1.IDE = CAN_ID_STD;
    TxHeader_Node1.RTR = CAN_RTR_DATA;
    TxHeader_Node1.StdId = 0x012;
    TxHeader_Node1.TransmitGlobalTime = DISABLE;

    TxHeader_Node2.DLC = 8;
    TxHeader_Node2.ExtId = 0x02;
    TxHeader_Node2.IDE = CAN_ID_STD;
    TxHeader_Node2.RTR = CAN_RTR_DATA;
    TxHeader_Node2.StdId = 0x0A2;
    TxHeader_Node2.TransmitGlobalTime = DISABLE;

      HAL_CAN_Start(&hcan1);
      HAL_CAN_Start(&hcan2);

      //CAN1 filter
      CAN_FilterTypeDef can1filterconfig;

          can1filterconfig.FilterActivation = CAN_FILTER_ENABLE;
          can1filterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones
          can1filterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
          can1filterconfig.FilterIdHigh = 0x0A2 << 5;
          can1filterconfig.FilterIdLow = 0x0000;
          can1filterconfig.FilterMaskIdHigh = 0x0A2 << 5;
          can1filterconfig.FilterMaskIdLow = 0x0000;
          can1filterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
          can1filterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
          can1filterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

          HAL_CAN_ConfigFilter(&hcan1, &can1filterconfig);
      //End CAN1 filter

      //CAN1 filter
      CAN_FilterTypeDef can2filterconfig;

      	can2filterconfig.FilterActivation = CAN_FILTER_ENABLE;
          can2filterconfig.FilterBank = 14;  // which filter bank to use from the assigned ones
          can2filterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
          can2filterconfig.FilterIdHigh = 0x012 << 5;
          can2filterconfig.FilterIdLow = 0x0000;
          can2filterconfig.FilterMaskIdHigh = 0x012 << 5;
          can2filterconfig.FilterMaskIdLow = 0x0000;
          can2filterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
          can2filterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
          can2filterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

          HAL_CAN_ConfigFilter(&hcan2, &can2filterconfig);
       //End CAN1 filter

          HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
          HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);

  lcd_init();
  ST7789_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Node1, TxData_Node1, &TxMailbox);



	  HAL_Delay(1000);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 20;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LCD_RESET_Pin|LCD_CS_Pin
                          |LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 LCD_RESET_Pin LCD_CS_Pin
                           LCD_DC_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LCD_RESET_Pin|LCD_CS_Pin
                          |LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
