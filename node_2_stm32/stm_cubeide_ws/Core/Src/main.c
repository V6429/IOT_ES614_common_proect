/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

// TODO: CREATE A MESSAGE FORMAT
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <strings.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "LoRa.h"
// THIS IS A CUSTOM MESSGE REFER HEADER FILE
#include "message_1.h"

#define BUFF_SIZE 50
#define MESSAGESIZE 12

#define NODE_ID 0xAB
#define SENDTO_ID 0xAC

#define DIGITAL_OUT1 0x01
#define DIGITAL_OUT2 0x02
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RECEIVE // TRANSMIT // RECEIVE
#define TRANSMIT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t LORA_RECEIVED_SOMETHING = 0;
uint8_t ADC_CONVERTED = 0;
uint8_t NODE_TRANSMIT = 1; // start with 1
uint8_t NODE_RECEIVE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Over riding the call back handler

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


 if(htim==&htim2) // 1 sec delay
 {
  NODE_RECEIVE = 0;
  HAL_TIM_Base_Stop(&htim2);
 }

 if(htim==&htim5)
 {
  NODE_TRANSMIT = 1;  //NODE_RECEIVE = 0 already
  HAL_TIM_Base_Stop(&htim5);

 }



 }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  char printbuffer[40] = {0};

  if (GPIO_Pin == dio_lora_pb2_Pin)
  {
    LORA_RECEIVED_SOMETHING = 1;
#ifdef RECEIVE
// snprintf(printbuffer,sizeof(printbuffer),"\nReceived something");
#endif
  }
  if (GPIO_Pin == B1_Pin)
  {
    snprintf(printbuffer, sizeof(printbuffer), "\nSomebody pressed the BLUE button!!");
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_UART_Transmit(&huart2, (uint8_t *)printbuffer, 40, 50);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  ADC_CONVERTED = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char x = 'x', newline = '\n', temp;
  char receivedval = 0;
  unsigned int status, i, rssi, lora_connect;
  char buffer[BUFF_SIZE] = {0};
  uint8_t airquality_input = 0;
  uint8_t send_data[MESSAGESIZE];
  uint8_t received_data[MESSAGESIZE];
  uint8_t packet_size = 0;
  uint16_t adc_value = 0;
  MSGrcv msg_rcv;
  MSGsend msg_snd;
  LoRa myLoRa;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // LORA SETTINGS
  myLoRa = newLoRa(); // sets frequency spreading factor to default value
  myLoRa.CS_pin = cs_lora_pb0_Pin;
  myLoRa.CS_port = cs_lora_pb0_GPIO_Port;
  myLoRa.reset_pin = reset_lora_pb1_Pin;
  myLoRa.reset_port = reset_lora_pb1_GPIO_Port;
  myLoRa.DIO0_port = dio_lora_pb2_GPIO_Port;
  myLoRa.DIO0_pin = dio_lora_pb2_Pin;
  myLoRa.hSPIx = &hspi2;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // start adc conversion through interrupt (air quality)
  HAL_ADC_Start_IT(&hadc1);

  // check lora
  lora_connect = LoRa_init(&myLoRa);

  // CHECK LORA STATUS
  if (lora_connect == LORA_OK)
  {
    sniprintf(buffer, BUFF_SIZE, "Lora connected...status= %d\n", lora_connect);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);
  }
  else
    while (1)
    {
      sniprintf(buffer, BUFF_SIZE, "\n LORA NOT CONNECTED");
      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
      HAL_Delay(1500);
    }

  // super loop
  //    update readings & clear buffer
  //     NODE_TRANSMIT_
  //                  tx stuff
  //                  strt timer 1 sec & rx=1
  //     RX
  //          loop(RX)
  //                receive stuff
  //          start timer 4 sec
  //          TX=0,RX=0

  while (1)
  {

    /// non tx rx stufff
    if (ADC_CONVERTED)
    {
      ADC_CONVERTED = 0;
      adc_value = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Start_IT(&hadc1);
      snprintf(buffer, BUFF_SIZE, "\nADC conversion done  %d", adc_value);
//      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    }
    airquality_input = HAL_GPIO_ReadPin(digital_sensor_in_GPIO_Port, digital_sensor_in_Pin);
    if (!airquality_input) // sensor has inverse digital logic returns 0 on bad air quality
    {
      snprintf(buffer, BUFF_SIZE, "\nBAD Airquality %d", airquality_input);
//      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    }
    else
    {
      snprintf(buffer, BUFF_SIZE, "\nGood Airquality %d", airquality_input);
//      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    }

    /// end of  non tx rx stufff



    if (NODE_TRANSMIT)
    {

    	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
#ifdef TRANSMIT
      msg_snd.ID = SENDTO_ID; // TODO
      msg_snd.digital = airquality_input ? 0x01 : 0x00;
      msg_snd.analog_converted = adc_value;
      msg_snd.future[3]=HAL_GPIO_ReadPin(digital_actu_out1_GPIO_Port, digital_actu_out1_Pin) <<1 | HAL_GPIO_ReadPin(digital_actu_out2_GPIO_Port, digital_actu_out2_Pin);



      message1_addData(&msg_snd, send_data, MESSAGESIZE);
      status = LoRa_transmit(&myLoRa, send_data, MESSAGESIZE, 50);
      if (status == 1)
      {
        snprintf(buffer, BUFF_SIZE, "\nTransmitted the message packets :");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        HAL_UART_Transmit(&huart2, (uint8_t *)send_data, MESSAGESIZE, 100);
      }
#endif
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_TIM_Base_Start_IT(&htim2); // start 1 second timer
      NODE_RECEIVE=1;
      NODE_TRANSMIT=0;
    }


    if (NODE_RECEIVE)
    {
     HAL_GPIO_WritePin(LD3_RX_MODE_GPIO_Port, LD3_RX_MODE_Pin, GPIO_PIN_SET);
#ifdef RECEIVE
      LoRa_startReceiving(&myLoRa);
      while (NODE_RECEIVE){
        //	 	if(LORA_RECEIVED_SOMETHING==1)
        packet_size = LoRa_receive(&myLoRa, received_data, BUFF_SIZE);

        // use the recived data for something
         if (packet_size == 12){
           snprintf(buffer, BUFF_SIZE, "\nReceived %d bytes", packet_size);
           HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
           message1_getData(&msg_rcv, received_data, packet_size);
           snprintf(buffer, BUFF_SIZE, "\nID=%c", msg_rcv.ID);
           HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);


           // if(msg_rcv.ID==NODEID)
           if (msg_rcv.digital & DIGITAL_OUT1)
             HAL_GPIO_WritePin(digital_actu_out1_GPIO_Port, digital_actu_out1_Pin, GPIO_PIN_SET);
           else
             HAL_GPIO_WritePin(digital_actu_out1_GPIO_Port, digital_actu_out1_Pin, GPIO_PIN_RESET);

           if (msg_rcv.digital & DIGITAL_OUT2)
             HAL_GPIO_WritePin(digital_actu_out2_GPIO_Port, digital_actu_out2_Pin, GPIO_PIN_SET);
           else
             HAL_GPIO_WritePin(digital_actu_out2_GPIO_Port, digital_actu_out2_Pin, GPIO_PIN_RESET);

         }
         packet_size = 0;
         HAL_Delay(200);
      }
#endif
  	HAL_GPIO_WritePin(LD3_RX_MODE_GPIO_Port, LD3_RX_MODE_Pin, GPIO_PIN_RESET);
    LoRa_gotoMode(&myLoRa, STNBY_MODE);
    NODE_TRANSMIT=0;
    HAL_TIM_Base_Start_IT(&htim5); // start 4 second timer
    }

  }
  //

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 21000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (rcvtime*4000)-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32767;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 21000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = (sleeptime*4000)-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LD3_RX_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(cs_lora_pb0_GPIO_Port, cs_lora_pb0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(reset_lora_pb1_GPIO_Port, reset_lora_pb1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, digital_actu_out1_Pin|digital_actu_out2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_RX_MODE_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_RX_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : cs_lora_pb0_Pin reset_lora_pb1_Pin */
  GPIO_InitStruct.Pin = cs_lora_pb0_Pin|reset_lora_pb1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : dio_lora_pb2_Pin */
  GPIO_InitStruct.Pin = dio_lora_pb2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(dio_lora_pb2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : digital_sensor_in_Pin */
  GPIO_InitStruct.Pin = digital_sensor_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(digital_sensor_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : digital_actu_out1_Pin digital_actu_out2_Pin */
  GPIO_InitStruct.Pin = digital_actu_out1_Pin|digital_actu_out2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
