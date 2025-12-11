/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */
/* Formal WeWalk Code */
WeWalkUIHandle                      hUI;
uint32_t                            ui_scheduler_count = 0;
float                               ui_scheduler_time_check = 0.0f;
WeWalkUnilateralHandle              hWeWalkRight, hWeWalkLeft;
GaitPhaseDetectionHandle            hGaitPhaseDetectionRight, hGaitPhaseDetectionLeft;
WeWalkHandle                        hWeWalk;
uint32_t                            main_scheduler_count = 0;
float                               main_scheduler_time_check = 0.0f;
SerialProtocolEchoHandle            hSerialPC;
FDCAN_RxHeaderTypeDef               rightMotorRxHeader;
FDCAN_TxHeaderTypeDef               rightMotorTxHeader = 
{
  .Identifier = 0x140 + WEWALKMIDDLEWARE_RIGHT_MOTOR_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_8,
  .FDFormat = FDCAN_CLASSIC_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS
};

FDCAN_FilterTypeDef                 rightMotorRxFilter = 
{
	.IdType = FDCAN_STANDARD_ID,
	.FilterIndex = 0,
	.FilterType = FDCAN_FILTER_DUAL,
	.FilterID1 = 0x140 + WEWALKMIDDLEWARE_RIGHT_MOTOR_ID,
	.FilterID2 = 0x01,
	.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP,
	.RxBufferIndex = 0
};

FDCAN_FilterTypeDef                 rightThighIMURx1Filter = 
{
	.IdType = FDCAN_STANDARD_ID,
	.FilterIndex = 1,
	.FilterType = FDCAN_FILTER_DUAL,
	.FilterID1 = 10,
	.FilterID2 = 11,
	.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP,
	.RxBufferIndex = 0
};
FDCAN_FilterTypeDef                 rightThighIMURx2Filter = 
{
	.IdType = FDCAN_STANDARD_ID,
	.FilterIndex = 2,
	.FilterType = FDCAN_FILTER_DUAL,
	.FilterID1 = 12,
	.FilterID2 = 0,
	.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP,
	.RxBufferIndex = 0
};

FDCAN_FilterTypeDef                 rightShankIMURx1Filter = 
{
	.IdType = FDCAN_STANDARD_ID,
	.FilterIndex = 3,
	.FilterType = FDCAN_FILTER_DUAL,
	.FilterID1 = 13,
	.FilterID2 = 14,
	.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP,
	.RxBufferIndex = 0
};
FDCAN_FilterTypeDef                 rightShankIMURx2Filter = 
{
	.IdType = FDCAN_STANDARD_ID,
	.FilterIndex = 4,
	.FilterType = FDCAN_FILTER_DUAL,
	.FilterID1 = 15,
	.FilterID2 = 20,
	.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP,
	.RxBufferIndex = 0
};

LKTECH_MG_Handle                    hMotorLeft, hMotorRight;
uint32_t                            motor_scheduler_count = 0;
float                               motor_scheduler_time_check = 0.0f;

/*IMU*/
union Int16UInt8 accThigh[3], gyroThigh[3], accShank[3], gyroShank[3];
union FloatUInt8 angleThigh[3], angleShank[3];
float thighIMURxCount = 0.0f;
float shankIMURxCount = 0.0f;

//For new task schedulaer logic testing//
uint8_t wewalk_ui_task_trigger_flag = 0;
uint8_t wewalk_main_task_trigger_flag = 0;
uint8_t wewalk_main_task_datalog_trigger = 0;
const uint8_t wewalk_main_task_datalog_trigger_threshold = 1;
uint8_t motor_task_trigger_flag = 0;
///////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_QUADSPI_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //////////////////////////////////////
  /////////Communication and UI/////////
  //////////////////////////////////////
  hSerialPC = SERIALPROTOCOLECHO_Create(&huart4, &hdma_memtomem_dma2_stream0);
  SERIALPROTOCOLECHO_EnableCommunication_CircularDMAMode(&hSerialPC);
  hUI = WEWALKUI_Create();
  hUI.serialUI = SERIALPROTOCOLECHO_Create(&huart1, &hdma_memtomem_dma2_stream0);
  SERIALPROTOCOLECHO_EnableCommunication_CircularDMAMode(&hUI.serialUI);
  SERIALPROTOCOLECHO_SetNewDatalogSendLabelFunction(&hSerialPC, Datalog_SendLabel);
	SERIALPROTOCOLECHO_SetNewDatalogSlot(&hSerialPC, datalog_data);
  SERIALPROTOCOLECHO_SetNewDatalogSlotLength(&hSerialPC, 16);
  
  /////////////////////////////////////
  //////////////Actuators//////////////
  /////////////////////////////////////
  LKTECH_MG_Init(&hMotorRight, &hfdcan1, &rightMotorTxHeader, &rightMotorRxHeader, &rightMotorRxFilter, 1, 36.0f, 1.0f);
  //////////////////////////////////////
  /////////////////IMUs/////////////////
  //////////////////////////////////////
  HAL_FDCAN_ConfigFilter(&hfdcan1, &rightThighIMURx1Filter);
  HAL_FDCAN_ConfigFilter(&hfdcan1, &rightThighIMURx2Filter);
  HAL_FDCAN_ConfigFilter(&hfdcan1, &rightShankIMURx1Filter);
  HAL_FDCAN_ConfigFilter(&hfdcan1, &rightShankIMURx2Filter);
//  HAL_FDCAN_ConfigFilter(&hfdcan1, &rightFootPlateSensorRxFilter);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
  /////////////////////////////////////
  //////////////Scheduler//////////////
  /////////////////////////////////////
  hGaitPhaseDetectionRight = WEWALK_GaitPhaseDetectionCreate(0.00032f, 1.0f);
  hWeWalkRight = WEWALKUNILATERAL_Create(&htim8, TIM_CHANNEL_1, &hGaitPhaseDetectionRight);
  hWeWalk = WEWALK_Create(&hUI, &hSerialPC, &hWeWalkLeft, &hWeWalkRight, WEWALK_APPLICATION_TYPE_UNILATERAL_LEG_RIGHT);
  
  //////////////////////////////////////
  ////////Flash and Load Settings///////
  //////////////////////////////////////
  W25Q_Init();
  WEWALKFLASH_LoadSettings(&hWeWalk);
  //////////////////////////////////////
  //////////////Start Tasks/////////////
  //////////////////////////////////////
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  WEWALKUI_SetLEDLogoPWMCCR(hWeWalk.right, hWeWalk.right->setting.ledLogoBrightnessPWMCCR);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (hWeWalk.ifSaveSettingRequired)
    {
      WEWALKFLASH_SaveSettings(&hWeWalk);
      hWeWalk.ifSaveSettingRequired = 0;
      HAL_Delay(1000);
    }

    if (wewalk_ui_task_trigger_flag)
    {
      wewalk_ui_task_trigger_flag = 0;
      SERIALPROTOCOLECHO_ReceiveCargo(&hUI.serialUI);
      WEWALKUI_Host(&hWeWalk);
      ui_scheduler_count += 2;
      ui_scheduler_time_check = (float)HAL_GetTick() - (float)ui_scheduler_count;
    }
    
    if (wewalk_main_task_trigger_flag)
    {
      wewalk_main_task_trigger_flag = 0;
      
      SERIALPROTOCOLECHO_ReceiveCargo(&hSerialPC);
      SERIALPROTOCOLECHO_MasterHost(&hSerialPC);
      WEWALK_Main(&hWeWalk);
      main_scheduler_count += 10;
      main_scheduler_time_check = (float)HAL_GetTick() - (float)main_scheduler_count;
    }
    
    if (wewalk_main_task_datalog_trigger == wewalk_main_task_datalog_trigger_threshold)
    {
      datalog_data[0].f = hWeWalkRight.data.accThigh[0].f;
      datalog_data[1].f = hWeWalkRight.data.accThigh[1].f;
      datalog_data[2].f = hWeWalkRight.data.accThigh[2].f;
      datalog_data[3].f = hWeWalkRight.data.gyroThigh[0].f;
      datalog_data[4].f = hWeWalkRight.data.gyroThigh[1].f;
      datalog_data[5].f = hWeWalkRight.data.gyroThigh[2].f;
      datalog_data[6].f = hWeWalkRight.data.accShank[0].f;
      datalog_data[7].f = hWeWalkRight.data.accShank[1].f;
      datalog_data[8].f = hWeWalkRight.data.accShank[2].f;
      datalog_data[9].f = hWeWalkRight.data.gyroShank[0].f;
      datalog_data[10].f = hWeWalkRight.data.gyroShank[1].f;
      datalog_data[11].f = hWeWalkRight.data.gyroShank[2].f;
      datalog_data[12].f = hWeWalkRight.data.motorPos.f;
      datalog_data[13].f = hWeWalkRight.data.motorVel.f;
      datalog_data[14].f = hWeWalkRight.data.motorTorque.f;
      datalog_data[15].f = hWeWalkRight.data.gaitPhaseIndicator.f;
      hWeWalk.serialPC->ifNewDatalogPiece2Send = 1;
      wewalk_main_task_datalog_trigger = 0;
    }
    
    if (motor_task_trigger_flag)
    {
      motor_task_trigger_flag = 0;
      WEWALKMIDDLEWARE_MotorControlManager(&hWeWalk, &hMotorLeft, &hMotorRight);
    }
    
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 76;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 20;
  PeriphClkInitStruct.PLL2.PLL2N = 240;
  PeriphClkInitStruct.PLL2.PLL2P = 3;
  PeriphClkInitStruct.PLL2.PLL2Q = 3;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 3;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 5;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 5;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 5;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 5;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 2;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 2;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 6;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 3;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 3;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 2;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 2;
  hfdcan2.Init.TxBuffersNbr = 2;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 2;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 50;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 24-1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47500-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47500-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim5.Init.Prescaler = 4750-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2375-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1382400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE2 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef temp_rxheader;
  uint8_t temp_can_rx[8];
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &temp_rxheader, temp_can_rx);
  if (temp_rxheader.Identifier == hMotorRight.canID)
  {
    LKTECH_MG_GetFeedback(&hMotorRight, temp_can_rx);
    motor_scheduler_count += 1;
    motor_scheduler_time_check = (float)HAL_GetTick() - (float)motor_scheduler_count;
    hWeWalk.right->data.motorCur.f = hMotorRight.current.f;
    hWeWalk.right->data.motorTorque.f = hMotorRight.current.f * hWeWalk.right->setting.motorKt.f;
    hWeWalk.right->data.motorPosRaw.f = hMotorRight.angle.f;
    hWeWalk.right->data.motorPos.f = hWeWalk.right->data.motorPosRaw.f - hWeWalk.right->setting.motorKneeJointAngleOffset.f;
    hWeWalk.right->data.motorPosCos.f = cosf(hWeWalk.right->data.motorPos.f * deg2rad);
    hWeWalk.right->data.motorPos.f = acosf(hWeWalk.right->data.motorPosCos.f) * rad2deg;
    hWeWalk.right->data.motorVel.f = -hMotorRight.speedDeg.f;
  }

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfdcan);
  UNUSED(RxFifo0ITs);

  /* NOTE: This function Should not be modified, when the callback is needed,
            the HAL_FDCAN_RxFifo0Callback could be implemented in the user file
   */
}

void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_RxHeaderTypeDef temp_rxheader;
  uint8_t temp_can_rx[8];
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_BUFFER0, &temp_rxheader, temp_can_rx);
  UNUSED(hfdcan);

}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  FDCAN_RxHeaderTypeDef temp_rxheader;
  uint8_t temp_can_rx[8];
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &temp_rxheader, temp_can_rx);
  if (temp_rxheader.Identifier == 10)
  {
    memcpy(&accThigh[0].b8[0], &temp_can_rx[0], 6);
    memcpy(&gyroThigh[0].b8, &temp_can_rx[6], 2);
    hWeWalkRight.data.accThigh[0].f = ((float)accThigh[0].b16) * 0.0008974358974f;
    hWeWalkRight.data.accThigh[1].f = ((float)accThigh[1].b16) * 0.0008974358974f;
    hWeWalkRight.data.accThigh[2].f = ((float)accThigh[2].b16) * 0.0008974358974f;
    hWeWalkRight.data.gyroThigh[0].f = ((float)gyroThigh[0].b16) * 0.00106526443603169529841533860381f;
    thighIMURxCount += 0.00166666666f;
  }
  else if (temp_rxheader.Identifier == 11)
  {
    memcpy(&gyroThigh[1].b8[0], &temp_can_rx[0], 4);
    memcpy(&angleThigh[0].b8[0], &temp_can_rx[4], 4);
    hWeWalkRight.data.gyroThigh[1].f = ((float)gyroThigh[1].b16) * 0.00106526443603169529841533860381f;
    hWeWalkRight.data.gyroThigh[2].f = ((float)gyroThigh[2].b16) * 0.00106526443603169529841533860381f;
    hWeWalkRight.data.angleThigh[0].f = angleThigh[0].f;
    thighIMURxCount += 0.00166666666f;
  }
  else if (temp_rxheader.Identifier == 12)
  {
    memcpy(&angleThigh[1].b8[0], &temp_can_rx[0], 8);
    hWeWalkRight.data.angleThigh[1].f = angleThigh[1].f;
    hWeWalkRight.data.angleThigh[2].f = angleThigh[2].f;
    thighIMURxCount += 0.00166666666f;
  }
  
  if (temp_rxheader.Identifier == 13)
  {
    memcpy(&accShank[0].b8[0], &temp_can_rx[0], 6);
    memcpy(&gyroShank[0].b8, &temp_can_rx[6], 2);
    hWeWalkRight.data.accShank[0].f = ((float)accShank[0].b16) * 0.0008974358974f;
    hWeWalkRight.data.accShank[1].f = ((float)accShank[1].b16) * 0.0008974358974f;
    hWeWalkRight.data.accShank[2].f = ((float)accShank[2].b16) * 0.0008974358974f;
    hWeWalkRight.data.gyroShank[0].f = ((float)gyroShank[0].b16) * 0.00106526443603169529841533860381f;
    shankIMURxCount += 1.0f/600.0f;
  }
  else if (temp_rxheader.Identifier == 14)
  {
    memcpy(&gyroShank[1].b8[0], &temp_can_rx[0], 4);
    memcpy(&angleShank[0].b8[0], &temp_can_rx[4], 4);
    hWeWalkRight.data.gyroShank[1].f = ((float)gyroShank[1].b16) * 0.00106526443603169529841533860381f;
    hWeWalkRight.data.gyroShank[2].f = ((float)gyroShank[2].b16) * 0.00106526443603169529841533860381f;
    hWeWalkRight.data.angleShank[0].f = angleShank[0].f;
    shankIMURxCount += 1.0f/600.0f;
  }
  else if (temp_rxheader.Identifier == 15)
  {
    memcpy(&angleShank[1].b8[0], &temp_can_rx[0], 8);
    hWeWalkRight.data.angleShank[1].f = angleShank[1].f;
    hWeWalkRight.data.angleShank[2].f = angleShank[2].f;
    shankIMURxCount += 1.0f/600.0f;
  }
  else if (temp_rxheader.Identifier == 20)//Right foot plate sensor
  {
    memcpy(&hWeWalkRight.data.forceSensorToe.b8[0], &temp_can_rx[0], 4);
    memcpy(&hWeWalkRight.data.forceSensorHeel.b8[0], &temp_can_rx[4], 4);
  }

  UNUSED(hfdcan);
  UNUSED(RxFifo1ITs);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)//WeWalk UI frequency: 237.5M / 47500 / 10 = 500 Hz (0.002 ms)
    wewalk_ui_task_trigger_flag = 1;
  else if (htim == &htim4)//WeWalk main frequency: 237.5M / 47500 / 50 = 100 Hz
  {
    wewalk_main_task_trigger_flag = 1;
    wewalk_main_task_datalog_trigger++;
  }
  else if (htim == &htim5)//Motor Control frequency: 237.5M / 4750 / 50 = 1000 Hz
    motor_task_trigger_flag = 1;
	UNUSED(htim);
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
  __disable_irq();
  while (1)
  {
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
