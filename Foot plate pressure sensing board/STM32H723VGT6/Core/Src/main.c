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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "w25q64.h"
#include <stdio.h>
#include "bsp_fdcan.h"
//#include "os_threads.h"
#include "dwt_stm32_delay.h"
#include "encoder.h"
//#include "lktech_mg_motor.h"
#include "api.h"
#include "lcd.h"
//#include "lcdfont.h"
#include "pic.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "bsp_buzzer.h"
#include "bsp_user_key.h"
#include "vofa.h"
#include "imu.h"
#include "ADS1115.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

OSPI_HandleTypeDef hospi2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart9;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LcdTask */
osThreadId_t LcdTaskHandle;
uint32_t LcdTaskBuffer[ 128 ];
osStaticThreadDef_t LcdTaskControlBlock;
const osThreadAttr_t LcdTask_attributes = {
  .name = "LcdTask",
  .cb_mem = &LcdTaskControlBlock,
  .cb_size = sizeof(LcdTaskControlBlock),
  .stack_mem = &LcdTaskBuffer[0],
  .stack_size = sizeof(LcdTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
uint32_t ImuTaskBuffer[ 128 ];
osStaticThreadDef_t ImuTaskControlBlock;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .cb_mem = &ImuTaskControlBlock,
  .cb_size = sizeof(ImuTaskControlBlock),
  .stack_mem = &ImuTaskBuffer[0],
  .stack_size = sizeof(ImuTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SystemStatusTas */
osThreadId_t SystemStatusTasHandle;
uint32_t SystemStatusTasBuffer[ 128 ];
osStaticThreadDef_t SystemStatusTasControlBlock;
const osThreadAttr_t SystemStatusTas_attributes = {
  .name = "SystemStatusTas",
  .cb_mem = &SystemStatusTasControlBlock,
  .cb_size = sizeof(SystemStatusTasControlBlock),
  .stack_mem = &SystemStatusTasBuffer[0],
  .stack_size = sizeof(SystemStatusTasBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
uint32_t MotorControlTasBuffer[ 128 ];
osStaticThreadDef_t MotorControlTasControlBlock;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .cb_mem = &MotorControlTasControlBlock,
  .cb_size = sizeof(MotorControlTasControlBlock),
  .stack_mem = &MotorControlTasBuffer[0],
  .stack_size = sizeof(MotorControlTasBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for APITask */
osThreadId_t APITaskHandle;
uint32_t APITaskBuffer[ 128 ];
osStaticThreadDef_t APITaskControlBlock;
const osThreadAttr_t APITask_attributes = {
  .name = "APITask",
  .cb_mem = &APITaskControlBlock,
  .cb_size = sizeof(APITaskControlBlock),
  .stack_mem = &APITaskBuffer[0],
  .stack_size = sizeof(APITaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
uint32_t ADCTaskBuffer[ 128 ];
osStaticThreadDef_t ADCTaskControlBlock;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .cb_mem = &ADCTaskControlBlock,
  .cb_size = sizeof(ADCTaskControlBlock),
  .stack_mem = &ADCTaskBuffer[0],
  .stack_size = sizeof(ADCTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
uint32_t EncoderTaskBuffer[ 128 ];
osStaticThreadDef_t EncoderTaskControlBlock;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .cb_mem = &EncoderTaskControlBlock,
  .cb_size = sizeof(EncoderTaskControlBlock),
  .stack_mem = &EncoderTaskBuffer[0],
  .stack_size = sizeof(EncoderTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
uint32_t KeyTaskBuffer[ 128 ];
osStaticThreadDef_t KeyTaskControlBlock;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .cb_mem = &KeyTaskControlBlock,
  .cb_size = sizeof(KeyTaskControlBlock),
  .stack_mem = &KeyTaskBuffer[0],
  .stack_size = sizeof(KeyTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

//USBD_HandleTypeDef *usbpdev;	//JZP
//LKTECH_MG_Handle hMotorHip, hMotorKnee;
//ADS1115_HandleTypeDef hadc1115;

int iled;
int iledcount;
float vbus=0;
#define W25Qxx_NumByteToTest   	32*1024					// Test data length

int32_t OSPI_Status ; 		 //calibrate byte

uint8_t  W25Qxx_WriteBuffer[W25Qxx_NumByteToTest];		//write block
uint8_t  W25Qxx_ReadBuffer[W25Qxx_NumByteToTest];		//read block
uint16_t adc_val[2];
extern float controlSigCount, angleFeedbackCount;
CE100EncoderHandle hEncoder1, hEncoder2, hEncoder3;
uint8_t encoder_read_stage;
uint32_t encoder_crc_error_counts;
extern TIM_HandleTypeDef htim12;

uint32_t can1_tx_freelevel = 0;

uint8_t rxfifo0detected = 0;
uint8_t rxfifo1detected = 0;

uint8_t ifStartReadMotorAngleHip, ifStartReadMotorAngleKnee;
uint8_t ifRequestEnableHip, ifRequestEnableKnee;
uint8_t ifRequestDisableHip, ifRequestDisableKnee;

float controlSigCount = 0.0f, angleFeedbackCount = 0.0f;
float cannotReadAngleCount = 0.0f;

uint8_t ifMotorCANOnline = 0;
uint8_t ifPCCANOnline = 0;
uint8_t ifPreSyncSignal = 0;
uint32_t shortBeepTimestamp = 0;
uint32_t longBeepTimmestamp = 0;
uint8_t shortBeepCount = 0;
uint32_t lastCANMsgRxFrPC = 0;
uint32_t lastCANMsgRxFrHipMotor = 0;
uint32_t lastCANMsgRxFrKneeMotor = 0;
uint32_t aligned_time_offset = 0;
union UInt32UInt8 aligned_time;
union FloatUInt8 aligned_time_f;
uint32_t last_led_blink_time;
int count=1;
uint32_t Force_Left_Toe,Force_Left_Heel,Force_Right_Toe,Force_Right_Heel;
LKTECH_MG_Handle hMotorHip, hMotorKnee;
//extern volatile uint8_t dma_complete = 0;
float adc_results[4]; // Stores adc results for all 4 channels
uint8_t devicecounti2c=0;
uint8_t devicei2c[5];
uint8_t i2caddress=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM12_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI6_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_OCTOSPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART9_Init(void);
void StartDefaultTask(void *argument);
void LcdTask_Entry(void *argument);
void ImuTask_Entry(void *argument);
void SystemStatusTaskFunc(void *argument);
void MotorControlTaskFunc(void *argument);
void APITaskFunc(void *argument);
void ADCTaskFunc(void *argument);
void EncoderTaskFunc(void *argument);
void KeyTask_Entry(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t OSPI_W25Qxx_Test(void)		//Flash��д����
{
    uint32_t i = 0X8000;	// ��������
    uint32_t W25Qxx_TestAddr  =	0	;							// ���Ե�ַ	
    uint32_t ExecutionTime_Begin;		// ��ʼʱ��
    uint32_t ExecutionTime_End;		// ����ʱ��
    uint32_t ExecutionTime;				// ִ��ʱ��	
    float    ExecutionSpeed;			// ִ���ٶ�
	char buffer[128];

// ���� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
    
     LCD_Init();//LCD��ʼ��
	//HAL_Delay(100);
    LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
   
    ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
    OSPI_Status 			= OSPI_W25Qxx_BlockErase_32K(W25Qxx_TestAddr);	// ����32K�ֽ�
    ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
    
    ExecutionTime = ExecutionTime_End - ExecutionTime_Begin; // �������ʱ�䣬��λms
    
    if( OSPI_Status == OSPI_W25Qxx_OK )
    {
        sprintf (buffer,"W25Q64 erase succeed, time: %d ms",ExecutionTime);		
			LCD_ShowString(10,10,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
    }
    else
    {
        sprintf (buffer,"erase error!!!!!  ERROR CODE:%d",OSPI_Status);
			LCD_ShowString(10,30,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
        while (1);
    }   
    
// д�� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    for(i=0;i<W25Qxx_NumByteToTest;i++)  //�Ƚ�����д������
    {
        W25Qxx_WriteBuffer[i] = i;
    }
    ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
    OSPI_Status				= OSPI_W25Qxx_WriteBuffer(W25Qxx_WriteBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest); // д������
    ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
    
    ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 		// �������ʱ�䣬��λms
    ExecutionSpeed = (float)W25Qxx_NumByteToTest / ExecutionTime ; // ����д���ٶȣ���λ KB/S
    if( OSPI_Status == OSPI_W25Qxx_OK )
    {
        sprintf (buffer,"write succeed, data size: %d KB, time: %d ms, speed: %.2f KB/s",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
			LCD_ShowString(10,50,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
    }
    else
    {
        sprintf (buffer,"write error!!!!!  error code: %d",OSPI_Status);
			LCD_ShowString(10,70,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
        while (1);
    }	
    
// ��ȡ	>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 

    
    ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms	
    OSPI_Status				= OSPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// ��ȡ����
    ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
    
    ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 					// �������ʱ�䣬��λms
    ExecutionSpeed = (float)W25Qxx_NumByteToTest/1024/1024 / ExecutionTime*1000 ; 	// �����ȡ�ٶȣ���λ MB/S 
    
    if( OSPI_Status == OSPI_W25Qxx_OK )
    {
        sprintf (buffer,"read succeed, size: %d KB, time: %d ms, speed: %.2f MB/s",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
			LCD_ShowString(10,90,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
    }
    else
    {
        sprintf (buffer,"read error!!!!!  error code:%d",OSPI_Status);
			LCD_ShowString(10,110,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
        while (1);
    }   
// ����У�� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
    
    for(i=0;i<W25Qxx_NumByteToTest;i++)	//��֤�����������Ƿ����д�������
    {
        if( W25Qxx_WriteBuffer[i] != W25Qxx_ReadBuffer[i] )	//������ݲ���ȣ��򷵻�0	
        {
            sprintf (buffer,"data check error!!!!!pos: %d",i);	
			LCD_ShowString(10,130,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
            while(1);
        }
    }   
    sprintf (buffer,"check pass!!!!!"); 
			LCD_ShowString(10,150,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
    
// ��ȡ��ƬFlash�����ݣ����Բ����ٶ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    //printf ("\r\n*****************************************************************************************************\r\n");		
    //printf ("\r\nIn the above test, the data read is relatively small and takes a short time. In addition, the minimum unit of measurement is ms, and the calculated reading speed has a large error.\r\n");		
    //printf ("\r\nNext, read the entire flash data to test the speed. The speed error obtained in this way is relatively small.\r\n");		
    sprintf (buffer,"read start>>>>\r\n");		
			LCD_ShowString(10,170,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
    ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms		
    
    for(i=0;i<W25Qxx_FlashSize/(W25Qxx_NumByteToTest);i++)	// ÿ�ζ�ȡ W25Qxx_NumByteToTest �ֽڵ�����
    {
        OSPI_Status     = OSPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// ��ȡ����
        W25Qxx_TestAddr = W25Qxx_TestAddr + W25Qxx_NumByteToTest;		
    }
    ExecutionTime_End   = HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
    
    ExecutionTime       = ExecutionTime_End - ExecutionTime_Begin; 								// �������ʱ�䣬��λms
    ExecutionSpeed      = (float)W25Qxx_FlashSize/1024/1024 / ExecutionTime*1000  ; 	// �����ȡ�ٶȣ���λ MB/S 

    if( OSPI_Status == OSPI_W25Qxx_OK )
    {
        sprintf (buffer,"\r\nread succeed, size: %d MB, time: %d ms, speed: %.2f MB/s \r\n",W25Qxx_FlashSize/1024/1024,ExecutionTime,ExecutionSpeed);		
			LCD_ShowString(50,70,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
    }
    else
    {
        sprintf (buffer,"\r\nread error!!!!!  error code:%d\r\n",OSPI_Status);
			LCD_ShowString(50,70,(uint8_t *)buffer,RED,BLACK,12,0);
			HAL_Delay(100);
        while (1);
    }	
    
    return OSPI_W25Qxx_OK ;  // ����ͨ��				
    
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    if (!CE100ENCODER_ConfirmData(encoder_general_rx_buf))
    {
      if (encoder_general_rx_buf[0] == hEncoder1.id)
        CE100ENCODER_GetData(&hEncoder1, encoder_general_rx_buf);
      else if (encoder_general_rx_buf[0] == hEncoder2.id)
        CE100ENCODER_GetData(&hEncoder2, encoder_general_rx_buf);
      else if (encoder_general_rx_buf[0] == hEncoder3.id)
        CE100ENCODER_GetData(&hEncoder3, encoder_general_rx_buf);
    }
    else
      encoder_crc_error_counts++;
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
  //General codes
  FDCAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  //HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &temRxHeader, temRxData);
  HAL_FDCAN_GetRxMessage(hcan,FDCAN_RX_FIFO1, &temRxHeader, temRxData);
   
  //Application specific codes
  if (temRxHeader.Identifier == hMotorHip.canID)
  {
    LKTECH_MG_GetFeedback(&hMotorHip, &temRxHeader, temRxData);
    ifStartReadMotorAngleHip = 1;
		lastCANMsgRxFrHipMotor = HAL_GetTick();
  }
  else if (temRxHeader.Identifier == hMotorKnee.canID)
  {
    LKTECH_MG_GetFeedback(&hMotorKnee, &temRxHeader, temRxData);
    ifStartReadMotorAngleKnee = 1;
		lastCANMsgRxFrKneeMotor = HAL_GetTick();
  }
  rxfifo1detected++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
  //General codes
  FDCAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  //HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &temRxHeader, temRxData);
  HAL_FDCAN_GetRxMessage(hcan,FDCAN_RX_FIFO0, &temRxHeader, temRxData);
  //Application specific codes
  if (temRxHeader.Identifier == 0x110)//Velocity Control
  {
    motorTaskHip = MOTOR_TASK_VELOCITY_CONTROL;
    motorTaskKnee = MOTOR_TASK_VELOCITY_CONTROL;
    memcpy(&mHipSetVel.b8[0], &temRxData[0], 4);
    memcpy(&mKneeSetVel.b8[0], &temRxData[4], 4);
  }
  else if (temRxHeader.Identifier == 0x111)//Current Control
  {
    motorTaskHip = MOTOR_TASK_CURRENT_CONTROL;
    motorTaskKnee = MOTOR_TASK_CURRENT_CONTROL;
    memcpy(&mHipSetCur.b8[0], &temRxData[0], 4);
    memcpy(&mKneeSetCur.b8[0], &temRxData[4], 4);
  }
  else if (temRxHeader.Identifier == 0x112)//Position Control
  {
    motorTaskHip = MOTOR_TASK_POSITION_CONTROL;
    motorTaskKnee = MOTOR_TASK_POSITION_CONTROL;
    memcpy(&mHipSetPos.b8[0], &temRxData[0], 4);
    memcpy(&mKneeSetPos.b8[0], &temRxData[4], 4);
  }
  else if (temRxHeader.Identifier == 0x101)
  {
    ifRequestEnableHip = 1;
    motorTaskHip = MOTOR_TASK_ENABLE;
  }
  else if (temRxHeader.Identifier == 0x102)
  {
    ifRequestEnableKnee = 1;
    motorTaskKnee = MOTOR_TASK_ENABLE;
  }
  else if (temRxHeader.Identifier == 0x103)
  {
    ifRequestDisableHip = 1;
    motorTaskHip = MOTOR_TASK_DISABLE;
  }
  else if (temRxHeader.Identifier == 0x104)
  {
    ifRequestDisableKnee = 1;
    motorTaskKnee = MOTOR_TASK_DISABLE;
  }
  else if (temRxHeader.Identifier == 0x105)//Gear Ratio
  {
    hMotorHip.gearRatio = (float)temRxData[0];
    hMotorKnee.gearRatio = (float)temRxData[1];
  }
	lastCANMsgRxFrPC = HAL_GetTick();
  rxfifo0detected++;
}

void FDCAN_ConfigFilters(void)
{
    FDCAN_FilterTypeDef fdcan_filter;
    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = hMotorHip.canID;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(hMotorHip.hcan, &fdcan_filter);
	
    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = hMotorKnee.canID;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(hMotorKnee.hcan, &fdcan_filter);
	
    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x100;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = hMotorHip.canID;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x101;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x102;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x103;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x104;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x105;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x110;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x111;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x112;
    fdcan_filter.FilterID2 = 0x7FF;		//Mask
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
}

void Buzzer(uint8_t if_motor_online, uint8_t if_pc_online, uint8_t* if_pre_sync_signal, \
						uint32_t* short_beep_timestamp, uint32_t* long_beep_timestamp)
{
	uint32_t longBeepDuration = 2000;
	uint32_t longBeepSoundTime = 601;
	uint32_t shortBeepDuration = 200;
	if (if_pc_online && if_motor_online) return;
	if (!if_pc_online)	{
		longBeepDuration = 2000;
		longBeepSoundTime = 601;
	}
	else if (!if_motor_online)	{
		longBeepDuration = 1400;
		longBeepSoundTime = 401;
	}
	if ((HAL_GetTick() - *long_beep_timestamp) < longBeepSoundTime)		{
		if ((HAL_GetTick() - *short_beep_timestamp) < shortBeepDuration/2){
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		}
		else if ((HAL_GetTick() - *short_beep_timestamp) < shortBeepDuration){
 			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		}
		else			{
			*short_beep_timestamp = HAL_GetTick();
 			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
		}
	}
	else
	{
		if ((HAL_GetTick() - *long_beep_timestamp) < longBeepDuration)		{
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		}
		else			{
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			*short_beep_timestamp = HAL_GetTick();
			*long_beep_timestamp = HAL_GetTick();
		}
	}

/*	if (HAL_GPIO_ReadPin(INPUT1_GPIO_Port, INPUT1_Pin))
	{
		htim3.Instance->CCR1 = 50;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		//HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
		*if_pre_sync_signal = 1;
	}
	else if (*if_pre_sync_signal)
	{
		htim3.Instance->CCR1 = 230;
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		//HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
		*if_pre_sync_signal = 0;
	}
		*/
}
/*
uint32_t Get_Adc(ADC_HandleTypeDef hadcn,uint8_t ch)
{
	hadcn.Instance->PCSEL|=1<<ch;
	hadcn.Instance->SQR1&=~(0X1F<<6*1);
	hadcn.Instance->SQR1|=ch<<6*1;
	hadcn.Instance->CR|=1<<2;
	while(!(hadcn.Instance->ISR&1<<2));
	return hadcn.Instance->DR;
}*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  //SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

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
  MX_ADC1_Init();
  MX_TIM12_Init();
  MX_SPI1_Init();
  MX_SPI6_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_OCTOSPI2_Init();
  MX_I2C1_Init();
  MX_UART9_Init();
  /* USER CODE BEGIN 2 */
  API_Init();
	OSPI_W25Qxx_Init();     // ��ʼ��OSPI��W25Q64
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
  //HAL_FDCAN_ActivateNotification(&hfdcan1, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
  //HAL_FDCAN_ActivateNotification(&hfdcan2, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
	
  DWT_Delay_Init();

  LKTECH_MG_Init(&hMotorHip, &hfdcan1, 1, 6.0f, 1.93f);//3.63987f
  LKTECH_MG_Init(&hMotorKnee, &hfdcan1, 2, 6.0f, 1.107715f);
  FDCAN_ConfigFilters();
  CE100ENCODER_Init(&hEncoder1, &huart2, 1);
  CE100ENCODER_Init(&hEncoder2, &huart2, 2);
  CE100ENCODER_Init(&hEncoder3, &huart2, 3);
  API_Init();
  ifStartReadMotorAngleHip = 0;
  ifStartReadMotorAngleKnee = 0;
  ifRequestEnableHip = 0;
  ifRequestEnableKnee = 0;
  ifRequestDisableHip = 0;
  ifRequestDisableKnee = 0;
  
  HAL_Delay(1000);
  OSPI_W25Qxx_Test();     // Flash��д����
	
	//ADS1115_Init(&hadc1115,&hi2c1);
	//bsp_can_init();
	//LCD_Init();//LCD��ʼ��
	//LCD_Fill(0,0,LCD_W, LCD_H,BLACK);	
	//HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,1);	// ��ȡADC������ֵ

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LcdTask */
  LcdTaskHandle = osThreadNew(LcdTask_Entry, NULL, &LcdTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(ImuTask_Entry, NULL, &ImuTask_attributes);

  /* creation of SystemStatusTas */
  SystemStatusTasHandle = osThreadNew(SystemStatusTaskFunc, NULL, &SystemStatusTas_attributes);

  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(MotorControlTaskFunc, NULL, &MotorControlTas_attributes);

  /* creation of APITask */
  APITaskHandle = osThreadNew(APITaskFunc, NULL, &APITask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(ADCTaskFunc, NULL, &ADCTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(EncoderTaskFunc, NULL, &EncoderTask_attributes);

  /* creation of KeyTask */
  KeyTaskHandle = osThreadNew(KeyTask_Entry, NULL, &KeyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 24;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
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
  hfdcan2.Init.NominalPrescaler = 24;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0x406;
  hfdcan2.Init.StdFiltersNbr = 10;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 32;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 32;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 32;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 10;
  hfdcan2.Init.TxBuffersNbr = 10;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */

  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */

  /* USER CODE END OCTOSPI2_Init 1 */
  /* OCTOSPI2 parameter configuration*/
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 8;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi2.Init.DeviceSize = 23;
  hospi2.Init.ChipSelectHighTime = 1;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
  hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi2.Init.ClockPrescaler = 3;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi2.Init.ChipSelectBoundary = 0;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi2.Init.MaxTran = 0;
  hospi2.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */

  /* USER CODE END OCTOSPI2_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 230;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 199;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 249;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART9_Init(void)
{

  /* USER CODE BEGIN UART9_Init 0 */

  /* USER CODE END UART9_Init 0 */

  /* USER CODE BEGIN UART9_Init 1 */

  /* USER CODE END UART9_Init 1 */
  huart9.Instance = UART9;
  huart9.Init.BaudRate = 460800;
  huart9.Init.WordLength = UART_WORDLENGTH_8B;
  huart9.Init.StopBits = UART_STOPBITS_1;
  huart9.Init.Parity = UART_PARITY_NONE;
  huart9.Init.Mode = UART_MODE_TX_RX;
  huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart9.Init.OverSampling = UART_OVERSAMPLING_16;
  huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART9_Init 2 */

  /* USER CODE END UART9_Init 2 */

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
  huart1.Init.BaudRate = 921600;
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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin|POWER_24V_1_Pin|POWER_5V_Pin|ACC_CS_Pin
                          |GYRO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, WIFI_RST_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BLK_Pin|LCD_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : POWER_24V_2_Pin POWER_24V_1_Pin POWER_5V_Pin */
  GPIO_InitStruct.Pin = POWER_24V_2_Pin|POWER_24V_1_Pin|POWER_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_CS_Pin GYRO_CS_Pin */
  GPIO_InitStruct.Pin = ACC_CS_Pin|GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_INT_Pin GYRO_INT_Pin PE0 */
  GPIO_InitStruct.Pin = ACC_INT_Pin|GYRO_INT_Pin|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RST_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BLK_Pin LCD_RES_Pin */
  GPIO_InitStruct.Pin = LCD_BLK_Pin|LCD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT1_Pin */
  GPIO_InitStruct.Pin = INPUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin|POWER_24V_1_Pin|POWER_5V_Pin, GPIO_PIN_SET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */
__weak void LcdTask_Entry(void *argument)
{
  /* USER CODE BEGIN LcdTask_Entry */
  /* Infinite loop */
    //LCD_Init();//LCD��ʼ��
    LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
		//HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,1);	// ��ȡADC������ֵ
    
    /* Infinite loop */
    for(;;)
    {
        //adc_results[0]=Read_ADS1115_Channel(0);
				//LCD_ShowString(120, 72,(uint8_t *)"dmBot", BRRED, BLACK, 24, 0);
        //LCD_ShowChinese(84, 100, (uint8_t *)"����Ƽ�", WHITE, BLACK, 32, 0);
        //LCD_DrawLine(10, 0, 10,  280,WHITE);
        //LCD_DrawLine(270,0, 270, 280,WHITE);
				LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
        LCD_ShowIntNum(50, 140, adc_val[0], 5, WHITE, BLACK, 32);
				LCD_ShowIntNum(50, 180, adc_val[1], 5, WHITE, BLACK, 32);
        //LCD_ShowFloatNum(50, 180, (float)adc_val[1]*3.3/65536.0, 3, 2, WHITE, BLACK, 32);
				LCD_ShowFloatNum(10,20,adc_results[0],3,2,WHITE,BLACK,32);
				LCD_ShowFloatNum(120,20,adc_results[1],3,2,WHITE,BLACK,32);
				LCD_ShowFloatNum(10,60,adc_results[2],3,2,WHITE,BLACK,32);
				LCD_ShowFloatNum(120,60,adc_results[3],3,2,WHITE,BLACK,32);
        osDelay(100);
    }
  /* USER CODE END LcdTask_Entry */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void *argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_SystemStatusTaskFunc */
/**
* @brief Function implementing the SystemStatusTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SystemStatusTaskFunc */
void SystemStatusTaskFunc(void *argument)
{
  /* USER CODE BEGIN SystemStatusTaskFunc */
  /* Infinite loop */
	last_led_blink_time = HAL_GetTick();
	uint8_t rl=1,gl=1,bl=1;
  for(;;)
  {
		//if (((float)(HAL_GetTick() - last_led_blink_time)*1.0021004f) >= 100.0f)
		{
			WS2812_Ctrl(rl,gl,bl);
			rl=rl+2;
			gl=gl+4;
			bl=bl+6;
			last_led_blink_time = 0;
			osDelay(399);
			WS2812_Ctrl(0,0,0);
			osDelay(499);
			
			//HAL_GetTick();*/
		}

    osDelay(100);
  }
  /* USER CODE END SystemStatusTaskFunc */
}

/* USER CODE BEGIN Header_MotorControlTaskFunc */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTaskFunc */
void MotorControlTaskFunc(void *argument)
{
  /* USER CODE BEGIN MotorControlTaskFunc */
  /* Infinite loop */
	static	uint8_t i_i;
  for(;;)
  {
    ifStartReadMotorAngleHip = 0;
    ifStartReadMotorAngleKnee = 0;
		i_i=motorTaskHip;
		switch (i_i) {
			case MOTOR_TASK_VELOCITY_CONTROL:
				LETECH_MG_SpeedControl(&hMotorHip, mHipSetVel.f);
				break;
			case MOTOR_TASK_CURRENT_CONTROL:
				LETECH_MG_CurrentControl(&hMotorHip, mHipSetCur.f);
				break;
			case MOTOR_TASK_POSITION_CONTROL:
				LETECH_MG_PositionControl1MultiTurn(&hMotorHip, mHipSetPos.f);
				break;
			case MOTOR_TASK_ENABLE:
				if (ifRequestEnableHip) {
					LETECH_MG_Enable(&hMotorHip);
					ifRequestEnableHip = 0;
				}
				else if (motorTaskHip != MOTOR_TASK_OFF) LETECH_MG_ReadCondition2(&hMotorHip);
				break;
			case MOTOR_TASK_DISABLE:
				if (ifRequestDisableHip) {
					LETECH_MG_Shutdown(&hMotorHip);
					ifRequestDisableHip = 0;
				}
				else if (motorTaskHip != MOTOR_TASK_OFF) LETECH_MG_ReadCondition2(&hMotorHip);
				break;
			default:
					break;
			}
		i_i=motorTaskKnee;
		switch (i_i) {
			case MOTOR_TASK_VELOCITY_CONTROL:
				LETECH_MG_SpeedControl(&hMotorKnee, mKneeSetVel.f);
				break;
			case MOTOR_TASK_CURRENT_CONTROL:
				LETECH_MG_CurrentControl(&hMotorKnee, mKneeSetCur.f);
				break;
			case MOTOR_TASK_POSITION_CONTROL:
				LETECH_MG_PositionControl1MultiTurn(&hMotorKnee, mKneeSetPos.f);
				break;
			case MOTOR_TASK_ENABLE:
				if (ifRequestEnableHip) {
					LETECH_MG_Enable(&hMotorKnee);
					ifRequestEnableKnee = 0;
				}
				else LETECH_MG_ReadCondition2(&hMotorKnee);
				break;
			case MOTOR_TASK_DISABLE:
				if (ifRequestDisableHip) {
					LETECH_MG_Shutdown(&hMotorKnee);
					ifRequestDisableKnee = 0;
				}
				else LETECH_MG_ReadCondition2(&hMotorKnee);
				break;
			default:
					break;
			}
/*			
    if (motorTaskHip == MOTOR_TASK_VELOCITY_CONTROL)
      LETECH_MG_SpeedControl(&hMotorHip, mHipSetVel.f);
    else if (motorTaskHip == MOTOR_TASK_CURRENT_CONTROL)
      LETECH_MG_CurrentControl(&hMotorHip, mHipSetCur.f);
    else if (motorTaskHip == MOTOR_TASK_POSITION_CONTROL)
      LETECH_MG_PositionControl1MultiTurn(&hMotorHip, mHipSetPos.f);
    else if (motorTaskHip == MOTOR_TASK_ENABLE && ifRequestEnableHip)
    {
      LETECH_MG_Enable(&hMotorHip);
      ifRequestEnableHip = 0;
    }
    else if (motorTaskHip == MOTOR_TASK_DISABLE && ifRequestDisableHip)
    {
      LETECH_MG_Shutdown(&hMotorHip);
      ifRequestDisableHip = 0;
    }
    else if (motorTaskHip != MOTOR_TASK_OFF)
      LETECH_MG_ReadCondition2(&hMotorHip);
			
    
    
    
    if (motorTaskKnee == MOTOR_TASK_VELOCITY_CONTROL)
      LETECH_MG_SpeedControl(&hMotorKnee, mKneeSetVel.f);
    else if (motorTaskKnee == MOTOR_TASK_CURRENT_CONTROL)
      LETECH_MG_CurrentControl(&hMotorKnee, mKneeSetCur.f);
    else if (motorTaskKnee == MOTOR_TASK_POSITION_CONTROL)
      LETECH_MG_PositionControl1MultiTurn(&hMotorKnee, mKneeSetPos.f);
    else if (motorTaskKnee == MOTOR_TASK_ENABLE && ifRequestEnableKnee)
    {
      LETECH_MG_Enable(&hMotorKnee);
      ifRequestEnableKnee = 0;
    }
    else if (motorTaskKnee == MOTOR_TASK_DISABLE && ifRequestDisableKnee)
    {
      LETECH_MG_Shutdown(&hMotorKnee);
      ifRequestDisableKnee = 0;
    }
    else if (motorTaskKnee != MOTOR_TASK_OFF)
      LETECH_MG_ReadCondition2(&hMotorKnee);
*/    
    osDelay(2);
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 3)
    {
      LETECH_MG_ReadAngleSingleTurn(&hMotorHip);
      LETECH_MG_ReadAngleSingleTurn(&hMotorKnee);
    }
    else
      cannotReadAngleCount += 1.0f;
		
   
    osDelay(1);
  }


  /* USER CODE END MotorControlTaskFunc */
}

/* USER CODE BEGIN Header_APITaskFunc */
/**
* @brief Function implementing the APITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APITaskFunc */
void APITaskFunc(void *argument)
{
  /* USER CODE BEGIN APITaskFunc */
  /* Infinite loop */
	TIM12->CCR2=02;
  for(;;)
  {
		if ((HAL_GetTick() - lastCANMsgRxFrHipMotor > 1000) ||
				(HAL_GetTick() - lastCANMsgRxFrKneeMotor > 1000))
			ifMotorCANOnline = 0;
		else
			ifMotorCANOnline = 1;
		if (HAL_GetTick() - lastCANMsgRxFrPC > 2000)
			ifPCCANOnline = 0;
		else
			ifPCCANOnline = 1;
 		if (((float)(HAL_GetTick() - last_led_blink_time)*1.0021004f) >= 100.0f)
		{
			Buzzer(ifMotorCANOnline, ifPCCANOnline, &ifPreSyncSignal, &shortBeepTimestamp, &longBeepTimmestamp);
		}
    API_Host();
    osDelay(3);
  }
  /* USER CODE END APITaskFunc */
}

/* USER CODE BEGIN Header_ADCTaskFunc */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADCTaskFunc */
void ADCTaskFunc(void *argument)
{
  /* USER CODE BEGIN ADCTaskFunc */
  /* Infinite loop */
    
    // Start first conversion
  for(;;)
  {
        // Your main loop can now use hadc1115.lastVoltage[0..3]
        // without blocking for I2C transactions
		adc_results[0]=ADS1115_ReadVoltage(&hi2c1,3,ADS1115_PGA_4_096V);
    osDelay(1);
		adc_results[1]=ADS1115_ReadVoltage(&hi2c1,0,ADS1115_PGA_4_096V);
    osDelay(1);
		adc_results[2]=ADS1115_ReadVoltage(&hi2c1,1,ADS1115_PGA_4_096V);
    osDelay(1);
		adc_results[3]=ADS1115_ReadVoltage(&hi2c1,2,ADS1115_PGA_4_096V);
    osDelay(1);
		
  }
  /* USER CODE END ADCTaskFunc */
}

/* USER CODE BEGIN Header_EncoderTaskFunc */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderTaskFunc */
void EncoderTaskFunc(void *argument)
{
  /* USER CODE BEGIN EncoderTaskFunc */
  /* Infinite loop */
  encoder_read_stage = 1;
  encoder_crc_error_counts = 0;
  for(;;)
  {
    can1_tx_freelevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    if (encoder_read_stage == 1)
      CE100ENCODER_ReadRequest(&hEncoder1, encoder_general_rx_buf);
    else if (encoder_read_stage == 2)
      CE100ENCODER_ReadRequest(&hEncoder2, encoder_general_rx_buf);
    else if (encoder_read_stage == 3)
      CE100ENCODER_ReadRequest(&hEncoder3, encoder_general_rx_buf);
    
    encoder_read_stage++;
    if (encoder_read_stage == 4)
      encoder_read_stage = 1;
    osDelay(3);
  }
  /* USER CODE END EncoderTaskFunc */
}

/* USER CODE BEGIN Header_KeyTask_Entry */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask_Entry */
void KeyTask_Entry(void *argument)
{
  /* USER CODE BEGIN KeyTask_Entry */
	//set beep volumn 0 or 10%
  /* Infinite loop */
	uint8_t keyon=0;
    //BSP_Buzzer_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);
		TIM12->CCR2=2;
    //WS2812_Ctrl(0, 10, 0);
    /* Infinite loop */
    for(;;)
    {
        if (BSP_UserKey_Detect() == BUTTON_PRESSED)
        {
            //BSP_Buzzer_Toggle();
					if (keyon==0) {
						TIM12->CCR2=2;
						keyon=1;
					}
					else {
						TIM12->CCR2=0;
						keyon=0;
					}
        }
        vbus = (adc_val[1]*3.3f/65535);//*11.0f;
        osDelay(20);
    }
  /* USER CODE END KeyTask_Entry */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
