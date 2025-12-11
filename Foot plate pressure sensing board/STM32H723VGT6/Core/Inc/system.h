#ifndef SYSTEM_H
#define SYSTEM_H
#include "stm32h7xx_hal.h"

#define ONBOARD_LED1_Pin GPIO_PIN_6
#define ONBOARD_LED1_GPIO_Port GPIOC
#define ONBOARD_LED2_Pin GPIO_PIN_7
#define ONBOARD_LED2_GPIO_Port GPIOC
#define ONBOARD_LED3_Pin GPIO_PIN_8
#define ONBOARD_LED3_GPIO_Port GPIOC
#define ONBOARD_LED4_Pin GPIO_PIN_9
#define ONBOARD_LED4_GPIO_Port GPIOC
#define RS485_TX_Pin GPIO_PIN_5
#define RS485_TX_GPIO_Port GPIOD
#define RS485_RX_Pin GPIO_PIN_6
#define RS485_RX_GPIO_Port GPIOD
#define RS485_DE_Pin GPIO_PIN_7
#define RS485_DE_GPIO_Port GPIOD
#define INPUT1_Pin GPIO_PIN_3
#define INPUT1_GPIO_Port GPIOB
#define INPUT3_Pin GPIO_PIN_5
#define INPUT3_GPIO_Port GPIOB
#define INPUT4_Pin GPIO_PIN_6
#define INPUT4_GPIO_Port GPIOB
#define INPUT5_Pin GPIO_PIN_7
#define INPUT5_GPIO_Port GPIOB
#define INPUT6_Pin GPIO_PIN_8
#define INPUT6_GPIO_Port GPIOB



void SystemClock_Config(void);
void MX_DMA_Init(void);
void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void MX_GPIO_Init(void);
void SYSTEM_Init(void);
void MX_USART2_UART_Init(void);
void CAN_ConfigureFilters(void);
void MX_TIM3_Init(void);


extern FDCAN_HandleTypeDef hcan1;
extern FDCAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
#endif
