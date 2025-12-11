#ifndef ENCODER_H
#define ENCODER_H
#include "stm32h7xx_hal.h"
#include "main.h"

#define CE100_COMMAND_GET_VERSION             0x0A
#define CE100_COMMAND_READ_SYS_PARAM          0x0C
#define CE100_COMMAND_CONFIG_SYS_PARAM        0x0D
#define CE100_COMMAND_SINGLE_TURN_ZEROING     0x21
#define CE100_COMMAND_MULTI_TURN_ZEROING      0x22
#define CE100_COMMAND_READ_DATA               0x2F


uint16_t crc16(uint8_t *buf,unsigned char Len);

typedef struct
{
  UART_HandleTypeDef*     huart;
  uint8_t                 id;
  uint8_t                 index;
  uint8_t                 txBuf[10];
  uint8_t                 rxBuf[20];
  union FloatUInt8        angle;
  union UInt16UInt8       angleRaw;
  uint8_t                 direction;
}CE100EncoderHandle;

void CE100ENCODER_Init(CE100EncoderHandle* hencoder, UART_HandleTypeDef* huart, uint8_t id);
void CE100ENCODER_Zeroing(CE100EncoderHandle* hencoder);
void CE100ENCODER_ReadRequest(CE100EncoderHandle* hencoder, uint8_t* rx_buf);
void CE100ENCODER_GetData(CE100EncoderHandle* hencoder, uint8_t* rx_buf);
uint8_t CE100ENCODER_ConfirmData(uint8_t* rx);

extern uint8_t encoder_general_rx_buf[19];

#endif
