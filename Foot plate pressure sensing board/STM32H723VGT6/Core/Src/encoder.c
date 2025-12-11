#include "encoder.h"
#include "system.h"

uint8_t encoder_general_rx_buf[19];


uint16_t crc16(uint8_t *buf,unsigned char Len)
{
  unsigned int temp = 0xffff;
  unsigned char n,i;
  
 for( n = 0; n < Len; n++)          
 {       
     temp = buf[n] ^ temp;
     for( i = 0;i < 8;i++)            
   { 
        if(temp & 0x01)
    {
             temp = temp >> 1;
             temp = temp ^ 0xa001;
        }   
        else
    {
             temp = temp >> 1;
        }   
     }   
  }   
 return temp;                          
}

void CE100ENCODER_Zeroing(CE100EncoderHandle* hencoder)
{
  
}

void CE100ENCODER_Init(CE100EncoderHandle* hencoder, UART_HandleTypeDef* huart, uint8_t id)
{
  hencoder->huart = huart;
  hencoder->id = id;
}

void CE100ENCODER_ReadRequest(CE100EncoderHandle* hencoder, uint8_t* rx_buf)
{
  hencoder->txBuf[0] = hencoder->id;
  hencoder->txBuf[1] = 0x04;
  hencoder->txBuf[2] = 0x00;
  hencoder->txBuf[3] = 0x00;
  hencoder->txBuf[4] = 0x00;
  hencoder->txBuf[5] = 0x07;
  uint16_t crc = crc16(hencoder->txBuf, 6);
  hencoder->txBuf[6] = (uint8_t)(crc&0xFF);
  hencoder->txBuf[7] = (uint8_t)((crc>>8)&0xFF);
  HAL_UART_Receive_DMA(hencoder->huart, rx_buf, 19);
  HAL_GPIO_WritePin(GPIOD, RS485_DE_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(hencoder->huart, hencoder->txBuf, 8, 1);
  HAL_GPIO_WritePin(GPIOD, RS485_DE_Pin, GPIO_PIN_RESET);
}

void CE100ENCODER_GetData(CE100EncoderHandle* hencoder, uint8_t* rx_buf)
{
  memcpy(hencoder->rxBuf, rx_buf, 19);
  hencoder->angle.f = ((float)(hencoder->rxBuf[4] | hencoder->rxBuf[3] << 8)) * 360.0f / 32768.0f;
  hencoder->angleRaw.b8[0] = hencoder->rxBuf[4];
  hencoder->angleRaw.b8[1] = hencoder->rxBuf[3];
  hencoder->direction = hencoder->rxBuf[16];
}

uint8_t CE100ENCODER_ConfirmData(uint8_t* rx)
{
  uint16_t crc_rx = *(rx+17) | (*(rx+18))<<8;
  if (crc_rx == crc16(rx, 17))
    return 0;
  else
    return 1;
}
