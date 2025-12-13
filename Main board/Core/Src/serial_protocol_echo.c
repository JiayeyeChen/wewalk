#include "serial_protocol_echo.h"


SerialProtocolEchoHandle SERIALPROTOCOLECHO_Create(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma)
{
	SerialProtocolEchoHandle hserial;
	hserial.huart = huart;
	hserial.hdma = hdma;
	hserial.ifNewMsg = 0;
  hserial.invalidRxMsgCount = 0;
	hserial.readMsgRawPtr = 0;
	hserial.task = SERIALPROTOCOLECHO_TASK_FREE;
	hserial.dma_clear_value = 0x00;
	
	hserial.datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_NA;
	hserial.ifNewDatalogPiece2Send = 0;
	hserial.datalogIndex.b32 = 0;
	hserial.dataSlotLen = 0;
	hserial.datalogLabel2SendPtr = 0;
	hserial.txMsg[0] = 0xAA;
  hserial.txMsg[1] = 0xCC;
	
	memset(hserial.rxMsgRaw, 0xFF, 256);
	
	return hserial;
}

void SERIALPROTOCOLECHO_TransmitCargo(SerialProtocolEchoHandle* hserial, uint8_t* buf, uint8_t size)
{
  hserial->txLen = size + 6;
  hserial->txMsg[2] = size;
  memcpy(&hserial->txMsg[3], buf, size);
  
  union UInt16UInt8 crc;
  crc.b16 = CRC16_Modbus(hserial->txMsg, size + 3);
  
  hserial->txMsg[size + 3] = crc.b8[0];
  hserial->txMsg[size + 4] = crc.b8[1];
  hserial->txMsg[size + 5] = 0x55;
  HAL_UART_Transmit_DMA(hserial->huart, hserial->txMsg, hserial->txLen);
}

void SERIALPROTOCOLECHO_ReceiveCargoUARTIdleITCallback(SerialProtocolEchoHandle* hserial)
{
  HAL_UART_DMAStop(hserial->huart);//Restart the DMA receiver
  uint8_t i = 0;
//  hserial->dmaPtr  = __HAL_DMA_GET_COUNTER(hserial->huart->hdmarx);
  while (i != 255)//Pointer for scanning through the buffer
  {
    if (hserial->rxMsgRaw[i] == 0xAA && hserial->rxMsgRaw[i + 1] == 0xCC)//Find the start delimiter
    {
      uint8_t tem_size = hserial->rxMsgRaw[i + 2];
      if (tem_size + i > 250)//Msg out of RX buffer range
      {
        hserial->invalidRxMsgCount++;
        break;
      }
      if (hserial->rxMsgRaw[tem_size + 5 + i] == 0x55)//Find the end delimiter
      {
        hserial->rxDataLen = tem_size;
        union UInt16UInt8 temCRC;
        temCRC.b16 = CRC16_Modbus(hserial->rxMsgRaw + i, tem_size + 3);
        if (temCRC.b8[0] == hserial->rxMsgRaw[tem_size + 3 + i] && temCRC.b8[1] == hserial->rxMsgRaw[tem_size + 4 + i])//CRC16 modebus check
        {
          hserial->ifNewMsg = 1;
          memcpy(hserial->rxMsgCfm, hserial->rxMsgRaw + 3 + i, hserial->rxDataLen);
          break;
        }
        else
          hserial->invalidRxMsgCount++;
      }
    }
    i++;
  }
  HAL_UARTEx_ReceiveToIdle_DMA(hserial->huart, hserial->rxMsgRaw, 255);
}

void SERIALPROTOCOLECHO_ReceiveCargo(SerialProtocolEchoHandle* hserial)
{
	for (uint8_t temp_read_ptr = hserial->readMsgRawPtr; temp_read_ptr != (uint8_t)(hserial->readMsgRawPtr - 1); temp_read_ptr++) // Scan one by one and looking for 0xAA and 0xCC
	{
		if (hserial->rxMsgRaw[temp_read_ptr] == 0xAA && hserial->rxMsgRaw[(uint8_t)(temp_read_ptr+1)] == 0xCC) // Found the start delimiter
		{
			uint8_t temp_num;
			temp_num = hserial->rxMsgRaw[(uint8_t)(temp_read_ptr+2)]; // Get potential valid data size
			if (hserial->rxMsgRaw[(uint8_t)(temp_read_ptr + temp_num + 5)] == 0x55) // Validation criteria
			{
				/*Check CRC*/
				union UInt16UInt8 temCRC;
				temCRC.b16 = CRC16_Modbus256(hserial->rxMsgRaw, temp_read_ptr, temp_num + 3);
				if (temCRC.b8[0] == hserial->rxMsgRaw[(uint8_t)(temp_read_ptr + temp_num + 3)] && temCRC.b8[1] == hserial->rxMsgRaw[(uint8_t)(temp_read_ptr + temp_num + 4)])
				{
					hserial->ifNewMsg = 1;
					hserial->rxDataLen = temp_num;
					hserial->hdma->Lock = HAL_UNLOCKED;
					hserial->hdma->State = HAL_DMA_STATE_READY;
					if ((uint8_t)(temp_read_ptr + temp_num + 2) > ((uint8_t)(temp_read_ptr + 3)))
					{
						memcpy(hserial->rxMsgCfm, hserial->rxMsgRaw + temp_read_ptr + 3, temp_num);
						HAL_DMA_Start(hserial->hdma, (uint32_t)(&(hserial->dma_clear_value)), (uint32_t)(hserial->rxMsgRaw + temp_read_ptr), temp_num + 6);
					}
					else
					{
						memcpy(hserial->rxMsgCfm, hserial->rxMsgRaw + temp_read_ptr + 3, 253 - temp_read_ptr);
						memcpy(hserial->rxMsgCfm + 253 - temp_read_ptr, hserial->rxMsgRaw, temp_num + temp_read_ptr - 253);
						HAL_DMA_Start(hserial->hdma, (uint32_t)(&(hserial->dma_clear_value)), (uint32_t)(hserial->rxMsgRaw + temp_read_ptr), 256 - temp_read_ptr);
					}
				}
				else
					hserial->invalidRxMsgCount++;
				
				hserial->readMsgRawPtr = temp_read_ptr + temp_num + 6;
				break;
			}
		}
	}
}

void SERIALPROTOCOLECHO_SendText(SerialProtocolEchoHandle* hserial, char text[])
{
  SERIALPROTOCOLECHO_TransmitCargo(hserial, (uint8_t*)text, strlen(text));
}

uint8_t SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(SerialProtocolEchoHandle* hserial, char str[], uint8_t pos)
{
  if (hserial->ifNewMsg)
  {
    if (!strncmp((const char*)(hserial->rxMsgCfm + pos), (const char*)str, strlen(str)))
    {
      hserial->ifNewMsg = 0;
      return 1;
    }
  }
  return 0;
}

void SERIALPROTOCOLECHO_EnableCommunication_Rx2IdleMode(SerialProtocolEchoHandle* hserial)
{
  HAL_UARTEx_ReceiveToIdle_DMA(hserial->huart, hserial->rxMsgRaw, 256);
}

void SERIALPROTOCOLECHO_EnableCommunication_CircularDMAMode(SerialProtocolEchoHandle* hserial)
{
	HAL_UART_Receive_DMA(hserial->huart, hserial->rxMsgRaw, 256);
}

void SERIALPROTOCOLECHO_MasterHost(SerialProtocolEchoHandle* hserial)
{
	/////////////////////////////////////////////////////
	////////////////Check Receive Message////////////////
	/////////////////////////////////////////////////////
	if (hserial->ifNewMsg)
	{
		switch (hserial->task)
    {
    	case SERIALPROTOCOLECHO_TASK_FREE:
				
			if(SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog start", 0))
				SERIALPROTOCOLECHO_StartDatalog(hserial);
    		break;
    	case SERIALPROTOCOLECHO_TASK_DATALOG:
				SERIALPROTOCOLECHO_DatalogManagerReceive(hserial);
    		break;
    	default:
    		break;
    }
	}
	/////////////////////////////////////////////////////
	///////////////////Transmit Message//////////////////
	/////////////////////////////////////////////////////
	switch (hserial->task)
	{
		case SERIALPROTOCOLECHO_TASK_FREE:
			SERIALPROTOCOLECHO_SendText(hserial, "Serial Protocol Echo Master: State Free");
			break;
		case SERIALPROTOCOLECHO_TASK_DATALOG:
			SERIALPROTOCOLECHO_DatalogManagerTransmit(hserial);
			break;
		default:
			break;
	}
}

void SERIALPROTOCOLECHO_DatalogManagerTransmit(SerialProtocolEchoHandle* hserial)
{
	switch (hserial->datalogTask)
  {
  	case SERIALPROTOCOLECHO_DATALOG_TASK_START:
			SERIALPROTOCOLECHO_SendText(hserial, "Datalog start request");
  		break;
  	case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN:
			SERIALPROTOCOLECHO_SendDataSlotLen(hserial);
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL:
			hserial->SetLabelFunc();
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA:
			if (hserial->ifNewDatalogPiece2Send)
			{
				SERIALPROTOCOLECHO_DatalogSingleCargoTransmit(hserial, hserial->dataSlot);
				hserial->ifNewDatalogPiece2Send = 0;
			}
			break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_END:
			SERIALPROTOCOLECHO_SendText(hserial, "Datalog end request");
			break;
  	default:
  		break;
  }
}

void SERIALPROTOCOLECHO_DatalogManagerReceive(SerialProtocolEchoHandle* hserial)
{
	switch (hserial->datalogTask)
  {
  	case SERIALPROTOCOLECHO_DATALOG_TASK_START:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog start request received", 0))
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN;
  		break;
  	case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog length received", 0))
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL;
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL:
    {
      if (hserial->rxMsgCfm[0] == hserial->datalogLabel2SendPtr + 1)
      {
				hserial->datalogLabel2SendPtr++;
      }
			if (hserial->datalogLabel2SendPtr >= hserial->dataSlotLen)
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA;
  		break;
    }
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog end", 0))
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_END;
			break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_END:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog end request received", 0))
			{
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_NA;
				hserial->task = SERIALPROTOCOLECHO_TASK_FREE;
			}
			break;
  	default:
  		break;
  }
  hserial->ifNewMsg = 0;
}

void SERIALPROTOCOLECHO_StartDatalog(SerialProtocolEchoHandle* hserial)
{
	hserial->task = SERIALPROTOCOLECHO_TASK_DATALOG;
	hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_START;
	hserial->ifNewDatalogPiece2Send = 0;
	hserial->datalogIndex.b32 = 0;
	hserial->datalogLabel2SendPtr = 0;
	hserial->datalogStartTimestamp = HAL_GetTick();
}

void SERIALPROTOCOLECHO_SetNewDatalogSlotLength(SerialProtocolEchoHandle* hserial, uint8_t len)
{
	hserial->dataSlotLen = len;
}

void SERIALPROTOCOLECHO_SendDataSlotLen(SerialProtocolEchoHandle* hserial)
{
  char numStr[2];
  char txStr[7];
  strncpy(txStr, "len: ", 5);
  int numStrLen;
  numStrLen = sprintf(numStr, "%d", hserial->dataSlotLen);
  strncpy(&txStr[5], numStr, numStrLen);
  SERIALPROTOCOLECHO_TransmitCargo(hserial, (uint8_t*)txStr, numStrLen + 5);
}

void SERIALPROTOCOLECHO_SendDataSlotLabel(SerialProtocolEchoHandle* hserial, char label_1[], ...)
{
	va_list label_ptr;
  va_start(label_ptr, label_1);
 
  uint8_t numOfLabels = atoi(label_1);
  static char buf[20];
  for (uint8_t i = 0; i < numOfLabels; i++)
  {
    buf[0] = i + 1;
//    strcpy(&buf[1], va_arg(label_ptr, char*));
    char* tem_char = va_arg(label_ptr, char*);
    memcpy(&buf[1], tem_char, 20);
    if (i == hserial->datalogLabel2SendPtr)
    {
      SERIALPROTOCOLECHO_TransmitCargo(hserial, (uint8_t*)buf, strlen(buf));
      break;
    }
  }
  va_end(label_ptr);
}

void SERIALPROTOCOLECHO_SetNewDatalogSendLabelFunction(SerialProtocolEchoHandle* hserial, FuncTypeVoidVoid func)
{
	hserial->SetLabelFunc = func;
}

void SERIALPROTOCOLECHO_SetNewDatalogSlot(SerialProtocolEchoHandle* hserial, union FloatUInt8 (*data_slot))
{
	hserial->dataSlot = data_slot;
}

void SERIALPROTOCOLECHO_DatalogSingleCargoTransmit(SerialProtocolEchoHandle* hserial, union FloatUInt8 data_slots[])
{
	uint8_t i = 0;
  union UInt32UInt8 sysTick;
  sysTick.b32 = HAL_GetTick() - hserial->datalogStartTimestamp;
  //Index
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[0];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[1];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[2];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[3];
  hserial->datalogIndex.b32++;
  //Time stamp
  hserial->datalogBuf[i++] = sysTick.b8[0];
  hserial->datalogBuf[i++] = sysTick.b8[1];
  hserial->datalogBuf[i++] = sysTick.b8[2];
  hserial->datalogBuf[i++] = sysTick.b8[3];
  //Data
  for (uint8_t j = 0; j < hserial->dataSlotLen; j++)
  {
    hserial->datalogBuf[i++] = data_slots[j].b8[0];
    hserial->datalogBuf[i++] = data_slots[j].b8[1];
    hserial->datalogBuf[i++] = data_slots[j].b8[2];
    hserial->datalogBuf[i++] = data_slots[j].b8[3];
  }
  SERIALPROTOCOLECHO_TransmitCargo(hserial, hserial->datalogBuf, hserial->dataSlotLen * 4 + 8);
}

void SERIALPROTOCOLECHO_EndDatalog(SerialProtocolEchoHandle* hserial)
{
	hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_END;
}
