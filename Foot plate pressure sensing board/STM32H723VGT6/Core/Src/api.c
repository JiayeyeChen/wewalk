#include "api.h"

extern CE100EncoderHandle hEncoder1, hEncoder2, hEncoder3;
extern LKTECH_MG_Handle hMotorHip, hMotorKnee;
extern uint32_t aligned_time_offset;
extern union UInt32UInt8 aligned_time;
extern union FloatUInt8 aligned_time_f;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

uint8_t txMsg1[8], txMsg2[8], txMsg3[8];
uint32_t txMsg1Mailbox, txMsg2Mailbox, txMsg3Mailbox;
FDCAN_TxHeaderTypeDef txMsg1Header, txMsg2Header, txMsg3Header;
uint8_t apiIndex;//5 bits

enum MotorTaskEnum motorTaskHip, motorTaskKnee;
union FloatUInt8 mHipSetVel, mHipSetPos, mHipSetCur, mKneeSetVel, mKneeSetPos, mKneeSetCur;

void API_Init(void)
{
  motorTaskHip = MOTOR_TASK_DISABLE;
  motorTaskKnee = MOTOR_TASK_DISABLE;
  
  //txMsg1Header.IDE = CAN_ID_STD;
  //txMsg1Header.DLC = 8;
  //txMsg1Header.StdId = 0x7F0;
  //txMsg1Header.RTR = CAN_RTR_DATA;
	
	txMsg1Header.Identifier=0x7F0;
  txMsg1Header.IdType=FDCAN_STANDARD_ID;
  txMsg1Header.TxFrameType=FDCAN_DATA_FRAME;
  txMsg1Header.DataLength=8;
  txMsg1Header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
  txMsg1Header.BitRateSwitch=FDCAN_BRS_OFF; 
  txMsg1Header.FDFormat=FDCAN_CLASSIC_CAN;
  txMsg1Header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  txMsg1Header.MessageMarker=0;                           
	
  
  //txMsg2Header.IDE = CAN_ID_STD;
  //txMsg2Header.DLC = 8;
  //txMsg2Header.StdId = 0x7F1;
  //txMsg2Header.RTR = CAN_RTR_DATA;

	txMsg2Header.Identifier=0x7F1;
  txMsg2Header.IdType=FDCAN_STANDARD_ID;
  txMsg2Header.TxFrameType=FDCAN_DATA_FRAME;
  txMsg2Header.DataLength=8;
  txMsg2Header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
  txMsg2Header.BitRateSwitch=FDCAN_BRS_OFF; 
  txMsg2Header.FDFormat=FDCAN_CLASSIC_CAN;
  txMsg2Header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  txMsg2Header.MessageMarker=0;                           

  //txMsg3Header.IDE = CAN_ID_STD;
  //txMsg3Header.DLC = 8;
  //txMsg3Header.StdId = 0x7F2;
  //txMsg3Header.RTR = CAN_RTR_DATA;

	txMsg3Header.Identifier=0x7F2;
  txMsg3Header.IdType=FDCAN_STANDARD_ID;
  txMsg3Header.TxFrameType=FDCAN_DATA_FRAME;
  txMsg3Header.DataLength=8;
  txMsg3Header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
  txMsg3Header.BitRateSwitch=FDCAN_BRS_OFF; 
  txMsg3Header.FDFormat=FDCAN_CLASSIC_CAN;
  txMsg3Header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  txMsg3Header.MessageMarker=0;                           

  API_ClearAllSetControlValues();
  apiIndex = 0;
}

void API_Host(void)
{
  API_SendFeedback();
  
}

void API_SendFeedback(void)
{
  apiIndex++;
  txMsg1[0] = hEncoder1.angleRaw.b8[0];
  txMsg1[1] = hEncoder1.angleRaw.b8[1];
	aligned_time.b32 = HAL_GetTick() - aligned_time_offset;
	aligned_time_f.f = ((float)aligned_time.b32) * 1.0021004f;
  txMsg1[2] = aligned_time_f.b8[0];
  txMsg1[3] = aligned_time_f.b8[1];
  txMsg1[4] = aligned_time_f.b8[2];
  txMsg1[5] = aligned_time_f.b8[3];
	if (HAL_GPIO_ReadPin(INPUT1_GPIO_Port, INPUT1_Pin) == GPIO_PIN_SET)
		txMsg1[6] = 1;//GPIO signal
  else
		txMsg1[6] = 0;//GPIO signal
  txMsg1[7] = apiIndex;
  //HAL_CAN_AddTxMessage(&hfdcan2, &txMsg1Header, txMsg1, &txMsg1Mailbox);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txMsg1Header, txMsg1);
  
  union UInt16UInt8 angle;
  angle.b16 = (uint16_t)(hMotorHip.angle.f * 65535.0f / 360.0f);
  txMsg2[0] = angle.b8[0];
  txMsg2[1] = angle.b8[1];
  txMsg2[2] = hMotorHip.speedRawDeg.b8[0];
  txMsg2[3] = hMotorHip.speedRawDeg.b8[1];
  txMsg2[4] = hMotorHip.currentRaw.b8[0];
  txMsg2[5] = hMotorHip.currentRaw.b8[1];
  txMsg2[6] = (uint8_t)hMotorHip.temperature.f;
  txMsg2[7] = apiIndex;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txMsg2Header, txMsg2);
  
  DWT_Delay_us(100);
  angle.b16 = (uint16_t)(hMotorKnee.angle.f * 65535.0f / 360.0f);
  txMsg3[0] = angle.b8[0];
  txMsg3[1] = angle.b8[1];
  txMsg3[2] = hMotorKnee.speedRawDeg.b8[0];
  txMsg3[3] = hMotorKnee.speedRawDeg.b8[1];
  txMsg3[4] = hMotorKnee.currentRaw.b8[0];
  txMsg3[5] = hMotorKnee.currentRaw.b8[1];
  txMsg3[6] = (uint8_t)hMotorKnee.temperature.f;
  txMsg3[7] = apiIndex;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txMsg3Header, txMsg3);
}

void API_ClearAllSetControlValues(void)
{
  mHipSetVel.f = 0.0f;
  mHipSetPos.f = 0.0f;
  mHipSetCur.f = 0.0f;
  mKneeSetVel.f = 0.0f;
  mKneeSetPos.f = 0.0f;
  mKneeSetCur.f = 0.0f;
}
