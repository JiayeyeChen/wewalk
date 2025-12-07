#include "wewalk_debug_gui_interface.h"

MyTouchGFXUtilities_Joystick hManualControlJoystick;

WeWalkDebugUIHandheldHandle WEWALKDEBUGUIHANDHELD_Create(SerialProtocolEchoHandle* hserial, uint32_t loop_period_ms)
{
  WeWalkDebugUIHandheldHandle huihandheld;
  huihandheld.hSerial = hserial;
  huihandheld.mainTask = UI_SERIAL_COMMAND_REQUEST_HEARTBEAT;
  huihandheld.loop_period_ms = loop_period_ms;
  huihandheld.bluetoothState = 0;
  huihandheld.ifWeWalkIsOnline = 0;
  huihandheld.wewalkOffLineThresholdCriteriaDuration = 1000;
  huihandheld.lastReceivedTimeStamp = 0;
  huihandheld.countForTest = 0;
  huihandheld.dataLeftPtr = 0;
  huihandheld.dataRightPtr = 0;
  huihandheld.settingLeftPtr = 0;
  huihandheld.settingRightPtr = 0;
  huihandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FREEWALKING;
  memset(huihandheld.particularFeedbackPtrLeft, 0xFF, sizeof(huihandheld.particularFeedbackPtrLeft));
  memset(huihandheld.particularFeedbackPtrRight, 0xFF, sizeof(huihandheld.particularFeedbackPtrRight));
  
  WeWalkDebugUIHandheld_ManualControlInit(&huihandheld);
  return huihandheld;
}

void WeWalkDebugUIHandheld_Host(WeWalkDebugUIHandheldHandle* huihandheld)
{
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  //////////////////////////Safety Precaution//////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  if (!huihandheld->ifWeWalkIsOnline)
    huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
  
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  ///////////////////////////Receive message///////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  if (huihandheld->hSerial->ifNewMsg)
  {
    uint16_t main_cmd;
    memcpy(&main_cmd, &huihandheld->hSerial->rxMsgCfm[0], 2);
    switch (main_cmd)
    {
    	case UI_SERIAL_COMMAND_REQUEST_HEARTBEAT:
      {
    		break;
      }
    	case UI_SERIAL_COMMAND_REQUEST_NORMAL_FEEDBACK:
      {
        memcpy(&huihandheld->dataRightPtr, &huihandheld->hSerial->rxMsgCfm[4], 4);
        memcpy((&huihandheld->dataRight.accThigh[0].b8[0]) + huihandheld->dataRightPtr*4, &huihandheld->hSerial->rxMsgCfm[8], 4);
    		break;
      }
      case UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT:
      {
        
        break;
      }
      case UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT:
      {
        
        break;
      }
      case UI_SERIAL_COMMAND_MANUAL_CONTROL:
      {
        uint16_t sub_cmd;
        memcpy(&sub_cmd, &huihandheld->hSerial->rxMsgCfm[2], 2);
        memcpy(&huihandheld->dataRight.motorPos.b8[0], &huihandheld->hSerial->rxMsgCfm[4], 4);
        memcpy(&huihandheld->dataRight.motorVel.b8[0], &huihandheld->hSerial->rxMsgCfm[8], 4);
        memcpy(&huihandheld->dataRight.motorCur.b8[0], &huihandheld->hSerial->rxMsgCfm[12], 4);
        memcpy(&huihandheld->dataLeft.motorPos.b8[0], &huihandheld->hSerial->rxMsgCfm[16], 4);
        memcpy(&huihandheld->dataLeft.motorVel.b8[0], &huihandheld->hSerial->rxMsgCfm[20], 4);
        memcpy(&huihandheld->dataLeft.motorCur.b8[0], &huihandheld->hSerial->rxMsgCfm[24], 4);
        break;
      }
      case UI_SERIAL_COMMAND_READ_SETTINGS:
      {
        memcpy(&huihandheld->settingRightPtr, &huihandheld->hSerial->rxMsgCfm[4], 4);
        memcpy((&huihandheld->settingsRight.motorKt.b8[0]) + huihandheld->settingRightPtr*4, &huihandheld->hSerial->rxMsgCfm[8], 4);
        break;
      }
      case UI_SERIAL_COMMAND_SAVE_SETTINGS:
      {
        break;
      }
      case UI_SERIAL_COMMAND_UPLOAD_SETTINGS:
      {
        break;
      }
      case UI_SERIAL_COMMAND_START_CONTROL:
      {
        break;
      }
      case UI_SERIAL_COMMAND_STOP_CONTROL:
      {
        break;
      }
    	default:
    		break;
    }
    huihandheld->hSerial->ifNewMsg = 0;
    huihandheld->lastReceivedTimeStamp = HAL_GetTick();
  }
  
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  ///////////////////////////Send message//////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  switch (huihandheld->mainTask)
  {
  	case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_REQUEST_HEARTBEAT;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
  		break;
    }
  	case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_NORMAL_DATA_FEEDBACK:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_REQUEST_NORMAL_FEEDBACK;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
  		break;
    }
    case UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT:
    {
      uint8_t num_of_data = 0;
      for (uint8_t i = 0; i <= sizeof(huihandheld->particularFeedbackPtrRight)/4; i++)
      {
        if (huihandheld->particularFeedbackPtrRight[i] != 0xFFFFFFFF)
          num_of_data++;
        else
          break;
      }
      uint8_t tx[4 + num_of_data * 4];
      uint32_t cmd = UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT;
      memcpy(&tx[0], &cmd, 4);
      memcpy(&tx[4], huihandheld->particularFeedbackPtrRight, num_of_data * 4);
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)tx, sizeof(tx));
      break;
    }
    case UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT:
    {
      uint8_t num_of_data = 0;
      for (uint8_t i = 0; i <= sizeof(huihandheld->particularFeedbackPtrLeft)/4; i++)
      {
        if (huihandheld->particularFeedbackPtrLeft[i] != 0xFFFFFFFF)
          num_of_data++;
        else
          break;
      }
      uint8_t tx[4 + num_of_data * 4];
      uint32_t cmd = UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT;
      memcpy(&tx[0], &cmd, 4);
      memcpy(&tx[4], huihandheld->particularFeedbackPtrLeft, num_of_data * 4);
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)tx, sizeof(tx));
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_MANUAL_CONTROL:
    {
      uint8_t tx[8];
      uint32_t cmd = UI_SERIAL_COMMAND_MANUAL_CONTROL | huihandheld->manualControl.manualControlTask << 16;
      memcpy(&tx[0], &cmd, 4);
      memcpy(&tx[4], &hManualControlJoystick.outputY.b8[0], 4);
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)tx, 8);
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_UPLOAD_SETTINGS:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_UPLOAD_SETTINGS;
//      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_SAVE_SETTINGS:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_SAVE_SETTINGS;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_READ_SETTINGS:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_READ_SETTINGS;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_START_CONTROL:
    {
      uint32_t cmd = UI_SERIAL_COMMAND_START_CONTROL;
      if (huihandheld->algorithm == WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FREEWALKING)
        cmd |= (UI_SERIAL_COMMAND_CONTROL_MODE_FREE_WALKING << 16);
      else if (huihandheld->algorithm == WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_PROPORTIONATE_ACCELERATION_CONTROL)
        cmd |= (UI_SERIAL_COMMAND_CONTROL_MODE_ACC_PRO_ASSISTANCE << 16);
      else if (huihandheld->algorithm == WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_PURE_IMPEDANCE_CONTROL)
        cmd |= (UI_SERIAL_COMMAND_CONTROL_MODE_PURE_IMPEDANCE_CONTROL << 16);
      else if (huihandheld->algorithm == WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_SINE_SWING)
        cmd |= (UI_SERIAL_COMMAND_CONTROL_MODE_SINE_SWING << 16);
      else if (huihandheld->algorithm == WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FULL_CONTROL_1)
        cmd |= (UI_SERIAL_COMMAND_CONTROL_MODE_FULL_ASSISTANCE_1 << 16);
      else if (huihandheld->algorithm == WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_TORQUE_PULSE)
        cmd |= (UI_SERIAL_COMMAND_CONTROL_MODE_TORQUE_PULSE << 16);
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 4);
      huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_CONTROL:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_STOP_CONTROL;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_START_DATALOG:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_START_DATALOG;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_DATALOG:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_STOP_DATALOG;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_GENERAL_TRIGGER1:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_GENERAL_TRIGGER_1;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
      break;
    }
    case WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_GENERAL_TRIGGER2:
    {
      uint16_t cmd = UI_SERIAL_COMMAND_GENERAL_TRIGGER_2;
      SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)&cmd, 2);
      huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
      break;
    }
  	default:
  		break;
  }
}

void WeWalkOnlineOfflineCheck(WeWalkDebugUIHandheldHandle* huihandheld)
{
  if (HAL_GetTick() - huihandheld->lastReceivedTimeStamp > huihandheld->wewalkOffLineThresholdCriteriaDuration)
    huihandheld->ifWeWalkIsOnline = 0;
  else
    huihandheld->ifWeWalkIsOnline = 1;
}

void WeWalkDebugUIHandheld_ManualControlInit(WeWalkDebugUIHandheldHandle* huihandheld)
{
  huihandheld->mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_MANUAL_CONTROL;
  huihandheld->manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_DISABLE;
}

void WeWalkDebugUIHandheld_SendSetting(WeWalkDebugUIHandheldHandle* huihandheld)
{
  uint8_t tx[12];
  uint32_t cmd = UI_SERIAL_COMMAND_UPLOAD_SETTINGS;
  memcpy(&tx[0], &cmd, 4);
  memcpy(&tx[4], &huihandheld->settingUploadRightPtr, 4);
  memcpy(&tx[8], &huihandheld->settingValueToSend, 4);
  SERIALPROTOCOLECHO_TransmitCargo(huihandheld->hSerial, (uint8_t*)tx, 12);
}
