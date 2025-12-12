#include "wewalk_application.h"



/******************************************/
/******************************************/
/**************User Interface**************/
/******************************************/
/******************************************/
WeWalkUIHandle WEWALKUI_Create(void)
{
  WeWalkUIHandle hui;
  hui.debugCounter = 0;
  hui.joystickX.f = 0.0f;
  hui.joystickY.f = 0.0f;
  hui.lastUISignalReceivedTimestamp = 0;
  hui.ifUIOnline = 0;
  hui.uiOffLineThresholdCriteriaDuration = 500;
  return hui;
}

void WEWALKUI_SendNormalFeedbackData(WeWalkHandle* hwewalk)
{
  uint8_t tx_feedback[12];
  uint16_t cmd = UI_SERIAL_COMMAND_REQUEST_NORMAL_FEEDBACK;
  memcpy(tx_feedback, &cmd, 4);
  memcpy(tx_feedback + 4, &hwewalk->right->dataPtr, 4);
  memcpy(tx_feedback + 8, (&hwewalk->right->data.accThigh[0].b8[0]) + hwewalk->right->dataPtr * 4, 4);
  hwewalk->right->dataPtr++;
  if (hwewalk->right->dataPtr == sizeof(hwewalk->right->data)/4)
    hwewalk->right->dataPtr = 0;
  SERIALPROTOCOLECHO_TransmitCargo(&hwewalk->hUI->serialUI, (uint8_t*)tx_feedback, 12);
}

void WEWALKUI_SendParticularFeedback(WeWalkHandle* hwewalk, uint8_t left_right)
{
  uint8_t num_of_data = 0;
  for (uint8_t i = 4; i <= 254; i+=4)
  {
    
  }
  if (left_right == UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT)
  {
    
  }
  else if (left_right == UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT)
  {
  }
}

void WEWALKUI_SendSettings(WeWalkHandle* hwewalk)
{
  hwewalk->mainTask = WEWALK_MAIN_TASK_IDLE;
  uint8_t tx_settings[12];
  uint16_t cmd = UI_SERIAL_COMMAND_READ_SETTINGS;
  memcpy(tx_settings, &cmd, 4);
  memcpy(tx_settings + 4, &hwewalk->right->settingPtr, 4);
  memcpy(tx_settings + 8, (&hwewalk->right->setting.motorKt.b8[0]) + hwewalk->right->settingPtr * 4, 4);
  hwewalk->right->settingPtr++;
  if (hwewalk->right->settingPtr == sizeof(hwewalk->right->setting)/4)
    hwewalk->right->settingPtr = 0;
  SERIALPROTOCOLECHO_TransmitCargo(&hwewalk->hUI->serialUI, (uint8_t*)tx_settings, 12);
}

void WEWALKUI_Host(WeWalkHandle* hwewalk)
{
  if (hwewalk->hUI->serialUI.ifNewMsg)
  {
    uint16_t cmd = hwewalk->hUI->serialUI.rxMsgCfm[0] | hwewalk->hUI->serialUI.rxMsgCfm[1] << 8;
    switch (cmd)
    {
    	case UI_SERIAL_COMMAND_REQUEST_HEARTBEAT:
      {
        union UInt16UInt8 tx;
        tx.b16 = UI_SERIAL_COMMAND_REQUEST_HEARTBEAT;
        SERIALPROTOCOLECHO_TransmitCargo(&hwewalk->hUI->serialUI, (uint8_t*)&tx, 2);
    		break;
      }
    	case UI_SERIAL_COMMAND_REQUEST_NORMAL_FEEDBACK:
      {
        WEWALKUI_SendNormalFeedbackData(hwewalk);
    		break;
      }
      case UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT:
      {
        WEWALKUI_SendParticularFeedback(hwewalk, UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT);
        break;
      }
      case UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT:
      {
        WEWALKUI_SendParticularFeedback(hwewalk, UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT);
        break;
      }
      case UI_SERIAL_COMMAND_MANUAL_CONTROL:
      {
        hwewalk->mainTask = WEWALK_MAIN_TASK_MANUAL_CONTROL;
        uint16_t sub_cmd = hwewalk->hUI->serialUI.rxMsgCfm[2] | hwewalk->hUI->serialUI.rxMsgCfm[3] << 8;
        switch (sub_cmd)
        {
        	case UI_SERIAL_COMMAND_MANUAL_CONTROL_DISABLE:
          {
            hwewalk->left->motorCommand.ifEnabled = 0;
            hwewalk->right->motorCommand.ifEnabled = 0;
        		break;
          }
          case UI_SERIAL_COMMAND_MANUAL_CONTROL_POSITION_CONTROL_RIGHT:
          {
            hwewalk->left->motorCommand.ifEnabled = 0;
            hwewalk->right->motorCommand.ifEnabled = 1;
            hwewalk->right->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_INCREMENTAL_POSITION_CONTROL;
            memcpy(&hwewalk->hUI->joystickY.b8[0], &hwewalk->hUI->serialUI.rxMsgCfm[4], 4);
            hwewalk->right->motorCommand.cmd.f = hwewalk->hUI->joystickY.f * 5.0f;
            break;
          }
          case UI_SERIAL_COMMAND_MANUAL_CONTROL_VELOCITY_CONTROL_RIGHT:
          {
            hwewalk->left->motorCommand.ifEnabled = 0;
            hwewalk->right->motorCommand.ifEnabled = 1;
            hwewalk->right->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_VELOCITY_CONTROL;
            memcpy(&hwewalk->hUI->joystickY.b8[0], &hwewalk->hUI->serialUI.rxMsgCfm[4], 4);
            hwewalk->right->motorCommand.cmd.f = hwewalk->hUI->joystickY.f * hwewalk->right->setting.motorManualcontrolVelocityMultiplier.f;
            break;
          }
          case UI_SERIAL_COMMAND_MANUAL_CONTROL_CURRENT_CONTROL_RIGHT:
          {
            hwewalk->left->motorCommand.ifEnabled = 0;
            hwewalk->right->motorCommand.ifEnabled = 1;
            hwewalk->right->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL;
            memcpy(&hwewalk->hUI->joystickY.b8[0], &hwewalk->hUI->serialUI.rxMsgCfm[4], 4);
            hwewalk->right->motorCommand.cmd.f = hwewalk->hUI->joystickY.f * hwewalk->right->setting.motorManualcontrolCurrentMultiplier.f;
            break;
          }
        	default:
        		break;
        }
        
        uint8_t tx[28];
        memcpy(&tx[0], &cmd, 2);
        memcpy(&tx[2], &sub_cmd, 2);
        memcpy(&tx[4], &hwewalk->right->data.motorPos.b8[0], 4);
        memcpy(&tx[8], &hwewalk->right->data.motorVel.b8[0], 4);
        memcpy(&tx[12], &hwewalk->right->data.motorCur.b8[0], 4);
        memcpy(&tx[16], &hwewalk->left->data.motorPos.b8[0], 4);
        memcpy(&tx[20], &hwewalk->left->data.motorVel.b8[0], 4);
        memcpy(&tx[24], &hwewalk->left->data.motorCur.b8[0], 4);
        SERIALPROTOCOLECHO_TransmitCargo(&hwewalk->hUI->serialUI, (uint8_t*)&tx, 28);
        break;
      }
      case UI_SERIAL_COMMAND_START_CONTROL:
      {
        uint16_t sub_cmd = hwewalk->hUI->serialUI.rxMsgCfm[2] | hwewalk->hUI->serialUI.rxMsgCfm[3] << 8;
        hwewalk->mainTask = WEWALK_MAIN_TASK_WALK_ASSISTANCE;
        if (sub_cmd == UI_SERIAL_COMMAND_CONTROL_MODE_FREE_WALKING)
          hwewalk->assistanceMode = WEWALK_WALK_ASSISTANCE_FREE_WALKING;
        else if (sub_cmd == UI_SERIAL_COMMAND_CONTROL_MODE_ACC_PRO_ASSISTANCE)
          hwewalk->assistanceMode = WEWALK_WALK_ASSISTANCE_ACC_PRO_ASSISTANCE;
        else if (sub_cmd == UI_SERIAL_COMMAND_CONTROL_MODE_PURE_IMPEDANCE_CONTROL)
          hwewalk->assistanceMode = WEWALK_WALK_ASSISTANCE_PURE_IMPEDANCE_CONTROL;
        else if (sub_cmd == UI_SERIAL_COMMAND_CONTROL_MODE_SINE_SWING)
        {
          hwewalk->assistanceMode = WEWALK_WALK_ASSISTANCE_SINE_SWING;
          WEWALK_SineSwingInit(hwewalk->right);
        }
        else if (sub_cmd == UI_SERIAL_COMMAND_CONTROL_MODE_FULL_ASSISTANCE_1)
          hwewalk->assistanceMode = WEWALK_WALK_ASSISTANCE_FULL_ASSISTANCE_1;
        else if (sub_cmd == UI_SERIAL_COMMAND_CONTROL_MODE_TORQUE_PULSE)
        {
          hwewalk->assistanceMode = WEWALK_WALK_ASSISTANCE_TORQUE_PULSE;
          WEWALK_TorquePulseControlTrigger(hwewalk->right, 0.0f);
        }
        break;
      }
      case UI_SERIAL_COMMAND_STOP_CONTROL:
      {
        hwewalk->mainTask = WEWALK_MAIN_TASK_IDLE;
        break;
      }
      case UI_SERIAL_COMMAND_START_DATALOG:
      {
        SERIALPROTOCOLECHO_StartDatalog(hwewalk->serialPC);
        break;
      }
      case UI_SERIAL_COMMAND_STOP_DATALOG:
      {
        SERIALPROTOCOLECHO_EndDatalog(hwewalk->serialPC);
        break;
      }
      case UI_SERIAL_COMMAND_READ_SETTINGS:
      {
        WEWALKUI_SendSettings(hwewalk);
        break;
      }
      case UI_SERIAL_COMMAND_UPLOAD_SETTINGS:
      {
        hwewalk->mainTask = WEWALK_MAIN_TASK_IDLE;
        WEWALKUI_ReceiveSettings(hwewalk);
        break;
      }
      case UI_SERIAL_COMMAND_SAVE_SETTINGS:
      {
        hwewalk->ifSaveSettingRequired = 1;
        hwewalk->mainTask = WEWALK_MAIN_TASK_IDLE;
        break;
      }
      case UI_SERIAL_COMMAND_GENERAL_TRIGGER_1:
      {
        WEWALK_TorquePulseControlTrigger(hwewalk->right, 1.0f);
        break;
      }
      case UI_SERIAL_COMMAND_GENERAL_TRIGGER_2:
      {
        WEWALK_TorquePulseControlTrigger(hwewalk->right, -1.0f);
        break;
      }
    	default:
    		break;
    }
    hwewalk->hUI->serialUI.ifNewMsg = 0;
    hwewalk->hUI->lastUISignalReceivedTimestamp = HAL_GetTick();
  }
}

void WEWALKUI_ReceiveSettings(WeWalkHandle* hwewalk)
{
  uint32_t setting_ptr;
  memcpy(&setting_ptr, &hwewalk->hUI->serialUI.rxMsgCfm[4], 4);
  if (setting_ptr == 0)//motorKt
    memcpy(&hwewalk->right->setting.motorKt.b8[0], &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 1)//motorFrictionCompensationCurrent
    memcpy(&hwewalk->right->setting.motorFrictionCompensationCurrent.b8[0], &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 2)//ledLogoBrightnessPWMCCR
  {
    memcpy(&hwewalk->right->setting.ledLogoBrightnessPWMCCR, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
    WEWALKUI_SetLEDLogoPWMCCR(hwewalk->right, hwewalk->right->setting.ledLogoBrightnessPWMCCR);
  }
  else if (setting_ptr == 3)//motorManualcontrolVelocityMultiplier
    memcpy(&hwewalk->right->setting.motorManualcontrolVelocityMultiplier, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 4)//motorManualcontrolCurrentMultiplier
    memcpy(&hwewalk->right->setting.motorManualcontrolCurrentMultiplier, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 5)//accelerationProportionateAssistanceGain
    memcpy(&hwewalk->right->setting.accelerationProportionateAssistanceGain, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 6)//accelerationProportionateAssistanceGain
    memcpy(&hwewalk->right->setting.accelerationProportionateAssistanceSigmoidCoefficient, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 7)//motorKneeJointAngleOffset
    memcpy(&hwewalk->right->setting.motorKneeJointAngleOffset, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 8)//impedanceControlSpringCoefficient
    memcpy(&hwewalk->right->setting.impedanceControlSpringCoefficient, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 9)//impedanceControlDampingCoefficient
    memcpy(&hwewalk->right->setting.impedanceControlDampingCoefficient, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 10)//sineSwingControlMagnitude
    memcpy(&hwewalk->right->setting.sineSwingControlMagnitude, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 11)//sineSwingControlFrequency
    memcpy(&hwewalk->right->setting.sineSwingControlFrequency, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 12)//accelerationFieldMethodGaitPhaseDetectionXPosition
    memcpy(&hwewalk->right->setting.accelerationFieldMethodGaitPhaseDetectionXPosition, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 13)//accelerationFieldMethodGaitPhaseDetectionYPosition
    memcpy(&hwewalk->right->setting.accelerationFieldMethodGaitPhaseDetectionYPosition, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 14)//accelerationFieldMethodGaitPhaseDetectionThreshold
    memcpy(&hwewalk->right->setting.accelerationFieldMethodGaitPhaseDetectionThreshold, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 15)//gppA
    memcpy(&hwewalk->right->setting.gppA, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 16)//gppB
    memcpy(&hwewalk->right->setting.gppB, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 17)//gppC
    memcpy(&hwewalk->right->setting.gppC, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 18)//gppD
    memcpy(&hwewalk->right->setting.gppD, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 19)//gppE
    memcpy(&hwewalk->right->setting.gppE, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 20)//gppF
    memcpy(&hwewalk->right->setting.gppF, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 21)//gppG
    memcpy(&hwewalk->right->setting.gppG, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
  else if (setting_ptr == 22)//gppH
    memcpy(&hwewalk->right->setting.gppH, &hwewalk->hUI->serialUI.rxMsgCfm[8], 4);
}
void WEWALKUI_SetLEDLogoPWMCCR(WeWalkUnilateralHandle* hwewalk, uint32_t ccr)
{
  memcpy((void*)(&(hwewalk->hLEDLogoPWM->Instance->CCR1) + hwewalk->ledLogoPWMChannel * 4), &ccr, 4);
}

void WEWALKUI_OnlineOfflineCheck(WeWalkHandle* hwewalk)
{
  if (HAL_GetTick() - hwewalk->hUI->lastUISignalReceivedTimestamp < hwewalk->hUI->uiOffLineThresholdCriteriaDuration)
    hwewalk->hUI->ifUIOnline = 1;
  else
    hwewalk->hUI->ifUIOnline = 0;
}

/******************************************/
/******************************************/
/**************Main Scheduler**************/
/******************************************/
/******************************************/

WeWalkUnilateralHandle WEWALKUNILATERAL_Create(TIM_HandleTypeDef* hledlogopwm, uint32_t pwm_channel, GaitPhaseDetectionHandle* hgaitphasedetection)
{
  WeWalkUnilateralHandle hunilateral;
  hunilateral.dataPtr = 0;
  hunilateral.settingPtr = 0;
  hunilateral.hLEDLogoPWM = hledlogopwm;
  hunilateral.ledLogoPWMChannel = pwm_channel;
  hunilateral.motorCommand.ifEnabled = 0;
  hunilateral.motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL;
  hunilateral.motorCommand.cmd.f = 0.0f;
  hunilateral.hGaitPhaseDetect = hgaitphasedetection;
  WEWALK_SineSwingInit(&hunilateral);
  return hunilateral;
}

WeWalkHandle WEWALK_Create(WeWalkUIHandle* hui, SerialProtocolEchoHandle* hserial_pc, \
                         WeWalkUnilateralHandle* left_leg, WeWalkUnilateralHandle* right_leg, \
                         uint8_t application_type)
{
  WeWalkHandle hwewalk;
  hwewalk.hUI = hui;
  hwewalk.serialPC = hserial_pc;
  hwewalk.left = left_leg;
  hwewalk.right = right_leg;
  hwewalk.application_type = application_type;
  hwewalk.mainTask = WEWALK_MAIN_TASK_IDLE;
  hwewalk.assistanceMode = WEWALK_WALK_ASSISTANCE_FREE_WALKING;
  hwewalk.ifSaveSettingRequired = 0;
  return hwewalk;
}

void WEWALK_Main(WeWalkHandle* hwewalk)
{
  WEWALKUI_OnlineOfflineCheck(hwewalk);
  switch (hwewalk->mainTask)
  {
  	case WEWALK_MAIN_TASK_IDLE:
    {
      hwewalk->left->motorCommand.ifEnabled = 0;
      hwewalk->right->motorCommand.ifEnabled = 0;
  		break;
    }
  	case WEWALK_MAIN_TASK_MANUAL_CONTROL:
    {
  		break;
    }
    case WEWALK_MAIN_TASK_WALK_ASSISTANCE:
    {
      hwewalk->left->motorCommand.ifEnabled = 1;
      hwewalk->right->motorCommand.ifEnabled = 1;
			WEWALK_GaitPhaseDetection(hwewalk->right);
      if (hwewalk->assistanceMode == WEWALK_WALK_ASSISTANCE_FREE_WALKING)
        WEWALK_FreeWalkingControl(hwewalk);
//////      else if (hwewalk->assistanceMode == WEWALK_WALK_ASSISTANCE_ACC_PRO_ASSISTANCE)
//////        WEWALK_AccelerationProportionateControl(hwewalk);
//////      else if (hwewalk->assistanceMode == WEWALK_WALK_ASSISTANCE_PURE_IMPEDANCE_CONTROL)
//////        WEWALK_ImpedanceControl(hwewalk);
//////      else if (hwewalk->assistanceMode == WEWALK_WALK_ASSISTANCE_SINE_SWING)
//////        WEWALK_SineSwingControl(hwewalk->right);
//////      else if (hwewalk->assistanceMode == WEWALK_WALK_ASSISTANCE_FULL_ASSISTANCE_1)
//////        WEWALK_FullAssistance_1(hwewalk);
//////      else if (hwewalk->assistanceMode == WEWALK_WALK_ASSISTANCE_TORQUE_PULSE)
//////        WEWALK_TorquePulseControl(hwewalk->right);
      else
      {
        hwewalk->left->motorCommand.ifEnabled = 0;
        hwewalk->right->motorCommand.ifEnabled = 0;
      }
      break;
    }
  	default:
  		break;
  }
}

/************************************************/
/************************************************/
/**************Gait Phase Detection**************/
/************************************************/
/************************************************/

GaitPhaseDetectionHandle WEWALK_GaitPhaseDetectionCreate(float loop_period_in_second, float gait_phase_detection_cutoff_frequency)
{
  GaitPhaseDetectionHandle hgaitphasedetection;
  hgaitphasedetection.gaitPhaseDetectionType = WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM_ETH;
  LowPassFilter_Init(&hgaitphasedetection.hFilterGaitPhaseDetectionIndicator, gait_phase_detection_cutoff_frequency, loop_period_in_second);
  return hgaitphasedetection;
}

void WEWALK_GaitPhaseDetection(WeWalkUnilateralHandle* hwewalk)
{
  switch (hwewalk->hGaitPhaseDetect->gaitPhaseDetectionType)
  {
    case WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM_ETH:
    {
			
      break;
    }
    case WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM_ICR:
    {
      break;
    }
    default:
      break;
  }
}




/***********************************/
/***********************************/
/**************Control**************/
/***********************************/
/***********************************/
void WEWALK_UpdatePerception(WeWalkHandle* hwewalk)
{
}

void WEWALK_FreeWalkingControl(WeWalkHandle* hwewalk)
{
	hwewalk->right->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL;
	hwewalk->right->motorCommand.cmd.f = 0.0f;
}

void WEWALK_AccelerationProportionateControl(WeWalkHandle* hwewalk)
{

}
void WEWALK_ImpedanceControl(WeWalkHandle* hwewalk)
{
  hwewalk->right->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL;
  hwewalk->right->motorCommand.cmd.f = hwewalk->right->setting.impedanceControlSpringCoefficient.f * hwewalk->right->data.motorPos.f + \
                                      hwewalk->right->setting.impedanceControlDampingCoefficient.f * hwewalk->right->data.motorVel.f;
}

void WEWALK_SineSwingControl(WeWalkUnilateralHandle* hwewalk)
{
  hwewalk->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_VELOCITY_CONTROL;
  float time_now = ((float)HAL_GetTick()) / 1000.0f;
  hwewalk->motorCommand.cmd.f = hwewalk->setting.sineSwingControlMagnitude.f * sinf((2.0f * pi / hwewalk->setting.sineSwingControlFrequency.f) * (time_now - hwewalk->hSineSwing.t0TimeSecond));
  
}

void WEWALK_SineSwingInit(WeWalkUnilateralHandle* hwewalk)
{
  hwewalk->hSineSwing.t0TimeSecond = ((float)HAL_GetTick()) / 1000.0f;
}

void WEWALK_FullAssistance_1(WeWalkHandle* hwewalk)
{
  if (hwewalk->right->data.ifSwingPhase.b32)
    WEWALK_AccelerationProportionateControl(hwewalk);
  else
    WEWALK_ImpedanceControl(hwewalk);
}

void WEWALK_TorquePulseControlTrigger(WeWalkUnilateralHandle* hwewalk, float direction)
{
  hwewalk->hTorquePulse.direction = direction;
  hwewalk->hTorquePulse.t0TimeMilliSecond = (float)HAL_GetTick();
}

void WEWALK_TorquePulseControl(WeWalkUnilateralHandle* hwewalk)
{
  hwewalk->motorCommand.controlMode = WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL;
  if (((float)HAL_GetTick()) - hwewalk->hTorquePulse.t0TimeMilliSecond < hwewalk->setting.gppC.f)
  {
    hwewalk->motorCommand.cmd.f = hwewalk->hTorquePulse.direction * hwewalk->setting.gppA.f * hwewalk->setting.gppB.f * sinf(pi * ((float)HAL_GetTick() - hwewalk->hTorquePulse.t0TimeMilliSecond) / hwewalk->setting.gppC.f) / hwewalk->setting.motorKt.f;
  }
  else
    hwewalk->motorCommand.cmd.f = 0.0f;
}

