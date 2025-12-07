#include "wewalk_middleware.h"

volatile uint8_t motor_communication_stage = 0;
union FloatUInt8 datalog_data[16];

void WEWALKMIDDLEWARE_MotorControlManager(WeWalkHandle* hwewalk, LKTECH_MG_Handle* hmotor_left, LKTECH_MG_Handle* hmotor_right)
{
  if (hwewalk->application_type == WEWALK_APPLICATION_TYPE_UNILATERAL_LEG_RIGHT)
  {
    if (motor_communication_stage == 1)
    {
      LETECH_MG_ReadAngleSingleTurn(hmotor_right);
      motor_communication_stage--;
    }
    else if (motor_communication_stage == 0)
    {
      if (hmotor_right->status == LKTECH_MG_MOTOR_STATUS_UNKNOWN || !hwewalk->right->motorCommand.ifEnabled)
      {
        LETECH_MG_Disable(hmotor_right);
      }
      else if (hwewalk->right->motorCommand.ifEnabled && hmotor_right->status != LKTECH_MG_MOTOR_STATUS_ENABLED)
      {
        LETECH_MG_Enable(hmotor_right);
      }
      else if (hwewalk->right->motorCommand.ifEnabled && hmotor_right->status == LKTECH_MG_MOTOR_STATUS_ENABLED)
      {
        //Send control commands
        if (hwewalk->right->motorCommand.controlMode == WEWALK_UNILATERAL_MOTOR_TASK_VELOCITY_CONTROL)
          LETECH_MG_SpeedControl(hmotor_right, hwewalk->right->motorCommand.cmd.f);
        else if (hwewalk->right->motorCommand.controlMode == WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL)
          LETECH_MG_CurrentControl(hmotor_right, hwewalk->right->motorCommand.cmd.f);
        else if (hwewalk->right->motorCommand.controlMode == WEWALK_UNILATERAL_MOTOR_TASK_INCREMENTAL_POSITION_CONTROL)
          LETECH_MG_PositionControl5Increment(hmotor_right, hwewalk->right->motorCommand.cmd.f);
      }
      motor_communication_stage++;
    }
  }
  else if (hwewalk->application_type == WEWALK_APPLICATION_TYPE_UNILATERAL_LEG_LEFT)
  {
  }
  else if (hwewalk->application_type == WEWALK_APPLICATION_TYPE_BILATERAL_LEG)
  {
  }
}

void Datalog_SendLabel(void)
{
	SERIALPROTOCOLECHO_SendDataSlotLabel(&hSerialPC, "16", "accThighX", "accThighY", "accThighZ", "gyroThighX", "gyroThighY", "gyroThighZ", \
                                       "accShankX", "accShankY", "accShankZ", "gyroShankX", "gyroShankY", "gyroShankZ", \
                                       "MotorPos", "MotorVel", "MotorTorque", "GaitPhaseIndicator");
}
