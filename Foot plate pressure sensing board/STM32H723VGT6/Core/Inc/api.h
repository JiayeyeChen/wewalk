#ifndef API_H
#define API_H

#include "lktech_mg_motor.h"
#include "encoder.h"
//#include "system.h"
#include "main.h"
#include "dwt_stm32_delay.h"

enum MotorTaskEnum
{
  MOTOR_TASK_OFF=0,
  MOTOR_TASK_IDLE=1,//Go to idle after having not received control command from PC for a certain time
  MOTOR_TASK_VELOCITY_CONTROL=2,
  MOTOR_TASK_CURRENT_CONTROL=3,
  MOTOR_TASK_POSITION_CONTROL=4,
  MOTOR_TASK_ENABLE=5,
  MOTOR_TASK_DISABLE=6
};


void API_Host(void);

void API_SendFeedback(void);

void API_Init(void);

void API_ClearAllSetControlValues(void);

extern enum MotorTaskEnum motorTaskHip, motorTaskKnee;
extern union FloatUInt8 mHipSetVel, mHipSetPos, mHipSetCur, mKneeSetVel, mKneeSetPos, mKneeSetCur;

#endif
