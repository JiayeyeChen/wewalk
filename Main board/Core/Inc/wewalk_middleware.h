#ifndef WEWALK_MIDDLEWARE_H
#define WEWALK_MIDDLEWARE_H

#include "main.h"
#include "lktech_mg_motor_canfd.h"

#define WEWALKMIDDLEWARE_RIGHT_MOTOR_ID 1

void WEWALKMIDDLEWARE_MotorControlManager(WeWalkHandle* hwewalk, LKTECH_MG_Handle* hmotor_left, LKTECH_MG_Handle* hmotor_right);

void Datalog_SendLabel(void);

extern union FloatUInt8 datalog_data[16];

#endif
