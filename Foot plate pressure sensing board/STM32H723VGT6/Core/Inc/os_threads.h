#ifndef OS_THREADS_H
#define OS_THREADS_H

#include "cmsis_os.h"
#include "task.h"
#include "FreeRTOS.h"


void StartDefaultTask(void *argument);//JZP
void KeyTask_Entry(void *argument);
void LcdTask_Entry(void *argument);
void ImuTask_Entry(void *argument);
void FunTest_Entry(void *argument);


void EncoderTaskFunc(void *argument);
void SystemStatusTaskFunc(void *argument);
void MotorControlTaskFunc(void *argument);
void APITaskFunc(void *argument);
void Threads_Init(void);






#endif
