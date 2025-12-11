#include "os_threads.h"

/* Definitions for ADCTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes= {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SystemStatusTas */
extern osThreadId_t SystemStatusTasHandle;
extern const osThreadAttr_t SystemStatusTas_attributes;
/* Definitions for MotorControlTas */
extern osThreadId_t MotorControlTasHandle;
extern const osThreadAttr_t MotorControlTas_attributes;
extern osThreadId_t APITaskHandle;
extern const osThreadAttr_t APITask_attributes;
extern osThreadId_t defaultTaskHandle;//JZP
extern const osThreadAttr_t defaultTask_attributes;//JZP

extern osThreadId_t KeyTaskHandle;
extern const osThreadAttr_t KeyTask_attributes;
extern osThreadId_t LcdTaskHandle;
extern const osThreadAttr_t LcdTask_attributes;
extern osThreadId_t ImuTaskHandle;
extern const osThreadAttr_t ImuTask_attributes;
extern osThreadId_t FunTestHandle;
extern const osThreadAttr_t FunTest_attributes;

void Threads_Init(void)
{
  defaultTaskHandle=osThreadNew(StartDefaultTask,NULL,&defaultTask_attributes);//JZP
	KeyTaskHandle=osThreadNew(KeyTask_Entry,NULL,&KeyTask_attributes);
	LcdTaskHandle=osThreadNew(LcdTask_Entry,NULL,&LcdTask_attributes);
	ImuTaskHandle=osThreadNew(ImuTask_Entry,NULL,&ImuTask_attributes);
	FunTestHandle=osThreadNew(FunTest_Entry,NULL,&FunTest_attributes);
	SystemStatusTasHandle = osThreadNew(SystemStatusTaskFunc, NULL, &SystemStatusTas_attributes);
  EncoderTaskHandle = osThreadNew(EncoderTaskFunc, NULL, &EncoderTask_attributes);
  MotorControlTasHandle = osThreadNew(MotorControlTaskFunc, NULL, &MotorControlTas_attributes);
  APITaskHandle = osThreadNew(APITaskFunc, NULL, &APITask_attributes);
}
