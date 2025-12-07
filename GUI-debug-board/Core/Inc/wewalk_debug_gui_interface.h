#ifndef WEWALK_DEBUG_GUI_INTERFACE_H
#define WEWALK_DEBUG_GUI_INTERFACE_H

#include "shared.h"
#include "serial_protocol_echo.h"
#include "myTouchGFXUtilities.h"
#include "common.h"




enum WeWalkDebugUIHandHeldMainTask
{
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_NORMAL_DATA_FEEDBACK,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_PARTICULAR_DATA_FEEDBACK_LEFT,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_PARTICULAR_DATA_FEEDBACK_RIGHT,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_MANUAL_CONTROL,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_UPLOAD_SETTINGS,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_SAVE_SETTINGS,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_READ_SETTINGS,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_START_CONTROL,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_CONTROL,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_START_DATALOG,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_DATALOG,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_GENERAL_TRIGGER1,
  WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_GENERAL_TRIGGER2
};

enum WeWalkDebugUIHandHeldSelectedAlgorithm
{
  WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FREEWALKING,
  WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_PROPORTIONATE_ACCELERATION_CONTROL,
  WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_PURE_IMPEDANCE_CONTROL,
  WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_SINE_SWING,
  WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FULL_CONTROL_1,
  WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_TORQUE_PULSE
};

typedef struct
{
  uint16_t         manualControlTask;
}WeWalkDebugUIHandHeldManualControlHandle;

typedef struct
{
  SerialProtocolEchoHandle*                   hSerial;
  enum WeWalkDebugUIHandHeldMainTask               mainTask;
  enum WeWalkDebugUIHandHeldSelectedAlgorithm      algorithm;
  WeWalkDebugUIHandHeldManualControlHandle         manualControl;
  WeWalkUnilateralDataStruct                   dataLeft, dataRight;
  WeWalkUnilateralSettingsStruct               settingsLeft, settingsRight;
  uint32_t                                    dataLeftPtr, dataRightPtr, settingLeftPtr, settingRightPtr;
  uint32_t                                    particularFeedbackPtrLeft[10], particularFeedbackPtrRight[10];
  uint32_t                                    settingUploadLeftPtr, settingUploadRightPtr;
  uint32_t                                    settingValueToSend;
  uint32_t                                    loop_period_ms;
  uint8_t                                     bluetoothState;
  uint8_t                                     ifWeWalkIsOnline;
  uint32_t                                    wewalkOffLineThresholdCriteriaDuration;
  uint32_t                                    lastReceivedTimeStamp;
  uint32_t                                    countForTest;
}WeWalkDebugUIHandheldHandle;

WeWalkDebugUIHandheldHandle WEWALKDEBUGUIHANDHELD_Create(SerialProtocolEchoHandle* hserial, uint32_t loop_period_ms);

void WeWalkDebugUIHandheld_Host(WeWalkDebugUIHandheldHandle* huihandheld);
void WeWalkOnlineOfflineCheck(WeWalkDebugUIHandheldHandle* huihandheld);
void WeWalkDebugUIHandheld_ManualControlInit(WeWalkDebugUIHandheldHandle* huihandheld);
void WeWalkDebugUIHandheld_SendSetting(WeWalkDebugUIHandheldHandle* huihandheld);


#endif
