#ifndef WEWALK_APPLICATION_H
#define WEWALK_APPLICATION_H
/**************Includes**************/
#include "serial_protocol_echo.h"
#include "shared.h"
#include "my_math.h"

#define WEWALK_APPLICATION_TYPE_UNILATERAL_LEG_RIGHT     0
#define WEWALK_APPLICATION_TYPE_UNILATERAL_LEG_LEFT      1
#define WEWALK_APPLICATION_TYPE_BILATERAL_LEG            2



/******************************************/
/******************************************/
/******Main Scheduler & User Interface*****/
/******************************************/
/******************************************/

enum WEWALK_MAIN_TASK_ENUM
{
	WEWALK_MAIN_TASK_IDLE,
  WEWALK_MAIN_TASK_MANUAL_CONTROL,
	WEWALK_MAIN_TASK_WALK_ASSISTANCE
};

enum WEWALK_WALK_ASSISTANCE_MODE
{
  WEWALK_WALK_ASSISTANCE_FREE_WALKING,
  WEWALK_WALK_ASSISTANCE_ACC_PRO_ASSISTANCE,
  WEWALK_WALK_ASSISTANCE_PURE_IMPEDANCE_CONTROL,
  WEWALK_WALK_ASSISTANCE_SINE_SWING,
  WEWALK_WALK_ASSISTANCE_FULL_ASSISTANCE_1,
  WEWALK_WALK_ASSISTANCE_TORQUE_PULSE
};

enum WEWALK_UNILATERAL_MOTOR_CONTROL_MODE
{
  WEWALK_UNILATERAL_MOTOR_TASK_VELOCITY_CONTROL,
  WEWALK_UNILATERAL_MOTOR_TASK_INCREMENTAL_POSITION_CONTROL,
  WEWALK_UNILATERAL_MOTOR_TASK_CURRENT_CONTROL
};

enum WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM
{
  WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM_ETH,
  WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM_ICR
};

typedef struct
{
  uint8_t                                   ifEnabled;
  enum WEWALK_UNILATERAL_MOTOR_CONTROL_MODE  controlMode;
  union FloatUInt8                          cmd;
}WeWalkMotorCommand;

typedef struct
{
  float        t0TimeSecond;
}SineSwingHandle;

typedef struct
{
  float t0TimeMilliSecond;
  float direction;
}TorquePulseHandle;

typedef struct
{
  float targetPositionAccX, targetPositionAccY, targetPositionAccNorm;
  
}AccelerationBasedGaitPhaseDetectorHandle;

typedef struct
{
  enum WEWALK_UNILATERAL_GAIT_PHASE_DETECTION_ALGORITHM      gaitPhaseDetectionType;
  LowPassFilterHandle                                        hFilterGaitPhaseDetectionIndicator;
}GaitPhaseDetectionHandle;

typedef struct
{
  WeWalkUnilateralDataStruct             data;
  WeWalkUnilateralSettingsStruct         setting;
  uint32_t                               dataPtr, settingPtr;
  TIM_HandleTypeDef*                     hLEDLogoPWM;
  uint32_t                               ledLogoPWMChannel;
  WeWalkMotorCommand                     motorCommand;
  SineSwingHandle                        hSineSwing;
  TorquePulseHandle                      hTorquePulse;
  GaitPhaseDetectionHandle*              hGaitPhaseDetect;
}WeWalkUnilateralHandle;

typedef struct
{
  SerialProtocolEchoHandle serialUI;
  uint32_t debugCounter;
  union FloatUInt8 joystickX, joystickY;
  uint32_t                        lastUISignalReceivedTimestamp;
  uint8_t                         ifUIOnline;
  uint32_t                        uiOffLineThresholdCriteriaDuration;
}WeWalkUIHandle;

typedef struct
{
  WeWalkUIHandle*                 hUI;
  SerialProtocolEchoHandle*       serialPC;
  WeWalkUnilateralHandle*         left, *right;
  uint8_t                         application_type;
  enum WEWALK_MAIN_TASK_ENUM       mainTask;
  enum WEWALK_WALK_ASSISTANCE_MODE assistanceMode;
  uint8_t                         ifSaveSettingRequired;
}WeWalkHandle;

WeWalkUnilateralHandle WEWALKUNILATERAL_Create(TIM_HandleTypeDef* hledlogopwm, uint32_t pwm_channel, GaitPhaseDetectionHandle* hgaitphasedetection);

WeWalkHandle WEWALK_Create(WeWalkUIHandle* hui, SerialProtocolEchoHandle* hserial_pc, \
                         WeWalkUnilateralHandle* left_leg, WeWalkUnilateralHandle* right_leg, \
                         uint8_t application_type);

void WEWALK_Main(WeWalkHandle* hwewalk);
WeWalkUIHandle WEWALKUI_Create(void);
void WEWALKUI_Host(WeWalkHandle* hwewalk);
void WEWALKUI_ReceiveSettings(WeWalkHandle* hwewalk);
void WEWALKUI_SetLEDLogoPWMCCR(WeWalkUnilateralHandle* hwewalk, uint32_t ccr);
void WEWALKUI_SendNormalFeedbackData(WeWalkHandle* hwewalk);
void WEWALKUI_SendParticularFeedback(WeWalkHandle* hwewalk, uint8_t left_right);
void WEWALKUI_SendSettings(WeWalkHandle* hwewalk);
void WEWALKUI_OnlineOfflineCheck(WeWalkHandle* hwewalk);
/************************************************/
/************************************************/
/**************Gait Phase Detection**************/
/************************************************/
/************************************************/

GaitPhaseDetectionHandle WEWALK_GaitPhaseDetectionCreate(float loop_period_in_second, float gait_phase_detection_cutoff_frequency);
void WEWALK_GaitPhaseDetection(WeWalkUnilateralHandle* hwewalk);
/***********************************/
/***********************************/
/**************Control**************/
/***********************************/
/***********************************/

void WEWALK_UpdatePerception(WeWalkHandle* hwewalk);
void WEWALK_AccelerationProportionateControl(WeWalkHandle* hwewalk);
void WEWALK_FreeWalkingControl(WeWalkHandle* hwewalk);
void WEWALK_ImpedanceControl(WeWalkHandle* hwewalk);
void WEWALK_SineSwingInit(WeWalkUnilateralHandle* hwewalk);
void WEWALK_SineSwingControl(WeWalkUnilateralHandle* hwewalk);
void WEWALK_FullAssistance_1(WeWalkHandle* hwewalk);
void WEWALK_TorquePulseControlTrigger(WeWalkUnilateralHandle* hwewalk, float direction);
void WEWALK_TorquePulseControl(WeWalkUnilateralHandle* hwewalk);

#endif
