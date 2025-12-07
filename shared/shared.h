#ifndef SHARED_H
#define SHARED_H

#define       UI_SERIAL_COMMAND_REQUEST_HEARTBEAT                 0x0000
#define       UI_SERIAL_COMMAND_REQUEST_NORMAL_FEEDBACK           0x0001
#define       UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_LEFT  0x0002
#define       UI_SERIAL_COMMAND_REQUEST_PARTICULAR_FEEDBACK_RIGHT 0x0003
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL                    0x0004
#define       UI_SERIAL_COMMAND_START_CONTROL                     0x0005
#define       UI_SERIAL_COMMAND_STOP_CONTROL                      0x0006
#define       UI_SERIAL_COMMAND_READ_SETTINGS                     0x0007
#define       UI_SERIAL_COMMAND_SAVE_SETTINGS                     0x0008
#define       UI_SERIAL_COMMAND_UPLOAD_SETTINGS                   0x0009
#define       UI_SERIAL_COMMAND_START_DATALOG                     0x000A
#define       UI_SERIAL_COMMAND_STOP_DATALOG                      0x000B
#define       UI_SERIAL_COMMAND_GENERAL_TRIGGER_1                 0x000C
#define       UI_SERIAL_COMMAND_GENERAL_TRIGGER_2                 0x000D


#define       UI_SERIAL_COMMAND_CONTROL_MODE_FREE_WALKING           0x0000
#define       UI_SERIAL_COMMAND_CONTROL_MODE_ACC_PRO_ASSISTANCE     0x0001
#define       UI_SERIAL_COMMAND_CONTROL_MODE_PURE_IMPEDANCE_CONTROL 0x0002
#define       UI_SERIAL_COMMAND_CONTROL_MODE_SINE_SWING             0x0003
#define       UI_SERIAL_COMMAND_CONTROL_MODE_FULL_ASSISTANCE_1      0x0004
#define       UI_SERIAL_COMMAND_CONTROL_MODE_TORQUE_PULSE           0x0005

#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_DISABLE                 0x0000
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_POSITION_CONTROL_LEFT   0x0001
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_VELOCITY_CONTROL_LEFT   0x0002
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_CURRENT_CONTROL_LEFT    0x0003
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_POSITION_CONTROL_RIGHT  0x0004
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_VELOCITY_CONTROL_RIGHT  0x0005
#define       UI_SERIAL_COMMAND_MANUAL_CONTROL_CURRENT_CONTROL_RIGHT   0x0006


#define       WEWALK_APPLICATION_TYPE_UNILATERAL_LEFT                      0x00
#define       WEWALK_APPLICATION_TYPE_UNILATERAL_RIGHT                     0x01
#define       WEWALK_APPLICATION_TYPE_BILATERAL                            0x02

#include "common.h"

typedef struct
{
  union FloatUInt8 \
    accThigh[3], accShank[3], gyroThigh[3], gyroShank[3], \
    motorCur, motorTorque, motorPos, motorPosRaw, motorPosCos, motorVel, \
    angularAccelerationGlobal;
  union UInt32UInt8 ifSwingPhase;
  union FloatUInt8 gaitPhaseIndicator;
}WeWalkUnilateralDataStruct;

typedef struct
{
  union FloatUInt8 motorKt, motorFrictionCompensationCurrent;
  uint32_t         ledLogoBrightnessPWMCCR;
  union FloatUInt8 motorManualcontrolVelocityMultiplier, \
                   motorManualcontrolCurrentMultiplier;
  union FloatUInt8 accelerationProportionateAssistanceGain, accelerationProportionateAssistanceSigmoidCoefficient;
  union FloatUInt8 motorKneeJointAngleOffset;
  union FloatUInt8 impedanceControlSpringCoefficient, impedanceControlDampingCoefficient;
  union FloatUInt8 sineSwingControlMagnitude, sineSwingControlFrequency;//#11
  union FloatUInt8 accelerationFieldMethodGaitPhaseDetectionXPosition, accelerationFieldMethodGaitPhaseDetectionYPosition; //#13
  union FloatUInt8 accelerationFieldMethodGaitPhaseDetectionThreshold; //#14
  union FloatUInt8 gppA, gppB, gppC, gppD, gppE, gppF, gppG, gppH; //#22
}WeWalkUnilateralSettingsStruct;



#endif
