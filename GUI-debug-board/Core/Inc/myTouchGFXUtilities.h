#ifndef MYTOUCHGFXUTILITIES_H
#define MYTOUCHGFXUTILITIES_H

#include "common.h"
#include "touch_800x480.h"

typedef struct
{
  volatile TouchStructure* hTouchInfo;
  uint16_t containerPosX, containerPosY, containerWidth, rodRadius;
  uint16_t rodCirclePosToDrawX, rodCirclePosToDrawY;
  uint32_t bufferAreaColorCodeNormal, bufferAreaColorCodeWarning;
  uint8_t ifWarningApproachingLimit;
  float radiusBuffer;
  float joystickStaticCenterX, joystickStaticCenterY, joystickRadiusOfMotion;
  union FloatUInt8 outputX, outputY;
}MyTouchGFXUtilities_Joystick;

MyTouchGFXUtilities_Joystick MYTOUCHGFXUTILITIES_JoystickCreate(volatile TouchStructure* htouch, uint16_t container_x, uint16_t container_y, uint16_t containerWidth, uint16_t rodRadius, float bufferRadius, uint32_t bufferColorNormal, uint32_t bufferColorWarning);

void MYTOUCHGFXUTILITIES_JoystickUpdate(MyTouchGFXUtilities_Joystick* hjoy);





#endif
