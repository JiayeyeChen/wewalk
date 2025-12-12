#include "myTouchGFXUtilities.h"

MyTouchGFXUtilities_Joystick MYTOUCHGFXUTILITIES_JoystickCreate(volatile TouchStructure* htouch, uint16_t container_x, uint16_t container_y, uint16_t containerWidth, uint16_t rodRadius, float bufferRadius, uint32_t bufferColorNormal, uint32_t bufferColorWarning)
{
  MyTouchGFXUtilities_Joystick hjoy;
  hjoy.hTouchInfo = htouch;
  hjoy.containerPosX = container_x;
  hjoy.containerPosY = container_y;
  hjoy.containerWidth = containerWidth;
  hjoy.rodRadius = rodRadius;
  hjoy.rodCirclePosToDrawX = 0;
  hjoy.rodCirclePosToDrawY = 0;
  hjoy.joystickStaticCenterX = (float)hjoy.containerPosX + (float)hjoy.containerWidth / 2.0f;
  hjoy.joystickStaticCenterY = (float)hjoy.containerPosY + (float)hjoy.containerWidth / 2.0f;
  hjoy.joystickRadiusOfMotion = (float)hjoy.containerWidth / 2.0f;
  hjoy.radiusBuffer = bufferRadius;
  hjoy.outputX.f = 0.0f;
  hjoy.outputY.f = 0.0f;
  hjoy.ifWarningApproachingLimit = 0;
  hjoy.bufferAreaColorCodeNormal = bufferColorNormal;
  hjoy.bufferAreaColorCodeWarning = bufferColorWarning;
  return hjoy;
}

void MYTOUCHGFXUTILITIES_JoystickUpdate(MyTouchGFXUtilities_Joystick* hjoy)
{
  if (hjoy->hTouchInfo->flag)
  {
    float temp_distance = sqrtf(powf((float)hjoy->hTouchInfo->x[0] - hjoy->joystickStaticCenterX, 2.0f) + powf((float)hjoy->hTouchInfo->y[0] - hjoy->joystickStaticCenterY, 2.0f));
    if (temp_distance < hjoy->joystickRadiusOfMotion)
    {
      hjoy->ifWarningApproachingLimit = 0;
      hjoy->rodCirclePosToDrawX = hjoy->hTouchInfo->x[0] - hjoy->containerPosX - hjoy->rodRadius;
      hjoy->rodCirclePosToDrawY = hjoy->hTouchInfo->y[0] - hjoy->containerPosY - hjoy->rodRadius;
      hjoy->outputX.f = ((float)hjoy->hTouchInfo->x[0] - hjoy->joystickStaticCenterX) / hjoy->joystickRadiusOfMotion;
      hjoy->outputY.f = -((float)hjoy->hTouchInfo->y[0] - hjoy->joystickStaticCenterY) / hjoy->joystickRadiusOfMotion;
    }
    else if (temp_distance >= hjoy->joystickRadiusOfMotion && temp_distance <= hjoy->joystickRadiusOfMotion + hjoy->radiusBuffer)
    {
      hjoy->ifWarningApproachingLimit = 1;
      hjoy->rodCirclePosToDrawX = hjoy->hTouchInfo->x[0] - hjoy->containerPosX - hjoy->rodRadius;
      hjoy->rodCirclePosToDrawY = hjoy->hTouchInfo->y[0] - hjoy->containerPosY - hjoy->rodRadius;
      hjoy->outputX.f = ((float)hjoy->hTouchInfo->x[0] - hjoy->joystickStaticCenterX) / temp_distance;
      hjoy->outputY.f = -((float)hjoy->hTouchInfo->y[0] - hjoy->joystickStaticCenterY) / temp_distance;
    }
    else
    {
      hjoy->ifWarningApproachingLimit = 0;
      hjoy->rodCirclePosToDrawX = hjoy->containerWidth/2 - hjoy->rodRadius;
      hjoy->rodCirclePosToDrawY = hjoy->containerWidth/2 - hjoy->rodRadius;
      hjoy->outputX.f = 0.0f;
      hjoy->outputY.f = 0.0f;
    }
  }
  else
  {
    hjoy->ifWarningApproachingLimit = 0;
    hjoy->rodCirclePosToDrawX = hjoy->containerWidth/2 - hjoy->rodRadius;
    hjoy->rodCirclePosToDrawY = hjoy->containerWidth/2 - hjoy->rodRadius;
    hjoy->outputX.f = 0.0f;
    hjoy->outputY.f = 0.0f;
  }
}
