#include <gui/screen_wewalkmanualcontrol_screen/Screen_WeWalkManualControlView.hpp>

extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern MyTouchGFXUtilities_Joystick hManualControlJoystick;
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

Screen_WeWalkManualControlView::Screen_WeWalkManualControlView()
{

}

void Screen_WeWalkManualControlView::setupScreen()
{
  Screen_WeWalkManualControlViewBase::setupScreen();
  uint16_t temp_r;
  joystick_rod_circle.getRadius(temp_r);
  hManualControlJoystick = MYTOUCHGFXUTILITIES_JoystickCreate(&touchInfo, joystick_container.getX(), joystick_container.getY(), joystick_container.getHeight(), temp_r, 50, 0x71748A, 0xF2A60F);
  WeWalkDebugUIHandheld_ManualControlInit(&hUIHandheld);
}

void Screen_WeWalkManualControlView::tearDownScreen()
{
    Screen_WeWalkManualControlViewBase::tearDownScreen();
}

void Screen_WeWalkManualControlView::handleTickEvent()
{
  tickCounter++;
  MYTOUCHGFXUTILITIES_JoystickUpdate(&hManualControlJoystick);
  if (hManualControlJoystick.ifWarningApproachingLimit)
    joystick_range_of_motion_circlePainter.setColor(hManualControlJoystick.bufferAreaColorCodeWarning);
  else
    joystick_range_of_motion_circlePainter.setColor(hManualControlJoystick.bufferAreaColorCodeNormal);
  joystick_range_of_motion_circle.invalidate();
  joystick_rod_circle.moveTo(hManualControlJoystick.rodCirclePosToDrawX,hManualControlJoystick.rodCirclePosToDrawY);
  
  touchgfx::Unicode::UnicodeChar num_str[15];
  Unicode::snprintfFloat(num_str, 10, "%#.1f", (const float)hUIHandheld.dataRight.motorPos.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightAngleBuffer, num_str, 10);
  textArea_RightAngle.setWildcard(textArea_RightAngleBuffer);
  textArea_RightAngle.invalidate();
  Unicode::snprintfFloat(num_str, 10, "%#.1f", (const float)hUIHandheld.dataRight.motorVel.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightVelocityBuffer, num_str, 10);
  textArea_RightVelocity.setWildcard(textArea_RightVelocityBuffer);
  textArea_RightVelocity.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.7f", (const float)(hUIHandheld.dataRight.motorCur.f * hUIHandheld.settingsRight.motorKt.f));
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightCurrentBuffer, num_str, 10);
  textArea_RightCurrent.setWildcard(textArea_RightCurrentBuffer);
  textArea_RightCurrent.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.1f", (const float)hUIHandheld.dataLeft.motorPos.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_LeftAngleBuffer, num_str, 10);
  textArea_LeftAngle.setWildcard(textArea_LeftAngleBuffer);
  textArea_LeftAngle.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.1f", (const float)hUIHandheld.dataLeft.motorVel.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_LeftVelocityBuffer, num_str, 10);
  textArea_LeftVelocity.setWildcard(textArea_LeftVelocityBuffer);
  textArea_LeftVelocity.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.1f", (const float)hUIHandheld.dataLeft.motorCur.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_LeftCurrentBuffer, num_str, 10);
  textArea_LeftCurrent.setWildcard(textArea_LeftCurrentBuffer);
  textArea_LeftCurrent.invalidate();
  
  
  if (hUIHandheld.ifWeWalkIsOnline)
    hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_MANUAL_CONTROL;
}

void Screen_WeWalkManualControlView::StartManualControl()
{
  if (radioButtonGroup2.getSelectedRadioButton() == &radioButton_LeftLeg)
  {
    if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_CurrentControl)
      hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_CURRENT_CONTROL_LEFT;
    else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_VelocityControl)
      hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_VELOCITY_CONTROL_LEFT;
    else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_PositionControl)
      hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_POSITION_CONTROL_LEFT;
  }
  else if (radioButtonGroup2.getSelectedRadioButton() == &radioButton_RightLeg)
  {
    if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_CurrentControl)
      hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_CURRENT_CONTROL_RIGHT;
    else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_VelocityControl)
      hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_VELOCITY_CONTROL_RIGHT;
    else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_PositionControl)
      hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_POSITION_CONTROL_RIGHT;
  }
  
  

}

void Screen_WeWalkManualControlView::StopManualControl()
{
  hUIHandheld.manualControl.manualControlTask = UI_SERIAL_COMMAND_MANUAL_CONTROL_DISABLE;
}
