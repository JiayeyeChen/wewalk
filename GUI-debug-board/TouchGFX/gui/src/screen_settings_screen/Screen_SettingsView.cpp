#include <gui/screen_settings_screen/Screen_SettingsView.hpp>

extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

Screen_SettingsView::Screen_SettingsView()
{
  keyboard.setPosition(480, 240, 320, 240);
  add(keyboard);
}

void Screen_SettingsView::setupScreen()
{
  Screen_SettingsViewBase::setupScreen();
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_CONTROL;
}

void Screen_SettingsView::tearDownScreen()
{
    Screen_SettingsViewBase::tearDownScreen();
}

void Screen_SettingsView::UploadButtonClicked()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_UPLOAD_SETTINGS;
  
  
  if (hUIHandheld.settingUploadRightPtr == 2)//Cases for all integers
    hUIHandheld.settingValueToSend = Unicode::atoi(keyboard.getBuffer());
  else//Cases for float
  {
    char str_utf8[15];
    Unicode::toUTF8(keyboard.getBuffer(), (uint8_t*)str_utf8, 15);
    float tempF = (float)atof((const char*)str_utf8);
    memcpy(&hUIHandheld.settingValueToSend, &tempF, 4);
  }
  WeWalkDebugUIHandheld_SendSetting(&hUIHandheld);
}

void Screen_SettingsView::ReadButtonClicked()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_READ_SETTINGS;
  touchgfx::Unicode::UnicodeChar num_str[15];
  Unicode::utoa(hUIHandheld.settingsRight.ledLogoBrightnessPWMCCR, num_str, 10, 10);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightLogoPWMCCRBuffer, num_str, 10);
  textArea_RightLogoPWMCCR.setWildcard(textArea_RightLogoPWMCCRBuffer);
  textArea_RightLogoPWMCCR.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.motorKt.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightKtBuffer, num_str, 10);
  textArea_RightKt.setWildcard(textArea_RightKtBuffer);
  textArea_RightKt.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.motorFrictionCompensationCurrent.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightFrictionCompensationCurrentBuffer, num_str, 10);
  textArea_RightFrictionCompensationCurrent.setWildcard(textArea_RightFrictionCompensationCurrentBuffer);
  textArea_RightFrictionCompensationCurrent.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.motorManualcontrolVelocityMultiplier.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightManualControlVelocityMultiplierBuffer, num_str, 10);
  textArea_RightManualControlVelocityMultiplier.setWildcard(textArea_RightManualControlVelocityMultiplierBuffer);
  textArea_RightManualControlVelocityMultiplier.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.motorManualcontrolCurrentMultiplier.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightManualControlCurrentMultiplierBuffer, num_str, 10);
  textArea_RightManualControlCurrentMultiplier.setWildcard(textArea_RightManualControlCurrentMultiplierBuffer);
  textArea_RightManualControlCurrentMultiplier.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.accelerationProportionateAssistanceGain.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightAccelerationProportionateAssistanceGainBuffer, num_str, 10);
  textArea_RightAccelerationProportionateAssistanceGain.setWildcard(textArea_RightAccelerationProportionateAssistanceGainBuffer);
  textArea_RightAccelerationProportionateAssistanceGain.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.accelerationProportionateAssistanceSigmoidCoefficient.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightAccelerationProportionateAssistanceSigmoidCoefficientBuffer, num_str, 10);
  textArea_RightAccelerationProportionateAssistanceSigmoidCoefficient.setWildcard(textArea_RightAccelerationProportionateAssistanceSigmoidCoefficientBuffer);
  textArea_RightAccelerationProportionateAssistanceSigmoidCoefficient.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.motorKneeJointAngleOffset.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightMotorKneeJointAngleOffsetBuffer, num_str, 10);
  textArea_RightMotorKneeJointAngleOffset.setWildcard(textArea_RightMotorKneeJointAngleOffsetBuffer);
  textArea_RightMotorKneeJointAngleOffset.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.impedanceControlSpringCoefficient.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightImpedanceControlSpringCoefficientBuffer, num_str, 10);
  textArea_RightImpedanceControlSpringCoefficient.setWildcard(textArea_RightImpedanceControlSpringCoefficientBuffer);
  textArea_RightImpedanceControlSpringCoefficient.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.impedanceControlDampingCoefficient.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightImpedanceControlDampingCoefficientBuffer, num_str, 10);
  textArea_RightImpedanceControlDampingCoefficient.setWildcard(textArea_RightImpedanceControlDampingCoefficientBuffer);
  textArea_RightImpedanceControlDampingCoefficient.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.sineSwingControlMagnitude.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightSineSwingControlMagnitudeBuffer, num_str, 10);
  textArea_RightSineSwingControlMagnitude.setWildcard(textArea_RightSineSwingControlMagnitudeBuffer);
  textArea_RightSineSwingControlMagnitude.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.sineSwingControlFrequency.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightSineSwingControlFrequencyBuffer, num_str, 10);
  textArea_RightSineSwingControlFrequency.setWildcard(textArea_RightSineSwingControlFrequencyBuffer);
  textArea_RightSineSwingControlFrequency.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.accelerationFieldMethodGaitPhaseDetectionXPosition.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightAccFieldMethodTargetPositionXBuffer, num_str, 10);
  textArea_RightAccFieldMethodTargetPositionX.setWildcard(textArea_RightAccFieldMethodTargetPositionXBuffer);
  textArea_RightAccFieldMethodTargetPositionX.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.accelerationFieldMethodGaitPhaseDetectionYPosition.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightAccFieldMethodTargetPositionYBuffer, num_str, 10);
  textArea_RightAccFieldMethodTargetPositionY.setWildcard(textArea_RightAccFieldMethodTargetPositionYBuffer);
  textArea_RightAccFieldMethodTargetPositionY.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.accelerationFieldMethodGaitPhaseDetectionThreshold.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightAccFieldMethodThresholdBuffer, num_str, 10);
  textArea_RightAccFieldMethodThreshold.setWildcard(textArea_RightAccFieldMethodThresholdBuffer);
  textArea_RightAccFieldMethodThreshold.invalidate();
  
  ////////////////General Purpose Parameters////////////////
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppA.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPaBuffer, num_str, 10);
  textArea_RightGPPa.setWildcard(textArea_RightGPPaBuffer);
  textArea_RightGPPa.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppB.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPbBuffer, num_str, 10);
  textArea_RightGPPb.setWildcard(textArea_RightGPPbBuffer);
  textArea_RightGPPb.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppC.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPcBuffer, num_str, 10);
  textArea_RightGPPc.setWildcard(textArea_RightGPPcBuffer);
  textArea_RightGPPc.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppD.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPdBuffer, num_str, 10);
  textArea_RightGPPd.setWildcard(textArea_RightGPPdBuffer);
  textArea_RightGPPd.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppE.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPeBuffer, num_str, 10);
  textArea_RightGPPe.setWildcard(textArea_RightGPPeBuffer);
  textArea_RightGPPe.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppF.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPfBuffer, num_str, 10);
  textArea_RightGPPf.setWildcard(textArea_RightGPPfBuffer);
  textArea_RightGPPf.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppG.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPgBuffer, num_str, 10);
  textArea_RightGPPg.setWildcard(textArea_RightGPPgBuffer);
  textArea_RightGPPg.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.settingsRight.gppH.f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_RightGPPhBuffer, num_str, 10);
  textArea_RightGPPh.setWildcard(textArea_RightGPPhBuffer);
  textArea_RightGPPh.invalidate();
}

void Screen_SettingsView::SaveButtonClicked()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_SAVE_SETTINGS;
}

void Screen_SettingsView::function_RightLogoPWMCCR_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 2;
}
void Screen_SettingsView::function_RightKt_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 0;
}
void Screen_SettingsView::function_RightFrictionCompensation_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 1;
}
void Screen_SettingsView::function_RightManualControlVelocityMultiplier_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 3;
}
void Screen_SettingsView::function_RightManualControlCurrentMultiplier_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 4;
}
void Screen_SettingsView::function_RightAccelerationProportionateAssistanceGain_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 5;
}
void Screen_SettingsView::function_RightAccelerationProportionateAssistanceSigmoidCoefficient_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 6;
}
void Screen_SettingsView::function_RightMotorKneeJointAngleOffset_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 7;
}
void Screen_SettingsView::function_RightImpedanceControlSpringCoefficient_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 8;
}
void Screen_SettingsView::function_RightImpedanceControlDampingCoefficient_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 9;
}
void Screen_SettingsView::function_RightSineSwingControlMagnitude_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 10;
}
void Screen_SettingsView::function_RightSineSwingControlFrequency_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 11;
}
void Screen_SettingsView::function_RightAccFieldMethodTargetPositionX_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 12;
}
void Screen_SettingsView::function_RightAccFieldMethodTargetPositionY_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 13;
}
void Screen_SettingsView::function_RightAccFieldMethodThreshold_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 14;
}
//////
void Screen_SettingsView::function_GPPa_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 15;
}
void Screen_SettingsView::function_GPPb_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 16;
}
void Screen_SettingsView::function_GPPc_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 17;
}
void Screen_SettingsView::function_GPPd_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 18;
}
void Screen_SettingsView::function_GPPe_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 19;
}
void Screen_SettingsView::function_GPPf_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 20;
}
void Screen_SettingsView::function_GPPg_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 21;
}
void Screen_SettingsView::function_GPPh_ButtonClicked()
{
  hUIHandheld.settingUploadRightPtr = 22;
}
