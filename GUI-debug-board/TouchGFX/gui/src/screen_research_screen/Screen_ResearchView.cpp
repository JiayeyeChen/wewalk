#include <gui/screen_research_screen/Screen_ResearchView.hpp>
#include <touchgfx/Color.hpp>

extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

Screen_ResearchView::Screen_ResearchView()
{

}

void Screen_ResearchView::setupScreen()
{
    Screen_ResearchViewBase::setupScreen();
    hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
}

void Screen_ResearchView::tearDownScreen()
{
    Screen_ResearchViewBase::tearDownScreen();
}

void Screen_ResearchView::UpdateInformation()
{
  static char str_utf8_online[] = "Online ";
  static char str_utf8_offline[] = "Offline";
  static Unicode::UnicodeChar str[7];
  if (hUIHandheld.ifWeWalkIsOnline)
  {
    Unicode::fromUTF8((const uint8_t*)str_utf8_online,str, 7);
    textArea_SystemStatus.setColor(touchgfx::Color::getColorFromRGB(0, 255, 0));
    textArea_SystemStatus.setWildcard(str);
  }
  else
  {
    Unicode::fromUTF8((const uint8_t*)str_utf8_offline,str, 7);
    textArea_SystemStatus.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
    textArea_SystemStatus.setWildcard(str);
  }
  textArea_SystemStatus.invalidate();
  
}

void Screen_ResearchView::EnableAlgorithms()
{
  if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_FreeWalking)
    hUIHandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FREEWALKING;
  else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_AccelerationProportionateAssistance)
    hUIHandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_PROPORTIONATE_ACCELERATION_CONTROL;
  else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_PureImpedanceControl)
    hUIHandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_PURE_IMPEDANCE_CONTROL;
  else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_SinSwing)
    hUIHandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_SINE_SWING;
  else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_FullControl1)
    hUIHandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_FULL_CONTROL_1;
  else if (radioButtonGroup1.getSelectedRadioButton() == &radioButton_TorquePulseTest)
    hUIHandheld.algorithm = WEWALK_DEBUG_UI_HANDHELD_ALGORITHM_TORQUE_PULSE;
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_START_CONTROL;
}

void Screen_ResearchView::StopAlgorithms()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_CONTROL;
}

void Screen_ResearchView::RequestDatalogStart()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_START_DATALOG;
}

void Screen_ResearchView::RequestDatalogEnd()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_STOP_DATALOG;
}

void Screen_ResearchView::function_UpdateInformation()
{
  dynamicGraph_Torque.addDataPoint(hUIHandheld.dataRight.motorCur.f);
}

void Screen_ResearchView::function_GeneralTrigger1_ButtonClicked()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_GENERAL_TRIGGER1;
}

void Screen_ResearchView::function_GeneralTrigger2_ButtonClicked()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_GENERAL_TRIGGER2;
}
