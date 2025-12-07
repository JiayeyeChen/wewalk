#include <gui/screen_wewalk_screen/SCREEN_WeWalkView.hpp>
#include <touchgfx/Color.hpp>

extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern WeWalkDebugUIHandheldHandle hUIHandheld;

}



SCREEN_WeWalkView::SCREEN_WeWalkView()
{

}

void SCREEN_WeWalkView::setupScreen()
{
  SCREEN_WeWalkViewBase::setupScreen();
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
}

void SCREEN_WeWalkView::tearDownScreen()
{
    SCREEN_WeWalkViewBase::tearDownScreen();
}

void SCREEN_WeWalkView::CheckSystemStatus()
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
