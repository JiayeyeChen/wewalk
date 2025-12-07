#include <gui/screen_kneex_screen/SCREEN_KneeXView.hpp>
#include <touchgfx/Color.hpp>

extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern LingXiUIHandHeldHandle hUIHandheld;

}



SCREEN_KneeXView::SCREEN_KneeXView()
{

}

void SCREEN_KneeXView::setupScreen()
{
    SCREEN_KneeXViewBase::setupScreen();
    hUIHandheld.mainTask = LINGXI_UI_HANDHELD_MAIN_TASK_REQUEST_HEARTBREAK;
}

void SCREEN_KneeXView::tearDownScreen()
{
    SCREEN_KneeXViewBase::tearDownScreen();
}

void SCREEN_KneeXView::CheckSystemStatus()
{
  static char str_utf8_online[] = "Online ";
  static char str_utf8_offline[] = "Offline";
  static Unicode::UnicodeChar str[7];
  if (hUIHandheld.ifLingXiIsOnline)
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
