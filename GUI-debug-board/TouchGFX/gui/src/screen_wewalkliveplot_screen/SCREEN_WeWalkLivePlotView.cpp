#include <gui/screen_wewalkliveplot_screen/SCREEN_WeWalkLivePlotView.hpp>
#include <touchgfx/Color.hpp>
extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}


SCREEN_WeWalkLivePlotView::SCREEN_WeWalkLivePlotView()
{

}

void SCREEN_WeWalkLivePlotView::setupScreen()
{
  SCREEN_WeWalkLivePlotViewBase::setupScreen();
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_NORMAL_DATA_FEEDBACK;
}

void SCREEN_WeWalkLivePlotView::tearDownScreen()
{
    SCREEN_WeWalkLivePlotViewBase::tearDownScreen();
}

void SCREEN_WeWalkLivePlotView::UpdateGraphs()
{
  hUIHandheld.mainTask = WEWALK_DEBUG_UI_HANDHELD_MAIN_TASK_REQUEST_NORMAL_DATA_FEEDBACK;
//  if (swipeContainer1.getSelectedPage() == 0)//Acc lower and upper page
//  {
//    memset(hUIHandheld.particularFeedbackPtrRight, 0xFF, sizeof(hUIHandheld.particularFeedbackPtrRight));
//    hUIHandheld.particularFeedbackPtrRight[0] = 0;//Acc lower X
//    hUIHandheld.particularFeedbackPtrRight[1] = 1;//Acc lower Y
//    hUIHandheld.particularFeedbackPtrRight[2] = 2;//Acc lower Z
//    hUIHandheld.particularFeedbackPtrRight[3] = 3;//Acc upper X
//    hUIHandheld.particularFeedbackPtrRight[4] = 4;//Acc upper Y
//    hUIHandheld.particularFeedbackPtrRight[5] = 5;//Acc upper Z
//  }
//  else if (swipeContainer1.getSelectedPage() == 1)//Acceleration gradient page
//  {
//    memset(hUIHandheld.particularFeedbackPtrRight, 0xFF, sizeof(hUIHandheld.particularFeedbackPtrRight));
//    hUIHandheld.particularFeedbackPtrRight[0] = 6;//Acc gradient X
//    hUIHandheld.particularFeedbackPtrRight[1] = 7;//Acc gradient Y
//    hUIHandheld.particularFeedbackPtrRight[2] = 8;//Acc gradient Z
//  }
//  else if (swipeContainer1.getSelectedPage() == 2)//Gyroz page
//  {
//    memset(hUIHandheld.particularFeedbackPtrRight, 0xFF, sizeof(hUIHandheld.particularFeedbackPtrRight));
//    hUIHandheld.particularFeedbackPtrRight[0] = 12;
//  }
//  else if (swipeContainer1.getSelectedPage() == 3)//Global angular acceleration page
//  {
//    memset(hUIHandheld.particularFeedbackPtrRight, 0xFF, sizeof(hUIHandheld.particularFeedbackPtrRight));
//    hUIHandheld.particularFeedbackPtrRight[0] = 21;
//    dynamicGraph_GlobalAngularAcceleration.addDataPoint(hUIHandheld.dataRight.angularAccelerationGlobal.f);
//  }
//  else if (swipeContainer1.getSelectedPage() == 4)//Torque page
//  {
//    memset(hUIHandheld.particularFeedbackPtrRight, 0xFF, sizeof(hUIHandheld.particularFeedbackPtrRight));
//    hUIHandheld.particularFeedbackPtrRight[0] = 16;//Motor torque
//    dynamicGraph_MotorTorque.addDataPoint(hUIHandheld.dataRight.motorTorque.f);
//    touchgfx::Unicode::UnicodeChar num_str[15];
//    Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.dataRight.motorTorque.f);
//    Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_MotorTorqueBuffer, num_str, 10);
//    textArea_MotorTorque.setWildcard(textArea_MotorTorqueBuffer);
//    textArea_MotorTorque.invalidate();
//  }
//  else if (swipeContainer1.getSelectedPage() == 5)//Gait Phase Indicator
//  {
//    dynamicGraph_GaitPhaseIndicator.addDataPoint(hUIHandheld.dataRight.gaitPhaseIndicator.f);
//    touchgfx::Unicode::UnicodeChar num_str[15];
//    Unicode::snprintfFloat(num_str, 10, "%#.5f", (const float)hUIHandheld.dataRight.gaitPhaseIndicator.f);
//    Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_GaitPhaseIndicatorBuffer, num_str, 10);
//    textArea_GaitPhaseIndicator.setWildcard(textArea_GaitPhaseIndicatorBuffer);
//    textArea_GaitPhaseIndicator.invalidate();
//    
//    if (hUIHandheld.dataRight.ifSwingPhase.b32)
//    {
//      boxWithBorder_SwingPhase.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
//      boxWithBorder_StancePhase.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
//      boxWithBorder_SwingPhase.invalidate();
//      boxWithBorder_StancePhase.invalidate();
//    }
//    else
//    {
//      boxWithBorder_SwingPhase.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
//      boxWithBorder_StancePhase.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
//      boxWithBorder_SwingPhase.invalidate();
//      boxWithBorder_StancePhase.invalidate();
//    }
//  }
}