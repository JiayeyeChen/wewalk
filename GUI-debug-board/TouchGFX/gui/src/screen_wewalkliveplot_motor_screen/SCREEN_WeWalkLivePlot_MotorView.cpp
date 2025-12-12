#include <gui/screen_wewalkliveplot_motor_screen/SCREEN_WeWalkLivePlot_MotorView.hpp>
extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

SCREEN_WeWalkLivePlot_MotorView::SCREEN_WeWalkLivePlot_MotorView()
{

}

void SCREEN_WeWalkLivePlot_MotorView::setupScreen()
{
    SCREEN_WeWalkLivePlot_MotorViewBase::setupScreen();
}

void SCREEN_WeWalkLivePlot_MotorView::tearDownScreen()
{
    SCREEN_WeWalkLivePlot_MotorViewBase::tearDownScreen();
}

void SCREEN_WeWalkLivePlot_MotorView::UpdateGraphs()
{
  graph_torque.addDataPoint(hUIHandheld.dataRight.motorTorque.f);
  graph_current.addDataPoint(hUIHandheld.dataRight.motorCur.f);
  graph_angle.addDataPoint(hUIHandheld.dataRight.motorPos.f);
  graph_velocity.addDataPoint(hUIHandheld.dataRight.motorVel.f);
}
