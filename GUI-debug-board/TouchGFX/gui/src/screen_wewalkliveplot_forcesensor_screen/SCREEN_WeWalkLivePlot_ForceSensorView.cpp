#include <gui/screen_wewalkliveplot_forcesensor_screen/SCREEN_WeWalkLivePlot_ForceSensorView.hpp>
extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

SCREEN_WeWalkLivePlot_ForceSensorView::SCREEN_WeWalkLivePlot_ForceSensorView()
{

}

void SCREEN_WeWalkLivePlot_ForceSensorView::setupScreen()
{
    SCREEN_WeWalkLivePlot_ForceSensorViewBase::setupScreen();
}

void SCREEN_WeWalkLivePlot_ForceSensorView::tearDownScreen()
{
    SCREEN_WeWalkLivePlot_ForceSensorViewBase::tearDownScreen();
}

void SCREEN_WeWalkLivePlot_ForceSensorView::UpdateGraphs()
{
  graph_toe.addDataPoint(hUIHandheld.dataRight.forceSensorToe.f);
  graph_heel.addDataPoint(hUIHandheld.dataRight.forceSensorHeel.f);
}
