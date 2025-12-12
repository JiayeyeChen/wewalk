#include <gui/screen_wewalkliveplot_accelerometer_screen/SCREEN_WeWalkLivePlot_AccelerometerView.hpp>
extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

SCREEN_WeWalkLivePlot_AccelerometerView::SCREEN_WeWalkLivePlot_AccelerometerView()
{

}

void SCREEN_WeWalkLivePlot_AccelerometerView::setupScreen()
{
    SCREEN_WeWalkLivePlot_AccelerometerViewBase::setupScreen();
}

void SCREEN_WeWalkLivePlot_AccelerometerView::tearDownScreen()
{
    SCREEN_WeWalkLivePlot_AccelerometerViewBase::tearDownScreen();
}

void SCREEN_WeWalkLivePlot_AccelerometerView::UpdateGraphs()
{
  graph_acc_x_shank.addDataPoint(hUIHandheld.dataRight.accShank[0].f);
  graph_acc_y_shank.addDataPoint(hUIHandheld.dataRight.accShank[1].f);
  graph_acc_z_shank.addDataPoint(hUIHandheld.dataRight.accShank[2].f);
  
  graph_acc_x_thigh.addDataPoint(hUIHandheld.dataRight.accThigh[0].f);
  graph_acc_y_thigh.addDataPoint(hUIHandheld.dataRight.accThigh[1].f);
  graph_acc_z_thigh.addDataPoint(hUIHandheld.dataRight.accThigh[2].f);
}
