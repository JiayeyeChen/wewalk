#include <gui/screen_wewalkliveplot_gyroscope_screen/SCREEN_WeWalkLivePlot_GyroscopeView.hpp>
extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}

SCREEN_WeWalkLivePlot_GyroscopeView::SCREEN_WeWalkLivePlot_GyroscopeView()
{

}

void SCREEN_WeWalkLivePlot_GyroscopeView::setupScreen()
{
    SCREEN_WeWalkLivePlot_GyroscopeViewBase::setupScreen();
}

void SCREEN_WeWalkLivePlot_GyroscopeView::tearDownScreen()
{
    SCREEN_WeWalkLivePlot_GyroscopeViewBase::tearDownScreen();
}

void SCREEN_WeWalkLivePlot_GyroscopeView::UpdateGraphs()
{
  graph_gyro_x_shank.addDataPoint(hUIHandheld.dataRight.gyroShank[0].f);
  graph_gyro_y_shank.addDataPoint(hUIHandheld.dataRight.gyroShank[1].f);
  graph_gyro_z_shank.addDataPoint(hUIHandheld.dataRight.gyroShank[2].f);
  
  graph_gyro_x_thigh.addDataPoint(hUIHandheld.dataRight.gyroThigh[0].f);
  graph_gyro_y_thigh.addDataPoint(hUIHandheld.dataRight.gyroThigh[1].f);
  graph_gyro_z_thigh.addDataPoint(hUIHandheld.dataRight.gyroThigh[2].f);
}
