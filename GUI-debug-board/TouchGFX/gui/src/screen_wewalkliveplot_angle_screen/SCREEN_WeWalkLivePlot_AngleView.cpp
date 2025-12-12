#include <gui/screen_wewalkliveplot_angle_screen/SCREEN_WeWalkLivePlot_AngleView.hpp>
extern "C"
{
  #include "main.h"
  #include "wewalk_debug_gui_interface.h"
  #include "touch_800x480.h"
  extern volatile TouchStructure touchInfo;
  extern WeWalkDebugUIHandheldHandle hUIHandheld;
}


SCREEN_WeWalkLivePlot_AngleView::SCREEN_WeWalkLivePlot_AngleView()
{

}

void SCREEN_WeWalkLivePlot_AngleView::setupScreen()
{
    SCREEN_WeWalkLivePlot_AngleViewBase::setupScreen();
}

void SCREEN_WeWalkLivePlot_AngleView::tearDownScreen()
{
    SCREEN_WeWalkLivePlot_AngleViewBase::tearDownScreen();
}

void SCREEN_WeWalkLivePlot_AngleView::UpdateGraphs()
{
  graph_angle_x_shank.addDataPoint(hUIHandheld.dataRight.angleShank[0].f);
  graph_angle_y_shank.addDataPoint(hUIHandheld.dataRight.angleShank[1].f);
  graph_angle_z_shank.addDataPoint(hUIHandheld.dataRight.angleShank[2].f);
  
  graph_angle_x_thigh.addDataPoint(hUIHandheld.dataRight.angleThigh[0].f);
  graph_angle_y_thigh.addDataPoint(hUIHandheld.dataRight.angleThigh[1].f);
  graph_angle_z_thigh.addDataPoint(hUIHandheld.dataRight.angleThigh[2].f);
}
