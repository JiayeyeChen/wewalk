#include <gui/screen_wewalkliveplot_angle_screen/SCREEN_WeWalkLivePlot_AngleView.hpp>
#include <touchgfx/Color.hpp>
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
  
  touchgfx::Unicode::UnicodeChar num_str[15];
  Unicode::snprintfFloat(num_str, 10, "%#.5f", hUIHandheld.dataRight.angleShank[0].f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_ShankAngleXBuffer, num_str, 10);
  textArea_ShankAngleX.setWildcard(textArea_ShankAngleXBuffer);
  textArea_ShankAngleX.invalidate();
  
  Unicode::snprintfFloat(num_str, 10, "%#.5f", hUIHandheld.dataRight.angleShank[1].f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_ShankAngleYBuffer, num_str, 10);
  textArea_ShankAngleY.setWildcard(textArea_ShankAngleYBuffer);
  textArea_ShankAngleY.invalidate();
  

  Unicode::snprintfFloat(num_str, 10, "%#.5f", hUIHandheld.dataRight.angleShank[2].f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_ShankAngleZBuffer, num_str, 10);
  textArea_ShankAngleZ.setWildcard(textArea_ShankAngleZBuffer);
  textArea_ShankAngleZ.invalidate();
  

  Unicode::snprintfFloat(num_str, 10, "%#.5f", hUIHandheld.dataRight.angleThigh[0].f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_ThighAngleXBuffer, num_str, 10);
  textArea_ThighAngleX.setWildcard(textArea_ThighAngleXBuffer);
  textArea_ThighAngleX.invalidate();
  

  Unicode::snprintfFloat(num_str, 10, "%#.5f", hUIHandheld.dataRight.angleThigh[1].f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_ThighAngleYBuffer, num_str, 10);
  textArea_ThighAngleY.setWildcard(textArea_ThighAngleYBuffer);
  textArea_ThighAngleY.invalidate();
  

  Unicode::snprintfFloat(num_str, 10, "%#.5f", hUIHandheld.dataRight.angleThigh[2].f);
  Unicode::strncpy((touchgfx::Unicode::UnicodeChar*)textArea_ThighAngleZBuffer, num_str, 10);
  textArea_ThighAngleZ.setWildcard(textArea_ThighAngleZBuffer);
  textArea_ThighAngleZ.invalidate();
}
