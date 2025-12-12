#ifndef SCREEN_WEWALKLIVEPLOT_FORCESENSORVIEW_HPP
#define SCREEN_WEWALKLIVEPLOT_FORCESENSORVIEW_HPP

#include <gui_generated/screen_wewalkliveplot_forcesensor_screen/SCREEN_WeWalkLivePlot_ForceSensorViewBase.hpp>
#include <gui/screen_wewalkliveplot_forcesensor_screen/SCREEN_WeWalkLivePlot_ForceSensorPresenter.hpp>

class SCREEN_WeWalkLivePlot_ForceSensorView : public SCREEN_WeWalkLivePlot_ForceSensorViewBase
{
public:
    SCREEN_WeWalkLivePlot_ForceSensorView();
    virtual ~SCREEN_WeWalkLivePlot_ForceSensorView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void UpdateGraphs();
protected:
};

#endif // SCREEN_WEWALKLIVEPLOT_FORCESENSORVIEW_HPP
