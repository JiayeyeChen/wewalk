#ifndef SCREEN_WEWALKLIVEPLOT_GYROSCOPEVIEW_HPP
#define SCREEN_WEWALKLIVEPLOT_GYROSCOPEVIEW_HPP

#include <gui_generated/screen_wewalkliveplot_gyroscope_screen/SCREEN_WeWalkLivePlot_GyroscopeViewBase.hpp>
#include <gui/screen_wewalkliveplot_gyroscope_screen/SCREEN_WeWalkLivePlot_GyroscopePresenter.hpp>

class SCREEN_WeWalkLivePlot_GyroscopeView : public SCREEN_WeWalkLivePlot_GyroscopeViewBase
{
public:
    SCREEN_WeWalkLivePlot_GyroscopeView();
    virtual ~SCREEN_WeWalkLivePlot_GyroscopeView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void UpdateGraphs();
protected:
};

#endif // SCREEN_WEWALKLIVEPLOT_GYROSCOPEVIEW_HPP
