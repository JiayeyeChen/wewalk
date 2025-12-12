#ifndef SCREEN_WEWALKLIVEPLOT_ANGLEVIEW_HPP
#define SCREEN_WEWALKLIVEPLOT_ANGLEVIEW_HPP

#include <gui_generated/screen_wewalkliveplot_angle_screen/SCREEN_WeWalkLivePlot_AngleViewBase.hpp>
#include <gui/screen_wewalkliveplot_angle_screen/SCREEN_WeWalkLivePlot_AnglePresenter.hpp>

class SCREEN_WeWalkLivePlot_AngleView : public SCREEN_WeWalkLivePlot_AngleViewBase
{
public:
    SCREEN_WeWalkLivePlot_AngleView();
    virtual ~SCREEN_WeWalkLivePlot_AngleView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN_WEWALKLIVEPLOT_ANGLEVIEW_HPP
