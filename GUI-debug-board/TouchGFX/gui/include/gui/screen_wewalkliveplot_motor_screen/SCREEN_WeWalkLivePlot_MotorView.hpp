#ifndef SCREEN_WEWALKLIVEPLOT_MOTORVIEW_HPP
#define SCREEN_WEWALKLIVEPLOT_MOTORVIEW_HPP

#include <gui_generated/screen_wewalkliveplot_motor_screen/SCREEN_WeWalkLivePlot_MotorViewBase.hpp>
#include <gui/screen_wewalkliveplot_motor_screen/SCREEN_WeWalkLivePlot_MotorPresenter.hpp>

class SCREEN_WeWalkLivePlot_MotorView : public SCREEN_WeWalkLivePlot_MotorViewBase
{
public:
    SCREEN_WeWalkLivePlot_MotorView();
    virtual ~SCREEN_WeWalkLivePlot_MotorView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN_WEWALKLIVEPLOT_MOTORVIEW_HPP
