#ifndef SCREEN_WEWALKLIVEPLOT_ACCELEROMETERVIEW_HPP
#define SCREEN_WEWALKLIVEPLOT_ACCELEROMETERVIEW_HPP

#include <gui_generated/screen_wewalkliveplot_accelerometer_screen/SCREEN_WeWalkLivePlot_AccelerometerViewBase.hpp>
#include <gui/screen_wewalkliveplot_accelerometer_screen/SCREEN_WeWalkLivePlot_AccelerometerPresenter.hpp>

class SCREEN_WeWalkLivePlot_AccelerometerView : public SCREEN_WeWalkLivePlot_AccelerometerViewBase
{
public:
    SCREEN_WeWalkLivePlot_AccelerometerView();
    virtual ~SCREEN_WeWalkLivePlot_AccelerometerView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN_WEWALKLIVEPLOT_ACCELEROMETERVIEW_HPP
