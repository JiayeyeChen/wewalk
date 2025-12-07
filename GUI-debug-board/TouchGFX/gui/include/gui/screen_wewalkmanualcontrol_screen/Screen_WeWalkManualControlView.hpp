#ifndef SCREEN_WEWALKMANUALCONTROLVIEW_HPP
#define SCREEN_WEWALKMANUALCONTROLVIEW_HPP

#include <gui_generated/screen_wewalkmanualcontrol_screen/Screen_WeWalkManualControlViewBase.hpp>
#include <gui/screen_wewalkmanualcontrol_screen/Screen_WeWalkManualControlPresenter.hpp>

class Screen_WeWalkManualControlView : public Screen_WeWalkManualControlViewBase
{
public:
    Screen_WeWalkManualControlView();
    virtual ~Screen_WeWalkManualControlView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    int tickCounter;
    virtual void handleTickEvent();
    virtual void StartManualControl();
    virtual void StopManualControl();
};

#endif // SCREEN_WEWALKMANUALCONTROLVIEW_HPP
