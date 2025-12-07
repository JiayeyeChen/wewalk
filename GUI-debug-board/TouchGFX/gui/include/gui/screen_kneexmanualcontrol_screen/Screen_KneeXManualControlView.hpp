#ifndef SCREEN_KNEEXMANUALCONTROLVIEW_HPP
#define SCREEN_KNEEXMANUALCONTROLVIEW_HPP

#include <gui_generated/screen_kneexmanualcontrol_screen/Screen_KneeXManualControlViewBase.hpp>
#include <gui/screen_kneexmanualcontrol_screen/Screen_KneeXManualControlPresenter.hpp>

class Screen_KneeXManualControlView : public Screen_KneeXManualControlViewBase
{
public:
    Screen_KneeXManualControlView();
    virtual ~Screen_KneeXManualControlView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    int tickCounter;
    void handleTickEvent();
    virtual void StartManualControl();
    virtual void StopManualControl();
};

#endif // SCREEN_KNEEXMANUALCONTROLVIEW_HPP
