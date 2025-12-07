#ifndef SCREEN_WEWALKVIEW_HPP
#define SCREEN_WEWALKVIEW_HPP

#include <gui_generated/screen_wewalk_screen/SCREEN_WeWalkViewBase.hpp>
#include <gui/screen_wewalk_screen/SCREEN_WeWalkPresenter.hpp>

class SCREEN_WeWalkView : public SCREEN_WeWalkViewBase
{
public:
    SCREEN_WeWalkView();
    virtual ~SCREEN_WeWalkView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    virtual void CheckSystemStatus();
};

#endif // SCREEN_WEWALKVIEW_HPP
