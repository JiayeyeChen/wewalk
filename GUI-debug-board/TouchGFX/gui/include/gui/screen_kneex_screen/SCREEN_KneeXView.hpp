#ifndef SCREEN_KNEEXVIEW_HPP
#define SCREEN_KNEEXVIEW_HPP

#include <gui_generated/screen_kneex_screen/SCREEN_KneeXViewBase.hpp>
#include <gui/screen_kneex_screen/SCREEN_KneeXPresenter.hpp>

class SCREEN_KneeXView : public SCREEN_KneeXViewBase
{
public:
    SCREEN_KneeXView();
    virtual ~SCREEN_KneeXView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    virtual void CheckSystemStatus();
};

#endif // SCREEN_KNEEXVIEW_HPP
