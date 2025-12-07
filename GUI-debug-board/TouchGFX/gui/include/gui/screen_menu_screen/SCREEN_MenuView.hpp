#ifndef SCREEN_MENUVIEW_HPP
#define SCREEN_MENUVIEW_HPP

#include <gui_generated/screen_menu_screen/SCREEN_MenuViewBase.hpp>
#include <gui/screen_menu_screen/SCREEN_MenuPresenter.hpp>

class SCREEN_MenuView : public SCREEN_MenuViewBase
{
public:
    SCREEN_MenuView();
    virtual ~SCREEN_MenuView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN_MENUVIEW_HPP
