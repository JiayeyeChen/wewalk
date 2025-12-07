#ifndef SCREEN_MENUPRESENTER_HPP
#define SCREEN_MENUPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SCREEN_MenuView;

class SCREEN_MenuPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SCREEN_MenuPresenter(SCREEN_MenuView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~SCREEN_MenuPresenter() {}

private:
    SCREEN_MenuPresenter();

    SCREEN_MenuView& view;
};

#endif // SCREEN_MENUPRESENTER_HPP
