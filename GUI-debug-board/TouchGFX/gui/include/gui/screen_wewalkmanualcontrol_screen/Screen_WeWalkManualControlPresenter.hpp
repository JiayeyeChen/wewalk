#ifndef SCREEN_WEWALKMANUALCONTROLPRESENTER_HPP
#define SCREEN_WEWALKMANUALCONTROLPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen_WeWalkManualControlView;

class Screen_WeWalkManualControlPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen_WeWalkManualControlPresenter(Screen_WeWalkManualControlView& v);

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

    virtual ~Screen_WeWalkManualControlPresenter() {}

private:
    Screen_WeWalkManualControlPresenter();

    Screen_WeWalkManualControlView& view;
};

#endif // SCREEN_WEWALKMANUALCONTROLPRESENTER_HPP
