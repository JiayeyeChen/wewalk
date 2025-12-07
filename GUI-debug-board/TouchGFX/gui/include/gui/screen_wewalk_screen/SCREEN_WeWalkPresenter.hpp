#ifndef SCREEN_WEWALKPRESENTER_HPP
#define SCREEN_WEWALKPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SCREEN_WeWalkView;

class SCREEN_WeWalkPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SCREEN_WeWalkPresenter(SCREEN_WeWalkView& v);

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

    virtual ~SCREEN_WeWalkPresenter() {}

private:
    SCREEN_WeWalkPresenter();

    SCREEN_WeWalkView& view;
};

#endif // SCREEN_WEWALKPRESENTER_HPP
