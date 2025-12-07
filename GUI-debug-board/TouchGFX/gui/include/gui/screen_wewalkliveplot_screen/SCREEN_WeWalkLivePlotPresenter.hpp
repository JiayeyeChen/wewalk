#ifndef SCREEN_WEWALKLIVEPLOTPRESENTER_HPP
#define SCREEN_WEWALKLIVEPLOTPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SCREEN_WeWalkLivePlotView;

class SCREEN_WeWalkLivePlotPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SCREEN_WeWalkLivePlotPresenter(SCREEN_WeWalkLivePlotView& v);

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

    virtual ~SCREEN_WeWalkLivePlotPresenter() {}

private:
    SCREEN_WeWalkLivePlotPresenter();

    SCREEN_WeWalkLivePlotView& view;
};

#endif // SCREEN_WEWALKLIVEPLOTPRESENTER_HPP
