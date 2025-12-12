#ifndef SCREEN_WEWALKLIVEPLOT_GYROSCOPEPRESENTER_HPP
#define SCREEN_WEWALKLIVEPLOT_GYROSCOPEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SCREEN_WeWalkLivePlot_GyroscopeView;

class SCREEN_WeWalkLivePlot_GyroscopePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SCREEN_WeWalkLivePlot_GyroscopePresenter(SCREEN_WeWalkLivePlot_GyroscopeView& v);

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

    virtual ~SCREEN_WeWalkLivePlot_GyroscopePresenter() {}

private:
    SCREEN_WeWalkLivePlot_GyroscopePresenter();

    SCREEN_WeWalkLivePlot_GyroscopeView& view;
};

#endif // SCREEN_WEWALKLIVEPLOT_GYROSCOPEPRESENTER_HPP
