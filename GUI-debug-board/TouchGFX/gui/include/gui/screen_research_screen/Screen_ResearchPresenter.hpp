#ifndef SCREEN_RESEARCHPRESENTER_HPP
#define SCREEN_RESEARCHPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen_ResearchView;

class Screen_ResearchPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen_ResearchPresenter(Screen_ResearchView& v);

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

    virtual ~Screen_ResearchPresenter() {}

private:
    Screen_ResearchPresenter();

    Screen_ResearchView& view;
};

#endif // SCREEN_RESEARCHPRESENTER_HPP
