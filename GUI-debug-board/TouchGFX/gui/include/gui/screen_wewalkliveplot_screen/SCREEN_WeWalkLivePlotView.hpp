#ifndef SCREEN_WEWALKLIVEPLOTVIEW_HPP
#define SCREEN_WEWALKLIVEPLOTVIEW_HPP

#include <gui_generated/screen_wewalkliveplot_screen/SCREEN_WeWalkLivePlotViewBase.hpp>
#include <gui/screen_wewalkliveplot_screen/SCREEN_WeWalkLivePlotPresenter.hpp>

class SCREEN_WeWalkLivePlotView : public SCREEN_WeWalkLivePlotViewBase
{
public:
    SCREEN_WeWalkLivePlotView();
    virtual ~SCREEN_WeWalkLivePlotView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void UpdateGraphs();
protected:
  
};

#endif // SCREEN_WEWALKLIVEPLOTVIEW_HPP
