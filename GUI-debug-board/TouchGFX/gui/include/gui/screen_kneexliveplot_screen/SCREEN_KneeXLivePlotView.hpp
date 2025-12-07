#ifndef SCREEN_KNEEXLIVEPLOTVIEW_HPP
#define SCREEN_KNEEXLIVEPLOTVIEW_HPP

#include <gui_generated/screen_kneexliveplot_screen/SCREEN_KneeXLivePlotViewBase.hpp>
#include <gui/screen_kneexliveplot_screen/SCREEN_KneeXLivePlotPresenter.hpp>

class SCREEN_KneeXLivePlotView : public SCREEN_KneeXLivePlotViewBase
{
public:
    SCREEN_KneeXLivePlotView();
    virtual ~SCREEN_KneeXLivePlotView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    virtual void UpdateGraphs();
};

#endif // SCREEN_KNEEXLIVEPLOTVIEW_HPP
