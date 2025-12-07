#ifndef SCREEN_RESEARCHVIEW_HPP
#define SCREEN_RESEARCHVIEW_HPP

#include <gui_generated/screen_research_screen/Screen_ResearchViewBase.hpp>
#include <gui/screen_research_screen/Screen_ResearchPresenter.hpp>

class Screen_ResearchView : public Screen_ResearchViewBase
{
public:
    Screen_ResearchView();
    virtual ~Screen_ResearchView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    virtual void UpdateInformation();
    virtual void EnableAlgorithms();
    virtual void StopAlgorithms();
    virtual void RequestDatalogStart();
    virtual void RequestDatalogEnd();
    virtual void function_UpdateInformation();
    virtual void function_GeneralTrigger1_ButtonClicked();
    virtual void function_GeneralTrigger2_ButtonClicked();
};

#endif // SCREEN_RESEARCHVIEW_HPP
