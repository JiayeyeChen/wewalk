#ifndef SCREEN_SETTINGSVIEW_HPP
#define SCREEN_SETTINGSVIEW_HPP

#include <gui_generated/screen_settings_screen/Screen_SettingsViewBase.hpp>
#include <gui/screen_settings_screen/Screen_SettingsPresenter.hpp>
#include <gui/common/CustomKeyboard.hpp>

class Screen_SettingsView : public Screen_SettingsViewBase
{
public:
    Screen_SettingsView();
    virtual ~Screen_SettingsView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void function_RightLogoPWMCCR_ButtonClicked();
    virtual void function_RightKt_ButtonClicked();
    virtual void function_RightFrictionCompensation_ButtonClicked();
    virtual void function_RightManualControlVelocityMultiplier_ButtonClicked();
    virtual void function_RightManualControlCurrentMultiplier_ButtonClicked();
    virtual void function_RightAccelerationProportionateAssistanceGain_ButtonClicked();
    virtual void function_RightAccelerationProportionateAssistanceSigmoidCoefficient_ButtonClicked();
    virtual void function_RightMotorKneeJointAngleOffset_ButtonClicked();
    virtual void function_RightImpedanceControlSpringCoefficient_ButtonClicked();
    virtual void function_RightImpedanceControlDampingCoefficient_ButtonClicked();
    virtual void function_RightSineSwingControlMagnitude_ButtonClicked();
    virtual void function_RightSineSwingControlFrequency_ButtonClicked();
    virtual void function_GPPa_ButtonClicked();
    virtual void function_GPPb_ButtonClicked();
    virtual void function_GPPc_ButtonClicked();
    virtual void function_GPPd_ButtonClicked();
    virtual void function_GPPe_ButtonClicked();
    virtual void function_GPPf_ButtonClicked();
    virtual void function_GPPg_ButtonClicked();
    virtual void function_GPPh_ButtonClicked();
    virtual void function_GPPi_ButtonClicked();
    virtual void function_GPPj_ButtonClicked();
    virtual void function_GPPk_ButtonClicked();
    virtual void function_GPPl_ButtonClicked();
    virtual void function_GPPm_ButtonClicked();
    virtual void function_GPPn_ButtonClicked();
    virtual void function_GPPo_ButtonClicked();
    virtual void function_GPPp_ButtonClicked();
    virtual void function_GPPq_ButtonClicked();
    virtual void function_GPPr_ButtonClicked();
    virtual void function_GPPs_ButtonClicked();
    virtual void function_GPPt_ButtonClicked();
    virtual void function_GPPu_ButtonClicked();
    virtual void function_GPPv_ButtonClicked();
    virtual void function_GPPw_ButtonClicked();
    virtual void function_GPPx_ButtonClicked();
    virtual void function_GPPy_ButtonClicked();
    virtual void function_GPPz_ButtonClicked();
protected:
    CustomKeyboard keyboard;
    int namemod;
    virtual void UploadButtonClicked();
    virtual void ReadButtonClicked();
    virtual void SaveButtonClicked();
};

#endif // SCREEN_SETTINGSVIEW_HPP
