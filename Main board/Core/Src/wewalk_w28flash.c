#include "wewalk_w28flash.h"

void WEWALKFLASH_LoadSettings(WeWalkHandle* hwewalk)
{
//  //LED Logo brightness
//  setting_map = WEWALK_UNILATERAL_SETTINGS_LOCATION_FLASH_PAGE_NUMBER_LED_LOGO_BRIGHTNESS;
//  W25Q_ReadData((u8_t*)&hwewalk->right->setting.ledLogoBrightnessPWMCCR, 4, WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT * 16, (u32_t) setting_map);
//  //Motor friction compensation current
//  setting_map = WEWALK_UNILATERAL_SETTINGS_LOCATION_FLASH_PAGE_NUMBER_QDD_FRICTION_COMPENSATION_CURRENT;
//  W25Q_ReadData((u8_t*)&hwewalk->right->setting.motorFrictionCompensationCurrent.b8[0], 4, WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT * 16, (u32_t) setting_map);
//  //Motor Kt
//  setting_map = WEWALK_UNILATERAL_SETTINGS_LOCATION_FLASH_PAGE_NUMBER_QDD_KT;
//  W25Q_ReadData((u8_t*)&hwewalk->right->setting.motorKt.b8[0], 4, WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT * 16, (u32_t) setting_map);
  for (uint16_t i = 0; i < sizeof(hwewalk->right->setting) / 4; i++)
  {
    W25Q_ReadData(&hwewalk->right->setting.motorKt.b8[0] + 4 * i, 4, WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT * 16, i);
  }
}

void WEWALKFLASH_SaveSettings(WeWalkHandle* hwewalk)
{
  W25Q_EraseSector(WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT);
  W25Q_EraseSector(WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT + 1);
  for (uint16_t i = 0; i < sizeof(hwewalk->right->setting) / 4; i++)
  {
    W25Q_ProgramData(&hwewalk->right->setting.motorKt.b8[0] + 4 * i, 4, WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT * 16, i);
  }
}
