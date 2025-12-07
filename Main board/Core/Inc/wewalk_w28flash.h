#ifndef WEWALK_W28FLASH_H
#define WEWALK_W28FLASH_H

#include "main.h"
#include "w25q_mem.h"
//Right leg settings sector number
#define WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_RIGHT                                        0U
//Left leg settings sector number
#define WEWALK_FLASH_SECTOR_NUMBER_UNILATERAL_SETTINGS_LEFT                                         1U
//Right/Left leg settings page number (without offset)

#define WEWALK_UNILATERAL_NUMBER_OF_SETTINGS                                                        2U     //All settings stored as a 32-bit values regardness of type
//enum WeWalkUnilateralSettingsLocalPageNumber
//{
//  WEWALK_UNILATERAL_SETTINGS_LOCATION_FLASH_PAGE_NUMBER_LED_LOGO_BRIGHTNESS,
//  WEWALK_UNILATERAL_SETTINGS_LOCATION_FLASH_PAGE_NUMBER_QDD_FRICTION_COMPENSATION_CURRENT,
//  WEWALK_UNILATERAL_SETTINGS_LOCATION_FLASH_PAGE_NUMBER_QDD_KT
//};

void WEWALKFLASH_LoadSettings(WeWalkHandle* hwewalk);
void WEWALKFLASH_SaveSettings(WeWalkHandle* hwewalk);






extern enum WeWalkUnilateralSettingsLocalPageNumber setting_map;
#endif
