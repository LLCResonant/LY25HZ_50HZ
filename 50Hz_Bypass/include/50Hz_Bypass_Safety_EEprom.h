/*
 * 50Hz_Bypass_Safety_EEprom.h
 *
 *  Created on: 2018-9-7
 *      Author: GAOXUN
 */

#ifndef SAFETY_EEPROM_H_
#define SAFETY_EEPROM_H_

struct Time
{
	Uint16 u16Second;
	Uint16 u16Minute;
	Uint16 u16Hour_L;
	Uint16 u16Hour_H;
	Uint16 u16OverFlow;
	Uint16 u16TimeCheck;
};
extern  struct Time  RunningTime;

extern void Times_ReadFromEEPROM();
extern void Times_WriteToEEPROM();
extern void Sample_ReadFromEEPROM();
extern void Sample_WriteToEEPROM();
extern void EEPROMParamDefault();

#endif /* 50HZ_BYPASS_SAFETY_EEPROM_H_ */
