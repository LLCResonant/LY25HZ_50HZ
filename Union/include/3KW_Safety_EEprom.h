/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_Safety_EEprom.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/

#ifndef  SAFETY_EEPROM_H
#define SAFETY_EEPROM_H

struct Time
{
	Uint16 	Second;
	Uint16 	Minute;
	Uint16 	Hour_L;
	Uint16 	Hour_H;
	Uint16 	OverFlow;
	Uint16 	TimeCheck;
};
extern  struct Time  RunningTime;

extern void Times_ReadFromEEPROM();
extern void Times_WriteToEEPROM();
extern void VoltsRef_ReadFromEEPROM();
extern void VoltsRef_WriteToEEPROM();
extern void Sample_ReadFromEEPROM();
extern void Sample_WriteToEEPROM();
extern void EEPROMParamDefault();

#endif
/* 3KW_SAFETY_EEPROM_H_ */
