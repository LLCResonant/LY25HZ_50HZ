/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : Epwm.h
 *
 *  PURPOSE  :
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/

#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"				// Main include file

#ifndef EPWM_H
#define EPWM_H

/*=============================================================================*
 * 	Extern functions declaration
 *============================================================================*/
/* ePWM module initialization */


/* Start ePWM timer5 for ADC SOC by PWM module */
//extern void StartEPWM5_TimerForADC(void);

/* Start ePWM timer6 for GeneralADC */
//extern void StartEPWM6_Timer(void);

/* Enable PWM output */
extern void PfcPWMOutputsEnable(void);
extern void InvPWMOutputsEnable(void);

/* Disable PWM output */
extern void PfcPWMOutputsDisable(void);
extern void InvPWMOutputsDisable(void);


#endif

//--- end of file -----------------------------------------------------

