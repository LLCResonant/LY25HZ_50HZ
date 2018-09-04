/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : Epwm.h 
 *
 *  PURPOSE  : Header file for EPwm.c
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-16      V0.1           Ken      	    Created
 *
 *
 *============================================================================*/


#ifndef EPWM_H
#define EPWM_H

#include "IQmathLib.h"

/* 288 sampling points in 20ms, 48*3 = 144(10ms), 144*2(20ms) = 288 */
/* 20KHz switching frequency for AD interruption */
#define AD_FREQ		20000

/* (period) for AD interruption with 30MHz TBCLK */
#define AD_PERIOD							(HSPCLK_FREQ/AD_FREQ)

/* (period) for AD interruption with 30MHz TBCLK */
#define AD_HALF_PERIOD				((HSPCLK_FREQ/AD_FREQ)/2)
#define AD_QUARTER_PERIOD			((HSPCLK_FREQ/AD_FREQ)/4)

/* 15KHz switching frequency for SCR PWM */
#define PWM_FREQ	15000
/* (period) for PWM  with 30MHz TBCLK */
#define PWM_PERIOD	(HSPCLK_FREQ/PWM_FREQ)

/* (period) for PWM  with 30MHz TBCLK */
#define PWM_HALF_PERIOD			((HSPCLK_FREQ/PWM_FREQ)/2)
#define PWM_QUARTER_PERIOD		((HSPCLK_FREQ/PWM_FREQ)/4)

/* 5us deadband with 30MHz TBCLK
#define DeadBand_Duration	50*/

/* Enable PWM output */
extern void SCR_INV_Enable(void);
extern void SCR_Bypass_Enable(void);
/* Disable PWM output */
extern void SCR_INV_Disable(void);
extern void SCR_Bypass_Disable(void);












#endif

//--- end of file -----------------------------------------------------

