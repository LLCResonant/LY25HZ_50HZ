/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : EPwm.c 
 *
 *  PURPOSE  : ePWM Module configuration and Timer/PWMoutput En/Disable process..
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-26      V0.1           Ken      	    Created
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *      
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *    
 *============================================================================*/
 
//#include "DSP2803x_Device.h"		// Peripheral address definitions
#include "F28035_example.h"

/*=============================================================================*
 * FUNCTION: InitEPwm()
 * PURPOSE : Initializes the Enhanced PWM modules.
 *			 To precisely capture the most valuable sampling point L current 
 *			 with high frequency sampling for Energy, additional Timer5 and 
 *			 Timer6 are used to supply additional leading-type SOC and equal 
 *			 interval sampling, with :
 *			 TBCLK=SYSCLKOUT/(HSPCLKDIV*CLKDIV)=60MHz/(1*1)=60MHz   
 * INPUT: 
 *     void
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *
 * CALLED BY: 
 *     Main.c  
 * 
 *============================================================================*/
void InitEPwm(void)
{ // start of InitEPwm()

	/* Must disable the clock to the ePWM modules if you want all ePMW modules 
	   synchronized, according to : TMS320x2833x, 2823x System Control and 
	   Interrupts Reference Guide (Rev. C).pdf. */

	asm(" EALLOW");					// Enable EALLOW protected register access
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	asm(" EDIS");					// Disable EALLOW protected register access

	// Configure ePWM1/2/3/4/5/6 for symmetric PWM

	// ePWM1 Configuration   用于产生AD中断
	EPwm1Regs.TBPRD = AD_HALF_PERIOD;						// actual period = 2*PWM_HALF_PERIOD

	EPwm1Regs.CMPA.half.CMPA = AD_QUARTER_PERIOD;
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode  40KHz
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// Reload from shadow
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// Sync down-stream module
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;	//
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;			
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;			
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD			
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD			

    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; 					// High
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;				// Low
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm1Regs.AQSFRC.bit.RLDCSF = 0x3;			// Load immediately
	EPwm1Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;		// Forces a continuous low on output
	EPwm1Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;		// Forces a continuous low on output

	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;	//
	EPwm1Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled

	EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;				// Gerneate EPWMxSOCA pulse on the first event 20KHz
	EPwm1Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm1Regs.ETSEL.bit.SOCASEL= ET_CTR_PRD;

//-----------------------------------------------------------------------------------------------------
	// ePWM2 Configuration  用于Scr.Inv的驱动
	EPwm2Regs.TBPRD = PWM_HALF_PERIOD;						// actual period = 2*PWM_HALF_PERIOD
	//EPwm2Regs.TBPHS.half.TBPHS =PWM_HALF_PERIOD;		//  Phase = 180 Degree

	EPwm2Regs.CMPA.half.CMPA = PWM_QUARTER_PERIOD;
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;	 	// up-down mode
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// Reload from shadow
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// Sync down-stream module
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;	//
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD

	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;	 			//
	EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;					//

	EPwm2Regs.AQSFRC.bit.RLDCSF = 0x3;			// Load immediately
	EPwm2Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;		// Forces a continuous low on output
	EPwm2Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;		// Forces a continuous low on output

	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;	//
/*
	EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Gerneate EPWMxSOCA pulse on the first event
	EPwm2Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm2Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/
//-----------------------------------------------------------------------------------------------------
	// ePWM3 Configuration  用于Scr.Bypass的驱动
	EPwm3Regs.TBPRD = PWM_PERIOD;						// actual period = 2*PWM_HALF_PERIOD
	//EPwm3Regs.TBPHS.half.TBPHS = PWM_HALF_PERIOD;		// Phase = 180 Degree

	EPwm3Regs.CMPA.half.CMPA = PWM_QUARTER_PERIOD;
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// Reload from shadow
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// Sync down-stream module
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;	//
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;		// Count Down after the sync event

	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD

	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;	 			//
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;					//

	EPwm3Regs.AQSFRC.bit.RLDCSF = 0x3;			// Load immediately
	EPwm3Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;		// Forces a continuous low on output
	EPwm3Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;		// Forces a continuous low on output

	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;	//

/*	EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Gerneate EPWMxSOCA pulse on the first event
	EPwm3Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm3Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/
//-----------------------------------------------------------------------------------------------------

	asm(" EALLOW");					// Enable EALLOW protected register access
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// HSPCLK to ePWM modules enabled
	asm(" EDIS");					// Disable EALLOW protected register access


} // end of InitEPwm()


void SCR_INV_Enable(void)
{ // start of PWMOutputsEnable()

	HWI_disable();

	EPwm2Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
	//EPwm2Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;

	HWI_enable();

} // end of PWMOutputsEnable()

void SCR_Bypass_Enable(void)
{ // start of PWMOutputsEnable()

	HWI_disable();

	EPwm3Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
	//EPwm3Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;

	HWI_enable();
} // end of PWMOutputsEnable()

void SCR_INV_Disable(void)
{ // start of PWMOutputsDisable()

	HWI_disable();			
	EPwm2Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
	//EPwm2Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;

	HWI_enable();
} // end of PWMOutputsDisable()

void SCR_Bypass_Disable(void)
{ // start of PWMOutputsDisable()

	HWI_disable();
	EPwm3Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
	//EPwm3Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;

	HWI_enable();
} // end of PWMOutputsDisable()


//--- end of file -----------------------------------------------------

