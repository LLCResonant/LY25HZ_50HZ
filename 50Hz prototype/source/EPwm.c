/*=============================================================================*
 *         Copyright(c) 2009-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : EPwm.c 
 *
 *  PURPOSE  : ePWM Module configuration and Timer/PWMoutput En/Disable process..
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
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
 
#include "DSP2833x_Device.h"		// Peripheral address definitions
#include "3KW_MAINHEADER.h"				// Main include file


/*=============================================================================*
 * FUNCTION: InitEPwm()
 * PURPOSE : Initializes the Enhanced PWM modules.
 *			 To precisely capture the most valuable sampling point L current 
 *			 with high frequency sampling for Energy, additional Timer5 and 
 *			 Timer6 are used to supply additional leading-type SOC and equal 
 *			 interval sampling, with :
 *			 TBCLK=SYSCLKOUT/(HSPCLKDIV*CLKDIV)=150MHz/(2*1)=75MHz
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

	// ePWM1 Configuration 
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm1Regs.TBPRD = PWM_HALF_PERIOD;					// actual period = 2*PWM_HALF_PERIOD
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// reload from shadow
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// sync enable
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// prescaler = 2
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;				// prescaler = 1

	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD


    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; 					// High
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;				// Low
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm1Regs.AQSFRC.bit.RLDCSF = 0x3;					// Load immediately
	EPwm1Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;				// Forces a continuous low on output
	EPwm1Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;				// Forces a continuous low on output

	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
	
	EPwm1Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled

	EPwm1Regs.ETPS.bit.SOCAPRD = ET_2ND;		// Generate EPWMxSOCA pulse on the first event //2017.5.10 GX
	EPwm1Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm1Regs.ETSEL.bit.SOCASEL= ET_CTR_PRD;	// Enable event time-base counter equal to zero.//2017.11.11 GX

	// ePWM2 Configuration 
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm2Regs.TBPRD = PWM_HALF_PERIOD;					// actual period = 2*PWM_HALF_PERIOD
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// reload from shadow
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// sync enable
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// prescaler = 2
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;				// prescaler = 1

	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD


    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; 					// High
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;				// Low
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm2Regs.AQSFRC.bit.RLDCSF = 0x3;					// Load immediately
	EPwm2Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;				// Forces a continuous low on output
	EPwm2Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;				// Forces a continuous low on output

	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
	
	EPwm2Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled
/*
	EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Generate EPWMxSOCA pulse on the first event
	EPwm2Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm2Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/

	// ePWM3 Configuration 
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm3Regs.TBPRD = PWM_HALF_PERIOD;					// actual period = 2*PWM_HALF_PERIOD
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// reload from shadow
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// sync enable
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// prescaler = 2
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;				// prescaler = 1

	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD

    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR; 				// Low
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;					// High
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm3Regs.AQSFRC.bit.RLDCSF = 0x3;					// Load immediately
	EPwm3Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;				// Forces a continuous low on output
	EPwm3Regs.AQCSFRC.bit.CSFB = AQ_SET;				// Forces a continuous High on output

	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm3Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm3Regs.DBFED = DeadBand_Duration;				// Dead band duration for FED
	EPwm3Regs.DBRED = DeadBand_Duration;				// Dead band duration for RED
	
	EPwm3Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled

/*
	EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Generate EPWMxSOCA pulse on the first event
	EPwm3Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm3Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/

	// ePWM4 Configuration 
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm4Regs.TBPRD = PWM_HALF_PERIOD;					// actual period = 2*PWM_HALF_PERIOD
	EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// reload from shadow
	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// sync enable
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// prescaler = 2
	EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;				// prescaler = 1

	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD

    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR; 				// Low
	EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;					// High
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm4Regs.AQSFRC.bit.RLDCSF = 0x3;					// Load immediately
	EPwm4Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;				// Forces a continuous low on output
	EPwm4Regs.AQCSFRC.bit.CSFB = AQ_SET;				// Forces a continuous High on output

	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm4Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm4Regs.DBFED = DeadBand_Duration;				// Dead band duration for FED
	EPwm4Regs.DBRED = DeadBand_Duration;				// Dead band duration for RED
	
	EPwm4Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled

/*
	EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Generate EPWMxSOCA pulse on the first event
	EPwm4Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm4Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/

	// ePWM5 Configuration 
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm5Regs.TBPRD = PWM_HALF_PERIOD;					// actual period = 2*PWM_HALF_PERIOD
	EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// reload from shadow
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// sync enable
	EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// prescaler = 2
	EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;				// prescaler = 1

	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD

    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR; 				// Low
	EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;					// High
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm5Regs.AQSFRC.bit.RLDCSF = 0x3;					// Load immediately
	EPwm5Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;				// Forces a continuous low on output
	EPwm5Regs.AQCSFRC.bit.CSFB = AQ_SET;				// Forces a continuous High on output

	EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm5Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm5Regs.DBFED = 1.25f * DeadBand_Duration;				// Dead band duration for FED
	EPwm5Regs.DBRED = 1.25f * DeadBand_Duration;				// Dead band duration for RED
	
	EPwm5Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled

/*
	EPwm5Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Generate EPWMxSOCA pulse on the first event
	EPwm5Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm5Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/
	
	// ePWM6 Configuration 
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		// up-down mode
	EPwm6Regs.TBPRD = PWM_HALF_PERIOD;					// actual period = 2*PWM_HALF_PERIOD
	EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Slave module
	EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;				// reload from shadow
	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 		// sync enable
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// prescaler = 2
	EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;				// prescaler = 1

	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;		// Load on CTR=PRD
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;		// Load on CTR=PRD

    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; 				// Low
	EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;					// High
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR; 				// Low
	EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;					// High

	EPwm6Regs.AQSFRC.bit.RLDCSF = 0x3;					// Load immediately
	EPwm6Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;				// Forces a continuous low on output
	EPwm6Regs.AQCSFRC.bit.CSFB = AQ_SET;				// Forces a continuous High on output

	EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm6Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm6Regs.DBFED = 1.25f * DeadBand_Duration;				// Dead band duration for FED
	EPwm6Regs.DBRED = 1.25f * DeadBand_Duration;				// Dead band duration for RED
	
	EPwm6Regs.PCCTL.bit.CHPEN = CHP_DISABLE;			// PWM chopper unit disabled

/*
	EPwm6Regs.ETPS.bit.SOCAPRD = ET_1ST;		// Generate EPWMxSOCA pulse on the first event
	EPwm6Regs.ETSEL.bit.SOCAEN = SOCA_ENABLE;	// Enable SOCA
	EPwm6Regs.ETSEL.bit.SOCASEL= ET_CTR_ZERO;	// Enable event time-base counter equal to zero.
*/
	// initial TB counter
	EPwm1Regs.TBCTR = 0x0000;		// Clear timer counter
	EPwm2Regs.TBCTR = 0x0000;		// Clear timer counter
	EPwm3Regs.TBCTR = 0x0000;		// Clear timer counter
	EPwm4Regs.TBCTR = 0x0000;		// Clear timer counter
	EPwm5Regs.TBCTR = 0x0000;		// Clear timer counter
	EPwm6Regs.TBCTR = 0x0000;		// Clear timer counter
	asm(" EALLOW");					// Enable EALLOW protected register access
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// HSPCLK to ePWM modules enabled
	asm(" EDIS");					// Disable EALLOW protected register access

	PfcPWMOutputsDisable();              //2017.2.21 GX
	InvPWMOutputsDisable();
} // end of InitEPwm()




/*=============================================================================*
 * FUNCTION: PfcPWMOutputsEnable()
 * PURPOSE : ePWM1A/B, 2A/B, and 3A/B enabled, PWM Timer5 also enabled
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
 *     Main.c or MainStatusMachine.c 
 * 
 *============================================================================*/
void PfcPWMOutputsEnable(void)					// Johnny 2017.4.19
{ // start of PWMOutputsEnable()

	HWI_disable();
	#ifdef PFC_PWM_ENABLE
	EPwm1Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;//2017.6.29 GX inverter test
	EPwm1Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;  //2017.4.20 GX

	EPwm2Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;//2017.6.29 GX inverter test
	EPwm2Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;  //2017.4.20 GX
	#endif
	HWI_enable();
} // end of PWMOutputsEnable()

/*=============================================================================*
 * FUNCTION: InvPWMOutputsEnable()
 * PURPOSE : ePWM1A/B, 2A/B, and 3A/B enabled, PWM Timer5 also enabled
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
 *     Main.c or MainStatusMachine.c
 *
 *============================================================================*/
void InvPWMOutputsEnable(void)					// Johnny 2017.4.19
{ // start of PWMOutputsEnable()
	HWI_disable();
	#ifdef INV_PWM_ENABLE
	EPwm3Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;//2017.6.29 GX inverter test
	EPwm3Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;

	EPwm4Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
	EPwm4Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
	#endif
	HWI_enable();
} // end of PWMOutputsEnable()


/*=============================================================================*
 * FUNCTION: PfcPWMOutputsDisable()
 * PURPOSE : ePWM1A/B, 2A/B, and 3A/B enabled, PWM Timer5 also disabled
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
 *     Main.c or MainStatusMachine.c
 *
 *============================================================================*/
void PfcPWMOutputsDisable(void)				// Johnny 2017.4.19
{ // start of PWMOutputsDisable()

	HWI_disable();
	#ifdef PFC_PWM_DISABLE
	EPwm1Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
	EPwm1Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;

	EPwm2Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
	EPwm2Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
	#endif
	HWI_enable();
} // end of PWMOutputsDisable()


/*=============================================================================*
 * FUNCTION: PWMOutputsDisable()
 * PURPOSE : ePWM1A/B, 2A/B, and 3A/B enabled, PWM Timer5 also disabled
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
 *     Main.c or MainStatusMachine.c 
 * 
 *============================================================================*/
void InvPWMOutputsDisable(void)				// Johnny 2017.4.19
{ // start of PWMOutputsDisable()

	HWI_disable();			
	#ifdef INV_PWM_DISABLE
	EPwm3Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
	EPwm4Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
	DelayUs(10);
	EPwm3Regs.AQCSFRC.bit.CSFB = AQ_SET;
    EPwm4Regs.AQCSFRC.bit.CSFB = AQ_SET;
	#endif
	HWI_enable();

} // end of PWMOutputsDisable()




//--- end of file -----------------------------------------------------

