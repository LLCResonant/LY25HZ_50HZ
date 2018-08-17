/*=============================================================================*
 *         Copyright(c) 2009-2011,    ALL RIGHTS RESERVED
 *
 *  FILENAME : Adc.c 
 *
 *  PURPOSE  : ADC Module initialization configuration and basic data process
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

#include "DSP2833x_Device.h"			     // Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file


/*=============================================================================*
 * FUNCTION: InitAdc()
 * PURPOSE : ADC hardware module initialization.
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
void InitAdc(void)
{ // start of InitAdc()
	
	//--- Reset the ADC module
	AdcRegs.ADCTRL1.bit.RESET = 1;		// Reset the ADC
	asm(" RPT #22 || NOP");				// Must for ADC reset to take effect
	
	//--- Call the ADC_cal() function located in the Boot ROM.
	/* ADC_cal_func_ptr is a macro defined in the file 5KW_MAINHEADER.h This
	   macro simply defines ADC_cal_func_ptr to be a function pointer to
	   the correct address in the boot ROM. */
	(*ADC_cal_func_ptr)();

	//--- Select the ADC reference
	AdcRegs.ADCREFSEL.bit.REF_SEL = 0x01;	
	//bit1-0, 00-internal, 01-external 2.048V, 10-1.500V, 11-1.024V

	// Power-up reference and main ADC sampling mode
	AdcRegs.ADCTRL3.all = 0x00E7;	
	//bit15-8, 	0, reserved
	//bit7-6, 	11, ADCBGRFDN, reference power, 00=off, 11=on
	//bit5, 	1, ADCPWDN, main ADC power, 0=off, 1=on
	//bit4-1, 	0011, ADCCLKPS, clock prescaler, FCLK=HSPCLK/(2*ADCCLKPS)
	//			=75MHz/(2*3)=12.5MHz; can be changed to 12.5MHz, by 0110 to 0011
	//bit0, 	1, SMODE_SEL, 0=sequential sampling, 1=simultaneous sampling

	DelayUs(1000);	// Wait 5ms before using the ADC

	// ADC has started, necessary config for Channel and Conversion Mode needed.

	//--- for SVPWM data acquisition simultaneously in cascaded mode
	AdcRegs.ADCMAXCONV.all = 0x0007;	//8 double's conversions

	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0000;	// Convert Channel ADCINA/B0
	AdcRegs.ADCCHSELSEQ1.bit.CONV01	= 0x0001;	// Convert Channel ADCINA/B1
	AdcRegs.ADCCHSELSEQ1.bit.CONV02	= 0x0002;	// Convert Channel ADCINA/B2
	AdcRegs.ADCCHSELSEQ1.bit.CONV03	= 0x0003;	// Convert Channel ADCINA/B3
	AdcRegs.ADCCHSELSEQ2.bit.CONV04	= 0x0004;	// Convert Channel ADCINA/B4
	AdcRegs.ADCCHSELSEQ2.bit.CONV05	= 0x0005;	// Convert Channel ADCINA/B5
	AdcRegs.ADCCHSELSEQ2.bit.CONV06	= 0x0006;	// Convert Channel ADCINA/B6
	AdcRegs.ADCCHSELSEQ2.bit.CONV07	= 0x0007;	// Convert Channel ADCINA/B7

	
	AdcRegs.ADCTRL1.all = 0x0310; // 0000 0011 0001 0000
	//bit15,	0,	reserved
	//bit14,	0,	RESET, 0=no action, 1=reset ADC
	//bit13-12,	00,	SUSMOD, 00=ignore emulation suspend
	//bit11-8,	0011,	ACQ_PS (Acquisition), Sampling Window Width = 
	//			(ACQ_PS+1) x ADCCLK Period = 4*(1/12.5MHz)=0.32us.
	//bit7,		0,	CPS (Core clock), 0: ADCCLK=FCLK/1, 1: ADCCLK=FCLK/2
	//bit6,		0,	CONT_RUN, 0=start/stop mode, 1=continuous run
	//bit5,		0,	SEQ_OVRD, 0=disabled, 1=enabled
	//bit4,		1,	SEQ_CASC, 0=dual sequencer, 1=cascaded sequencer
	//bit3-0,0000,	reserved
	/* ADCCLK=HSPCLK/(2*ADCCLKPS_Patameter*(CPS_Parameter+1))=75/(2*3)
	   =12.5MHz, and the total ADC time is,	T(SMODE=1)=(2.5+(1+ACQ_PS)+5+(3+ACQ_PS)*7)*ADCCLK
	   =(29.5+ACQ_PS*8)*80ns=4.28us, this is the time duration from ADC_trigging
	   to All_ADC_result_ready. Proper adjustment should be done to satisfy 
	   minimum interval of other higher frequency sampling and digital process.
	*/

	AdcRegs.ADCTRL2.all = 0x0900; // 0000 1001 0000 0000
	// bit15,	0,	ePWM_SOCB_SEQ, 0=no action
	// bit14,	0,	RST_SEQ1, 0=no action
	// bit13,	0,	SOC_SEQ1, 0=clear any pending SOCs, 1=software trigger
	// bit12,	0,	reserved
	// bit11,	1,	INT_ENA_SEQ1, 1=enable interrupt
	// bit10,	0,	INT_MOD_SEQ1, 0=int on every SEQ1 conv, 
	//			                  1=int on every other SEQ1 conv
	// bit9,	0,	reserved
	// bit8,	1,	ePWM_SOCA_SEQ1, 1=SEQ1 start from ePWM_SOCA trigger
	// bit7,	0,	EXT_SOC_SEQ1, 1=SEQ1 start from ADCSOC pin
	// bit6,	0,	RST_SEQ2, 0=no action
	// bit5,	0,	SOC_SEQ2, no effect in cascaded mode
	// bit4,	0,	reserved
	// bit3,	0,	INT_ENA_SEQ2, 0=int disabled
	// bit2,	0,	INT_MOD_SEQ2, 0=int on every other SEQ2 conv
	// bit1,	0,	reserved
	// bit0,	0,	ePWM_SOCB_SEQ2, 0=no action

	//--- Enable the ADC interrupt
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// Enable ADCINT in PIE group 1
	IER |= M_INT1;						// Enable INT1 in IER to enable PIE group1


} // end of InitAdc()

//--- end of file -----------------------------------------------------
