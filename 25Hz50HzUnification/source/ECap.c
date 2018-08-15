/*=============================================================================*
 *         Copyright(c) 2009-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : ECap.c 
 *
 *  PURPOSE  : Capture falling/rising edge for Grid_Frequecy calculation and 
 *  		       phase sequency detection.
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


#include "DSP2833x_Device.h"	// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file



/*=============================================================================*
 * 	Local Variables declaration
 *============================================================================*/

/* Detect period of one Grid frequency */
int32 i32GridFrequencyPriodTemp = 0;
	


/*=============================================================================*
 * FUNCTION: InitECap()
 * PURPOSE : eCAP hardware module initialization.
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
void InitECap(void)
{ // start of InitECap()

//	Configure eCAP5 unit to test frequency
//GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;    // Enable pull-up on GPIO48 (CAP5)
//GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 0; // Synch to SYSCLKOUT GPIO48 (CAP5)
	DINT;
	ECap1Regs.ECEINT.all = 0x0000;		// Disable all eCAP interrupts
	ECap1Regs.ECCLR.all = 0xFFFF;       // Clear all CAP interrupt flags
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;	// Disabled loading of capture results
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;	// Stop the counter

	ECap1Regs.ECCTL2.bit.CONT_ONESHT =0;
	ECap1Regs.ECCTL2.bit.STOP_WRAP = 0;
	ECap1Regs.ECCTL1.bit.CAP1POL = 1;
	ECap1Regs.ECCTL1.bit.CAP2POL = 1;
	ECap1Regs.ECCTL1.bit.CAP3POL = 1;
	ECap1Regs.ECCTL1.bit.CAP4POL = 1;
	ECap1Regs.ECCTL1.bit.CTRRST1 = 1;
	ECap1Regs.ECCTL1.bit.CTRRST2 = 1;
	ECap1Regs.ECCTL1.bit.CTRRST3 = 1;
	ECap1Regs.ECCTL1.bit.CTRRST4 = 1;
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;

	ECap1Regs.ECCTL1.bit.PRESCALE = 0;

	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap1Regs.ECCTL2.bit.REARM = 1;
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;
	ECap1Regs.ECEINT.bit.CEVT1 = 1;

	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;		// Enable ECAP1_INT in PIE group 4


//	Configure eCAP5 unit to test frequency
//GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;    // Enable pull-up on GPIO48 (CAP5)
//GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 0; // Synch to SYSCLKOUT GPIO48 (CAP5)

	ECap2Regs.ECEINT.all = 0x0000;		// Disable all eCAP interrupts
	ECap2Regs.ECCLR.all = 0xFFFF;       // Clear all CAP interrupt flags
	ECap2Regs.ECCTL1.bit.CAPLDEN = 0;	// Disabled loading of capture results
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;	// Stop the counter

	ECap2Regs.ECCTL2.bit.CONT_ONESHT = 0;//2017.5.24 GX
	ECap2Regs.ECCTL2.bit.STOP_WRAP = 0;
	ECap2Regs.ECCTL1.bit.CAP1POL = 1;
	//ECap2Regs.ECCTL1.bit.CAP2POL = 1;
	//ECap2Regs.ECCTL1.bit.CAP3POL = 1;
	//ECap2Regs.ECCTL1.bit.CAP4POL = 1;
	ECap2Regs.ECCTL1.bit.CTRRST1 = 1;
	//ECap2Regs.ECCTL1.bit.CTRRST2 = 1;
	//ECap2Regs.ECCTL1.bit.CTRRST3 = 1;
	//ECap2Regs.ECCTL1.bit.CTRRST4 = 1;
	ECap2Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap2Regs.ECCTL1.bit.CAPLDEN = 1;

	ECap2Regs.ECCTL1.bit.PRESCALE = 0;

	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;
	//ECap2Regs.ECCTL2.bit.REARM = 1;
	ECap2Regs.ECCTL1.bit.CAPLDEN = 1;
	ECap2Regs.ECEINT.bit.CEVT1 = 1;

	PieCtrlRegs.PIEIER4.bit.INTx2 = 1;	// Enable ECAP2_INT in PIE group 4


//	Configure eCAP5 unit to test frequency
	ECap3Regs.ECEINT.all = 0x0000;		// Disable all eCAP interrupts
	ECap3Regs.ECCLR.all = 0xFFFF;       // Clear all CAP interrupt flags
	ECap3Regs.ECCTL1.bit.CAPLDEN = 0;	// Disabled loading of capture results
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;	// Stop the counter

	ECap3Regs.ECCTL2.bit.CONT_ONESHT =0;
	ECap3Regs.ECCTL2.bit.STOP_WRAP = 0;
	ECap3Regs.ECCTL1.bit.CAP1POL = 1;
	ECap3Regs.ECCTL1.bit.CAP2POL = 1;
	ECap3Regs.ECCTL1.bit.CAP3POL = 1;
	ECap3Regs.ECCTL1.bit.CAP4POL = 1;
	ECap3Regs.ECCTL1.bit.CTRRST1 = 1;
	ECap3Regs.ECCTL1.bit.CTRRST2 = 1;
	ECap3Regs.ECCTL1.bit.CTRRST3 = 1;
	ECap3Regs.ECCTL1.bit.CTRRST4 = 1;
	ECap3Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap3Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap3Regs.ECCTL1.bit.CAPLDEN = 1;

	ECap3Regs.ECCTL1.bit.PRESCALE = 0;

	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap3Regs.ECCTL2.bit.REARM = 1;
	ECap3Regs.ECCTL1.bit.CAPLDEN = 1;
	ECap3Regs.ECEINT.bit.CEVT1 = 1;

	PieCtrlRegs.PIEIER4.bit.INTx3 = 1;	// Enable ECAP1_INT in PIE group 4



//	Configure eCAP5 unit to test frequency

	ECap4Regs.ECEINT.all = 0x0000;		// Disable all eCAP interrupts
	ECap4Regs.ECCLR.all = 0xFFFF;       // Clear all CAP interrupt flags
	ECap4Regs.ECCTL1.bit.CAPLDEN = 0;	// Disabled loading of capture results
	ECap4Regs.ECCTL2.bit.TSCTRSTOP = 0;	// Stop the counter

	ECap4Regs.ECCTL2.bit.CONT_ONESHT =0;
	ECap4Regs.ECCTL2.bit.STOP_WRAP = 0;
	ECap4Regs.ECCTL1.bit.CAP1POL = 0;
	ECap4Regs.ECCTL1.bit.CAP2POL = 0;
	ECap4Regs.ECCTL1.bit.CAP3POL = 0;
	ECap4Regs.ECCTL1.bit.CAP4POL = 0;
	ECap4Regs.ECCTL1.bit.CTRRST1 = 1;
	ECap4Regs.ECCTL1.bit.CTRRST2 = 1;
	ECap4Regs.ECCTL1.bit.CTRRST3 = 1;
	ECap4Regs.ECCTL1.bit.CTRRST4 = 1;
	ECap4Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap4Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap4Regs.ECCTL1.bit.CAPLDEN = 1;

	ECap4Regs.ECCTL1.bit.PRESCALE = 0;

	ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap4Regs.ECCTL2.bit.REARM = 1;
	ECap4Regs.ECCTL1.bit.CAPLDEN = 1;
	ECap4Regs.ECEINT.bit.CEVT1 = 1;

	PieCtrlRegs.PIEIER4.bit.INTx4 = 1;	// Enable ECAP1_INT in PIE group 4


//	Configure eCAP5 unit to test frequency
//GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;    // Enable pull-up on GPIO48 (CAP5)
//GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 0; // Synch to SYSCLKOUT GPIO48 (CAP5)

	ECap5Regs.ECEINT.all = 0x0000;		// Disable all eCAP interrupts
	ECap5Regs.ECCLR.all = 0xFFFF;       // Clear all CAP interrupt flags
	ECap5Regs.ECCTL1.bit.CAPLDEN = 0;	// Disabled loading of capture results
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = 0;	// Stop the counter

	ECap5Regs.ECCTL2.bit.CONT_ONESHT =0;
	ECap5Regs.ECCTL2.bit.STOP_WRAP = 0;
	ECap5Regs.ECCTL1.bit.CAP1POL = 0;
	ECap5Regs.ECCTL1.bit.CAP2POL = 0;
	ECap5Regs.ECCTL1.bit.CAP3POL = 0;
	ECap5Regs.ECCTL1.bit.CAP4POL = 0;
	ECap5Regs.ECCTL1.bit.CTRRST1 = 1;
	ECap5Regs.ECCTL1.bit.CTRRST2 = 1;
	ECap5Regs.ECCTL1.bit.CTRRST3 = 1;
	ECap5Regs.ECCTL1.bit.CTRRST4 = 1;
	ECap5Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap5Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap5Regs.ECCTL1.bit.CAPLDEN = 1;

	ECap5Regs.ECCTL1.bit.PRESCALE = 0;

	ECap5Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap5Regs.ECCTL2.bit.REARM = 1;
	ECap5Regs.ECCTL1.bit.CAPLDEN = 1;
	ECap5Regs.ECEINT.bit.CEVT1 = 1;

	PieCtrlRegs.PIEIER4.bit.INTx5 = 1;	// Enable ECAP1_INT in PIE group 4


//  ecap6

	ECap6Regs.ECEINT.all = 0x0000;			// Disable all eCAP interrupts
	ECap6Regs.ECCLR.all = 0xFFFF;       	// Clear all CAP interrupt flags
	ECap6Regs.ECCTL1.bit.CAPLDEN = 0;	// Disabled loading of capture results
	ECap6Regs.ECCTL2.bit.TSCTRSTOP = 0;	// Stop the counter

	ECap6Regs.ECCTL2.bit.CONT_ONESHT =0;
	ECap6Regs.ECCTL2.bit.STOP_WRAP = 0;
	ECap6Regs.ECCTL1.bit.CAP1POL = 1;    //2017.5.25 GX
	ECap6Regs.ECCTL1.bit.CAP2POL = 1;
	ECap6Regs.ECCTL1.bit.CAP3POL = 1;
	ECap6Regs.ECCTL1.bit.CAP4POL = 1;
	ECap6Regs.ECCTL1.bit.CTRRST1 = 1;
	ECap6Regs.ECCTL1.bit.CTRRST2 = 1;
	ECap6Regs.ECCTL1.bit.CTRRST3 = 1;
	ECap6Regs.ECCTL1.bit.CTRRST4 = 1;
	ECap6Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap6Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap6Regs.ECCTL1.bit.CAPLDEN = 1;

	ECap6Regs.ECCTL1.bit.PRESCALE = 0;

	ECap6Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap6Regs.ECCTL2.bit.REARM = 1;
	ECap6Regs.ECCTL1.bit.CAPLDEN = 1;
	ECap6Regs.ECEINT.bit.CEVT1 = 1;

	PieCtrlRegs.PIEIER4.bit.INTx6 = 1;	// Enable ECAP1_INT in PIE group 4

	IER |= M_INT4;	// Enable INT4 in IER to enable PIE group 4
	EINT;
} // end of InitECap()



//--- end of file -----------------------------------------------------
