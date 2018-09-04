/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : ECap.c 
 *
 *  PURPOSE  : Capture falling/rising edge for Grid_Frequecy calculation and 
 *  		   phase sequency detection.
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


#include "DSP2803x_Device.h"		// Peripheral address definitions
#include "F28035_example.h"			// Main include file



/*=============================================================================*
 * 	Local Variables declaration
 *============================================================================*/
/* Detect and check phase sequency */
int32 g_i32PhaseSequencyTemp_Vvw = 0;
int32 g_i32PhaseSequencyTemp_Vwu = 0; 

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

//--------------------------------------------------------------------------------------
   ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;      // One-shot
   ECap1Regs.ECCTL2.bit.STOP_WRAP = 0;        // Stop at 4 events
   ECap1Regs.ECCTL1.bit.CAP1POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP2POL = 1;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP3POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP4POL = 1;          // Rising edge
   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;         // Disable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 2;        // Pass through
   ECap1Regs.ECCTL1.bit.PRESCALE= 0;
   //ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap1Regs.ECCTL2.bit.CAP_APWM=0;

   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap1Regs.ECCTL2.bit.REARM = 1;            // arm one-shot
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
   ECap1Regs.ECEINT.bit.CEVT1 = 1;            // 4 events = interrupt


PieCtrlRegs.PIEIER4.bit.INTx1 = 1;	// Enable ECAP1_INT in PIE group 4

	IER |= M_INT4;	// Enable INT4 in IER to enable PIE group 4

} // end of InitECap()


//--- end of file -----------------------------------------------------
