/**********************************************************************
* File: Flash.c
* Devices: TMS320F2803x
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   01/07/09 - original (D. Alter)
**********************************************************************/
#include "F28035_example.h"				// Main include file


/**********************************************************************
* Function: InitFlash()
* Description: Initializes the F2803x flash timing registers.
* Notes:
*  1) This function MUST be executed out of RAM.  Executing it out of
*     OTP/FLASH will produce unpredictable results.
*  2) The flash registers are code security module protected.  Therefore,
*     you must either run this function from secure RAM (L0),
*     or you must first unlock the CSM.  Note that unlocking the CSM as
*     part of the program flow can compromise the code security.
*  3) The wait-states configured below are for 60 MHz operation.  The
*     latest datasheet for the particular device of interest should
*     be consulted to confirm the flash timing specifications.
**********************************************************************/
#pragma CODE_SECTION(InitFlash, "secureRamFuncs")
void InitFlash(void)
{
	asm(" EALLOW");									// Enable EALLOW protected register access
	FlashRegs.FPWR.bit.PWR = 3;						// Pump and bank set to active mode
	FlashRegs.FSTATUS.bit.V3STAT = 1;				// Clear the 3VSTAT bit
	FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;	// Sleep to standby transition cycles
	FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;	// Standby to active transition cycles
	FlashRegs.FBANKWAIT.bit.RANDWAIT = 1;//2;			// Random access waitstates
	FlashRegs.FBANKWAIT.bit.PAGEWAIT = 1;//2;			// Paged access waitstates
	FlashRegs.FOTPWAIT.bit.OTPWAIT = 1;//3;				// OTP waitstates
	FlashRegs.FOPT.bit.ENPIPE = 1;					// Enable the flash pipeline
	asm(" EDIS");									// Disable EALLOW protected register access

// Force a complete pipeline flush to ensure that the write to the last register
// configured occurs before returning.  Safest thing is to wait 8 full cycles.

    asm(" RPT #6 || NOP");

} // end of InitFlash()


//--- end of file -----------------------------------------------------
