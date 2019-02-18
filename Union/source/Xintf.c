/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : Xintf.c
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/
#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"				// Main include file


/**********************************************************************
* Function: InitXintf()
*
* Description: Initializes the external memory interface on the F28335.
**********************************************************************/
#pragma CODE_SECTION(InitXintf, "InitFlash_InRAM")
void InitXintf(void)
{
//--- XINTCNF2 Register
	XintfRegs.XINTCNF2.bit.XTIMCLK = 0;			// XTIMCLK=SYSCLKOUT/1
	XintfRegs.XINTCNF2.bit.CLKOFF = 0;			// XCLKOUT is enabled
	XintfRegs.XINTCNF2.bit.CLKMODE = 0;			// XCLKOUT = XTIMCLK

// Make sure write buffer is empty before configuring buffering depth
	while(XintfRegs.XINTCNF2.bit.WLEVEL != 0);	// poll the WLEVEL bit
	XintfRegs.XINTCNF2.bit.WRBUFF = 0;			// No write buffering

//--- XBANK Register
// Example: Assume Zone 7 is slow, so add additional BCYC cycles whenever
// switching from Zone 7 to another Zone.  This will help avoid bus contention.
	XintfRegs.XBANK.bit.BCYC = 7;				// Add 7 cycles
	XintfRegs.XBANK.bit.BANK = 7;				// select zone 7
    
//--- Zone 0 Configuration
	XintfRegs.XTIMING0.bit.X2TIMING = 1;	// Timing scale factor = 1
	XintfRegs.XTIMING0.bit.XSIZE = 3;		// 3 means 16-bit interface
	XintfRegs.XTIMING0.bit.READYMODE = 1; 	// XREADY is asynchronous
	XintfRegs.XTIMING0.bit.USEREADY = 0;	// Disable XREADY
	XintfRegs.XTIMING0.bit.XRDLEAD = 3;		// Read lead time
	XintfRegs.XTIMING0.bit.XRDACTIVE = 7;	// Read active time
	XintfRegs.XTIMING0.bit.XRDTRAIL = 3;	// Read trail time
	XintfRegs.XTIMING0.bit.XWRLEAD = 3;		// Write lead time
	XintfRegs.XTIMING0.bit.XWRACTIVE = 7;	// Write active time
	XintfRegs.XTIMING0.bit.XWRTRAIL = 3;	// Write trail time

//--- Zone 6 Configuration
	XintfRegs.XTIMING6.bit.X2TIMING = 1;	// Timing scale factor = 1  0
	XintfRegs.XTIMING6.bit.XSIZE = 3;		// 3 means 16-bit interface 3
	XintfRegs.XTIMING6.bit.READYMODE = 1; 	// XREADY is asynchronous   1
	XintfRegs.XTIMING6.bit.USEREADY = 0;	// Disable XREADY           0
	XintfRegs.XTIMING6.bit.XRDLEAD = 3;		// Read lead time           1
	XintfRegs.XTIMING6.bit.XRDACTIVE = 7;	// Read active time         2 
	XintfRegs.XTIMING6.bit.XRDTRAIL = 3;	// Read trail time          0
	XintfRegs.XTIMING6.bit.XWRLEAD = 3;		// Write lead time          1
	XintfRegs.XTIMING6.bit.XWRACTIVE = 7;	// Write active time        2
	XintfRegs.XTIMING6.bit.XWRTRAIL = 3;	// Write trail time         0

//--- Zone 7 Configuration
	XintfRegs.XTIMING7.bit.X2TIMING = 1;	// Timing scale factor = 1
	XintfRegs.XTIMING7.bit.XSIZE = 3;		// 3 means 16-bit interface
	XintfRegs.XTIMING7.bit.READYMODE = 1; 	// XREADY is asynchronous
	XintfRegs.XTIMING7.bit.USEREADY = 0;	// Disable XREADY
	XintfRegs.XTIMING7.bit.XRDLEAD = 3;		// Read lead time
	XintfRegs.XTIMING7.bit.XRDACTIVE = 7;	// Read active time
	XintfRegs.XTIMING7.bit.XRDTRAIL = 3;	// Read trail time
	XintfRegs.XTIMING7.bit.XWRLEAD = 3;		// Write lead time
	XintfRegs.XTIMING7.bit.XWRACTIVE = 7;	// Write active time
	XintfRegs.XTIMING7.bit.XWRTRAIL = 3;	// Write trail time

//--- Force a complete pipeline flush to ensure that the write to the last register
//    configured occurs before returning.  Safest thing to do is wait 8 full cycles.
	asm(" RPT #6 || NOP");


} // end of InitXintf()


//--- end of file -----------------------------------------------------
