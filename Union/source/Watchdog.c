/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : Watchdog.c
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
* Function: InitWatchdog()
* Description: Initializes the F2833x Watchdog Timer.
* Notes:
*  1) As written, this function disables the WD timer.  This is really
*     just a placeholder though since the WD is already disabled in the
*     function CodeStartBranch.asm.
*  2) Edit this function as desired to re-enable the WD, and to
*     choose between a WD interrupt or system reset.
**********************************************************************/
void InitWatchdog(void)
{
	asm(" EALLOW");						// Enable EALLOW protected register access

	SysCtrlRegs.WDCR = 0x00E8;
// bit 15-8      0's:    reserved
// bit 7         1:      WDFLAG, write 1 to clear
// bit 6         1:      WDDIS, 1=disable WD
// bit 5-3       101:    WDCHK, WD check bits, always write as 101b
// bit 2-0       000:    WDPS, WD prescale bits, 000: WDCLK=OSCCLK/512/1

	SysCtrlRegs.SCSR = 0x0000;
// bit 15-3      0's:    reserved
// bit 2         0:      WDINTS, WD interrupt status bit (read-only)
// bit 1         0:      WDENINT, 0=WD causes reset, 1=WD causes WDINT
// bit 0         0:      WDOVERRIDE, write 1 to disable disabling of the WD (clear-only)

	asm(" EDIS");						// Disable EALLOW protected register access

} // end of InitWatchdog()


//--- end of file -----------------------------------------------------
