/*=============================================================================*
 *         
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 50KW_MAINHEADER.h 

 *  PURPOSE  : main include file involves constants and other including
 *			   files besides standard DSP2833x header files.  
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *
 ******************************************************************************/
 
   
/*=============================================================================*
 * 	Software Development Evironment Notification: 
 *	-- CCS 3.3.49.1
 * 	-- BIOS 5.33.06
 *	-- Code Generation v5.2.0 or v5.2.4
 *  
 *	CHECK CAREFULLY AND MAKE SURE IN ACCORDANCE WITH THE ABOVE EVIRONMENT 
 *
 *============================================================================*/

#ifndef LYX2BP1_PARA_SW_H
#define LYX2BP1_PARA_SW_H


/* Define Macros for system level clock parameters */
#define SYSCLOCK_FREQ		150000000
#define HSPCLK_FREQ			(SYSCLOCK_FREQ/2)		// defined in SysCtrlRegs
#define LSPCLK_FREQ			(SYSCLOCK_FREQ/4)		// defined in SysCtrlRegs
#define TBCLK_FREQ			(SYSCLOCK_FREQ/2)		// EPwm Timer clock


/* DSP/BIOS Macros for ADC calibration */
#define ADC_cal_func_ptr (void (*)(void))0x380080
//#define FAN_CHECK

/*
 * PFC PWM test
 * Warning: DON NOT SOLDER THE BUS FUSE
 */
//#define	PFC_OPEN_LOOP
//#define	PFC_PWM_ENABLE

/*
 * PFC_CLOSE_LOOP TEST
 * Warning: DON NOT SOLDER THE BUS FUSE
 */
//#define	PFC_CLOSE_LOOP
//#define	PFC_PWM_ENABLE
//#define	PFC_PWM_DISABLE


/*
 * INV PWM TEST
 * Warning: DON NOT SOLDER THE BUS FUSE
 */
//#define	INV_OPEN_LOOP
//#define	PFC_PWM_DISABLE
//#define	INV_PWM_ENABLE


/*
 * INV CLOSE TEST
 * Warning: SOLDER THE BUS FUSE
*/

#define	INV_CLOSE_LOOP
#define	PFC_CLOSE_LOOP
#define	PFC_PWM_ENABLE
#define	PFC_PWM_DISABLE
#define	INV_PWM_ENABLE
#define	INV_PWM_DISABLE

#define NORMAL_EEPROM
//#define RESET_EEPROM

//#define SECOND_EDITION
#define THIRD_EDITION
/*=============================================================================*
 * 	Includings 
 *============================================================================*/
/* Include Standard C Language Header Files */
#include <string.h>

/* Include DSP/BIOS related Header Files */
#include <gbl.h>
#include <hwi.h>

/* Include DSP/BIOS generated Header Files */
#include "3kswcfg.h"

/* Include 283xx specific Header Files */
#include "math.h"
#include "C28x_FPU_FastRTS.h"//km
/* Include user defined Header Files */
//#include "Adc.h"
//#include "Ecan.h"
#include "Ecap.h"
#include "Epwm.h"
//#include "EnergyCalc.h"
#include "3KW_DataAcquisition.h"
#include "3KW_ProtectionLogic.h"
#include "3KW_StateUpdate.h"
#include "3KW_Regulator.h"
#include "3KW_ParallelLogic.h"
#include "3KW_Scib_PCOsc.h"
#include "Flash28335_API.h"
#include "3KW_I2cEeprom.h"
#include "Ecan.h"
#include "3KW_ECan_Basic.h"
#include "3KW_EcanDataprotocol.h"
#include "Sci.h"
#include "3KW_Scia_LCDs.h"

/*=============================================================================*
 * 	Global Variables for system level initialization
 *============================================================================*/

/* BIOS symbols defined in the linker command file */

extern Uint16 hwi_vec_loadstart;
extern Uint16 hwi_vec_loadend;
extern Uint16 hwi_vec_runstart;
extern Uint16 trcdata_loadstart;
extern Uint16 trcdata_loadend;
extern Uint16 trcdata_runstart;

/* User defined symbols in the linker command file */

extern Uint16 g_u16InitFlash_InRAM_loadstart;
extern Uint16 g_u16InitFlash_InRAM_loadend;
extern Uint16 g_u16InitFlash_InRAM_runstart;

extern Uint16 g_u16ControlLoopInRAM_loadstart;
extern Uint16 g_u16ControlLoopInRAM_loadend;
extern Uint16 g_u16ControlLoopInRAM_runstart;

extern Uint16 g_u16sciaRamFun_loadstart;
extern Uint16 g_u16sciaRamFun_loadend;
extern Uint16 g_u16sciaRamFun_runstart;

extern void InitEPwm(void);
extern void InitECap(void);
extern void InitAdc(void);
extern void InitECanGpio(void);
extern void InitECan(void);

/*=============================================================================*
 * 	Global functions for system level initialization
 *============================================================================*/

/* System level initialization */
extern void DelayUs(Uint16);
extern void InitGpio(void);
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);
extern void InitWatchdog(void);
extern void SetDBGIER(Uint16);
extern void InitFlash(void);
extern void UserInit(void);

#endif

//--- end of file -----------------------------------------------------
