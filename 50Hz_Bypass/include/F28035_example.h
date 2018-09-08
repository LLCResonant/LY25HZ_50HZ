/**********************************************************************
* File: F28035_example.h
* Device: TMS320F28035
* Author: David M. Alter, Texas Instruments Inc.
* Description: Include file for example project.  Include this file in
*   all C-source files.
* Notes:
*   1) The following constants may be defined in the CCS project build
*      options:
*        EXAMPLE_NONBIOS
*        EXAMPLE_BIOS
*        EXAMPLE_FLASH
*        EXAMPLE_RAM
* History:
*   05/04/09 - original (D. Alter)
**********************************************************************/

#ifndef F28035_EXAMPLE_H
#define F28035_EXAMPLE_H

/*-----------------------------------------------------------------------------
      Specify the PLL control register (PLLCR) and divide select (DIVSEL) value.
-----------------------------------------------------------------------------*/

//#define DSP28_DIVSEL   0 // Enable /4 for SYSCLKOUT
//#define DSP28_DIVSEL   1 // Disable /4 for SYSCKOUT
#define DSP28_DIVSEL   2 // Enable /2 for SYSCLKOUT
//#define DSP28_DIVSEL   3 // Enable /1 for SYSCLKOUT


//#define DSP28_PLLCR   12    // Uncomment for 60 MHz devices [60 MHz = (10MHz * 12)/2]
//#define DSP28_PLLCR   11
//#define DSP28_PLLCR   10
//#define DSP28_PLLCR    9
//#define DSP28_PLLCR    8      // Uncomment for 40 MHz devices [40 MHz = (10MHz * 8)/2]
//#define DSP28_PLLCR    7
#define DSP28_PLLCR    6
//#define DSP28_PLLCR    5
//#define DSP28_PLLCR    4
//#define DSP28_PLLCR    3
//#define DSP28_PLLCR    2
//#define DSP28_PLLCR    1
//#define DSP28_PLLCR    0  // PLL is bypassed in this mode
//----------------------------------------------------------------------------

/*-----------------------------------------------------------------------------
      Specify the clock rate of the CPU (SYSCLKOUT) in nS.

      Take into account the input clock frequency and the PLL multiplier
      selected in step 1.

      Use one of the values provided, or define your own.
      The trailing L is required tells the compiler to treat
      the number as a 64-bit value.

      Only one statement should be uncommented.

      Example:   60 MHz devices:
                 CLKIN is a 10 MHz crystal or internal 10 MHz oscillator

                 In step 1 the user specified PLLCR = 0xC for a
                 60 MHz CPU clock (SYSCLKOUT = 60 MHz).

                 In this case, the CPU_RATE will be 16.667L
                 Uncomment the line: #define CPU_RATE 16.667L

-----------------------------------------------------------------------------*/

#define CPU_RATE   16.667L   // for a 60MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   20.000L   // for a 50MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   25.000L   // for a 40MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   33.333L   // for a 30MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   41.667L   // for a 24MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   50.000L   // for a 20MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   66.667L   // for a 15MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE  100.000L   // for a 10MHz CPU clock speed  (SYSCLKOUT)

//----------------------------------------------------------------------------

// The following pointer to a function call calibrates the ADC and internal oscillators
#define Device_cal (void   (*)(void))0x3D7C80

//---------------------------------------------------------------------------
// Constant Definitions
//
#define ADC_BUF_LEN			50				// ADC buffer length
#define ADC_SAMPLE_PERIOD	1199			// 1199 = 50 kHz sampling w/ 60 MHz SYSCLKOUT
#define PWM_DUTY_CYCLE		11250			// 25% duty cycle


/*=============================================================================*
 * 	Extern functions declaration
 *============================================================================*/
/* ePWM module initialization */
#define SYSCLOCK_FREQ		60000000
#define HSPCLK_FREQ			(SYSCLOCK_FREQ/2)			// defined in SysCtrlRegs        (SYSCLOCK_FREQ/2)
#define LSPCLK_FREQ			(SYSCLOCK_FREQ/4)			// defined in SysCtrlRegs
#define TBCLK_FREQ			(SYSCLOCK_FREQ)				// EPwm Timer clock   chu####    (SYSCLOCK_FREQ/2)

//---------------------------------------------------------------------------
// Include Standard C Language Header Files
//
#include <string.h>
//---------------------------------------------------------------------------
// Include any other Header Files
//
#include "DSP2803x_Device.h"				// Peripheral address definitions


#ifdef EXAMPLE_NONBIOS
	#include "DSP2803x_DefaultIsr.h"		// ISR definitions
#endif

#ifdef EXAMPLE_BIOS
	#ifdef EXAMPLE_FLASH
	    #include "F28035_example_BIOS_flashcfg.h"
	#endif
#endif



#include "50Hz_Bypass_DataAcquisition.h"
#include "50Hz_Bypass_ProtectionLogic.h"
#include "50Hz_Bypass_StateUpdate.h"
#include "50Hz_Bypass_SwitchLogic.h"
#include "50Hz_Bypass_Scib_PCOsc.h"
#include "50Hz_Bypass_SwitchLogic.h"
#include "50Hz_Bypass_ECan_Basic.h"
#include "50Hz_Bypass_EcanDataprotocol.h"
#include "50Hz_Bypass_I2cEeprom.h"
#include "50Hz_Bypass_Safety_EEprom.h"
#include "Sci.h"
#include "math.h"
#include "Epwm.h"

//---------------------------------------------------------------------------
// Function Prototypes
//
extern void DelayUs(Uint16);
extern void AdcOffsetSelfCal();

extern void InitECanGpio(void);
extern void InitECan(void);
extern void InitAdc(void);
extern void InitECap(void);
extern void InitEPwm(void);
extern void InitHRCap(void);

#ifdef EXAMPLE_FLASH
	extern void InitFlash(void);
#endif

extern void InitGpio(void);
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);
extern void InitWatchdog(void);
extern void SetDBGIER(Uint16);

#define KickDog ServiceDog     // For compatiblity with previous versions
extern void ServiceDog(void);
extern void DisableDog(void);
extern Uint16 CsmUnlock(void);
extern void IntOsc1Sel (void);
extern void IntOsc2Sel (void);
extern void XtalOscSel (void);
extern void ExtOscSel (void);
extern void InitPll(Uint16 pllcr, Uint16 clkindiv);
extern void InitPeripheralClocks(void);

#ifdef EXAMPLE_BIOS
	extern void UserInit(void);
#endif


//---------------------------------------------------------------------------
// Global symbols defined in the linker command file
//
#ifdef EXAMPLE_BIOS
	extern Uint16 hwi_vec_loadstart;
	extern Uint16 hwi_vec_loadsize;
	extern Uint16 hwi_vec_runstart;
	extern Uint16 trcdata_loadstart;
	extern Uint16 trcdata_loadsize;
	extern Uint16 trcdata_runstart;
#endif

#ifdef EXAMPLE_FLASH
	extern Uint16 secureRamFuncs_loadstart;
	extern Uint16 secureRamFuncs_loadsize;
	extern Uint16 secureRamFuncs_runstart;
#endif


//---------------------------------------------------------------------------
// Global Variable References
//
extern Uint16 AdcBuf[ADC_BUF_LEN];			// ADC data buffer allocation

#ifdef EXAMPLE_NONBIOS
	extern const struct PIE_VECT_TABLE PieVectTableInit;	// Pie vector table (non-BIOS only)
#endif


//---------------------------------------------------------------------------
// Macros
//
#define Device_cal_func_ptr (void (*)(void))0x3D7C80


//---------------------------------------------------------------------------
#endif  // end of F28035_EXAMPLE_H definition


//--- end of file -----------------------------------------------------
