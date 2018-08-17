// TI File $Revision: /main/2 $
// Checkin $Date:
//###########################################################################
//
// FILE:  Flash28335_API_Config.h
//
// TITLE: F28335 Flash Algo's - User Settings
//
// NOTE:  This file contains user defined settings that
//        are used by the F28335 Flash APIs.
//
//###########################################################################
// $TI Release:$
// $Release Date:$
//###########################################################################

#ifndef FLASH28335_API_CONFIG_H
#define FLASH28335_API_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// Variables that can be configured by the user. 

/*-----------------------------------------------------------------------------
   1. Specify the device.
      Define the device to be programmed as "1" (no quotes).
      Define all other devices as "0" (no quotes).  
-----------------------------------------------------------------------------*/

#define FLASH_F28335   1

/*-----------------------------------------------------------------------------
   2. Specify the clock rate of the CPU (SYSCLKOUT) in nS.

      Take into account the input clock frequency and the PLL multiplier
      that your application will use.
 
      Use one of the values provided, or define your own.
      The trailing L is required tells the compiler to treat 
      the number as a 64-bit value.  

      Only one statement should be uncommented.

      Example:  CLKIN is a 30MHz crystal. 
 
                If the application will set PLLCR = 0xA then the CPU clock 
                will be 150Mhz (SYSCLKOUT = 150MHz).  

                In this case, the CPU_RATE will be 6.667L
                Uncomment the line:  #define CPU_RATE  6.667L   
-----------------------------------------------------------------------------*/

#define CPU_RATE    6.667L   // for a 150MHz CPU clock speed (SYSCLKOUT)

//----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// **** DO NOT modify the code below this line ****
//-----------------------------------------------------------------------------
#define SCALE_FACTOR  1048576.0L*( (200L/CPU_RATE) )  // IQ20


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // -- end FLASH2833X_API_CONFIG_H 
