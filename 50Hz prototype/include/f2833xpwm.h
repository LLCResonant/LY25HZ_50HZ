/* ==================================================================================
File name:        F2833XPWM.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments
Description:  
Header file containing data type and object definitions and 
initializers. Also contains prototypes for the functions in F280XPWM.C.

Target: TMS320F2833x family
              
=====================================================================================
History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20: Using DSP280x v. 1.10 or higher 
------------------------------------------------------------------------------------*/

#ifndef __F2833X_PWM_H__
#define __F2833X_PWM_H__

#include "f2833xbmsk.h"

/*----------------------------------------------------------------------------
Initialization constant for the F2833X Time-Base Control Registers for PWM Generation. 
Sets up the timer to run free upon emulation suspend, count up-down mode
prescaler 1.
----------------------------------------------------------------------------*/
#define PWM_INIT_STATE ( FREE_RUN_FLAG +         \
                         PRDLD_IMMEDIATE  +       \
                         TIMER_CNT_UPDN +         \
                         HSPCLKDIV_PRESCALE_X_1 + \
                         CLKDIV_PRESCALE_X_1  +   \
                         PHSDIR_CNT_UP    +       \
                         CNTLD_DISABLE )

/*----------------------------------------------------------------------------
Initialization constant for the F2833X Compare Control Register. 
----------------------------------------------------------------------------*/
#define CMPCTL_INIT_STATE ( LOADAMODE_ZRO + \
                            LOADBMODE_ZRO + \
                            SHDWAMODE_SHADOW + \
                            SHDWBMODE_SHADOW )

/*----------------------------------------------------------------------------
Initialization constant for the F2833X Action Qualifier Output A Register. 
----------------------------------------------------------------------------*/
//#define AQCTLA_INIT_STATE ( CAU_SET + CAD_CLEAR )    //Clarke
#define AQCTLA_INIT_STATE ( CAU_CLEAR + CAD_SET )   //Park

/*----------------------------------------------------------------------------
Initialization constant for the F2833X Dead-Band Generator registers for PWM Generation. 
Sets up the dead band for PWM and sets up dead band values.
----------------------------------------------------------------------------*/
#define DBCTL_INIT_STATE  (BP_ENABLE + POLSEL_ACTIVE_HI_CMP)
//#define DBCTL_INIT_STATE  (BP_ENABLE + POLSEL_ACTIVE_LO_CMP) //set PWM ACL ,Active low complepentary mode


#define DBCNT_INIT_STATE   200   // 100 counts = 1 usec (delay) * 100 count/usec (for TBCLK = SYSCLK/1):100->1us @100MHz

/*----------------------------------------------------------------------------
Initialization constant for the F2833X PWM Chopper Control register for PWM Generation. 
----------------------------------------------------------------------------*/
#define  PCCTL_INIT_STATE  CHPEN_DISABLE

/*----------------------------------------------------------------------------
Initialization constant for the F2833X Trip Zone Select Register 
----------------------------------------------------------------------------*/
#define  TZSEL_INIT_STATE  DISABLE_TZSEL
              
/*----------------------------------------------------------------------------
Initialization constant for the F2833X Trip Zone Control Register 
----------------------------------------------------------------------------*/
#define  TZCTL_INIT_STATE ( TZA_HI_Z + TZB_HI_Z + \
                            DCAEVT1_HI_Z + DCAEVT2_HI_Z + \
                            DCBEVT1_HI_Z + DCBEVT2_HI_Z )
                                                                 
/*-----------------------------------------------------------------------------
Define the structure of the PWM Driver Object 
-----------------------------------------------------------------------------*/
typedef struct {   
        Uint32 PeriodMax;     // Parameter: PWM Half-Period in CPU clock cycles (Q0)
        int32 MfuncPeriod;    // Input: Period scaler (Q15) 2^15 
        int32 MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio (Q15)
        int32 MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio (Q15) 
        int32 MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio (Q15)
        int32 MfuncC4;        // Input: EPWM4 A&B Duty cycle ratio (Q15)
        int32 MfuncC5;        // Input: EPWM5 A&B Duty cycle ratio (Q15)
        int32 MfuncC6;        // Input: EPWM6 A&B Duty cycle ratio (Q15)
        int32 TaOffset;
        int32 TbOffset;
        int32 TcOffset;
        void (*update)();     // Pointer to the update function 
        } PWMGEN ;    

/*-----------------------------------------------------------------------------
Define a PWMGEN_handle
-----------------------------------------------------------------------------*/
typedef PWMGEN *PWMGEN_handle;

/*------------------------------------------------------------------------------
Default Initializers for the F2833X PWMGEN Object 
------------------------------------------------------------------------------*/
#define F2833X_FC_PWM_GEN    {1000,   \
                              0x7FFF, \
                              0x4000, \
                              0x4000, \
                              0x4000, \
                              0x4000, \
                              0x4000, \
                              0x4000, \
                              0x0000,\
                              0x0000,\
                              0x0000,\
                           (void (*)(Uint32))F2833X_PWM_Update \
                             }

#define PWMGEN_DEFAULTS 	F2833X_FC_PWM_GEN
/*------------------------------------------------------------------------------
 Prototypes for the functions in F2833XPWM.C
------------------------------------------------------------------------------*/
//void F2833X_PWM_Init(PWMGEN_handle);
void F2833X_PWM_Update(PWMGEN_handle);

#endif  // __F2833X_PWM_H__

