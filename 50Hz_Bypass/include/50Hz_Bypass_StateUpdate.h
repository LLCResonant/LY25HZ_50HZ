/***********************************************************************
 *    Copyright(c) 2010-2011,    Convertergy Co.,Ltd.

 *    FILENAME  : 5KW_StateUpdate.h
 *    PURPOSE  :  define constant, struct declaration
 *                        
 *    HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-24      V0.1           Ken      	    Created 
 *
 *		
 * 	Software Development Environment:
 *	-- CCS 3.3.81.6
 * 	-- BIOS 5.33.06
 *	-- Code Generation Tool v5.2.4 

 ************************************************************************/

#ifndef STATE_UPDATE  //MAINSTATUSMACHINE_H
#define STATE_UPDATE  //MAINSTATUSMACHINE_H

#define DC_Fan1_Enable				GpioDataRegs.GPASET.bit.GPIO3 = 1
#define DC_Fan1_State					GpioDataRegs.GPADAT.bit.GPIO3
#define DC_Fan1_Disable				GpioDataRegs.GPACLEAR.bit.GPIO3 = 1

#define DC_Fan2_Enable				GpioDataRegs.GPASET.bit.GPIO21 = 1
#define DC_Fan2_State					GpioDataRegs.GPADAT.bit.GPIO21
#define DC_Fan2_Disable				GpioDataRegs.GPACLEAR.bit.GPIO21 = 1

#define DryCntl_ON						GpioDataRegs.GPASET.bit.GPIO10 = 1
#define DryCntl_OFF						GpioDataRegs.GPACLEAR.bit.GPIO10 = 1

#define LED_Warning_ON				GpioDataRegs.GPASET.bit.GPIO17 = 1
#define LED_Warning_OFF			GpioDataRegs.GPACLEAR.bit.GPIO17 = 1
#define LED_Fault_ON					GpioDataRegs.GPASET.bit.GPIO8 = 1
#define LED_Fault_OFF					GpioDataRegs.GPACLEAR.bit.GPIO8 = 1
//================ Global variables =====================================

//System Status definition for StateMachine
enum   SYS_STATE                             
{
 	CheckState = 0,
 	NormalState = 1,
 	Standby = 2,
 	FaultState = 3,
 	ForceNormalState = 4,
 	LockState = 5
} ; 
extern enum	SYS_STATE	g_Sys_State;

union  STATE_CHECK 
{
    struct
    {
        Uint16  byte0:8;
        Uint16  byte1:8;
    }Word;

    struct
    {
		//byte 0
        Uint16  Zero_Crossing_Flag:1;    // B0
        Uint16  SelfPhaseOut_EN:1;    // B1
        Uint16  Time_Overflow:1;    // B2
        Uint16  DcFan1Fault:1;    // B3
        Uint16  DcFan2Fault:1;    // B4
        Uint16  ECAN_Fault:1;    // B5
        Uint16  ECAN_Noline:1;    // B6
        Uint16  AD_initial:1;    // B7     0: hardware  interrupt   1:softwar force  interrupt
		//byte 1
		Uint16  COM4:1;    // B0
	    Uint16  COM2:1;    // B1
	    Uint16  Fault:1;    // B2
	    Uint16  Force_Switch:1;    // B3
	    Uint16  Module_Lock:1;    // B4
	    Uint16  :1;    // B5
	    Uint16  :1;    // B6
	    Uint16  :1;	// B7
    }bit;
}; 
extern union STATE_CHECK  g_StateCheck; 

union  SYS_WARN_MESSAGE
{
    struct
    {
        Uint16  byte0:8;
        Uint16  byte1:8;
    }Word;

    struct
    {
		//byte 0
        Uint16  :1;    		// B0
        Uint16  :1;    		// B1
        Uint16  :1;    				// B2
        Uint16  OverTemp:1;    				// B3
        Uint16  Fan1Block:1; 				// B4
        Uint16  Fan2Block:1;				// B5
        Uint16  Fan_Fault:1;					// B6
        Uint16  :1;    		// B7
		//byte 1
        Uint16  :1;    		// B0
        Uint16  :1;      	// B1
        Uint16  :1; 			// B2
        Uint16  :1;  			// B3
        Uint16  :1;			// B4
        Uint16  :1;  //B5
        Uint16  :1; //B6
        Uint16  :1;    //B7
    }bit;
};
extern union SYS_WARN_MESSAGE  g_SysWarningMessage;

union  SYS_FAULT_MESSAGE   
{
    struct
    {
        Uint16  Bypass:8;
        Uint16  Overall:8;
    }Word;  
   
    struct
    {
        //byte0
        Uint16  VGridOverRating:1;     //B0
        Uint16  VGridUnderRating:1;     //B1
        Uint16  FreGridOverRating:1;    //B2
        Uint16  FreGridUnderRating:1;    //B3
        Uint16  :1;    //B4
        Uint16  HWADFault_VGrid:1;    //B5
        Uint16  HWADFault_VOut:1;    //B6
        Uint16  :1;    //B7

        //byte1
        Uint16  OverTempFault:1;   //B0
        Uint16  :1;    //B2          
      	Uint16  :1;    //
       	Uint16  :1;    //
      	Uint16  :1;     //B6
        Uint16  :1;    //B7
        Uint16  :1;    //B1  
        Uint16  :1;    //     

     }bit;
};
extern union SYS_FAULT_MESSAGE g_SysFaultMessage;

//===================== Global functions==================================
extern void SCROFF(void);
extern void SCRON(void);
extern void FanCntl(void);

#endif

//--- end of file -----------------------------------------------------


