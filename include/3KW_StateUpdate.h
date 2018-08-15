/***********************************************************************
 *    Copyright(c) 2010-2011,    

 *    FILENAME  : 3KW_StateUpdate.h
 *    PURPOSE  :  define constant, struct declaration
 *                        
 *    HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *     
 *
 *		
 * 	Software Development Evironment: 
 *	-- CCS 3.3.81.6
 * 	-- BIOS 5.33.06
 *	-- Code Generation Tool v5.2.4 

 ************************************************************************/

#ifndef STATE_UPDATE  //MAINSTATUSMACHINE_H
#define STATE_UPDATE  //MAINSTATUSMACHINE_H

#define SW_Version1 2012
#define SW_Version2 0101
#define SW_Version3 V1.2.1
#define CurLoop 0       //
#define OpenLoop 1 
#define MpptLoop 2
#define StabilizedVoltLoop 3      
#define WorkMode OpenLoop 

//#define WorkMode   MpptLoop
//#define WorkMode StabilizedVoltLoop

//safety define
/*
#define  CHINA_SUN  1
#define  VDE0126    2
#define  RD1663     3   //Spanish
#define  DK5940     4
#define FunctionSafety CHINA_SUN
*/
//================ Global variables =====================================

//System Status definition for StateMachine
enum   SYS_STATE                             
{
 WaitState, CheckState,NormalState,FaultState,PermanentState
} ; 
extern enum	SYS_STATE	g_Sys_Current_State;

union  STATE_CHECK 
{
    struct
    {
        Uint16  byte0:8;
        Uint16  byte1:8;
        Uint16  byte2:8;
        Uint16  byte3:8;
        Uint16  byte4:8;	
        Uint16  byte5:8;
        Uint16  byte6:8;
    }Word;

    struct
    {
		//byte 0
        Uint16  :1;    				// B0
        Uint16  GridAD_initial:1;    				// B1
        Uint16  InvAD_initial:1;    	// B2
        Uint16  :1;    					// B3
        Uint16  Grid_Zero_Crossing_Flag:1;   		// B4
        Uint16  Inv_Zero_Crossing_Flag:1;    		// B5
        Uint16  PfcSoftStart:1;    		// B6
        Uint16  PwmForceOffFlag:1;    			// B7     0: hardware  interrupt   1:softwar force  interrupt   
		//byte 1
        Uint16  OCP_InvH:1;    					// B0
        Uint16  OCP_InvL:1;      				// B1
        Uint16  OCP_AcGrid:1; 					// B2
        Uint16  OVP_InvH:1;     				// B3
        Uint16  OVP_InvL:1;     				// B4
        Uint16  RlyCheckOver:1;  				//B5
        Uint16  DcPreCharCheckOver:1; 			//B6
        Uint16  Slowupmode:1;    //B7	== 1 represents slowup mode
		//byte 2
        Uint16  :1;   			//
        Uint16  :1;    			//
		Uint16  :1;    			//
		Uint16  :1;    				//
		Uint16  :1;    		//
		Uint16  InvDrvEnable:1; 				//
		Uint16  Inv_SoftStart:1;    							//
		Uint16  :1;    							//
		//byte 3
		Uint16  DcFanControl:1;    //0
		Uint16  DcFan1Fault:1;    //1
		Uint16  DcFan2Fault:1;    //2
		Uint16  DcFanFault:1;    //3
		Uint16  Grid_PhaseLock:1;    ////2017.8.14 GX
		Uint16  Bus_OVP_Fault:1;    //
		Uint16  ECAN_Noline:1;    //
		Uint16  fflag:1;    //
		//byte 4
		Uint16  ACPowerDerating:1;    //
		Uint16  ECANPWMEnable:1;    //2017.12.18 GX
		Uint16  Time_Overflow:1;    //
		Uint16  ECAN_Fault:1;    //
		Uint16  Input_dip_Disable_GridOCP:1;    	//
		Uint16  Input_dip_PLL_Keep:1;    //
		Uint16  :1;    //
		Uint16  :1;    //
		//byte 5
		Uint16  VInvHUnderRating:1;    //
		Uint16  VInvLUnderRating:1;    //
		Uint16  Sync_Fault1:1;    //
		Uint16  Sync_Fault2:1;    //
		Uint16  ShortInv:1;    //
		Uint16  :1;    //
		Uint16  ParallelInvHOCP:1;    //
		Uint16  ParallelInvLOCP:1;    //
		//byte 6
		Uint16  HWADFault_IGrid_ave:1;    //B0
		Uint16  HWADFault_VGrid_ave:1;    //B1
		Uint16  HWADFault_VInvH_ave:1;    //B2
		Uint16  HWADFault_VInvL_ave:1;    //B3
		Uint16  HWADFault_IInvH_ave:1;    //B4
		Uint16  HWADFault_IInvL_ave:1;    //B5
		Uint16  HWADFault_VOutH_ave:1;    //B6
		Uint16  HWADFault_VOutL_ave:1;    //B7
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
        Uint16  ECAN_Fault:1;    		// B0
        Uint16  :1;    		// B1
        Uint16  :1;    				// B2
        Uint16  OverTemp:1;    				// B3
        Uint16  Fan1Block:1; 				// B4
        Uint16  Fan2Block:1;				// B5
        Uint16  InvAsyn:1;					// B6
        Uint16  :1;    		// B7
		//byte 1
        Uint16  VInvHUnderRating:1;    		// B0
        Uint16  VInvLUnderRating:1;      	// B1
        Uint16  HW_Bus_OVP:1; 			// B2  硬件过压是告警而非保护
        Uint16  InvH_OverLoad:1;  			// B3
        Uint16  InvL_OverLoad:1;			// B4
        Uint16  LCD_Comm_Error:1;  //B5
        Uint16  :1; //B6
        Uint16  :1;    //B7
    }bit;
};
extern union SYS_WARN_MESSAGE  g_SysWarningMessage;


union  SYS_FAULT_MESSAGE   
{
    struct
    {
        Uint16  byte0:8;
        Uint16  byte1:8;
        Uint16  byte2:8;
        Uint16  byte3:8;
        Uint16  byte4:8;	
        Uint16  unrecover1 :8;
        Uint16  unrecover2 :8;
        Uint16  unrecover3:8;
    }Word;  

    struct
    {
        //byte0
        Uint16  VGridOverRating:1;     	//B0
        Uint16  VGridUnderRating:1;    	//B1
        Uint16  FreGridOverRating:1;    //B2
        Uint16  FreGridUnderRating:1;   //B3
        Uint16  :1;    		//B4
        Uint16  :1;       //B5
        Uint16  :1;	//B6
        Uint16  BusVoltUnbalanceFault:1;   //B7

        //byte1
        Uint16  :1;		//B0
        Uint16  :1;		//B1
        Uint16  :1; 		//B2
        Uint16  :1;  		//B3
        Uint16  :1;    		//B4
        Uint16  :1;		//B5
        Uint16  InvTempOverLimit:1;		//B6
        Uint16  PfcOverTempFault:1;			//B7

        //byte2
        Uint16  :1; 	//B0
        Uint16  :1; 	//B1
        Uint16  :1;		//B2
        Uint16  :1;    					//B3
  	    Uint16  :1;    						//B4
        Uint16  :1;    						//B5
        Uint16  :1;        					//B6
        Uint16  :1;    						//B7

    	//byte3
        Uint16  :1;	//B0
        Uint16  unrecoverInvHCurrSharFault:1;	//B1
        Uint16  unrecoverInvLCurrSharFault:1;	//B2
        Uint16  OCP_AC_RMS:1;    //B3
        Uint16  :1;    		//B4
        Uint16  :1;   		//B5
        Uint16  :1;  					//B6
        Uint16  :1;		//B7

        //byte4
        Uint16  InvH_OverLoad:1;   		//B0
        Uint16  InvL_OverLoad:1;    	//B1
        Uint16  recoverHW_InvH_OCP:1;    	//B2
        Uint16  recoverHW_InvL_OCP:1;    	//B3
        Uint16  recoverSW_Bus_UVP:1;    	//B4  2018.1.24 GX
        Uint16  :1;   	//B5
        Uint16  :1;   	//B6
        Uint16  :1;   	//B7

        //unrecover1
        Uint16  :1;    	//B0
        Uint16  DcFuseFault:1;    	//B1
        Uint16  DcFanFault:1;     	//B2
        Uint16  HWADFault:1;    	//B3
        Uint16  :1;    	//B4
        Uint16  :1;      //B5
        Uint16  FreInvOverRating:1;     	//B6
        Uint16  FreInvUnderRating:1;     	//B7

        //unrecover2
        Uint16  unrecoverSW_OCP_Grid:1;  		//B0
        Uint16  unrecoverSW_OCP_InvH:1;			//B1
        Uint16  unrecoverSW_OCP_InvL:1;      	//B2
        Uint16  unrecover_RestartNum:1;    	//B3
        Uint16  :1;    		//B4
        Uint16  :1;   //B5
        Uint16  unrecoverHW_OCP_AC:1;   //B6
        Uint16  unrecoverHW_Bus_OVP:1;   //B7

        //unrecover3
        Uint16  unrecoverHW_SynLine_cut:1;   	//B0
        Uint16  unrecoverSW_Bus_OVP:1;   	//B1
      	Uint16  unrecoverHW_InvH_OCP:1;		//B2
       	Uint16  unrecoverSW_InvH_OVP:1;    	//B3
      	Uint16  unrecoverHW_InvH_OVP:1;    	//B4
        Uint16  unrecoverHW_InvL_OCP:1;		//B5
        Uint16  unrecoverSW_InvL_OVP:1;    	//B6
        Uint16  unrecoverHW_InvL_OVP:1;    	//B7

     }bit;
};
extern union SYS_FAULT_MESSAGE g_SysFaultMessage;

//===================== Global functions==================================
extern void RelaysOFF(void);
//main statemachine
//extern void SysParamDefault(); 
//extern void ProcessWaiting(void);
//extern void ProcessChecking(void);
//extern void ProcessRunning(void);
//extern void ProcessPermanent(void);        
//extern void ProcessFault(void);           


#endif

//--- end of file -----------------------------------------------------


