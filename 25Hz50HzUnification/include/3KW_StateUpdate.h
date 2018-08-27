/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_ProtectionLogic.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/

#ifndef STATE_UPDATE
#define STATE_UPDATE

#define FAN_CHECK
#define LY25HZ	0x05
//#define LY50HZ	0x07

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

#define SECOND_EDITION
//#define THIRD_EDITION

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
        Uint16  FreVOutH_Fault:1;   			//
        Uint16  FreVOutL_Fault:1;    			//
		Uint16  :1;    			//
		Uint16  :1;    				//
		Uint16  :1;    		//
		Uint16  InvDrvEnable:1; 				//
		Uint16  Inv_SoftStart:1;    							//
		Uint16  :1;    							//
		//byte 3
		Uint16  DcFanControl:1;    //B0
		Uint16  DcFan1Fault:1;    //B1
		Uint16  DcFan2Fault:1;    //B2
		Uint16  DcFanFault:1;    //B3
		Uint16  Grid_PhaseLock:1;    //B4
		Uint16  Bus_OVP_Fault:1;    //B5
		Uint16  ECAN_Noline:1;    //B6
		Uint16  fflag:1;    //B7
		//byte 4
		Uint16  ACPowerDerating:1;    //
		Uint16  ECANPWMEnable:1;    //
		Uint16  Time_Overflow:1;    //
		Uint16  ECAN_Fault:1;    //
		Uint16  VGridDip_Disable_GridOCP:1;    	//
		Uint16  :1;    //
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
        Uint16  HW_Bus_OVP:1; 			// B2
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
        Uint16  recoverSW_Bus_UVP:1;    	//B4
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

#endif

//--- end of file -----------------------------------------------------


