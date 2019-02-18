/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_StateUpdate.c
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/
 
//Main including header files
#include "DSP2833x_Device.h"		// Peripheral address definitions
#include "3KW_MAINHEADER.h"				// Main include file

//--- Global variables
/*==========================================
* NAME      :  g_StateCheck
* PURPOSE :  indication signal state that is checked
* RANGE    :  union Uint 16 bits
*==========================================*/ 
union STATE_CHECK  g_StateCheck ={0}; 

/*==========================================
* NAME      :  g_SysWarningMessage
* PURPOSE :  indication all warning  message
* RANGE    :  union Uint 16 bits
*==========================================*/
union SYS_WARN_MESSAGE  g_SysWarningMessage ={0};

/*==========================================
* NAME      :  g_SysFaultMessage
* PURPOSE :  indication all fault  message   
* RANGE    :  union
*==========================================*/ 
union SYS_FAULT_MESSAGE g_SysFaultMessage={0};

/*==========================================
* NAME      :  g_Sys_Current_State
* PURPOSE :  indication current state for pv inverter 
* RANGE    :  enum
*==========================================*/ 
enum SYS_STATE	 g_Sys_Current_State = WaitState;

//--- Global functions
void ProcessWaiting(void);
void ProcessChecking(void);
void ProcessRunning(void);
void ProcessPermanent(void);        
void ProcessFault(void);         
void DigitalIODetect(void);
void DigitalIOCheck(void);
void RelaysOFF(void);
Uint8 FaultHandling(void);
void FaultRecover(void);
Uint16 PermanentFaultCheck(void);    
Uint16 FaultCheck(void);
Uint16 FaultBackCheck(void);

/***********************************************************************************************
* FUNCION :  	State Switch
* PURPOSE :  	state changed for waiting,checking,running,stop,fault, permanent fault
* 						when condition is ready
* CALLED BY: 	DSP/BIOS  kernel task  every 2ms
**********************************************************************************************/
void StateSwitch(void)
{
    while (1) // start of task main loop 
    {
    	if (TRUE == SEM_pend(&TimeBase2msReady, SYS_FOREVER) )
    	{
    		DigitalIODetect();
			DigitalIOCheck();

			switch (g_Sys_Current_State)
			{
			case WaitState:
				ProcessWaiting();           	// waiting  state
            break;  
			case CheckState:
				ProcessChecking();         	// checking  state
            break;  
			case NormalState:
				ProcessRunning();          	// running  state
            break;   
			case FaultState:
				ProcessFault();             	 // fault state
            break;   
			case PermanentState:
                ProcessPermanent();    		 // forever fault state
            break; 
			default:
            break;	  
			}
    	}
    } // end of task main loop 
} // end of StateSwitch

/********************************************************************************************************************
* FUNCION :  	Waiting Process
* PURPOSE :  	go into Permanent Fault state when the fault is not come back happen,
*           		 	go into Fault state when the fault is come back happen, then close Relay, close pwm;
*            			go into check state when Bus voltage is larger that Standby voltage,
* CALLED BY:  	void StateSwitch(void)
**********************************************************************************************************************/
void ProcessWaiting(void)
{
	// start of ProcessWATING
	static Uint16 CntReconnectionDelayTime = 0;
	Uint8 Fault_stste_temp = 0;

	Fault_stste_temp = FaultHandling();
	if (Fault_stste_temp == 1)          //Permanent fault
	{
		CntReconnectionDelayTime = 0;
		g_Sys_Current_State = PermanentState;
	}
	else if (Fault_stste_temp == 2)    		 //Fault state
	{
		CntReconnectionDelayTime = 0;
		g_Sys_Current_State = FaultState;
	}
	else
	{
		CntReconnectionDelayTime++;
		//wait 2s for longer previous charge time
		if(CntReconnectionDelayTime > 1000 && g_StateCheck.bit.Grid_PhaseLock == 1)// 1000 * 2ms = 2s
		{
			CntReconnectionDelayTime = 0;
			g_StateCheck.bit.DcPreCharCheckOver = 0;
			g_Sys_Current_State = CheckState;
		}
	}
} // end of ProcessWATING

void ProcessChecking(void)
{ // start of ProcessCHECKING
	static Uint16 Count_DelayTime = 0;
	Uint8 Fault_stste_temp = 0;

	Fault_stste_temp = FaultHandling();
	if(Fault_stste_temp == 1)          			//Permanent fault
	{
		g_Sys_Current_State = PermanentState;
	}
	else if (Fault_stste_temp == 2)             //Fault state
	{
		g_Sys_Current_State = FaultState;
	}
	else
	{
		DC_Fan_Enable;

		if(0 == g_StateCheck.bit.DcPreCharCheckOver)
			DcPreCharCheck();

		if(1 == g_StateCheck.bit.DcPreCharCheckOver)
		{
			g_StateCheck.bit.DcPreCharCheckOver = 0;

			//wait 2 Grid period for advance charge after relay picking up.
			if(Count_DelayTime > 20)
			{
				Count_DelayTime = 0;
				HWI_disable();
				RectifierStage_Init();
				InverterStage_Init();
				HWI_enable();
				g_Sys_Current_State = NormalState;
			}
			else
			{
				Count_DelayTime ++ ;
			}
		}
	}
}

/**********************************************************************************************************************
* FUNCION :  	Running Process
* PURPOSE :  	go into Permanent Fault state when the fault is not come back happen,
* 						go into Fault  state when the fault is come back happen, then close relay,close pwm;
* CALLED BY:  	void StateSwitch(void)
*********************************************************************************************************************/
void ProcessRunning(void) 
{
	// start of ProcessRUNNING
	Uint8 Fault_stste_temp = 0;
	static Uint8 u8_count = 0;

	Fault_stste_temp = FaultHandling();
	if(Fault_stste_temp == 1)          			//Permanent fault
	{
		g_Sys_Current_State = PermanentState;
	}
	else if (Fault_stste_temp == 2)             //Fault state
	{
		g_Sys_Current_State = FaultState;
	}
	else
	{
		if (g_Sys_Bypass_State == Bypass &&\
				1 == g_StateCheck.bit.Inv_SoftStart && g_StateCheck.bit.InvOut == 0)
		{
			u8_count ++;
			if (u8_count >=10)
			{
				g_StateCheck.bit.InvOut = 1;
				u8_count = 0;
			}
		}
		if (g_Sys_Bypass_State == Bypass && g_StateCheck.bit.InvOut == 1)
		{
			SYNC_COM2_ON;
			InvL_CurrShare_or_COM4_OFF;
			//g_StateCheck.bit.BypToInv = 1;
		}
	}

} // end of ProcessRUNNING

/********************************************************************************************************************
* FUNCION :  	Fault Process
* PURPOSE :  	go into Permanent Fault state when the fault is not come back happen,then close relay, close pwm;
* 						go into  checking state if  the fault is  come back
* CALLED BY:  	void StateSwitch(void)
************************************************************************************************************************/
void ProcessFault(void) 
{ // start of ProcessFAULT

	static int16 i16Cnt_backChangeTemp = 0;
	static Uint16 i16Fault_recover_temp1 = 0;
	Uint8 Fault_stste_temp = 0;

	Fault_stste_temp = FaultHandling();
	if(Fault_stste_temp == 1)          			//Permanent fault
	{
		g_Sys_Current_State = PermanentState;
	}
	else
	{
		ShortRecover();
		i16Fault_recover_temp1 ++;
	    if (i16Fault_recover_temp1>=2000)//4s
	    {
	    	g_StateCheck.bit.PfcSoftStart = 0;
	    	g_SysFaultMessage.bit.recoverSW_Bus_UVP = 0; //Low Bus voltage automatic recover
	    	g_SysFaultMessage.bit.BusVoltUnbalanceFault = 0;//Bus voltage unbalanced automatic recover
	    	i16Fault_recover_temp1 = 0;
	    }
        if (FaultBackCheck())
        {
			i16Cnt_backChangeTemp++;
			if(i16Cnt_backChangeTemp >= 500)// 2ms * 500 = 1s
			{
				FaultRecover();
				i16Cnt_backChangeTemp = 0;
				g_Sys_Current_State = WaitState;
			}
        }
    }
} // end of ProcessFAULT

/**********************************************************************
* FUNCION :  Permanent Process
* PURPOSE :  close relay,close pwm
* CALLED BY:  void StateSwitch(void)             
**********************************************************************/
void ProcessPermanent(void)
{
	;
}

/****************************************************************************************
* FUNCION :  	Permanent Fault Check
* PURPOSE :  	check if Faults happen
* CALLED BY:  	void ProcessFault(void) ,void ProcessRunning(void) ,
* 						void ProcessChecking(void), void ProcessWaiting(void)
******************************************************************************************/
Uint16 FaultCheck(void)      
{
    if ((0 == g_SysFaultMessage.Word.byte0 ) && (0 == g_SysFaultMessage.Word.byte1) && (0 == g_SysFaultMessage.Word.byte2) && (0 == g_SysFaultMessage.Word.byte3 ) && (0 == g_SysFaultMessage.Word.byte4))
    {
    	return(0);
    }
    else
    {
    	return(1);
    }
}

/**********************************************************************
* FUNCION :  	Fault Check
* PURPOSE :  	check if  permanent Faults happen
* CALLED BY:  	void ProcessFault(void) ,void ProcessRunning(void) ,
* 						void ProcessChecking(void), void ProcessWaiting(void)
**********************************************************************/
Uint16 PermanentFaultCheck(void)
{
    if ((0 == g_SysFaultMessage.Word.unrecover1) && (0 == g_SysFaultMessage.Word.unrecover2) && (0 == g_SysFaultMessage.Word.unrecover3))
    {
    	return(0);
    }
    else
    {
    	return(1);
    }

}

/**********************************************************************
* FUNCION :  	Fault Back Check
* PURPOSE :  	check if Faults recover
* CALLED BY:  	void ProcessFault(void) ,void ProcessRunning(void) ,
* 						void ProcessChecking(void), void ProcessWaiting(void)
**********************************************************************/
Uint16 FaultBackCheck(void)
{
	if ((0 == g_SysFaultMessage.Word.byte0) && (0 == g_SysFaultMessage.Word.byte1) && (0 == g_SysFaultMessage.Word.byte2) && (0 == g_SysFaultMessage.Word.byte3) && (0 == g_SysFaultMessage.Word.byte4))
    {
		return(1);
    }
   	else
   	{
   		return(0);
    }
}

void RelaysOFF(void)
{
	INVH_RELY_OFF;
	INVL_RELY_OFF;

	InvH_CurrShare_OFF;
	#ifdef LY25HZ
	InvL_CurrShare_or_COM4_OFF;
	#endif
}

Uint8 FaultHandling(void)
{
	if (PermanentFaultCheck())          //Permanent fault
	{
		if (g_Sys_Bypass_State == noBypass)
		{
			SYNC_COM2_ON;
		}
		else
		{
			SYNC_COM2_OFF;
			InvL_CurrShare_or_COM4_ON;
			g_StateCheck.bit.Inv_SoftStart = 0;
			//if(COM4_LEVEL == 1)
			//{
				//g_StateCheck.bit.BypToInv = 0;
			//}
		}

		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		RelaysOFF();
		DryCrtl_OFF;
		return 1;
	}
	else if (FaultCheck())    		 //Fault state
	{
		if (g_Sys_Bypass_State == noBypass)
		{
			SYNC_COM2_ON;
		}
		else
		{
			SYNC_COM2_OFF;
			InvL_CurrShare_or_COM4_ON;
			g_StateCheck.bit.Inv_SoftStart = 0;
			//if(COM4_LEVEL == 1)
			//{
				//g_StateCheck.bit.BypToInv = 0;
			//}
		}

		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		RelaysOFF();
		DryCrtl_OFF;
		return 2;
	}
	else
		return 0;
}

void FaultRecover(void)
{
	//g_StateCheck.bit.InvAD_initial = 1;  //2019.1.12GX
	ADChannelOffset.f32VGrid = 0;
	ADChannelOffset.f32IGrid = 0;
	ADChannelOffset.f32VInvH = 0;
	ADChannelOffset.f32IInvH = 0;
	ADChannelOffset.f32VOutH = 0;
	ADChannelOffset.f32VInvL = 0;
	ADChannelOffset.f32IInvL = 0;
	ADChannelOffset.f32VOutL = 0;
	ADChannelOffset.f32VBusP = 0;
	ADChannelOffset.f32VBusN = 0;
	g_ParaLogic_State.bit.SelfPhaseOut_EN = 0;
	g_ParaLogic_State.bit.SyncPhase_Flag = 0;
	g_StateCheck.bit.InvOut = 0;

	SYNC_COM2_OFF;
	//if (g_Sys_Bypass_State == Bypass)
	//{
		//InvL_CurrShare_or_COM4_OFF;
	//}//2018.12.24

	DryCrtl_ON;
}

void DigitalIODetect()
{
	HwInvHOCPDetection();
	HwInvHOVPDetection();

	HwBusOVPDetection();
	HwGridOCPDetection();

	HwInvLOCPDetection();
	HwInvLOVPDetection();
}
void DigitalIOCheck()
{
	#ifdef FAN_CHECK
	DCFanCheck();
	#endif
	HwInvHOCPCheck();
	HwInvHOVPCheck();

	HwBusOVPCheck();
	HwGridOCPCheck();

	HwInvLOCPCheck();
	HwInvLOVPCheck();
}

//--- end of file -----------------------------------------------------
