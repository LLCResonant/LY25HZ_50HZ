/*=============================================================================*
 *         Copyright(c) 
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_StateUpdate.c

 *  PURPOSE  : state changed between wait state, check state,
 *             normal state,fault state, for 3KW Parallel Inverter.
 * 
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					Li Zhang
 *    												Xun Gao
 *    												Jianxi Zhu
 ******************************************************************************/
 
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
Uint16 PermanentFaultCheck(void);    
Uint16 FaultCheck(void);
Uint16 FaultBackCheck(void);
void Times_WriteToEEPROM();
void Ref_WriteToEEPROM();

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
	static Uint16 CntReconnectionDelayTime = 0;

	if (PermanentFaultCheck())          //Permanent fault
	{
		RelaysOFF();
		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		CntReconnectionDelayTime = 0;
		g_Sys_Current_State = PermanentState;
		DryCrtl_OFF;
	}
	else if (FaultCheck())    		 //Fault state
	{
		RelaysOFF();
		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		CntReconnectionDelayTime = 0;
		g_Sys_Current_State = FaultState;
		DryCrtl_OFF;
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
	if(PermanentFaultCheck())          //Permanent fault
	{
		RelaysOFF();//2017.4.13  GX  Johnny 2017.4.19
		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		g_Sys_Current_State = PermanentState;
		DryCrtl_OFF;
	}
	else if (FaultCheck())             //Fault state
	{
		RelaysOFF();//2017.4.13  GX  Johnny 2017.4.19
		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		g_Sys_Current_State = FaultState;
		DryCrtl_OFF;
	}
	else
	{
		DC_Fan_Enable;

		if(0 == g_StateCheck.bit.DcPreCharCheckOver)
		{
		  DcPreCharCheck(); //2017.6.29 GX inverter test
		  //g_StateCheck.bit.DcPreCharCheckOver = 1;
		  //GRID_RELY_ON;
		}

		if(1 == g_StateCheck.bit.DcPreCharCheckOver)  //2017.4.18 GX confirmed
		{
			g_StateCheck.bit.DcPreCharCheckOver = 0;
      
			HWI_disable();

			RectifierStage_Init();
			InverterStage_Init();

			if(Count_DelayTime > 20)	//2017.8.9 GX
			{
				Count_DelayTime = 0;
				g_Sys_Current_State = NormalState;
			}
			else
			{
				g_Sys_Current_State = CheckState;
				Count_DelayTime ++ ;
			}
		}
      HWI_enable();
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
	if (PermanentFaultCheck())        //Permanent fault
	{
		RelaysOFF();//2017.4.13  GX   Johnny 2017.4.19
	    PfcPWMOutputsDisable();
	    InvPWMOutputsDisable();
    	g_Sys_Current_State = PermanentState;
    	DryCrtl_OFF;
	}
	else if (FaultCheck())                 // fault state
	{
	  	RelaysOFF();//2017.4.13  GX   Johnny 2017.4.19
	    PfcPWMOutputsDisable();
	    InvPWMOutputsDisable();
    	g_Sys_Current_State = FaultState;
    	DryCrtl_OFF;
	}
	else
		;
}

/********************************************************************************************************************
* FUNCION :  	Fault Process
* PURPOSE :  	go into Permanent Fault state when the fault is not come back happen,then close relay, close pwm;
* 						go into  checking state if  the fault is  come back
* CALLED BY:  	void StateSwitch(void)
************************************************************************************************************************/
void ProcessFault(void) 
{
	static int16 i16Cnt_backChangeTemp = 0;
	static Uint16 temp1 = 0;
	static Uint16 temp2 = 0;

    if (PermanentFaultCheck())         //Permanent fault
    {
	    PfcPWMOutputsDisable();
	    InvPWMOutputsDisable();
	    RelaysOFF();//2017.4.13  GX  Johnny 2017.4.19
	    g_Sys_Current_State = PermanentState;
    }
    else 
    {
    	temp1 ++;
	    PfcPWMOutputsDisable();
	    InvPWMOutputsDisable();
	    RelaysOFF();
	    g_StateCheck.bit.PfcSoftStart = 0;// GX 2017.12.20
	    if (temp1>=2000)//4s
	    {
	    	g_SysFaultMessage.bit.recoverSW_Bus_UVP = 0; //母线欠压自动恢复
	    	g_SysFaultMessage.bit.BusVoltUnbalanceFault = 0;//母线电压不平衡自动恢复
	    	temp1 = 0;
	    }

    	ShortCheck();//WF 2018.5.23

    	if (temp2 == 0)
    	{
    		Times_WriteToEEPROM();
    		Ref_WriteToEEPROM();
    		temp2 =1;
    		SYNC_COM2_ON;
    		//g_StateCheck.bit.fflag = 1;
    	}

        if (FaultBackCheck())
        {
			i16Cnt_backChangeTemp++;
			if(i16Cnt_backChangeTemp >= 500)// 2ms * 500 = 1s
			{
				g_StateCheck.bit.InvAD_initial = 1;
				ADChannelOffset.f32VGrid = 0;
				ADChannelOffset.f32IGrid = 0;
				ADChannelOffset.f32VInv = 0;
      			ADChannelOffset.f32IInv = 0;
				ADChannelOffset.f32VOut = 0;
				ADChannelOffset.f32VBusP = 0;
				ADChannelOffset.f32VBusN = 0;
				SYNC_COM2_OFF;
				i16Cnt_backChangeTemp = 0;
				temp2 = 0;
				DryCrtl_ON;

				g_Sys_Current_State = WaitState;
			}
        }
    }		 
}

/**********************************************************************
* FUNCION :  Permanent Process
* PURPOSE :  close relay,close pwm
* CALLED BY:  void StateSwitch(void)             
**********************************************************************/
void ProcessPermanent(void)
{
	static Uint8 temp = 0;
	if (PermanentFaultCheck())
	{
		RelaysOFF();   //2017.4.13  GX   Johnny 2017.4.19
	    PfcPWMOutputsDisable();
	    InvPWMOutputsDisable();
        g_Sys_Current_State = PermanentState;
        if (temp == 0)
        {
        	Times_WriteToEEPROM();
        	Ref_WriteToEEPROM();
        	temp =1;
        }
	}
	PfcPWMOutputsDisable();   //2018.6.2 GX
	InvPWMOutputsDisable();	//2018.6.2 GX
	RelaysOFF();
}

/****************************************************************************************
* FUNCION :  	Permanent Fault Check
* PURPOSE :  	check if Faults happen
* CALLED BY:  	void ProcessFault(void) ,void ProcessRunning(void) ,
* 						void ProcessChecking(void), void ProcessWaiting(void)
******************************************************************************************/
Uint16 FaultCheck(void)      
{
    if ((0 == g_SysFaultMessage.Word.byte0 ) && (0 == g_SysFaultMessage.Word.byte1) && (0 == g_SysFaultMessage.Word.byte2)
     && (0 == g_SysFaultMessage.Word.byte3 ) && (0 == g_SysFaultMessage.Word.byte4))
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
	if ((0 == g_SysFaultMessage.Word.byte0 ) && (0 == g_SysFaultMessage.Word.byte1) &&(0 == g_SysFaultMessage.Word.byte2)
   	&& (0 == g_SysFaultMessage.Word.byte3) && (0 == g_SysFaultMessage.Word.byte4))
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
	INV_RELY_OFF;
	Inv_CurrShare_OFF;
	//SYNC_COM2_OFF; //Why?
}

void DigitalIODetect()
{
    HwInvOCPDetection();
	HwInvOVPDetection();

	HwBusOVPDetection();
	HwGridOCPDetection();
}
void DigitalIOCheck()
{
	#ifdef FAN_CHECK
	DCFanCheck();//2017.3.26 GX
	#endif

	HwInvOCPCheck();
	HwInvOVPCheck();

	HwBusOVPCheck();
	HwGridOCPCheck();
}

//--- end of file -----------------------------------------------------
