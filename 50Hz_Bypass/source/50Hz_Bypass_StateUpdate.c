/*=============================================================================*
 *         Copyright(c) 2010-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_StateUpdate.c 

 *  PURPOSE  : state changed between wait state, check state,
 *             normal state,fault state, 
 * 
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-24      V0.1           Ken      	    Created
 *
 ******************************************************************************/
 
//Main including header files
#include "DSP2803x_Device.h"		// Peripheral address definitions
#include "F28035_example.h"				// Main include file


//--- Global variables

/*==========================================
* NAME      :  g_StateCheck
* PURPOSE :  indication signal state that is checked
* RANGE    :  union Uint 16 bits
*
*
* CALLED BY: void StateSwitch(void),void EnergyCalcLowPrio(void)
*==========================================*/ 
union STATE_CHECK  g_StateCheck = {0};

/*==========================================
* NAME      :  g_SysFaultMessage
* PURPOSE :  indication all fault  message   
* RANGE    :  union
*
*
* CALLED BY: void StateSwitch(void),void EnergyCalcLowPrio(void)
*==========================================*/ 
union SYS_FAULT_MESSAGE g_SysFaultMessage = {0};
union SYS_WARN_MESSAGE  g_SysWarningMessage = {0};
/*==========================================
* NAME      :  g_Sys_Current_State
* PURPOSE :  indication current state for pv inverter 
* RANGE    :  enum
*
*
* CALLED BY: void StateSwitch(void),void EnergyCalcLowPrio(void)
*==========================================*/ 
enum SYS_STATE	g_Sys_State = CheckState;

//--- Global functions
void ProcessChecking(void);
void ProcessRunning(void);  	//旁路工作状态
void ProcessWaiting(void);		//逆变工作状态
void ProcessFault(void);
void ProcessLock(void);
void ProcessForceRunning(void);
void SCROFF(void);
void SCRON(void);
void FanCntl(void);

//extern void CheckSafety_PowerUp(void);

/**********************************************************************
* FUNCION :  State Switch
* PURPOSE :  state changed for waiting,checking,running,stop,fault when condition is ready
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY: DSP/BIOS  kernel task  every 2ms 
* 
**********************************************************************/
void StateSwitch(void)
{
	while (1) // start of task main loop
    {
		DCFanCheck();
		if (TRUE == SEM_pend(&TimeBase2msReady, SYS_FOREVER) )
		{
			switch (g_Sys_State)
			{
			case CheckState:
				ProcessChecking();         //  checking  state
				break;
			case NormalState:
				ProcessRunning();          // running  state
				break;
			case Standby:
				ProcessWaiting();          // running  state
				break;
			case FaultState:
				ProcessFault();            // fault state
				break;
			case LockState:
				ProcessLock();            // fault state
				break;
			case ForceNormalState:
				ProcessForceRunning();            // fault state
				break;
			default:
				break;
			}
		}
		if (g_Sys_State == FaultState)
		{
			LED_Fault_ON;
			DryCntl_OFF;
		}
		else
		{
			LED_Fault_OFF;
			DryCntl_ON;
		}

		if (g_SysWarningMessage.Word.byte0 == 0)
			LED_Warning_OFF;
		else
			LED_Warning_ON;
    } // end of task main loop
} // end of StateSwitch

/**********************************************************************
* FUNCION : ProcessChecking
* PURPOSE : 
*                  
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  void StateSwitch(void)             
* 
**********************************************************************/
void ProcessChecking(void)// start of ProcessWATING
{
	;
}

/**********************************************************************
* FUNCION :  Running Process
* PURPOSE :  go into Permanent Fault state when the fault is not come back happen, go into generic  Fault  state when the fault is come back happen, then close contactor,close pwm;
*                   go into waiting  state when bus voltage is less that  standbus voltage 
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  void StateSwitch(void)             
* 
**********************************************************************/
void ProcessWaiting(void) // start of ProcessRUNNING
{
	static Uint16 DelayTime = 0;
	SCR_INV_Enable();

	if (DelayTime >= 200)
		SCR_Bypass_Disable();
	else
		DelayTime ++;

	g_StateCheck.bit.SelfPhaseOut_EN = 0;
} // end of ProcessRUNNING

void ProcessRunning(void) // start of ProcessRUNNING
{
	SCR_Bypass_Enable();
	SCR_INV_Disable();
	g_StateCheck.bit.SelfPhaseOut_EN = 1;
} // end of ProcessRUNNING

void ProcessForceRunning(void) // start of ProcessRUNNING
{
	if (g_StateCheck.bit.Fault == 1)
	{
		SCR_Bypass_Disable();
		SCR_INV_Disable();
	}
	else
	{
		SCR_INV_Disable();
		SCR_Bypass_Enable();
	}
	g_StateCheck.bit.SelfPhaseOut_EN = 1;
} // end of ProcessRUNNING

void ProcessLock(void) // start of ProcessRUNNING
{
	SCR_Bypass_Disable();
	SCR_INV_Disable();
} // end of ProcessRUNNING
/**********************************************************************
* FUNCION :  Fault Process
* PURPOSE :  go into Permanent Fault state when the fault is not come back happen,then close contactor,close pwm; go into  checking state if  the fault is  come back 
*                   
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  void StateSwitch(void)             
* 
**********************************************************************/
void ProcessFault(void)  // start of ProcessFAULT
{
	SCR_Bypass_Disable();
} // end of ProcessFAULT

/**********************************************************************
* FUNCION :   Fault Check
* PURPOSE :  check if  Fault is happend
*                   
* INPUT :
*        void
* RETURN :
*        Uint16
* CALLS:
*        void
*
* CALLED BY: void ProcessRunning(void)       
* 
**********************************************************************/
/**********************************************************************
* FUNCION :  Fault Back Check
* PURPOSE :  check if  generic Fault is happend
*                   
* INPUT :
*        void
* RETURN :
*        Uint16
* CALLS:
*        void
*
* CALLED BY:  void ProcessFault(void)                   
* 
**********************************************************************/
void FanCntl(void)
{
	static Uint8 i8Cnt_Count = 0;
	static Uint8 i8Cnt_Hilimit = 0;

	if(i8Cnt_Count<=9)
		i8Cnt_Count++;
	else
		i8Cnt_Count=1;

	if (Calc_Result.i32TempAmb > 50)
		i8Cnt_Hilimit = 10;
	else if(Calc_Result.i32TempAmb > 45)
		i8Cnt_Hilimit = 9;
	else if(Calc_Result.i32TempAmb > 40)
		i8Cnt_Hilimit = 8;
	else if(Calc_Result.i32TempAmb > 35)
		i8Cnt_Hilimit = 7;
	else if(Calc_Result.i32TempAmb > 30)
		i8Cnt_Hilimit = 6;
	else
		i8Cnt_Hilimit = 5;

	if(i8Cnt_Hilimit >= i8Cnt_Count)
	{
		DC_Fan1_Enable;
		DC_Fan2_Enable;
		DcFanSpeedSense();
	}
	else
		DC_Fan1_Disable;
		DC_Fan2_Disable;
}


//--- end of file -----------------------------------------------------

