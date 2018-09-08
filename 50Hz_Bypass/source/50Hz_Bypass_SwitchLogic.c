/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_Regulator.c 
 *   
 *  PURPOSE  : SPWM algorithm process for control loop and 28335 specific board
 *				     for 5KW PV Inverter.
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-19      V0.1           Ken      	    Created
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *          
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *    
 *============================================================================*/


#include "DSP2803x_Device.h"			// Peripheral address definitions
#include "F28035_example.h"				// Main include file

/* Configure the DATA/CODE section running in RAM from flash */
//#pragma CODE_SECTION(CurrentPIDcontroller, "ControlLoopInRAM")

/*=============================================================================*
 * 	Local Variables declaration
 *============================================================================*/

/*=============================================================================*
 * 	Local functions declaration
 *============================================================================*/
struct PLL GridPLLReg;

void ADC_INT_Control(void);
void Switch_Logic_Control(void);
void HRCAP2_By_to_Inv(void);
void GridPLLControl(void);
Uint8 FaultCheck(void);

//#pragma CODE_SECTION(ADC_INT_Control, "ControlLoopInRAM")

void ADC_INT_Control(void)
{
	// start of SVPWM_Model
	Get_ADC_Result();
	ADAccCalc();
	GridPLLControl();

	g_StateCheck.bit.COM4 = COM4_LEVEL;
	g_StateCheck.bit.COM2 = COM2_LEVEL;
	g_StateCheck.bit.Fault = FaultCheck();

	Switch_Logic_Control();
} // end

void Switch_Logic_Control(void)
{
	if (g_StateCheck.Word.byte1 < 0x04 && g_Sys_State > 2)
		g_Sys_State = CheckState;
	else
	{
		if (g_StateCheck.bit.Module_Lock == 1)
			g_Sys_State = LockState;
		else if (g_StateCheck.bit.Force_Switch == 1)
			g_Sys_State = ForceNormalState;
		else if (g_StateCheck.bit.Fault == 1)
			g_Sys_State = FaultState;
		else if (g_StateCheck.bit.COM2 == 1)
			g_Sys_State = Standby;
		else if (g_StateCheck.bit.COM4 == 1)
			g_Sys_State = NormalState;
		else
			g_Sys_State = g_Sys_State;
	}
}

void GridPLLControl(void)
{
	static Uint8 s_u8temp = 0;

	_iq20 GridPLL_Kp = 5e-5f;
	_iq20 GridPLL_Ki = 2.5e-8f;
	_iq20 LPF_B0 = 0.00034605;
	_iq20 LPF_B1 = 0.00069210;
	_iq20 LPF_B2 = 0.00034605;
	_iq20 LPF_A1 = 1.94721182;
	_iq20 LPF_A2 = 0.94859602;
	_iq20 DELTA_ANGLE = GridTheta_StepRated;//2*pi*50/20000

	GridPLLReg.iq20Valpha = GetRealValue.iq20VGrid;
	s_u8temp++;

	if (4 == s_u8temp)
	{
		 GridPLLReg.iq20Input[2] = GridPLLReg.iq20Input[1];																					// x(k-2) = x(k-1)
		 GridPLLReg.iq20Input[1] = GridPLLReg.iq20Input[0];																					// x(k-1) = x(k)
		 GridPLLReg.iq20Input[0] = _IQmpy(GridPLLReg.iq20Valpha, GridPLLReg.iq20Cos_Theta);						// x(k)

		 GridPLLReg.iq20Output[2] = GridPLLReg.iq20Output[1];																			// y(k-2) = y(k-1)
		 GridPLLReg.iq20Output[1] = GridPLLReg.iq20Output[0];																			// y(k-1) = y(k)

		 GridPLLReg.iq20MAC =  _IQmpy(LPF_B0, GridPLLReg.iq20Input[0]);															// + b0 * x(k)
		 GridPLLReg.iq20MAC +=  _IQmpy(LPF_B1, GridPLLReg.iq20Input[1]);														// + b1 * x(k-1)
		 GridPLLReg.iq20MAC +=  _IQmpy(LPF_B2, GridPLLReg.iq20Input[2]);														// + b2 * x(k-2)

		 GridPLLReg.iq20MAC +=  _IQmpy(LPF_A1, GridPLLReg.iq20Output[1]);													// + a11 * y(k-1)
		 GridPLLReg.iq20MAC -=  _IQmpy(LPF_A2, GridPLLReg.iq20Output[2]);													// - a2 * y(k-2)

		 GridPLLReg.iq20Output[0] = GridPLLReg.iq20MAC;
		 s_u8temp= 0;
	}

	GridPLLReg.iq20PIDErr_Old = GridPLLReg.iq20PIDErr_New ;
	GridPLLReg.iq20PIDErr_New  = GridPLLReg.iq20Output[0] - GridPLLReg.iq20Refer;										// error(n) = y(k) - 0

	GridPLLReg.iq20PID_Output = GridPLLReg.iq20PID_Output \
		 						+  _IQmpy((GridPLL_Kp + GridPLL_Ki), GridPLLReg.iq20PIDErr_New) \
		 						-  _IQmpy(GridPLL_Kp, GridPLLReg.iq20PIDErr_Old);                // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

	GridPLLReg.iq20Delta_Theta =  DELTA_ANGLE + GridPLLReg.iq20PID_Output;

	if(GridPLLReg.iq20Delta_Theta > GridTheta_Step_Hi_Limit)
	{
		 GridPLLReg.iq20Delta_Theta = GridTheta_Step_Hi_Limit;
	}
	if(GridPLLReg.iq20Delta_Theta < GridTheta_Step_Low_Limit)
	{
		 GridPLLReg.iq20Delta_Theta = GridTheta_Step_Low_Limit;
	}

	GridPLLReg.iq20Theta += GridPLLReg.iq20Delta_Theta;
	if ( GridPLLReg.iq20Theta > Value_2Pi )
	{
		 GridPLLReg.iq20Theta = GridPLLReg.iq20Theta - Value_2Pi;
		 g_StateCheck.bit.Zero_Crossing_Flag = 1;
	}
	GridPLLReg.iq20Sin_Theta = _IQsin(GridPLLReg.iq20Theta);
	GridPLLReg.iq20Cos_Theta = _IQcos(GridPLLReg.iq20Theta);

	if(1 == g_StateCheck.bit.SelfPhaseOut_EN)
	{
		if(GridPLLReg.iq20Theta >= Value_Pi)
			SYNC_COM1_ON;
		if(GridPLLReg.iq20Theta < Value_Pi)
			SYNC_COM1_OFF;
	}
}

Uint8 FaultCheck(void)
{
	if ((0 == g_SysFaultMessage.Word.byte0 ) && (0 == g_SysFaultMessage.Word.byte1))
		return(0);
	else
		return(1);
}
