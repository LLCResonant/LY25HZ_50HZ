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
	g_StateCheck.bit.Module_Lock =MODULE_LOCK;

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
	_iq20 ThetaStep_Ref = GridTheta_StepRated;
	static _iq20 Theta_Ref = Value_2Pi * 0.25f;  //2017.3.29 GX
	static int i8temp = 0;

	_iq20 GridPLL_Kp = 5e-5f; //2017.8.14 GX //5e-6//2017.3.17 GX Kp1/40000/VGrid
	_iq20 GridPLL_Ki = 2.5e-8f; //2017.8.14 GX//3e-9 2017.5.10 GX//2017.3.17 GX KI1/40000/40000/VGrid
	_iq20 LPF_B0 = 0.00034605;//0.000346050740714038155762;
	_iq20 LPF_B1 = 0.00069210;//0.000692101481428076311525;
	_iq20 LPF_B2 = 0.00034605;//0.000346050740714038155762;
	_iq20 LPF_A1 = 1.94721182;//1.947211823488243620516869;
	_iq20 LPF_A2 = 0.94859602;//0.948596026451099749721152;
	_iq20 DELTA_ANGLE = GridTheta_StepRated;//2*pi*50/20000

	/*------------------- dqPLL PID regulator process ----------------------------*/
	Theta_Ref += ThetaStep_Ref;
	if(Theta_Ref > Value_2Pi)
	{
		 Theta_Ref = Theta_Ref - Value_2Pi;
	}
	//GridPLLConReg.f32Valpha = sin(Theta_Ref) * 100;

	GridPLLReg.i32Valpha = GetRealValue.i32VGrid;
	i8temp++;

	if (4 == i8temp)
	{
		 GridPLLReg.i32Input[2] = GridPLLReg.i32Input[1];																					// x(k-2) = x(k-1)
		 GridPLLReg.i32Input[1] = GridPLLReg.i32Input[0];																					// x(k-1) = x(k)
		 GridPLLReg.i32Input[0] = _IQmpy(GridPLLReg.i32Valpha, GridPLLReg.i32Cos_Theta);						// x(k)

		 GridPLLReg.i32Output[2] = GridPLLReg.i32Output[1];																			// y(k-2) = y(k-1)
		 GridPLLReg.i32Output[1] = GridPLLReg.i32Output[0];																			// y(k-1) = y(k)

		 //GridPLLReg.i32MAC -= ((S32)pllLPF.Output[0] << 16);																		// i32MAC Low 16bits

		 GridPLLReg.i32MAC =  _IQmpy(LPF_B0, GridPLLReg.i32Input[0]);															// + b0 * x(k)
		 GridPLLReg.i32MAC +=  _IQmpy(LPF_B1, GridPLLReg.i32Input[1]);														// + b1 * x(k-1)
		 GridPLLReg.i32MAC +=  _IQmpy(LPF_B2, GridPLLReg.i32Input[2]);														// + b2 * x(k-2)

		 GridPLLReg.i32MAC +=  _IQmpy(LPF_A1, GridPLLReg.i32Output[1]);													// + a11 * y(k-1)
		 GridPLLReg.i32MAC -=  _IQmpy(LPF_A2, GridPLLReg.i32Output[2]);													// - a2 * y(k-2)

		 GridPLLReg.i32Output[0] = GridPLLReg.i32MAC;
		 i8temp= 0;
	}

	GridPLLReg.i32PIDErr_Old = GridPLLReg.i32PIDErr_New ;
	GridPLLReg.i32PIDErr_New  = GridPLLReg.i32Output[0] - GridPLLReg.i32Refer;										// error(n) = y(k) - 0

	GridPLLReg.i32PID_Output = GridPLLReg.i32PID_Output \
		 						+  _IQmpy((GridPLL_Kp + GridPLL_Ki), GridPLLReg.i32PIDErr_New) \
		 						-  _IQmpy(GridPLL_Kp, GridPLLReg.i32PIDErr_Old);                // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

	GridPLLReg.i32Delta_Theta =  DELTA_ANGLE + GridPLLReg.i32PID_Output;

	if(GridPLLReg.i32Delta_Theta > GridTheta_Step_Hi_Limit)
	{
		 GridPLLReg.i32Delta_Theta = GridTheta_Step_Hi_Limit;
	}
	if(GridPLLReg.i32Delta_Theta < GridTheta_Step_Low_Limit)
	{
		 GridPLLReg.i32Delta_Theta = GridTheta_Step_Low_Limit;
	}

	GridPLLReg.i32Theta += GridPLLReg.i32Delta_Theta;
	if ( GridPLLReg.i32Theta > Value_2Pi )
	{
		 GridPLLReg.i32Theta = GridPLLReg.i32Theta - Value_2Pi;
		 g_StateCheck.bit.Zero_Crossing_Flag = 1;
	}
	GridPLLReg.i32Sin_Theta = _IQsin(GridPLLReg.i32Theta);
	GridPLLReg.i32Cos_Theta = _IQcos(GridPLLReg.i32Theta);

	if(1 == g_StateCheck.bit.SelfPhaseOut_EN)
	{
		if(GridPLLReg.i32Theta >= Value_Pi)
			SYNC_COM1_ON;
		if(GridPLLReg.i32Theta < Value_Pi)
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
