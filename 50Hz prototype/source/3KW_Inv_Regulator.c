/*=============================================================================*
 *         Copyright(c) 
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_Inv_Regulator.c
 *   
 *  PURPOSE  : SPWM algorithm process for control loop and 28335 specific board
 *				     for 3KW Parallel Inverter.
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
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


#include "DSP2833x_Device.h"				// Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file


/* Configure the DATA/CODE section running in RAM from flash */
#pragma CODE_SECTION(Inv_VoltControl, "ControlLoopInRAM")
#pragma CODE_SECTION(OutPLLcontroller, "ControlLoopInRAM")

#pragma CODE_SECTION(InvParallel_Control, "ControlLoopInRAM")
#pragma CODE_SECTION(Get_ADC_Result2, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccInvCalc, "ControlLoopInRAM")
//#pragma CODE_SECTION(ADC_INT_INV_Control, "ControlLoopInRAM")

/*=============================================================================*
 * 	Local Variables declaration
 *============================================================================*/
struct	PLLCONTREG	OutPLLConReg, VOutPLLConReg;
struct	PLL 					VOutPLLReg;
struct  INVVOLTCONTREG  InvVoltConReg;
struct  INVCURRCONTREG  InvCurrConReg;
struct  INVRESTARTCHECKREG InvRestartCheckReg;
/*=============================================================================*
 * 	Local functions declaration
 *============================================================================*/
/* Phase Lock Loop controller */
void OutPLLcontroller(void);
void VoutPhaseCalc(void);

/* PID controller for 220V-Inverter control */
void Inv_VoltControl(void);

/* Inverter Voltage Reference Slow Up*/
void InvVoltSlowup(void);
		
/* Initialize the starting Duty and loop PID parameters*/
void InverterStage_Init(void);

void ADC_INT_INV_Control(void);

/* Shortcut protection restart*/
void InvRestartCheck(void);

/*=============================================================================*
 * FUNCTION:	void ADC_INT_INV_Control(void)
 *
 * PURPOSE:	One of Interruption service program. Contain the inverter control logic.
 *
 * CALLED BY:	void SEQ1INT_ISR(void)
 *============================================================================*/
void ADC_INT_INV_Control(void)
{
	Get_ADC_Result2();
  	OutPLLcontroller();
  	VoutPhaseCalc();

	#ifdef INV_CLOSE_LOOP
  	if((NormalState==g_Sys_Current_State) && (1 == g_ParaLogic_State.bit.InvSoftStart_EN))
  	{
  		InvParallel_Control();		// Johnny 0426
  	  	Inv_VoltControl();
  	}
	#endif

	#ifdef INV_OPEN_LOOP
  	InvPWMOutputsEnable();
  	Inv_VoltControl();
	#endif

  	ADAccInvCalc();
  	Scib_SnatchGraph();

  	/*
  	 * Software instantaneous current protection, which is used in tuning and abandoned in usual
  	 */
  	/* if( NormalState == g_Sys_Current_State)
  	{
  		if((abs(GetRealValue.f32IInv) > 45))
  		{
  			InvPWMOutputsDisable();
  			RelaysOFF();
  			g_SysFaultMessage.bit.unrecoverSW_OCP_Inv = 1;
  		}
  	}
  	*/
} // end of SVPWM_Model

/*=============================================================================*
 * FUNCTION:	void InvH_VoltControl(void)
 *
 * PURPOSE:	INV(220V) voltage PR controller
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void Inv_VoltControl(void)
{ // start of Inv_VoltControl
	/*
	 * PR controller in s domain: Gpr =
	 * 				    kr * s
	 *  kp + --------------
	 *  		  s^2 + w0^2
	 */
	static float32 kp = 0.01f;//0.1;
	static float32 kr = 800 * 0.00005f;//800;
	float32 Coff_a2 = 0.0002467401f;  // (w0 * Ts) * (w0 * Ts)
	float32 Coff_a3 = 0.2499846f;     // 1/ (4+w0*w0*Ts*Ts)

	InvVoltConReg.f32VoltInst_Ref = (InvVoltConReg.f32VoltRms_Ref) \
			* 1.414f * OutPLLConReg.Sin_Theta * InvVoltConReg.f32VoltGain;
	InvVoltConReg.f32VoltInst_Fdb = GetRealValue.f32VInv;

	InvVoltConReg.f32VoltInst_ErrOld = InvVoltConReg.f32VoltInst_ErrNew;
	InvVoltConReg.f32VoltInst_ErrNew = InvVoltConReg.f32VoltInst_Ref - InvVoltConReg.f32VoltInst_Fdb ;//WF 2018.07.26  + 5.0f * Calc_Result.f32VDutyD_ave

  	 if(InvVoltConReg.f32VoltInst_ErrNew  > 10)
  		InvVoltConReg.f32VoltInst_ErrNew = 10;
	else if(InvVoltConReg.f32VoltInst_ErrNew  < -10)
		InvVoltConReg.f32VoltInst_ErrNew  = -10;


  	InvVoltConReg.Input[2] = InvVoltConReg.Input[1];																					// x(k-2) = x(k-1)
  	InvVoltConReg.Input[1] = InvVoltConReg.Input[0];																					// x(k-1) = x(k)
  	InvVoltConReg.Input[0] = InvVoltConReg.f32VoltInst_ErrNew;											// x(k)

  	InvVoltConReg.Output[2] = InvVoltConReg.Output[1];																				// y(k-2) = y(k-1)
  	InvVoltConReg.Output[1] = InvVoltConReg.Output[0];																				// y(k-1) = y(k)

  	InvVoltConReg.MAC = ((2*kr+4*kp+Coff_a2*kp) * Coff_a3) * InvVoltConReg.Input[0];                                                  // + a1 * x(k)
  	InvVoltConReg.MAC -=((8-2*Coff_a2) *kp * Coff_a3)* InvVoltConReg.Input[1];
  	InvVoltConReg.MAC += ((4*kp+Coff_a2*kp-2*kr)* Coff_a3) * InvVoltConReg.Input[2];											// + a3 * x(k-2)

  	InvVoltConReg.MAC += ((8-2*Coff_a2)* Coff_a3) * InvVoltConReg.Output[1];															// + b1 * y(k-1)
  	InvVoltConReg.MAC -= InvVoltConReg.Output[2];															// - b2 * y(k-2)

  	InvVoltConReg.Output[0] = InvVoltConReg.MAC;
  	InvVoltConReg.f32VoltInst_ErrOut = InvVoltConReg.Output[0];
  	InvCurrConReg.f32InvDuty =  InvVoltConReg.f32VoltInst_ErrOut;

  	//InvCurrConReg.f32InvDuty = Calc_Result.Coff_Dforward2 * InvVoltConReg.f32VoltInst_ErrOut; //2017.11.12 GX
  	//InvCurrConReg.f32InvDuty = 0.76f * PWM_HALF_PERIOD * OutPLLConReg.Sin_Theta;//2017.6.29 GX inverter test

  	if (InvCurrConReg.f32InvDuty >=  InvVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD)   			//
  		InvCurrConReg.f32InvDuty = InvVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
  	else if (InvCurrConReg.f32InvDuty <= -InvVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD )  		//
  	  	InvCurrConReg.f32InvDuty = -InvVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;

 	 //open loop just for test
	#ifdef INV_OPEN_LOOP
 	InvCurrConReg.f32InvDuty = 0.76f * PWM_HALF_PERIOD * OutPLLConReg.Sin_Theta;
	#endif

  	EPwm3Regs.CMPA.half.CMPA = (Uint16)(-InvCurrConReg.f32InvDuty);		// RH	High frequency switching with Low level in half cycle
  	EPwm3Regs.CMPB = (Uint16)(-InvCurrConReg.f32InvDuty);					// RL 	High frequency switching with High level in half cycle
  	EPwm4Regs.CMPA.half.CMPA = (Uint16)(InvCurrConReg.f32InvDuty);			// LH
  	EPwm4Regs.CMPB = (Uint16)(InvCurrConReg.f32InvDuty);

 } // end of InvH_VoltControl

/*=============================================================================*
 * FUNCTION:	 void OutPLLcontroller(void)
 *
 * PURPOSE:	Inverter output phase lock control
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
 void OutPLLcontroller(void)
{ 
	 // start of OutPLLcontroller()
	/*
	 * Actually it is a frequency produce function.
	 * 'OutPLLConReg.f32Theta_Step' equals 50Hz * 2 * pi / 20000Hz,
	 * which means angle step need to be accumulated in each ADC interruption.
	 */
	OutPLLConReg.f32Theta +=  OutPLLConReg.f32Theta_Step;

	/*
	* If synchronization is completed, the native phase signal need to be send to sync-bus
	* through the following code.
	*/
	if(1 == g_ParaLogic_State.bit.SelfPhaseOut_EN && g_Sys_Current_State != PermanentState && g_Sys_Current_State != FaultState)
	{
		if(OutPLLConReg.f32Theta >= Value_Pi)
			SYNC_COM1_ON;
		if(OutPLLConReg.f32Theta > Value_2Pi)
			SYNC_COM1_OFF;
	}
	else
	{
		SYNC_COM1_ON;
	}

	if ( OutPLLConReg.f32Theta > Value_2Pi )
	{
		OutPLLConReg.f32Theta = OutPLLConReg.f32Theta - Value_2Pi;
		g_StateCheck.bit.Inv_Zero_Crossing_Flag = 1;
	}

	/*
	 * INV(220V) uses 'OutPLLConReg.Sin_Theta'.
	 */
	sincos(OutPLLConReg.f32Theta, &(OutPLLConReg.Sin_Theta), &(OutPLLConReg.Cos_Theta));
}

 /*=============================================================================*
   * FUNCTION:	void VoutHPhaseCalc(void)
   *
   * PURPOSE:		Load voltage(220V) phase lock controller
   *
   * CALLED BY:	void ADC_INT_INV_Control(void)
   *============================================================================*/
void VoutPhaseCalc(void)
{
	static int i = 0;

 	float32 Vout_PLL_Kp = 5e-5f;
 	float32 Vout_PLL_Ki = 2.5e-8f;
 	float32 LPF_B0 = 0.00034605f;//0.00003913
 	float32 LPF_B1 = 0.00069210f;//0.00007826
 	float32 LPF_B2 = 0.00034605f;//0.00003913
 	float32 LPF_A1 = 1.94721182f;//1.9822318
 	float32 LPF_A2 = 0.94859602f;//0.98238832
 	float32 DELTA_ANGLE = 15.7e-3f;//2*pi*50/20000

 	VOutPLLConReg.f32Valpha = GetRealValue.f32VOut;
 	i++;

 	//The following is the different equation of a second order low pass filter
 	if (4 == i)
 	{
 		VOutPLLReg.Input[2] = VOutPLLReg.Input[1];																		// x(k-2) = x(k-1)
 	 	VOutPLLReg.Input[1] = VOutPLLReg.Input[0];																		// x(k-1) = x(k)
 	 	VOutPLLReg.Input[0] = VOutPLLConReg.f32Valpha * VOutPLLConReg.Cos_Theta;		// x(k)

  		VOutPLLReg.Output[2] = VOutPLLReg.Output[1];																// y(k-2) = y(k-1)
  		VOutPLLReg.Output[1] = VOutPLLReg.Output[0];																// y(k-1) = y(k)

  		VOutPLLReg.MAC = LPF_B0 * VOutPLLReg.Input[0];														// + b0 * x(k)
  		VOutPLLReg.MAC += LPF_B1 * VOutPLLReg.Input[1];														// + b1 * x(k-1)
  		VOutPLLReg.MAC += LPF_B2 * VOutPLLReg.Input[2];														// + b2 * x(k-2)

  		VOutPLLReg.MAC += LPF_A1 * VOutPLLReg.Output[1];													// + a11 * y(k-1)
  		VOutPLLReg.MAC -= LPF_A2 * VOutPLLReg.Output[2];													// - a2 * y(k-2)

  		VOutPLLReg.Output[0] = VOutPLLReg.MAC;
  		i = 0;
  	}

 	//PI regulator can achieve zero static error of phase
  	VOutPLLReg.f32PIDErr_Old = VOutPLLReg.f32PIDErr_New ;
  	VOutPLLReg.f32PIDErr_New  = VOutPLLReg.Output[0] - VOutPLLReg.f32Refer;				// error(n) = y(k) - 0

  	VOutPLLReg.f32PID_Output = VOutPLLReg.f32PID_Output \
  						+ (Vout_PLL_Kp + Vout_PLL_Ki) * VOutPLLReg.f32PIDErr_New \
  						- Vout_PLL_Kp * VOutPLLReg.f32PIDErr_Old;           // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

  	/*
  	* 'VOutHPLLReg.f32Delta_Theta' is the angle step which needs to be accumulated in each ADC interruption.
  	* 'DELTA_ANGLE_H' is the bias, which can acceleration regulating process
  	*/
  	VOutPLLReg.f32Delta_Theta =  DELTA_ANGLE + VOutPLLReg.f32PID_Output;

  	if(VOutPLLReg.f32Delta_Theta > InvTheta_Step_Hi_Limit)
  		VOutPLLReg.f32Delta_Theta = InvTheta_Step_Hi_Limit;
  	if(VOutPLLReg.f32Delta_Theta < InvTheta_Step_Low_Limit)
  		VOutPLLReg.f32Delta_Theta = InvTheta_Step_Low_Limit;

  	//'VOutHPLLConReg.f32Theta_Step' is used to calculate the frequency of the 220V load voltage
  	VOutPLLConReg.f32Theta_Step = VOutPLLReg.f32Delta_Theta ;
  	VOutPLLReg.f32Theta += VOutPLLReg.f32Delta_Theta;

  	if ( VOutPLLReg.f32Theta > Value_2Pi )
  		VOutPLLReg.f32Theta = VOutPLLReg.f32Theta - Value_2Pi;

  	sincos(VOutPLLReg.f32Theta, &(VOutPLLConReg.Sin_Theta), &(VOutPLLConReg.Cos_Theta));
}
 /*=============================================================================*
   * FUNCTION:	 void InvVoltSlowup(void)
   *
   * PURPOSE:		Inverter slow up function
   *
   * CALLED BY:	void TSK_InvVoltPeriod(void)
   *============================================================================*/
 //
void InvVoltSlowup(void)
{
	if((NormalState==g_Sys_Current_State) && (1 == g_ParaLogic_State.bit.InvSoftStart_EN))
	{
		//InvPWMOutputsEnable();//WF 2018.05.30
		if (g_StateCheck.bit.InvDrvEnable == 0)
		{
			g_StateCheck.bit.InvDrvEnable = 1;
			InvPWMOutputsEnable();
		}
		if(Calc_Result.f32VGrid_rms > 100)
		{
			if(InvVoltConReg.f32VoltGain < 1)// 220V  SlowUp
		 	{
				InvVoltConReg.f32VoltGain = InvVoltConReg.f32VoltGain + 0.01f;
				InvVoltConReg.f32VoltDutyUpLimit = InvVoltConReg.f32VoltDutyUpLimit + 0.01f;
		 	}
			else
			{
				InvVoltConReg.f32VoltGain = 1;
			   //InvVoltConReg.f32VoltDutyLowLimit = 0.025f; //Why?
				InvVoltConReg.f32VoltDutyUpLimit = 0.9f;
			}
		}
		else
			InvVoltConReg.f32VoltDutyUpLimit = 0.95f;
	}

	if(1 == InvVoltConReg.f32VoltGain)
	{
		g_StateCheck.bit.Inv_SoftStart = 1;
	 	//SYNC_COM2_ON;//Why
	}
	else
		g_StateCheck.bit.Inv_SoftStart = 0;

	if (g_StateCheck.bit.Inv_SoftStart == 1 && ShortCheck_Reg.Restart_times == 0)
	{
		if (InvVoltConReg.f32VoltRms_Ref >= SafetyReg.f32Inv_VoltRms_Ref)
		{
			InvVoltConReg.f32VoltRms_Ref -= 0.6;
			if (InvVoltConReg.f32VoltRms_Ref <= SafetyReg.f32Inv_VoltRms_Ref)
				InvVoltConReg.f32VoltRms_Ref = SafetyReg.f32Inv_VoltRms_Ref;
		}
		else if (InvVoltConReg.f32VoltRms_Ref <= SafetyReg.f32Inv_VoltRms_Ref)
		{
			InvVoltConReg.f32VoltRms_Ref += 0.6;
			if (InvVoltConReg.f32VoltRms_Ref >= SafetyReg.f32Inv_VoltRms_Ref)
				InvVoltConReg.f32VoltRms_Ref = SafetyReg.f32Inv_VoltRms_Ref;
		}
	}
}

/*=============================================================================*
  * FUNCTION:	void InverterStage_Init(void)
  *
  * PURPOSE:		Initialize the variable related to  inverter
  *
  * CALLED BY:	void ProcessChecking(void)
  *============================================================================*/
void InverterStage_Init(void)
{ // start of InverterStage_Init

	InvVoltConReg.f32VoltRms_ErrOld = 0;
	InvVoltConReg.f32VoltRms_ErrNew = 0;
	InvVoltConReg.f32VoltRms_Out = 0;
	InvVoltConReg.f32VoltRms_Ref_Delta = 3;

	InvVoltConReg.f32VoltInst_ErrOld = 0;
	InvVoltConReg.f32VoltInst_ErrNew = 0;
	InvVoltConReg.f32VoltInst_ErrOut = 0;

	InvCurrConReg.f32CurrInst_ErrOld = 0;
	InvCurrConReg.f32CurrInst_ErrNew = 0;

	InvCurrConReg.f32InvDuty_Con = 0;
	InvVoltConReg.f32VoltGain = 0;

	InvVoltConReg.f32VoltDutyUpLimit = 0.05f;
	InvVoltConReg.f32VoltDutyLowLimit = 0.01f;

	g_StateCheck.bit.InvDrvEnable = 0;
	g_StateCheck.bit.Inv_SoftStart = 0;
	g_ParaLogic_State.bit.InvSoftStart_EN = 0;

	Parallel_Reg.u16Cnt_SCR_ON = 0;
	InvVoltConReg.f32VoltRms_Ref = SafetyReg.f32Inv_VoltRms_Ref;

} // end of InverterStage_Init

/*=============================================================================*
  * FUNCTION:	void InverterStage_Init(void)
  *
  * PURPOSE:	The module need be restarted to produce a low output voltage after the shortcut protection,
  * 					which is used to test whether it is still shorted. If the system is back to normal, the module
  * 					will restart again with a normal output voltage.
  *
  * CALLED BY:	void TSK_InvVoltPeriod(void)
  *============================================================================*/
void InvRestartCheck(void)
{
	if((g_StateCheck.bit.Inv_SoftStart == 1) && (NormalState==g_Sys_Current_State))
	{
		if(ShortCheck_Reg.Restart_times >=1)
		{
			if(InvRestartCheckReg.temp1 <500)
				InvRestartCheckReg.temp1++;
			else
			{
				InvRestartCheckReg.temp1=0;
				ShortCheck_Reg.Restart_times=0;
				g_Sys_Current_State = FaultState;
				SafetyReg.f32Inv_VoltRms_Ref = Inv_VoltRef;
			}
		}
	}
	else
		InvRestartCheckReg.temp1 = 0;
}
//--- end of file -----------------------------------------------------

