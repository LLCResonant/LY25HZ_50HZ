/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME: 3KW_Inv_Regulator.c
 *
 *  PURPOSE :	Data acquisition and protection file of the module.
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					Li Zhang
 *    												Xun Gao
 *    												Jianxi Zhu
 *============================================================================*/

#include "DSP2833x_Device.h"				// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file

/* Configure the DATA/CODE section running in RAM from flash */
#pragma CODE_SECTION(InvH_VoltControl, "ControlLoopInRAM")
#pragma CODE_SECTION(InvL_VoltControl, "ControlLoopInRAM")
#pragma CODE_SECTION(OutPLLcontroller, "ControlLoopInRAM")

#pragma CODE_SECTION(InvParallel_Control, "ControlLoopInRAM")
#pragma CODE_SECTION(Get_ADC_Result2, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccInvCalc, "ControlLoopInRAM")
//#pragma CODE_SECTION(ADC_INT_INV_Control, "ControlLoopInRAM")

/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
struct	PLLCONTREG							OutPLLConReg, VOutLPLLConReg, VOutHPLLConReg;
struct	PLL 											VOutLPLLReg, VOutHPLLReg;
struct  INVVOLTCONTREG  				InvHVoltConReg, InvLVoltConReg;
struct  INVCURRCONTREG  				InvHCurrConReg, InvLCurrConReg;

/*=============================================================================*
 * 	functions declaration
 *============================================================================*/
/* Phase Lock Loop controller */
void OutPLLcontroller(void);
void VoutLPhaseCalc(void);
void VoutHPhaseCalc(void);

/* PID controller for 220V-Inverter control */
void InvH_VoltControl(void);

/* PID controller for 110V-Inverter control*/
void InvL_VoltControl(void);

/* Inverter Voltage Reference Slow Up */
void InvVoltSlowup(void);
		
/* Initialize the starting Duty and loop PID parameters */
void InverterStage_Init(void);

void ADC_INT_INV_Control(void);

/* Shortcut protection restart  */
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
	// start of ADC_INT_INV_Control()

  	Get_ADC_Result2();
  	OutPLLcontroller();
  	VoutHPhaseCalc();
  	VoutLPhaseCalc();

	#ifdef INV_CLOSE_LOOP
  	if((NormalState==g_Sys_Current_State) && (1 == g_ParaLogic_State.bit.InvSoftStart_EN))
  	{
  		InvParallel_Control();
  	  	InvH_VoltControl();
  	  	InvL_VoltControl();
  	}
	#endif

	#ifdef INV_OPEN_LOOP
	InvH_VoltControl();
	InvL_VoltControl();
	#endif

  	ADAccInvCalc();
  	Scib_SnatchGraph();

  	/*
  	 * Software instantaneous current protection, which is used in tuning and abandoned in usual
  	 */
  	/*if( NormalState == g_Sys_Current_State )
  	{
  		if ( (abs(GetRealValue.f32IInvH) > 10) )
  		{
  			InvPWMOutputsDisable();
  			RelaysOFF();
  			g_SysFaultMessage.bit.unrecoverSW_OCP_InvH = 1;  //2017.4.20 GX
  		}
  		if ( (abs (GetRealValue.f32IInvL) > 13) )
  		{
  			InvPWMOutputsDisable();
  			RelaysOFF();
  			g_SysFaultMessage.bit.unrecoverSW_OCP_InvL = 1;  //2017.4.20 GX
   		}
  	}*/

} // end of ADC_INT_INV_Control()

/*=============================================================================*
 * FUNCTION:	void InvH_VoltControl(void)
 *
 * PURPOSE:	INVH(220V) voltage PR controller
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void InvH_VoltControl(void)
{
	// start of InvH_VoltControl

	/*
	 * PR controller in s domain: Gpr =
	 * 				    kr * s
	 *  kp + --------------
	 *  		  s^2 + w0^2
	 */
	static float32 kp = 0.6f;                        //0.8f;//2016.6.25 GX
	static float32 kr = 400* 0.00005f;        //800 * 0.00005f;//2016.6.25 GX
	float32 Coff_a2 = 0.00006168503f;  	// (w0 * Ts) * (w0 * Ts), where w0 is 2 * pi * 25Hz; Ts is the sample period 1 / 20000Hz
	float32 Coff_a3 = 0.2499961f;     		// 1 / (4 + w0 * w0 * Ts * Ts)

	/*
	 *	Change INVH(220V) PR parameter according to output current amplitude.
	 */
	/*if(Calc_Result.f32IInvH_rms_instant <= 0.9f)
	{
		kp = 0.8f;
		kr = 800* 0.00005f;
	}
	else if(Calc_Result.f32IInvH_rms_instant > 0.9f && Calc_Result.f32IInvH_rms_instant <= 1.3f)
	{
		kp=kp;
		kr=kr;
	}
	else if(Calc_Result.f32IInvH_rms_instant > 1.3f && Calc_Result.f32IInvH_rms_instant <= 3.85f)
	{
		kp = 0.6f;
		kr = 400* 0.00005f;
	}
	else if(Calc_Result.f32IInvH_rms_instant > 3.85f && Calc_Result.f32IInvH_rms_instant <= 4.15f)
	{
		kp=kp;
		kr=kr;
	}
	else if(Calc_Result.f32IInvH_rms_instant > 4.15f)
	{
		kp = 0.8f;
		kr = 800* 0.00005f;
	}*/

	InvHVoltConReg.f32VoltInst_Ref = InvHVoltConReg.f32VoltRms_Ref * 1.414f * OutPLLConReg.Sin_Theta * InvHVoltConReg.f32VoltGain;
	InvHVoltConReg.f32VoltInst_Fdb = GetRealValue.f32VInvH;

	InvHVoltConReg.f32VoltInst_ErrOld = InvHVoltConReg.f32VoltInst_ErrNew;
	InvHVoltConReg.f32VoltInst_ErrNew = InvHVoltConReg.f32VoltInst_Ref - InvHVoltConReg.f32VoltInst_Fdb;

  	 if(InvHVoltConReg.f32VoltInst_ErrNew  > 10)
  		 InvHVoltConReg.f32VoltInst_ErrNew = 10;
  	 else if(InvHVoltConReg.f32VoltInst_ErrNew  < -10)
  		 InvHVoltConReg.f32VoltInst_ErrNew  = -10;

  	 /*
  	  * The following is the PR controller difference-equation
  	  */
  	 InvHVoltConReg.Input[2] = InvHVoltConReg.Input[1];																					// x(k-2) = x(k-1)
  	 InvHVoltConReg.Input[1] = InvHVoltConReg.Input[0];																					// x(k-1) = x(k)
  	 InvHVoltConReg.Input[0] = InvHVoltConReg.f32VoltInst_ErrNew;																// x(k)

  	 InvHVoltConReg.Output[2] = InvHVoltConReg.Output[1];																			// y(k-2) = y(k-1)
  	 InvHVoltConReg.Output[1] = InvHVoltConReg.Output[0];																			// y(k-1) = y(k)

  	 InvHVoltConReg.MAC = ((2*kr+4*kp+Coff_a2*kp) * Coff_a3) * InvHVoltConReg.Input[0];                         // + a1 * x(k)
  	 InvHVoltConReg.MAC -=((8-2*Coff_a2) *kp * Coff_a3)* InvHVoltConReg.Input[1];
  	 InvHVoltConReg.MAC += ((4*kp+Coff_a2*kp-2*kr)* Coff_a3) * InvHVoltConReg.Input[2];							// + a3 * x(k-2)

  	 InvHVoltConReg.MAC += ((8-2*Coff_a2)* Coff_a3) * InvHVoltConReg.Output[1];										// + b1 * y(k-1)
  	 InvHVoltConReg.MAC -= InvHVoltConReg.Output[2];																					// - b2 * y(k-2)

  	 InvHVoltConReg.Output[0] = InvHVoltConReg.MAC;
  	 InvHVoltConReg.f32VoltInst_ErrOut = InvHVoltConReg.Output[0];
  	 InvHCurrConReg.f32InvDuty =  InvHVoltConReg.f32VoltInst_ErrOut;

  	 if (InvHCurrConReg.f32InvDuty >=  InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD)
  		 InvHCurrConReg.f32InvDuty = InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
  	 else if (InvHCurrConReg.f32InvDuty <= -InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD )
  		 InvHCurrConReg.f32InvDuty = -InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
  	 else
  		 ;

  	 //open loop just for test
	 #ifdef INV_OPEN_LOOP
  	 InvHCurrConReg.f32InvDuty = 0.76f * PWM_HALF_PERIOD * OutPLLConReg.Sin_Theta;
	 #endif

  	 EPwm5Regs.CMPA.half.CMPA = (Uint16)(-InvHCurrConReg.f32InvDuty);
  	 EPwm5Regs.CMPB = (Uint16)(-InvHCurrConReg.f32InvDuty);
  	 EPwm6Regs.CMPA.half.CMPA = (Uint16)(InvHCurrConReg.f32InvDuty);
  	 EPwm6Regs.CMPB = (Uint16)(InvHCurrConReg.f32InvDuty);

 } // end of InvH_VoltControl

/*=============================================================================*
 * FUNCTION:	void InvL_VoltControl(void)
 *
 * PURPOSE:	INVL(110V) voltage PR controller
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void InvL_VoltControl(void)
{
	// start of InvL_VoltControl

	/*
	 * PR controller in s domain: Gpr =
	 * 				    kr * s
	 *  kp + --------------
	 *  		  s^2 + w0^2
	 */
	static float32 kp = 1.5f;//2.0f;  //2018.6.25 GX
	static float32 kr = 350* 0.00005f;//1200 * 0.00005f;  //2018.6.25 GX
	float32 Coff_a2 = 0.00006168503f;  				// (w0 * Ts) * (w0 * Ts), where w0 is 2 * pi * 25Hz; Ts is the sample period 1 / 20000Hz
	float32 Coff_a3 = 0.2499961f;     					// 1 / (4 + w0 * w0 * Ts * Ts)

	/*
	 *	Change INVH(220V) PR parameter according to output current amplitude.
	 */
	/*if(Calc_Result.f32IInvL_rms_instant <= 0.9f)
	{
		kp = 2.0f;
		kr = 1200* 0.00005f;
	}
	else if(Calc_Result.f32IInvL_rms_instant > 0.9f && Calc_Result.f32IInvL_rms_instant <= 1.1f)
	{
	    kp=kp;
	    kr=kr;
	}
	else if(Calc_Result.f32IInvL_rms_instant > 1.1f && Calc_Result.f32IInvL_rms_instant <= 4.8f)
	{
		kp = 2.0f;
		kr = 350* 0.00005f;
	}
	else if(Calc_Result.f32IInvL_rms_instant > 4.8f && Calc_Result.f32IInvL_rms_instant <= 5.1f)
	{
		kp=kp;
		kr=kr;
	}
	else if(Calc_Result.f32IInvL_rms_instant > 5.1f)
	{
		kp = 1.5f;
		kr = 1200* 0.00005f;
	}*/

	InvLVoltConReg.f32VoltInst_Ref =  InvLVoltConReg.f32VoltRms_Ref * 1.414f * OutPLLConReg.Cos_Theta * InvLVoltConReg.f32VoltGain;

    InvLVoltConReg.f32VoltInst_Fdb = GetRealValue.f32VInvL;

	InvLVoltConReg.f32VoltInst_ErrOld = InvLVoltConReg.f32VoltInst_ErrNew;
	InvLVoltConReg.f32VoltInst_ErrNew = InvLVoltConReg.f32VoltInst_Ref - InvLVoltConReg.f32VoltInst_Fdb;

  	if(InvLVoltConReg.f32VoltInst_ErrNew  > 10)
  	{
  		InvLVoltConReg.f32VoltInst_ErrNew = 10;
  	}
	else if(InvLVoltConReg.f32VoltInst_ErrNew  < -10)
  	{
		InvLVoltConReg.f32VoltInst_ErrNew  = -10;
	}

 	 /*
 	  * The following is the PR controller difference-equation
 	  */
	InvLVoltConReg.Input[2] = InvLVoltConReg.Input[1];																			// x(k-2) = x(k-1)
	InvLVoltConReg.Input[1] = InvLVoltConReg.Input[0];																			// x(k-1) = x(k)
	InvLVoltConReg.Input[0] = InvLVoltConReg.f32VoltInst_ErrNew;														// x(k)

	InvLVoltConReg.Output[2] = InvLVoltConReg.Output[1];																	// y(k-2) = y(k-1)
	InvLVoltConReg.Output[1] = InvLVoltConReg.Output[0];																	// y(k-1) = y(k)

	InvLVoltConReg.MAC = ((2*kr+4*kp+Coff_a2*kp) * Coff_a3) * InvLVoltConReg.Input[0];                // + a1 * x(k)
	InvLVoltConReg.MAC -=((8-2*Coff_a2) *kp * Coff_a3)* InvLVoltConReg.Input[1];
	InvLVoltConReg.MAC += ((4*kp+Coff_a2*kp-2*kr)* Coff_a3) * InvLVoltConReg.Input[2];				// + a3 * x(k-2)

	InvLVoltConReg.MAC += ((8-2*Coff_a2)* Coff_a3) * InvLVoltConReg.Output[1];								// + b1 * y(k-1)
	InvLVoltConReg.MAC -= InvLVoltConReg.Output[2];																		// - b2 * y(k-2)

	InvLVoltConReg.Output[0] = InvLVoltConReg.MAC;
	InvLVoltConReg.f32VoltInst_ErrOut = InvLVoltConReg.Output[0];
	InvLCurrConReg.f32InvDuty =  InvLVoltConReg.f32VoltInst_ErrOut;

	if (InvLCurrConReg.f32InvDuty >=  InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD)
		InvLCurrConReg.f32InvDuty = InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
    else if (InvLCurrConReg.f32InvDuty <= - InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD  )
    	InvLCurrConReg.f32InvDuty = - InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
    else if (InvLCurrConReg.f32InvDuty < InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD && \
    		InvLCurrConReg.f32InvDuty >= InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLCurrConReg.f32InvDuty = InvLCurrConReg.f32InvDuty;
    else if (InvLCurrConReg.f32InvDuty > -InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD && \
    		InvLCurrConReg.f32InvDuty <= -InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLCurrConReg.f32InvDuty = InvLCurrConReg.f32InvDuty;
    else if (InvLCurrConReg.f32InvDuty >= 0 && \
    		InvLCurrConReg.f32InvDuty <  InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLCurrConReg.f32InvDuty = InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD;
    else if (InvLCurrConReg.f32InvDuty < 0 && \
    		InvLCurrConReg.f32InvDuty >  -InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLCurrConReg.f32InvDuty = -InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD;
    else
    	;

	//open loop just for test
	#ifdef INV_OPEN_LOOP
	InvLCurrConReg.f32InvDuty = 0.38f * PWM_HALF_PERIOD * OutPLLConReg.Cos_Theta;
	#endif

    EPwm3Regs.CMPA.half.CMPA = (Uint16)(-InvLCurrConReg.f32InvDuty);
    EPwm3Regs.CMPB = (Uint16)(-InvLCurrConReg.f32InvDuty);
    EPwm4Regs.CMPA.half.CMPA = (Uint16)(InvLCurrConReg.f32InvDuty);
    EPwm4Regs.CMPB = (Uint16)(InvLCurrConReg.f32InvDuty);

} // end of InvL_VoltControl

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
	  * 'OutPLLConReg.f32Theta_Step' equals 25Hz * 2 * pi / 20000Hz,
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
	 * INVH(220V) uses 'OutPLLConReg.Sin_Theta', while INVL(110V) uses 'OutPLLConReg.Cos_Theta)'.
	 * So 110V lead half period than 220V
	 */
	sincos(OutPLLConReg.f32Theta, &(OutPLLConReg.Sin_Theta), &(OutPLLConReg.Cos_Theta));

} // end of OutPLLcontroller()

 /*=============================================================================*
  * FUNCTION:	void VoutLPhaseCalc(void)
  *
  * PURPOSE:		Load voltage(110V) phase lock controller
  *
  * CALLED BY:	void ADC_INT_INV_Control(void)
  *============================================================================*/
void VoutLPhaseCalc(void)
{
	static int Li = 0;

	float32 VoutL_PLL_Kp = 1e-5f;
	float32 VoutL_PLL_Ki = 5e-9f;
	float32 LPFL_B0 = 0.00003913f;
	float32 LPFL_B1 = 0.00007826f;
	float32 LPFL_B2 = 0.00003913f;
	float32 LPFL_A1 = 1.9822318f;
	float32 LPFL_A2 = 0.98238832f;
	float32 DELTA_ANGLE_L = 7.5398e-3f;

	VOutLPLLConReg.f32Valpha = GetRealValue.f32VOutL;
	Li++;

	//The following is the different equation of a second order low pass filter
	if (4 == Li)
	{
		VOutLPLLReg.Input[2] = VOutLPLLReg.Input[1];																		// x(k-2) = x(k-1)
		VOutLPLLReg.Input[1] = VOutLPLLReg.Input[0];																		// x(k-1) = x(k)
		VOutLPLLReg.Input[0] = VOutLPLLConReg.f32Valpha * VOutLPLLConReg.Cos_Theta;		// x(k)

		VOutLPLLReg.Output[2] = VOutLPLLReg.Output[1];																// y(k-2) = y(k-1)
		VOutLPLLReg.Output[1] = VOutLPLLReg.Output[0];																// y(k-1) = y(k)

		VOutLPLLReg.MAC = LPFL_B0 * VOutLPLLReg.Input[0];															// + b0 * x(k)
	 	VOutLPLLReg.MAC += LPFL_B1 * VOutLPLLReg.Input[1];														// + b1 * x(k-1)
	 	VOutLPLLReg.MAC += LPFL_B2 * VOutLPLLReg.Input[2];														// + b2 * x(k-2)

	 	VOutLPLLReg.MAC += LPFL_A1 * VOutLPLLReg.Output[1];													// + a11 * y(k-1)
	 	VOutLPLLReg.MAC -= LPFL_A2 * VOutLPLLReg.Output[2];														// - a2 * y(k-2)

	 	VOutLPLLReg.Output[0] = VOutLPLLReg.MAC;
	 	Li = 0;
	 }

	 //PI regulator can achieve zero static error of phase
	 VOutLPLLReg.f32PIDErr_Old = VOutLPLLReg.f32PIDErr_New ;
	 VOutLPLLReg.f32PIDErr_New  = VOutLPLLReg.Output[0] - VOutLPLLReg.f32Refer;				// error(n) = y(k) - 0

	 VOutLPLLReg.f32PID_Output = VOutLPLLReg.f32PID_Output \
	 					+ (VoutL_PLL_Kp + VoutL_PLL_Ki) * VOutLPLLReg.f32PIDErr_New \
	 					- VoutL_PLL_Kp * VOutLPLLReg.f32PIDErr_Old;                		// u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

	 /*
	  * 'VOutLPLLReg.f32Delta_Theta' is the angle step which needs to be accumulated in each ADC interruption.
	  * 'DELTA_ANGLE_L' is the bias, which can acceleration regulating process
	  */
	 VOutLPLLReg.f32Delta_Theta =  DELTA_ANGLE_L + VOutLPLLReg.f32PID_Output;

	 if(VOutLPLLReg.f32Delta_Theta > InvTheta_Step_Hi_Limit)
	 	VOutLPLLReg.f32Delta_Theta = InvTheta_Step_Hi_Limit;
	 if(VOutLPLLReg.f32Delta_Theta < InvTheta_Step_Low_Limit)
	 	VOutLPLLReg.f32Delta_Theta = InvTheta_Step_Low_Limit;

	 //'VOutLPLLConReg.f32Theta_Step' is used to calculate the frequency of the 110V load voltage
	 VOutLPLLConReg.f32Theta_Step = VOutLPLLReg.f32Delta_Theta ;
	 VOutLPLLReg.f32Theta += VOutLPLLReg.f32Delta_Theta;

	 if ( VOutLPLLReg.f32Theta > Value_2Pi )
	 	VOutLPLLReg.f32Theta = VOutLPLLReg.f32Theta - Value_2Pi;

	 sincos(VOutLPLLReg.f32Theta, &(VOutLPLLConReg.Sin_Theta), &(VOutLPLLConReg.Cos_Theta));
}

/*=============================================================================*
  * FUNCTION:	void VoutHPhaseCalc(void)
  *
  * PURPOSE:		Load voltage(220V) phase lock controller
  *
  * CALLED BY:	void ADC_INT_INV_Control(void)
  *============================================================================*/
void VoutHPhaseCalc(void)
{
	static int Hi = 0;

	 float32 VoutH_PLL_Kp = 1e-5f;
	 float32 VoutH_PLL_Ki = 5e-9f;
	 float32 LPFH_B0 = 0.00003913f;
	 float32 LPFH_B1 = 0.00007826f;
	 float32 LPFH_B2 = 0.00003913f;
	 float32 LPFH_A1 = 1.9822318f;
	 float32 LPFH_A2 = 0.98238832f;
	 float32 DELTA_ANGLE_H = 7.5398e-3f;

	 VOutHPLLConReg.f32Valpha = GetRealValue.f32VOutH;
	 Hi++;

	 //The following is the different equation of a second order low pass filter
	 if (4 == Hi)
	 {
	 	VOutHPLLReg.Input[2] = VOutHPLLReg.Input[1];																		// x(k-2) = x(k-1)
	 	VOutHPLLReg.Input[1] = VOutHPLLReg.Input[0];																		// x(k-1) = x(k)
	 	VOutHPLLReg.Input[0] = VOutHPLLConReg.f32Valpha * VOutHPLLConReg.Cos_Theta;		// x(k)

 		VOutHPLLReg.Output[2] = VOutHPLLReg.Output[1];																// y(k-2) = y(k-1)
 		VOutHPLLReg.Output[1] = VOutHPLLReg.Output[0];																// y(k-1) = y(k)

 		VOutHPLLReg.MAC = LPFH_B0 * VOutHPLLReg.Input[0];														// + b0 * x(k)
 		VOutHPLLReg.MAC += LPFH_B1 * VOutHPLLReg.Input[1];														// + b1 * x(k-1)
 		VOutHPLLReg.MAC += LPFH_B2 * VOutHPLLReg.Input[2];														// + b2 * x(k-2)

 		VOutHPLLReg.MAC += LPFH_A1 * VOutHPLLReg.Output[1];													// + a11 * y(k-1)
 		VOutHPLLReg.MAC -= LPFH_A2 * VOutHPLLReg.Output[2];													// - a2 * y(k-2)

 		VOutHPLLReg.Output[0] = VOutHPLLReg.MAC;
 		Hi = 0;
 	}

	//PI regulator can achieve zero static error of phase
 	VOutHPLLReg.f32PIDErr_Old = VOutHPLLReg.f32PIDErr_New ;
 	VOutHPLLReg.f32PIDErr_New  = VOutHPLLReg.Output[0] - VOutHPLLReg.f32Refer;				// error(n) = y(k) - 0

 	VOutHPLLReg.f32PID_Output = VOutHPLLReg.f32PID_Output \
 						+ (VoutH_PLL_Kp + VoutH_PLL_Ki) * VOutHPLLReg.f32PIDErr_New \
 						- VoutH_PLL_Kp * VOutHPLLReg.f32PIDErr_Old;           // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

 	/*
 	* 'VOutHPLLReg.f32Delta_Theta' is the angle step which needs to be accumulated in each ADC interruption.
 	* 'DELTA_ANGLE_H' is the bias, which can acceleration regulating process
 	*/
 	VOutHPLLReg.f32Delta_Theta =  DELTA_ANGLE_H + VOutHPLLReg.f32PID_Output;

 	if(VOutHPLLReg.f32Delta_Theta > InvTheta_Step_Hi_Limit)
 		VOutHPLLReg.f32Delta_Theta = InvTheta_Step_Hi_Limit;
 	if(VOutHPLLReg.f32Delta_Theta < InvTheta_Step_Low_Limit)
 		VOutHPLLReg.f32Delta_Theta = InvTheta_Step_Low_Limit;

 	//'VOutHPLLConReg.f32Theta_Step' is used to calculate the frequency of the 220V load voltage
 	VOutHPLLConReg.f32Theta_Step = VOutHPLLReg.f32Delta_Theta ;
 	VOutHPLLReg.f32Theta += VOutHPLLReg.f32Delta_Theta;

 	if ( VOutHPLLReg.f32Theta > Value_2Pi )
 		VOutHPLLReg.f32Theta = VOutHPLLReg.f32Theta - Value_2Pi;

 	sincos(VOutHPLLReg.f32Theta, &(VOutHPLLConReg.Sin_Theta), &(VOutHPLLConReg.Cos_Theta));
}

/*=============================================================================*
  * FUNCTION:	 void InvVoltSlowup(void)
  *
  * PURPOSE:		Inverter slow up function
  *
  * CALLED BY:	void TSK_InvVoltPeriod(void)
  *============================================================================*/
void InvVoltSlowup(void)
{
	if((NormalState==g_Sys_Current_State) && (1 == g_ParaLogic_State.bit.InvSoftStart_EN))
	{
		if (g_StateCheck.bit.InvDrvEnable == 0)
		{
			g_StateCheck.bit.InvDrvEnable = 1;
			InvPWMOutputsEnable();
		}
		if(Calc_Result.f32VGrid_rms > 100)
		{
			if(InvHVoltConReg.f32VoltGain < 1)
		 	{
				InvHVoltConReg.f32VoltGain = InvHVoltConReg.f32VoltGain + 0.01f;
				InvHVoltConReg.f32VoltDutyUpLimit = InvHVoltConReg.f32VoltDutyUpLimit + 0.01f;
		 	}
		 	else
		 	{
		 		InvHVoltConReg.f32VoltGain = 1;
		 		InvLVoltConReg.f32VoltDutyLowLimit = 0.025f;
		 		InvHVoltConReg.f32VoltDutyUpLimit = 0.95f;
		 	}
		}
		else
			InvHVoltConReg.f32VoltDutyUpLimit = 0.95f;

		InvLVoltConReg.f32VoltGain = InvHVoltConReg.f32VoltGain;
		InvLVoltConReg.f32VoltDutyUpLimit = 0.6f * InvHVoltConReg.f32VoltDutyUpLimit;
	 }

	 if(1 == InvHVoltConReg.f32VoltGain)
	 	g_StateCheck.bit.Inv_SoftStart = 1;
	 else
	 	g_StateCheck.bit.Inv_SoftStart = 0;

	 if (g_StateCheck.bit.Inv_SoftStart == 1 && ShortCheck_Reg.Restart_times == 0)
	 {
		 if (InvHVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvH_VoltRms_Ref)
		 {
			 InvHVoltConReg.f32VoltRms_Ref -= 0.6;
			 if (InvHVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvH_VoltRms_Ref)
				 InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
		 }
		 else if (InvHVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvH_VoltRms_Ref)
		 {
			 InvHVoltConReg.f32VoltRms_Ref += 0.6;
			 if (InvHVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvH_VoltRms_Ref)
				 InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
		 }
		 if (InvLVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvL_VoltRms_Ref)
		 {
			 InvLVoltConReg.f32VoltRms_Ref -= 0.6;
			 if (InvLVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvL_VoltRms_Ref)
				 InvLVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;
		 }
		 else if (InvLVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvL_VoltRms_Ref)
		 {
			 InvLVoltConReg.f32VoltRms_Ref += 0.6;
			 if (InvLVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvL_VoltRms_Ref)
				 InvLVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;
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
{
	// start of InverterStage_Init

	InvHVoltConReg.f32VoltRms_ErrOld = 0;
	InvHVoltConReg.f32VoltRms_ErrNew = 0;
	InvHVoltConReg.f32VoltRms_Out = 0;
	InvHVoltConReg.f32VoltRms_Ref_Delta = 3;

	InvHVoltConReg.f32VoltInst_ErrOld = 0;
	InvHVoltConReg.f32VoltInst_ErrNew = 0;
	InvHVoltConReg.f32VoltInst_ErrOut = 0;

	InvHCurrConReg.f32CurrInst_ErrOld = 0;
	InvHCurrConReg.f32CurrInst_ErrNew = 0;

	InvHCurrConReg.f32InvDuty_Con = 0;

	InvLVoltConReg.f32VoltRms_ErrOld = 0;
	InvLVoltConReg.f32VoltRms_ErrNew = 0;
	InvLVoltConReg.f32VoltRms_Out = 0;
	InvLVoltConReg.f32VoltRms_Ref_Delta = 1.5;

	InvLVoltConReg.f32VoltInst_ErrOld = 0;
	InvLVoltConReg.f32VoltInst_ErrNew = 0;
	InvLVoltConReg.f32VoltInst_ErrOut = 0;

	InvLCurrConReg.f32CurrInst_ErrOld = 0;
	InvLCurrConReg.f32CurrInst_ErrNew = 0;

	InvLCurrConReg.f32InvDuty_Con = 0;

	InvHVoltConReg.f32VoltGain = 0;
	InvLVoltConReg.f32VoltGain = 0;
	InvHVoltConReg.f32VoltDutyUpLimit = 0.05f;
	InvHVoltConReg.f32VoltDutyLowLimit = 0.01f;
	InvLVoltConReg.f32VoltDutyUpLimit = 0.05f;
	InvLVoltConReg.f32VoltDutyLowLimit = 0.01f;

	g_StateCheck.bit.InvDrvEnable = 0;
	g_StateCheck.bit.Inv_SoftStart = 0;
	g_ParaLogic_State.bit.InvSoftStart_EN = 0;

	Parallel_Reg.u16Cnt_SCR_ON = 0;
	InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
	InvLVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;

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
	static Uint16 u16temp1 = 0;

	if((g_StateCheck.bit.Inv_SoftStart == 1) && (NormalState==g_Sys_Current_State))
	{
		if(ShortCheck_Reg.Restart_times >=1)
		{
			if(u16temp1 <250)	//250 * 40ms = 10s
				u16temp1++;
			else
			{
				u16temp1=0;
				ShortCheck_Reg.Restart_times = 0;
				g_Sys_Current_State = FaultState;
				SafetyReg.f32InvH_VoltRms_Ref = InvH_RatedVolt_Ref;
			    SafetyReg.f32InvL_VoltRms_Ref = InvL_RatedVolt_Ref;
				//InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
				//InvLVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;
			}
		}
	}
	else
		u16temp1 = 0;
 }

//--- end of file -----------------------------------------------------

