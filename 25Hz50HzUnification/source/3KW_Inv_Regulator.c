/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_DataAcquisition.c
 *
 *  PURPOSE  : Data acquisition and protection file of the module.
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
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
struct  INVVOLTCONTREG  					InvHVoltConReg, InvLVoltConReg;

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

	InvHVoltConReg.f32VoltInst_Ref = InvHVoltConReg.f32VoltRms_Ref * 1.414f * OutPLLConReg.f32Sin_Theta * InvHVoltConReg.f32VoltGain;
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
  	 InvHVoltConReg.f32Input[2] = InvHVoltConReg.f32Input[1];																				// x(k-2) = x(k-1)
  	 InvHVoltConReg.f32Input[1] = InvHVoltConReg.f32Input[0];																				// x(k-1) = x(k)
  	 InvHVoltConReg.f32Input[0] = InvHVoltConReg.f32VoltInst_ErrNew;																	// x(k)

  	 InvHVoltConReg.f32Output[2] = InvHVoltConReg.f32Output[1];																			// y(k-2) = y(k-1)
  	 InvHVoltConReg.f32Output[1] = InvHVoltConReg.f32Output[0];																			// y(k-1) = y(k)

  	 InvHVoltConReg.f32MAC = ( ( 2 * InvHVoltConReg.f32Kr + 4 * InvHVoltConReg.f32Kp + \
  			 Inv_Volt_Coff_a2 * InvHVoltConReg.f32Kp ) * Inv_Volt_Coff_a3 ) * InvHVoltConReg.f32Input[0];                         // + a1 * x(k)
  	 InvHVoltConReg.f32MAC -=( ( 8 - 2 * Inv_Volt_Coff_a2 ) * InvHVoltConReg.f32Kp * \
  			 Inv_Volt_Coff_a3) * InvHVoltConReg.f32Input[1];
  	 InvHVoltConReg.f32MAC += ( ( 4 * InvHVoltConReg.f32Kp + Inv_Volt_Coff_a2 * InvHVoltConReg.f32Kp - \
  			 2 * InvHVoltConReg.f32Kr ) * Inv_Volt_Coff_a3) * InvHVoltConReg.f32Input[2];													// + a3 * x(k-2)

  	 InvHVoltConReg.f32MAC += ((8-2*Inv_Volt_Coff_a2)* Inv_Volt_Coff_a3) * InvHVoltConReg.f32Output[1];										// + b1 * y(k-1)
  	 InvHVoltConReg.f32MAC -= InvHVoltConReg.f32Output[2];																					// - b2 * y(k-2)

  	 InvHVoltConReg.f32Output[0] = InvHVoltConReg.f32MAC;
  	 InvHVoltConReg.f32VoltInst_ErrOut = InvHVoltConReg.f32Output[0];
  	 InvHVoltConReg.f32InvDuty =  InvHVoltConReg.f32VoltInst_ErrOut;

  	 if (InvHVoltConReg.f32InvDuty >=  InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD)
  		 InvHVoltConReg.f32InvDuty = InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
  	 else if (InvHVoltConReg.f32InvDuty <= -InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD )
  		 InvHVoltConReg.f32InvDuty = -InvHVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
  	 else
  		 ;

  	 //open loop just for test
	 #ifdef INV_OPEN_LOOP
  	 InvHVoltConReg.f32InvDuty = 0.76f * PWM_HALF_PERIOD * OutPLLConReg.f32Sin_Theta;
	 #endif

  	 EPwm5Regs.CMPA.half.CMPA = (Uint16)(-InvHVoltConReg.f32InvDuty);
  	 EPwm5Regs.CMPB = (Uint16)(-InvHVoltConReg.f32InvDuty);
  	 EPwm6Regs.CMPA.half.CMPA = (Uint16)(InvHVoltConReg.f32InvDuty);
  	 EPwm6Regs.CMPB = (Uint16)(InvHVoltConReg.f32InvDuty);

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

	InvLVoltConReg.f32VoltInst_Ref =  InvLVoltConReg.f32VoltRms_Ref * 1.414f * OutPLLConReg.f32Cos_Theta * InvLVoltConReg.f32VoltGain;

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
	InvLVoltConReg.f32Input[2] = InvLVoltConReg.f32Input[1];																			// x(k-2) = x(k-1)
	InvLVoltConReg.f32Input[1] = InvLVoltConReg.f32Input[0];																			// x(k-1) = x(k)
	InvLVoltConReg.f32Input[0] = InvLVoltConReg.f32VoltInst_ErrNew;														// x(k)

	InvLVoltConReg.f32Output[2] = InvLVoltConReg.f32Output[1];																	// y(k-2) = y(k-1)
	InvLVoltConReg.f32Output[1] = InvLVoltConReg.f32Output[0];																	// y(k-1) = y(k)

	InvLVoltConReg.f32MAC = ( ( 2 * InvLVoltConReg.f32Kr + 4 * InvLVoltConReg.f32Kp + \
			Inv_Volt_Coff_a2 * InvLVoltConReg.f32Kp ) * Inv_Volt_Coff_a3 ) * InvLVoltConReg.f32Input[0];                // + a1 * x(k)
	InvLVoltConReg.f32MAC -=( ( 8 - 2 * Inv_Volt_Coff_a2 ) * InvLVoltConReg.f32Kp *\
			Inv_Volt_Coff_a3 ) * InvLVoltConReg.f32Input[1];
	InvLVoltConReg.f32MAC += ( ( 4 * InvLVoltConReg.f32Kp + Inv_Volt_Coff_a2 * InvLVoltConReg.f32Kp - \
			2 * InvLVoltConReg.f32Kr ) * Inv_Volt_Coff_a3 ) * InvLVoltConReg.f32Input[2];				// + a3 * x(k-2)

	InvLVoltConReg.f32MAC += ( ( 8 - 2 * Inv_Volt_Coff_a2 ) * Inv_Volt_Coff_a3 ) * InvLVoltConReg.f32Output[1];								// + b1 * y(k-1)
	InvLVoltConReg.f32MAC -= InvLVoltConReg.f32Output[2];																		// - b2 * y(k-2)

	InvLVoltConReg.f32Output[0] = InvLVoltConReg.f32MAC;
	InvLVoltConReg.f32VoltInst_ErrOut = InvLVoltConReg.f32Output[0];
	InvLVoltConReg.f32InvDuty =  InvLVoltConReg.f32VoltInst_ErrOut;

	if (InvLVoltConReg.f32InvDuty >=  InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD)
		InvLVoltConReg.f32InvDuty = InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
    else if (InvLVoltConReg.f32InvDuty <= - InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD  )
    	InvLVoltConReg.f32InvDuty = - InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD;
    else if (InvLVoltConReg.f32InvDuty < InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD && \
    		InvLVoltConReg.f32InvDuty >= InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLVoltConReg.f32InvDuty = InvLVoltConReg.f32InvDuty;
    else if (InvLVoltConReg.f32InvDuty > -InvLVoltConReg.f32VoltDutyUpLimit * PWM_HALF_PERIOD && \
    		InvLVoltConReg.f32InvDuty <= -InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLVoltConReg.f32InvDuty = InvLVoltConReg.f32InvDuty;
    else if (InvLVoltConReg.f32InvDuty >= 0 && \
    		InvLVoltConReg.f32InvDuty <  InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLVoltConReg.f32InvDuty = InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD;
    else if (InvLVoltConReg.f32InvDuty < 0 && \
    		InvLVoltConReg.f32InvDuty >  -InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD)
    	InvLVoltConReg.f32InvDuty = -InvLVoltConReg.f32VoltDutyLowLimit * PWM_HALF_PERIOD;
    else
    	;

	//open loop just for test
	#ifdef INV_OPEN_LOOP
	InvLVoltConReg.f32InvDuty = 0.38f * PWM_HALF_PERIOD * OutPLLConReg.f32Cos_Theta;
	#endif

    EPwm3Regs.CMPA.half.CMPA = (Uint16)(-InvLVoltConReg.f32InvDuty);
    EPwm3Regs.CMPB = (Uint16)(-InvLVoltConReg.f32InvDuty);
    EPwm4Regs.CMPA.half.CMPA = (Uint16)(InvLVoltConReg.f32InvDuty);
    EPwm4Regs.CMPB = (Uint16)(InvLVoltConReg.f32InvDuty);

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
	 * INVH(220V) uses 'OutPLLConReg.f32Sin_Theta', while INVL(110V) uses 'OutPLLConReg.f32Cos_Theta)'.
	 * So 110V lead half period than 220V
	 */
	sincos(OutPLLConReg.f32Theta, &(OutPLLConReg.f32Sin_Theta), &(OutPLLConReg.f32Cos_Theta));

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
	static Uint8 u8cntL = 0;

	VOutLPLLConReg.f32Valpha = GetRealValue.f32VOutL;
	u8cntL++;

	//The following is the different equation of a second order low pass filter
	if (4 == u8cntL)
	{
		VOutLPLLConReg.f32Input[2] = VOutLPLLConReg.f32Input[1];																		// x(k-2) = x(k-1)
		VOutLPLLConReg.f32Input[1] = VOutLPLLConReg.f32Input[0];																		// x(k-1) = x(k)
		VOutLPLLConReg.f32Input[0] = VOutLPLLConReg.f32Valpha * VOutLPLLConReg.f32Cos_Theta;		// x(k)

		VOutLPLLConReg.f32Output[2] = VOutLPLLConReg.f32Output[1];																// y(k-2) = y(k-1)
		VOutLPLLConReg.f32Output[1] = VOutLPLLConReg.f32Output[0];																// y(k-1) = y(k)

		VOutLPLLConReg.f32MAC = LPF_B0_INV * VOutLPLLConReg.f32Input[0];															// + b0 * x(k)
	 	VOutLPLLConReg.f32MAC += LPF_B1_INV * VOutLPLLConReg.f32Input[1];														// + b1 * x(k-1)
	 	VOutLPLLConReg.f32MAC += LPF_B2_INV * VOutLPLLConReg.f32Input[2];														// + b2 * x(k-2)

	 	VOutLPLLConReg.f32MAC += LPF_A1_INV * VOutLPLLConReg.f32Output[1];													// + a11 * y(k-1)
	 	VOutLPLLConReg.f32MAC -= LPF_A2_INV * VOutLPLLConReg.f32Output[2];														// - a2 * y(k-2)

	 	VOutLPLLConReg.f32Output[0] = VOutLPLLConReg.f32MAC;
	 	u8cntL = 0;
	 }

	 //PI regulator can achieve zero static error of phase
	 VOutLPLLConReg.f32PIDErr_Old = VOutLPLLConReg.f32PIDErr_New ;
	 VOutLPLLConReg.f32PIDErr_New  = VOutLPLLConReg.f32Output[0] - VOutLPLLConReg.f32Refer;				// error(n) = y(k) - 0

	 VOutLPLLConReg.f32PID_Output = VOutLPLLConReg.f32PID_Output \
	 					+ (VOutLPLLConReg.f32Kp + VOutLPLLConReg.f32Ki) * VOutLPLLConReg.f32PIDErr_New \
	 					- VOutLPLLConReg.f32Kp * VOutLPLLConReg.f32PIDErr_Old;                		// u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

	 /*
	  * 'VOutLPLLConReg.f32Theta_Step' is the angle step which needs to be accumulated in each ADC interruption.
	  * 'DELTA_ANGLE_L' is the bias, which can acceleration regulating process
	  */
	 VOutLPLLConReg.f32Theta_Step =  DELTA_ANGLE_INV + VOutLPLLConReg.f32PID_Output;

	 if(VOutLPLLConReg.f32Theta_Step > InvTheta_Step_Hi_Limit)
	 	VOutLPLLConReg.f32Theta_Step = InvTheta_Step_Hi_Limit;
	 if(VOutLPLLConReg.f32Theta_Step < InvTheta_Step_Low_Limit)
	 	VOutLPLLConReg.f32Theta_Step = InvTheta_Step_Low_Limit;

	 //'VOutLPLLConReg.f32Theta_Step' is used to calculate the frequency of the 110V load voltage
	 VOutLPLLConReg.f32Theta += VOutLPLLConReg.f32Theta_Step;

	 if ( VOutLPLLConReg.f32Theta > Value_2Pi )
	 	VOutLPLLConReg.f32Theta = VOutLPLLConReg.f32Theta - Value_2Pi;

	 sincos(VOutLPLLConReg.f32Theta, &(VOutLPLLConReg.f32Sin_Theta), &(VOutLPLLConReg.f32Cos_Theta));
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
	static Uint8 u8cntH = 0;

	 VOutHPLLConReg.f32Valpha = GetRealValue.f32VOutH;
	 u8cntH++;

	 //The following is the different equation of a second order low pass filter
	 if (4 == u8cntH)
	 {
	 	VOutHPLLConReg.f32Input[2] = VOutHPLLConReg.f32Input[1];																		// x(k-2) = x(k-1)
	 	VOutHPLLConReg.f32Input[1] = VOutHPLLConReg.f32Input[0];																		// x(k-1) = x(k)
	 	VOutHPLLConReg.f32Input[0] = VOutHPLLConReg.f32Valpha * VOutHPLLConReg.f32Cos_Theta;		// x(k)

 		VOutHPLLConReg.f32Output[2] = VOutHPLLConReg.f32Output[1];																// y(k-2) = y(k-1)
 		VOutHPLLConReg.f32Output[1] = VOutHPLLConReg.f32Output[0];																// y(k-1) = y(k)

 		VOutHPLLConReg.f32MAC = LPF_B0_INV * VOutHPLLConReg.f32Input[0];														// + b0 * x(k)
 		VOutHPLLConReg.f32MAC += LPF_B1_INV * VOutHPLLConReg.f32Input[1];														// + b1 * x(k-1)
 		VOutHPLLConReg.f32MAC += LPF_B2_INV * VOutHPLLConReg.f32Input[2];														// + b2 * x(k-2)

 		VOutHPLLConReg.f32MAC += LPF_A1_INV * VOutHPLLConReg.f32Output[1];													// + a11 * y(k-1)
 		VOutHPLLConReg.f32MAC -= LPF_A2_INV * VOutHPLLConReg.f32Output[2];													// - a2 * y(k-2)

 		VOutHPLLConReg.f32Output[0] = VOutHPLLConReg.f32MAC;
 		u8cntH = 0;
 	}

	//PI regulator can achieve zero static error of phase
 	VOutHPLLConReg.f32PIDErr_Old = VOutHPLLConReg.f32PIDErr_New ;
 	VOutHPLLConReg.f32PIDErr_New  = VOutHPLLConReg.f32Output[0] - VOutHPLLConReg.f32Refer;				// error(n) = y(k) - 0

 	VOutHPLLConReg.f32PID_Output = VOutHPLLConReg.f32PID_Output \
 						+ (VOutHPLLConReg.f32Kp + VOutHPLLConReg.f32Ki) * VOutHPLLConReg.f32PIDErr_New \
 						- VOutHPLLConReg.f32Kp * VOutHPLLConReg.f32PIDErr_Old;           // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)

 	/*
 	* 'VOutHPLLConReg.f32Theta_Step' is the angle step which needs to be accumulated in each ADC interruption.
 	* 'DELTA_ANGLE_H' is the bias, which can acceleration regulating process
 	*/
 	VOutHPLLConReg.f32Theta_Step =  DELTA_ANGLE_INV + VOutHPLLConReg.f32PID_Output;

 	if(VOutHPLLConReg.f32Theta_Step > InvTheta_Step_Hi_Limit)
 		VOutHPLLConReg.f32Theta_Step = InvTheta_Step_Hi_Limit;
 	if(VOutHPLLConReg.f32Theta_Step < InvTheta_Step_Low_Limit)
 		VOutHPLLConReg.f32Theta_Step = InvTheta_Step_Low_Limit;

 	//'VOutHPLLConReg.f32Theta_Step' is used to calculate the frequency of the 220V load voltage
 	VOutHPLLConReg.f32Theta += VOutHPLLConReg.f32Theta_Step;

 	if ( VOutHPLLConReg.f32Theta > Value_2Pi )
 		VOutHPLLConReg.f32Theta = VOutHPLLConReg.f32Theta - Value_2Pi;

 	sincos(VOutHPLLConReg.f32Theta, &(VOutHPLLConReg.f32Sin_Theta), &(VOutHPLLConReg.f32Cos_Theta));
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
		 		InvHVoltConReg.f32VoltDutyUpLimit = 0.95f;
		 	}
		}
		else
			InvHVoltConReg.f32VoltDutyUpLimit = 0.05f;

		InvLVoltConReg.f32VoltGain = InvHVoltConReg.f32VoltGain;
		InvLVoltConReg.f32VoltDutyUpLimit = 0.6f * InvHVoltConReg.f32VoltDutyUpLimit;
	 }

	 if(1 == InvHVoltConReg.f32VoltGain)
	 	g_StateCheck.bit.Inv_SoftStart = 1;
	 else
	 	g_StateCheck.bit.Inv_SoftStart = 0;

	 if (g_StateCheck.bit.Inv_SoftStart == 1 && SafetyReg.u16Short_Restart_times == 0)
	 {
		 if (InvHVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvH_VoltRms_Ref)
		 {
			 InvHVoltConReg.f32VoltRms_Ref -= VoltRefDelta;
			 if (InvHVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvH_VoltRms_Ref)
				 InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
		 }
		 else if (InvHVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvH_VoltRms_Ref)
		 {
			 InvHVoltConReg.f32VoltRms_Ref += VoltRefDelta;
			 if (InvHVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvH_VoltRms_Ref)
				 InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
		 }
		 if (InvLVoltConReg.f32VoltRms_Ref >= SafetyReg.f32InvL_VoltRms_Ref)
		 {
			 InvLVoltConReg.f32VoltRms_Ref -= VoltRefDelta;
			 if (InvLVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvL_VoltRms_Ref)
				 InvLVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;
		 }
		 else if (InvLVoltConReg.f32VoltRms_Ref <= SafetyReg.f32InvL_VoltRms_Ref)
		 {
			 InvLVoltConReg.f32VoltRms_Ref += VoltRefDelta;
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
	InvHVoltConReg.f32VoltInst_ErrOld = 0;
	InvHVoltConReg.f32VoltInst_ErrNew = 0;
	InvHVoltConReg.f32VoltInst_ErrOut = 0;

	InvLVoltConReg.f32VoltInst_ErrOld = 0;
	InvLVoltConReg.f32VoltInst_ErrNew = 0;
	InvLVoltConReg.f32VoltInst_ErrOut = 0;

	InvHVoltConReg.f32VoltGain = 0;
	InvLVoltConReg.f32VoltGain = 0;
	InvHVoltConReg.f32VoltDutyUpLimit = 0.05f;
	InvHVoltConReg.f32VoltDutyLowLimit = 0.025f;
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
		if(SafetyReg.u16Short_Restart_times >=1)
		{
			if(u16temp1 <250)	//250 * 40ms = 10s
				u16temp1++;
			else
			{
				u16temp1=0;
				SafetyReg.u16Short_Restart_times = 0;
				g_Sys_Current_State = FaultState;
				SafetyReg.f32InvH_VoltRms_Ref = InvH_RatedVolt_Ref;
			    SafetyReg.f32InvL_VoltRms_Ref = InvL_RatedVolt_Ref;
			}
		}
	}
	else
		u16temp1 = 0;
 }

//--- end of file -----------------------------------------------------

