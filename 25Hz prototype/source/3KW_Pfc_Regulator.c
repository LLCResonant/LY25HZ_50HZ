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
#pragma CODE_SECTION(GridCurrentController, "ControlLoopInRAM")
#pragma CODE_SECTION(GridPLLcontroller, "ControlLoopInRAM")
#pragma CODE_SECTION(Get_ADC_Result1, "ControlLoopInRAM")
#pragma CODE_SECTION(BusVoltBalController, "ControlLoopInRAM")
#pragma CODE_SECTION(Calc_IGrid_reference, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccGridCalc, "ControlLoopInRAM")

//#pragma CODE_SECTION(ADC_INT_PFC_Control, "ControlLoopInRAM")

/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
struct  BUS_CON_REG                   		BusCon_Reg;
struct	CURRENTCONTREG	       		CurrConReg;
struct	PLLCONTREG							GridPLLConReg;
struct	PLL											GridPLLReg;

/*=============================================================================*
 * 	functions declaration
 *============================================================================*/
/* PI controller for voltage control */
void Calc_IGrid_reference(void);

/* Neutral Point Balance controller */
void BusVoltBalController(void);

/* Phase Lock Loop controller */
void GridPLLcontroller(void);

/* PI controller for current control */
void GridCurrentController(void);
		
/* Initialize the starting Duty and loop PID parameters */
void RectifierStage_Init(void);

/* Bus Voltage Reference Slow Up */
void BusVoltSlowup(void);

void ADC_INT_PFC_Control(void);

/* Voltage dip controller*/
void VGrid_Dip_Reset(void);

/* No load or light load controller */
void BusVoltHysteresis(void);

/*=============================================================================*
 * FUNCTION:	void ADC_INT_PFC_Control(void)
 *
 * PURPOSE:	One of Interruption service program. Contain the PFC control logic.
 *
 * CALLED BY:	void SEQ1INT_ISR(void)
 *============================================================================*/
void ADC_INT_PFC_Control(void)
{
	// start of ADC_INT_PFC_Control()

  	Get_ADC_Result1();
  	GridPLLcontroller();

  	#ifdef PFC_CLOSE_LOOP
  	if ( g_Sys_Current_State == NormalState )
  	{
  		if (CurrConReg.Driveopen == 0 && g_StateCheck.bit.Grid_Zero_Crossing_Flag == 1)
  		{
  			CurrConReg.f32PfcDuty_ff = 0;
  			CurrConReg.Driveopen = 1;
  			//send a signal to CPLD. The input current protection is set to be a Cycle-by-cycle protection
  			GRID_OCP_CBC_ON;
  		}

  		VGrid_Dip_Reset();

		if (CurrConReg.Driveopen >= 1 )
  		{
			Calc_IGrid_reference();
			BusVoltBalController();
			GridCurrentController();
  		}
 	}
	#endif

	#ifdef PFC_OPEN_LOOP
  	GridCurrentController();
  	CurrConReg.Driveopen = 1;
	#endif

	//Scib_SnatchGraph();
	ADAccGridCalc();

	/*
	* Software instantaneous current protection, which is used in tuning and abandoned in usual
	*/
	/*if((abs(GetRealValue.f32IGrid) > 40) && ( NormalState == g_Sys_Current_State))
	{
		g_SysFaultMessage.bit.unrecoverSW_OCP_Grid = 1;
		PfcPWMOutputsDisable();
	}*/

} // end of ADC_INT_PFC_Control()

/*=============================================================================*
 * FUNCTION:	void VGrid_Dip_Reset (void)
 *
 * PURPOSE:	Detect and handle Grid voltage dip according to Grid voltage RMS.
 * 						When voltage dip, the drive should be closed, and when voltage recover, the drive should be
 * 						opened and some variables should be clear.
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void VGrid_Dip_Reset (void)
{
	static Uint8 PFCPWM = 0;

	if(Calc_Result.f32VGrid_rms_previous <= 100)
	{
		PfcPWMOutputsDisable();
		CurrConReg.Driveopen = 0;
		PFCPWM = 1;
		g_StateCheck.bit.Input_dip_Disable_GridOCP = 1;
	}
	else if(Calc_Result.f32VGrid_rms_previous > 100 && PFCPWM == 1)
	{
		BusCon_Reg.f32BusVoltDiffErr_New = 0;
		BusCon_Reg.f32BusVoltDiffErr_Old = 0;
		BusCon_Reg.f32BusVoltDiff_Out = 0;

		CurrConReg.f32IGridErr_Old = 0;
		CurrConReg.f32IGridErr_New = 0;
		CurrConReg.f32PfcDuty_Con = 0;

		CurrConReg.Driveopen = 1;
		PFCPWM = 0;
	}
	else
		;
}

/*=============================================================================*
 * FUNCTION:	void GridCurrentController(void)
 *
 * PURPOSE:	PI controller for current control
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void GridCurrentController(void)
{
	// start of CurrentPIDcontroller

	static float32 Curr_Kp =40.0f;
	static float32 Curr_Ki =3.0f;
	static Uint8  BusOVP = 0;

	Curr_Kp = CurrConReg.f32Kp;
	Curr_Ki = CurrConReg.f32Ki;

	/*
	 * 'BusCon_Reg.f32IGridAmp_Ref * GridPLLConReg.Sin_Theta' is the original current reference
	 * 'BusCon_Reg.f32BusVoltDiff_Out' is the Neutral Point Balance superposition
	 * '2 * 3.14 * Calc_Result.f32GridFreq * 0.000005 * Calc_Result.f32VGrid_rms * 1.414 * GridPLLConReg.Cos_Theta'
	 * is the input capacitor current compensation
	 */
	CurrConReg.f32IGrid_Ref = BusCon_Reg.f32IGridAmp_Ref * GridPLLConReg.Sin_Theta - BusCon_Reg.f32BusVoltDiff_Out \
			- 2 * 3.14 * Calc_Result.f32GridFreq * 0.000005 * Calc_Result.f32VGrid_rms * 1.414 * GridPLLConReg.Cos_Theta;
	CurrConReg.f32IGrid_Fdb = GetRealValue.f32IGrid;

	CurrConReg.f32IGridErr_Old = CurrConReg.f32IGridErr_New;
	CurrConReg.f32IGridErr_New = CurrConReg.f32IGrid_Ref - CurrConReg.f32IGrid_Fdb;

	//PI calculating process
	CurrConReg.f32PfcDuty_Con = CurrConReg.f32PfcDuty_Con + (Curr_Kp + Curr_Ki) * CurrConReg.f32IGridErr_New - Curr_Kp * CurrConReg.f32IGridErr_Old;

	if (CurrConReg.Driveopen == 2)
		BusVoltHysteresis();

	//'CurrConReg.f32PfcDuty_ff' is the forward control superposition value
	CurrConReg.f32PfcDuty = CurrConReg.f32PfcDuty_Con - CurrConReg.f32PfcDuty_ff;

	if(CurrConReg.f32PfcDuty > PWM_HALF_PERIOD)
	{
		CurrConReg.f32PfcDuty = PWM_HALF_PERIOD;
		CurrConReg.f32PfcDuty_Con = CurrConReg.f32PfcDuty + CurrConReg.f32PfcDuty_ff;
	}
	else if(CurrConReg.f32PfcDuty < (-PWM_HALF_PERIOD))
	{
		CurrConReg.f32PfcDuty = -PWM_HALF_PERIOD;
		CurrConReg.f32PfcDuty_Con = CurrConReg.f32PfcDuty + CurrConReg.f32PfcDuty_ff;
	}

	// open loop just for test
	#ifdef PFC_OPEN_LOOP
	CurrConReg.f32PfcDuty = 0.8 * PWM_HALF_PERIOD * GridPLLConReg.Sin_Theta;
	#endif

	if(CurrConReg.f32PfcDuty <= 0)
	{
		EPwm2Regs.CMPA.half.CMPA = (Uint16)(-CurrConReg.f32PfcDuty);
		EPwm1Regs.CMPA.half.CMPA = 0;
	}
	else
	{
		EPwm1Regs.CMPA.half.CMPA = (Uint16)(CurrConReg.f32PfcDuty);
		EPwm2Regs.CMPA.half.CMPA = 0;
	}

	if (CurrConReg.Driveopen == 1) //2017.4.18 GX not confirmed
	{
		CurrConReg.Driveopen = 2;
		PfcPWMOutputsEnable();
	}
	else if (CurrConReg.Driveopen >= 1)
	{
		/*
		 * When Bus voltage increase unusually, the drive should be closed.
		 * When the voltage is back to normal, the drive should be opened and some variables should be clear.
		 */
		if((GetRealValue.f32VBusP+GetRealValue.f32VBusN) >= 900 && BusOVP == 0)
		{
			PfcPWMOutputsDisable();
			BusOVP = 1;
			g_StateCheck.bit.PwmForceOffFlag = 1;
		}
		else if((GetRealValue.f32VBusP+GetRealValue.f32VBusN) <= 870 && BusOVP == 1)
		{
			CurrConReg.f32PfcDuty_Con=0;
	        CurrConReg.f32IGridErr_New=0;
	        CurrConReg.f32IGridErr_Old=0;
	        BusCon_Reg.f32BusVoltDiffErr_New = 0;
	        BusCon_Reg.f32BusVoltDiffErr_Old = 0;
	        BusCon_Reg.f32BusVoltDiff_Out = 0;
	        CurrConReg.Driveopen = 1;
	        BusOVP = 0;
		}
	}
} // end of CurrentPIDcontroller

/*=============================================================================*
 * FUNCTION:	void BusVoltHysteresis(void)
 *
 * PURPOSE:	No load or light load controller.
 * 						'GetRealValue.f32VGrid * Calc_Result.Coff_Dforward' is the normal forward value,
 * 						which is the sine ware in time domain.
 * 						When  the output of voltage loop is smaller than 2, a square ware forward need be
 * 						considered to handle no load or light load condition.
 *
 * CALLED BY:	void GridCurrentController(void)
 *============================================================================*/
void BusVoltHysteresis(void)
{
	static float32 PfcDutyff_DCM = 0;

	CurrConReg.f32PfcDuty_ff = GetRealValue.f32VGrid * Calc_Result.Coff_Dforward;

	// when Grid voltage is positive
	if (BusCon_Reg.f32IGridAmp_Ref <= 2)
	{
		//'CurrConReg.f32PfcDuty_ff_factor' is the square wave amplitude
		CurrConReg.f32PfcDuty_ff_factor = 1 - 0.5 * BusCon_Reg.f32IGridAmp_Ref ;
		if (GridPLLConReg.Sin_Theta >= 0)
		{
			/*
			 * 'PWM_HALF_PERIOD' is the PWM counting value.
			 * 'PfcDutyff_DCM' is the square wave forward
			 */
			PfcDutyff_DCM = CurrConReg.f32PfcDuty_ff_factor * PWM_HALF_PERIOD;

			//The larger one will be the final forward value
			if (PfcDutyff_DCM > CurrConReg.f32PfcDuty_ff)
				CurrConReg.f32PfcDuty_ff = PfcDutyff_DCM;
		}
		else // when Grid voltage is negative
		{
			PfcDutyff_DCM = CurrConReg.f32PfcDuty_ff_factor * -1 * PWM_HALF_PERIOD;
			if(PfcDutyff_DCM < CurrConReg.f32PfcDuty_ff)
				CurrConReg.f32PfcDuty_ff = PfcDutyff_DCM;
		}
	}
}

/*=============================================================================*
 * FUNCTION:	void BusVoltBalController(void)
 *
 * PURPOSE:	Neutral Point Balance controller. The difference of 'Calc_Result.f32VBusP'
 * 						and 'Calc_Result.f32VBusN' is the controlled variable
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void BusVoltBalController(void)
{
	// start of BusVoltBalController

	float32 BusBal_Kp = 0.03f;
	float32 BusBal_Ki = 0.00002;

    BusCon_Reg.f32BusVoltDiffErr_Old = BusCon_Reg.f32BusVoltDiffErr_New;
	BusCon_Reg.f32BusVoltDiffErr_New = Calc_Result.f32VBusP - Calc_Result.f32VBusN;

  	if(BusCon_Reg.f32BusVoltDiffErr_New  > 10)
  	{
  		BusCon_Reg.f32BusVoltDiffErr_New = 10;
  	}
	else if(BusCon_Reg.f32BusVoltDiffErr_New  < -10)
  	{
	  	BusCon_Reg.f32BusVoltDiffErr_New  = -10;
	}

	BusCon_Reg.f32BusVoltDiff_Out = BusCon_Reg.f32BusVoltDiff_Out + (BusBal_Kp + BusBal_Ki) * \
			BusCon_Reg.f32BusVoltDiffErr_New - BusBal_Kp * BusCon_Reg.f32BusVoltDiffErr_Old;

    if (BusCon_Reg.f32BusVoltDiff_Out >=  10)
    {
        BusCon_Reg.f32BusVoltDiff_Out = 10;
    }
    else if ( BusCon_Reg.f32BusVoltDiff_Out <= -10 )
    {
        BusCon_Reg.f32BusVoltDiff_Out = -10;
    }

} // end of BusVoltBalController

/*=============================================================================*
 * FUNCTION:	void Calc_IGrid_reference(void)
 *
 * PURPOSE:	PI controller for voltage control. The output of voltage controller is the amplitude of input
 * 						current reference.
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void Calc_IGrid_reference(void)
{
	//start of Calc_IGrid_reference

	static float32 Bus_Kp = 0.2f;
	static float32 Bus_Ki = 0.001f;

	BusCon_Reg.f32BusFliter = 0.1f * (GetRealValue.f32VBusP + GetRealValue.f32VBusN) + \
			0.9f * BusCon_Reg.f32BusFliter;
	BusCon_Reg.f32BusVoltErr_Old = BusCon_Reg.f32BusVoltErr_New;
	BusCon_Reg.f32BusVoltErr_New = BusCon_Reg.f32BusVolt_Cmd - BusCon_Reg.f32BusFliter;

  	if(BusCon_Reg.f32BusVoltErr_New  > 10)
  		BusCon_Reg.f32BusVoltErr_New = 10;
	else if(BusCon_Reg.f32BusVoltErr_New  < -10)
	  	BusCon_Reg.f32BusVoltErr_New  = -10;

	BusCon_Reg.f32IGridAmp_Ref = BusCon_Reg.f32IGridAmp_Ref + (Bus_Kp + Bus_Ki) * \
			BusCon_Reg.f32BusVoltErr_New - Bus_Kp * BusCon_Reg.f32BusVoltErr_Old;

    if (BusCon_Reg.f32IGridAmp_Ref >= BusCon_Reg.f32IGrid_RefAmp_Restrict)
        BusCon_Reg.f32IGridAmp_Ref = BusCon_Reg.f32IGrid_RefAmp_Restrict;
    else if ( BusCon_Reg.f32IGridAmp_Ref <= 0 )
        BusCon_Reg.f32IGridAmp_Ref = 0;

} // end of Calc_IGrid_reference

/*=============================================================================*
 * FUNCTION:	 void GridPLLcontroller(void)
 *
 * PURPOSE:	Phase lock controller of Grid voltage
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
 void GridPLLcontroller(void)
{ 
	 // start of  Phase Lock Loop controller
	 static int i = 0;
	 static int Phase_Check_signal = 0;


	 float32 GridPLL_Kp = 5e-5f;
	 float32 GridPLL_Ki = 2.5e-8f;
	 float32 LPF_B0 = 0.00034605;		//0.000346050740714038155762;
	 float32 LPF_B1 = 0.00069210;		//0.000692101481428076311525;
	 float32 LPF_B2 = 0.00034605;		//0.000346050740714038155762;
	 float32 LPF_A1 = 1.94721182;		//1.947211823488243620516869;
	 float32 LPF_A2 = 0.94859602;		//0.948596026451099749721152;
	 float32 DELTA_ANGLE = 17.279e-3;	//2*pi*55/20000

	 GridPLLConReg.f32Valpha = GetRealValue.f32VGrid;
	 i++;

	 //The following is the different equation of a second order low pass filter
	 if (4 == i)
	 {
	 	GridPLLReg.Input[2] = GridPLLReg.Input[1];																	// x(k-2) = x(k-1)
	 	GridPLLReg.Input[1] = GridPLLReg.Input[0];																	// x(k-1) = x(k)
	 	GridPLLReg.Input[0] = GridPLLConReg.f32Valpha * GridPLLConReg.Cos_Theta;		// x(k)

	 	GridPLLReg.Output[2] = GridPLLReg.Output[1];															// y(k-2) = y(k-1)
	 	GridPLLReg.Output[1] = GridPLLReg.Output[0];															// y(k-1) = y(k)

	 	GridPLLReg.MAC = LPF_B0 * GridPLLReg.Input[0];														// + b0 * x(k)
	 	GridPLLReg.MAC += LPF_B1 * GridPLLReg.Input[1];														// + b1 * x(k-1)
	 	GridPLLReg.MAC += LPF_B2 * GridPLLReg.Input[2];														// + b2 * x(k-2)

	 	GridPLLReg.MAC += LPF_A1 * GridPLLReg.Output[1];													// + a11 * y(k-1)
	 	GridPLLReg.MAC -= LPF_A2 * GridPLLReg.Output[2];													// - a2 * y(k-2)

	 	GridPLLReg.Output[0] = GridPLLReg.MAC;
	 	i = 0;

	 	/*
	 	 * If  'GridPLLReg.Output[0]' can be small enough, the phase is successful locked
	 	 */
	 	if ( GridPLLReg.Output[0] < 40 && GridPLLReg.Output[0] > -40 && g_StateCheck.bit.Grid_PhaseLock == 0)  //2017.8.14 GX
	 		Phase_Check_signal ++;
	 	else
	 		Phase_Check_signal = 0;
	 	if ( Phase_Check_signal >200 )
	 		g_StateCheck.bit.Grid_PhaseLock = 1;
	 }

	 // The PI regulator
	 GridPLLReg.f32PIDErr_Old = GridPLLReg.f32PIDErr_New ;
	 GridPLLReg.f32PIDErr_New  = GridPLLReg.Output[0] - GridPLLReg.f32Refer;															// error(n) = y(k) - 0
	 GridPLLReg.f32PID_Output = GridPLLReg.f32PID_Output \
	 					+ (GridPLL_Kp + GridPLL_Ki) * GridPLLReg.f32PIDErr_New \
	 					- GridPLL_Kp * GridPLLReg.f32PIDErr_Old;               	 // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)
	 /*
	 * 'GridPLLReg.f32Delta_Theta' is the angle step which needs to be accumulated in each ADC interruption.
	 * 'DELTA_ANGLE' is the bias(55Hz), which can acceleration regulating process
	 */
	 GridPLLReg.f32Delta_Theta =  DELTA_ANGLE + GridPLLReg.f32PID_Output;

	 if(GridPLLReg.f32Delta_Theta > GridTheta_Step_Hi_Limit)	  //60 * 1.15Hz
	 {
	 	GridPLLReg.f32Delta_Theta = GridTheta_Step_Hi_Limit;
	 }
	 if(GridPLLReg.f32Delta_Theta < GridTheta_Step_Low_Limit)	//50 * 0.85Hz
	 {
	 	GridPLLReg.f32Delta_Theta = GridTheta_Step_Low_Limit;
	 }
	 GridPLLConReg.f32Theta_Step = GridPLLReg.f32Delta_Theta;

	 GridPLLReg.f32Theta += GridPLLReg.f32Delta_Theta;
	 if ( GridPLLReg.f32Theta > Value_2Pi )
	 {
	 	GridPLLReg.f32Theta = GridPLLReg.f32Theta - Value_2Pi;
	 	g_StateCheck.bit.Grid_Zero_Crossing_Flag = 1;
	 }

	 sincos((GridPLLReg.f32Theta), &(GridPLLConReg.Sin_Theta), &(GridPLLConReg.Cos_Theta));
} // end of  Phase Lock Loop controller

 /*=============================================================================*
  * FUNCTION:	void BusVoltSlowup(void)
  *
  * PURPOSE:		Bus voltage slow up control
  *
  * CALLED BY:	void ADC_INT_PFC_Control(void)
  *============================================================================*/
void BusVoltSlowup(void)
{
	if(NormalState == g_Sys_Current_State)
	{
		if(BusCon_Reg.f32BusVolt_Cmd < BusCon_Reg.f32BusVolt_Ref)
		{
			BusCon_Reg.f32BusVolt_Cmd = BusCon_Reg.f32BusVolt_Cmd + 2;
		}
		else
		{
			BusCon_Reg.f32BusVolt_Cmd = BusCon_Reg.f32BusVolt_Ref;
			g_StateCheck.bit.PfcSoftStart = 1;
		}
	}
}

/*=============================================================================*
 * FUNCTION:	void RectifierStage_Init(void)
 *
 * PURPOSE:	Bus voltage slow up control
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void RectifierStage_Init(void)
{
	// start of RectifierStage_Init

	//1.38 was tested to fulfill both start time and rush current
	BusCon_Reg.f32BusVolt_Cmd = Calc_Result.f32VGrid_rms * 1.38 * 2;

	BusCon_Reg.f32BusVoltErr_Old = 0;
	BusCon_Reg.f32BusVoltErr_New = 0;
    BusCon_Reg.f32IGridAmp_Ref = 0;

	BusCon_Reg.f32BusVoltDiffErr_New = 0;
	BusCon_Reg.f32BusVoltDiffErr_Old = 0;
	BusCon_Reg.f32BusVoltDiff_Out = 0;

	CurrConReg.f32IGridErr_Old = 0;
	CurrConReg.f32IGridErr_New = 0;
	CurrConReg.f32PfcDuty_Con = 0;
	CurrConReg.Driveopen = 0;
	CurrConReg.f32PfcDuty_ff_factor = 1;

	g_StateCheck.bit.PfcSoftStart = 0;
	g_StateCheck.bit.Grid_PhaseLock = 0;

} // end of RectifierStage_Init



int16 swGetStartADIsrPoint(void)
{
    return(0);
}

int16 swGetEndCONPoint(void) 
{
    return(0);
}

int16 swGetEndADIsrPoint(void)
{
    return(0);
}

//--- end of file -----------------------------------------------------

