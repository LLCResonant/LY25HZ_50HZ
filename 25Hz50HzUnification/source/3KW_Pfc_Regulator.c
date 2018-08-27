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
struct	CURRENTCONTREG	       			CurrConReg;
struct	PLLCONTREG							GridPLLConReg;
struct 	VOLTAGE_REVISE_REG 			Output_VoltRe_Reg;

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
void VGridFeedForwardCntl(void);

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
  		if (CurrConReg.u8Drive_Open == 0 && g_StateCheck.bit.Grid_Zero_Crossing_Flag == 1)
  		{
  			CurrConReg.f32PfcDuty_ff = 0;
  			CurrConReg.u8Drive_Open = 1;
  			//send a signal to CPLD. The input current protection is set to be a Cycle-by-cycle protection
  			GRID_OCP_CBC_ON;
  		}

  		VGrid_Dip_Reset();

		if (CurrConReg.u8Drive_Open >= 1 )
  		{
			Calc_IGrid_reference();
			BusVoltBalController();
			GridCurrentController();
  		}
 	}
	#endif

	#ifdef PFC_OPEN_LOOP
  	GridCurrentController();
  	CurrConReg.u8Drive_Open = 1;
	#endif

	ADAccGridCalc();
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
	static Uint8 u8PFCPWM = 0;

	if(Calc_Result.f32VGrid_rms_instant <= 100)
	{
		PfcPWMOutputsDisable();
		CurrConReg.u8Drive_Open = 0;
		u8PFCPWM = 1;
		g_StateCheck.bit.VGridDip_Disable_GridOCP = 1;
	}
	else if(Calc_Result.f32VGrid_rms_instant > 100 && u8PFCPWM == 1)
	{
		BusCon_Reg.f32BusVoltDiffErr_New = 0;
		BusCon_Reg.f32BusVoltDiffErr_Old = 0;
		BusCon_Reg.f32BusVoltDiff_Out = 0;

		CurrConReg.f32IGridErr_Old = 0;
		CurrConReg.f32IGridErr_New = 0;
		CurrConReg.f32PfcDuty_Con = 0;

		CurrConReg.u8Drive_Open = 1;
		u8PFCPWM = 0;
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
	static Uint8  u8BusOVP = 0;
	/*
	 * 'BusCon_Reg.f32IGridAmp_Ref * GridPLLConReg.Sin_Theta' is the original current reference
	 * 'BusCon_Reg.f32BusVoltDiff_Out' is the Neutral Point Balance superposition
	 * '2 * 3.14 * Calc_Result.f32GridFreq * 0.000005 * Calc_Result.f32VGrid_rms * 1.414 * GridPLLConReg.Cos_Theta'
	 * is the input capacitor current compensation
	 */
	CurrConReg.f32IGrid_Ref = BusCon_Reg.f32IGridAmp_Ref * GridPLLConReg.f32Sin_Theta - BusCon_Reg.f32BusVoltDiff_Out \
			- 2 * 3.14 * Calc_Result.f32GridFreq * 0.000005 * Calc_Result.f32VGrid_rms * 1.414 * GridPLLConReg.f32Cos_Theta;
	CurrConReg.f32IGrid_Fdb = GetRealValue.f32IGrid;

	CurrConReg.f32IGridErr_Old = CurrConReg.f32IGridErr_New;
	CurrConReg.f32IGridErr_New = CurrConReg.f32IGrid_Ref - CurrConReg.f32IGrid_Fdb;

	//PI calculating process
	CurrConReg.f32PfcDuty_Con = CurrConReg.f32PfcDuty_Con + (CurrConReg.f32Kp + CurrConReg.f32Ki) * CurrConReg.f32IGridErr_New - CurrConReg.f32Kp * CurrConReg.f32IGridErr_Old;

	if (CurrConReg.u8Drive_Open == 2)
		VGridFeedForwardCntl();

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

	if (CurrConReg.u8Drive_Open == 1) //2017.4.18 GX not confirmed
	{
		CurrConReg.u8Drive_Open = 2;
		PfcPWMOutputsEnable();
	}
	else if (CurrConReg.u8Drive_Open >= 1)
	{
		/*
		 * When Bus voltage increase unusually, the drive should be closed.
		 * When the voltage is back to normal, the drive should be opened and some variables should be clear.
		 */
		if((GetRealValue.f32VBusP+GetRealValue.f32VBusN) >= 900 && u8BusOVP == 0)
		{
			PfcPWMOutputsDisable();
			u8BusOVP = 1;
			g_StateCheck.bit.PwmForceOffFlag = 1;
		}
		else if((GetRealValue.f32VBusP+GetRealValue.f32VBusN) <= 870 && u8BusOVP == 1)
		{
			CurrConReg.f32PfcDuty_Con=0;
	        CurrConReg.f32IGridErr_New=0;
	        CurrConReg.f32IGridErr_Old=0;
	        BusCon_Reg.f32BusVoltDiffErr_New = 0;
	        BusCon_Reg.f32BusVoltDiffErr_Old = 0;
	        BusCon_Reg.f32BusVoltDiff_Out = 0;
	        CurrConReg.u8Drive_Open = 1;
	        u8BusOVP = 0;
		}
	}
} // end of CurrentPIDcontroller

/*=============================================================================*
 * FUNCTION:	void VGridFeedForwardCntl(void)
 *
 * PURPOSE:	No load or light load controller.
 * 						'GetRealValue.f32VGrid * Calc_Result.f32Coff_Dforward' is the normal forward value,
 * 						which is the sine ware in time domain.
 * 						When  the output of voltage loop is smaller than 2, a square ware forward need be
 * 						considered to handle no load or light load condition.
 *
 * CALLED BY:	void GridCurrentController(void)
 *============================================================================*/
void VGridFeedForwardCntl(void)
{
	static float32 PfcDutyff_DCM = 0;

	CurrConReg.f32PfcDuty_ff = GetRealValue.f32VGrid * Calc_Result.f32Coff_Dforward;

	// when Grid voltage is positive
	if (BusCon_Reg.f32IGridAmp_Ref <= 2)
	{
		//'CurrConReg.f32PfcDuty_ff_factor' is the square wave amplitude
		CurrConReg.f32PfcDuty_ff_factor = 1 - 0.5 * BusCon_Reg.f32IGridAmp_Ref ;
		if (GridPLLConReg.f32Sin_Theta >= 0)
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

	BusCon_Reg.f32BusVoltDiff_Out = BusCon_Reg.f32BusVoltDiff_Out + (BusCon_Reg.f32BusBal_Kp + BusCon_Reg.f32BusBal_Ki) * \
			BusCon_Reg.f32BusVoltDiffErr_New - BusCon_Reg.f32BusBal_Kp * BusCon_Reg.f32BusVoltDiffErr_Old;

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
	BusCon_Reg.f32BusFliter = 0.1f * (GetRealValue.f32VBusP + GetRealValue.f32VBusN) + \
			0.9f * BusCon_Reg.f32BusFliter;
	BusCon_Reg.f32BusVoltErr_Old = BusCon_Reg.f32BusVoltErr_New;
	BusCon_Reg.f32BusVoltErr_New = BusCon_Reg.f32BusVolt_Cmd - BusCon_Reg.f32BusFliter;

  	if(BusCon_Reg.f32BusVoltErr_New  > 10)
  		BusCon_Reg.f32BusVoltErr_New = 10;
	else if(BusCon_Reg.f32BusVoltErr_New  < -10)
	  	BusCon_Reg.f32BusVoltErr_New  = -10;

	BusCon_Reg.f32IGridAmp_Ref = BusCon_Reg.f32IGridAmp_Ref + (BusCon_Reg.f32Bus_Kp + BusCon_Reg.f32Bus_Ki) * \
			BusCon_Reg.f32BusVoltErr_New - BusCon_Reg.f32Bus_Kp * BusCon_Reg.f32BusVoltErr_Old;

    if (BusCon_Reg.f32IGridAmp_Ref >= BusCon_Reg.f32IGrid_RefAmp_Limit  )
        BusCon_Reg.f32IGridAmp_Ref = BusCon_Reg.f32IGrid_RefAmp_Limit  ;
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
	 static Uint8 	u8cnt = 0;
	 static Uint16 	u16Phase_Check_signal = 0;

	 GridPLLConReg.f32Valpha = GetRealValue.f32VGrid;
	 u8cnt ++;

	 //The following is the different equation of a second order low pass filter
	 if (4 == u8cnt )
	 {
	 	GridPLLConReg.f32Input[2] = GridPLLConReg.f32Input[1];																	// x(k-2) = x(k-1)
	 	GridPLLConReg.f32Input[1] = GridPLLConReg.f32Input[0];																	// x(k-1) = x(k)
	 	GridPLLConReg.f32Input[0] = GridPLLConReg.f32Valpha * GridPLLConReg.f32Cos_Theta;		// x(k)

	 	GridPLLConReg.f32Output[2] = GridPLLConReg.f32Output[1];															// y(k-2) = y(k-1)
	 	GridPLLConReg.f32Output[1] = GridPLLConReg.f32Output[0];															// y(k-1) = y(k)

	 	GridPLLConReg.f32MAC = LPF_B0_GRID * GridPLLConReg.f32Input[0];														// + b0 * x(k)
	 	GridPLLConReg.f32MAC += LPF_B1_GRID * GridPLLConReg.f32Input[1];														// + b1 * x(k-1)
	 	GridPLLConReg.f32MAC += LPF_B2_GRID * GridPLLConReg.f32Input[2];														// + b2 * x(k-2)

	 	GridPLLConReg.f32MAC += LPF_A1_GRID * GridPLLConReg.f32Output[1];													// + a11 * y(k-1)
	 	GridPLLConReg.f32MAC -= LPF_A2_GRID * GridPLLConReg.f32Output[2];													// - a2 * y(k-2)

	 	GridPLLConReg.f32Output[0] = GridPLLConReg.f32MAC;
	 	u8cnt  = 0;

	 	/*
	 	 * If  'GridPLLConReg.f32Output[0]' can be small enough, the phase is successful locked
	 	 */
	 	if ( GridPLLConReg.f32Output[0] < 40 && GridPLLConReg.f32Output[0] > -40 && g_StateCheck.bit.Grid_PhaseLock == 0)  //2017.8.14 GX
	 		u16Phase_Check_signal ++;
	 	else
	 		u16Phase_Check_signal = 0;
	 	if ( u16Phase_Check_signal >200 )
	 		g_StateCheck.bit.Grid_PhaseLock = 1;
	 }

	 // The PI regulator
	 GridPLLConReg.f32PIDErr_Old = GridPLLConReg.f32PIDErr_New ;
	 GridPLLConReg.f32PIDErr_New  = GridPLLConReg.f32Output[0] - GridPLLConReg.f32Refer;															// error(n) = y(k) - 0
	 GridPLLConReg.f32PID_Output = GridPLLConReg.f32PID_Output \
	 					+ (GridPLLConReg.f32Kp + GridPLLConReg.f32Ki) * GridPLLConReg.f32PIDErr_New \
	 					- GridPLLConReg.f32Kp * GridPLLConReg.f32PIDErr_Old;               	 // u(n) = u(n-1) + alpha * error(n) - beta * error(n-1)
	 /*
	 * 'GridPLLConReg.f32Theta_Step' is the angle step which needs to be accumulated in each ADC interruption.
	 * 'DELTA_ANGLE' is the bias(55Hz), which can acceleration regulating process
	 */
	 GridPLLConReg.f32Theta_Step =  DELTA_ANGLE_GRID + GridPLLConReg.f32PID_Output;

	 if(GridPLLConReg.f32Theta_Step > GridTheta_Step_Hi_Limit)	  //60 * 1.15Hz
	 {
	 	GridPLLConReg.f32Theta_Step = GridTheta_Step_Hi_Limit;
	 }
	 if(GridPLLConReg.f32Theta_Step < GridTheta_Step_Low_Limit)	//50 * 0.85Hz
	 {
	 	GridPLLConReg.f32Theta_Step = GridTheta_Step_Low_Limit;
	 }

	 GridPLLConReg.f32Theta += GridPLLConReg.f32Theta_Step;
	 if ( GridPLLConReg.f32Theta > Value_2Pi )
	 {
	 	GridPLLConReg.f32Theta = GridPLLConReg.f32Theta - Value_2Pi;
	 	g_StateCheck.bit.Grid_Zero_Crossing_Flag = 1;
	 }

	 sincos((GridPLLConReg.f32Theta), &(GridPLLConReg.f32Sin_Theta), &(GridPLLConReg.f32Cos_Theta));
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
	CurrConReg.u8Drive_Open = 0;
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

