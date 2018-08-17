/*=============================================================================*
 *       
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_DataAcquisition.c
 *
 *  PURPOSE  : Data acquisition file of 28335 for 3KW Parallel Inverter.
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *                	    Created
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
#pragma CODE_SECTION(Get_ADC_Result1, "ControlLoopInRAM")
#pragma CODE_SECTION(Get_ADC_Result2, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccGridCalc, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccInvCalc, "ControlLoopInRAM")

/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
struct	AD_Sample_Reg1	GeneralADbuffer, GetRealValue, ADGain,ADChannelOffset, ADCorrection;
struct	AD_ACC_Reg1	AD_Acc, AD_Sum, Calc_Result;
struct FanCntl_REG  FanControl_Reg;
float32	f32SumCounterReci;
float32	f32SumCounterInv;  //ZR 11.23
int16 i16Cnt_SysParaTemp = 0;   
extern 	 float32 f32TempTab[];

/*=============================================================================*
 * functions declaration
 *============================================================================*/
void Get_ADC_Result1();
void Get_ADC_Result2();
void ADAccGridCalc();
void ADAccInvCalc();
void TSK_GridPeriod();
void TSK_InvVoltPeriod();

void GridCurrentPI();
void GridIRefAmp();

//local function
void GridCurrentsAveCalc();
void InvCurrentsAveCalc();
void OutVoltsAveCalc();
void GridVoltsAveCalc();
void InvVoltsAveCalc();
void InvVoltsDutyAveCalc();
   				
void GridCurrentsRMSCalc();
void InvCurrentsRMSCalc();

void GridVoltsRMSCalc();
void InvVoltsRMSCalc();
void OutVoltsRMSCalc();
void BusVoltsCalc();

void OverTemperatureLimit(void);
void GridFrequencyCalc();
void OutFrequencyCalc();
void InvTempCalc();
void PFCTempCalc();

void FanCntl();

/*=============================================================================*
 * FUNCTION:	void Get_ADC_Result1(void)
 *
 * PURPOSE:	Get the PFC sample data from the ADC module of DSP28335
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void Get_ADC_Result1(void)
{
	 // start of function
	GeneralADbuffer.f32VGrid = (float32)AdcMirror.ADCRESULT1 - ADDefaultACOffset;        					// B0 result
	GetRealValue.f32VGrid = GeneralADbuffer.f32VGrid * ADGain.f32VGrid * ADCorrection.f32VGrid - ADChannelOffset.f32VGrid;
	GeneralADbuffer.f32IGrid = (float32)AdcMirror.ADCRESULT0 - ADDefaultACOffset;	                        // A0 result
	GetRealValue.f32IGrid = -(GeneralADbuffer.f32IGrid * ADGain.f32IGrid * ADCorrection.f32IGrid) - ADChannelOffset.f32IGrid;

	GeneralADbuffer.f32VBusP = (float32)AdcMirror.ADCRESULT2;	                        // A1 result
	GetRealValue.f32VBusP = GeneralADbuffer.f32VBusP * ADGain.f32VBusP * ADCorrection.f32VBusP - ADChannelOffset.f32VBusP;
	GeneralADbuffer.f32VBusN = (float32)AdcMirror.ADCRESULT4;	                        // A2 result
	GetRealValue.f32VBusN = GeneralADbuffer.f32VBusN * ADGain.f32VBusN * ADCorrection.f32VBusN - ADChannelOffset.f32VBusN;

	GeneralADbuffer.f32TempPFC = (float32)AdcMirror.ADCRESULT9;											// B5 result
	GetRealValue.f32TempPFC = ADCorrection.f32TempPFC * GeneralADbuffer.f32TempPFC;
}

/*=============================================================================*
 * FUNCTION:	void Get_ADC_Result2(void)
 *
 * PURPOSE:	Get the INV sample data from the ADC module of DSP28335
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void Get_ADC_Result2(void)//ZJX changed
{  // start of function
	GeneralADbuffer.f32IInv = (float32)AdcMirror.ADCRESULT6 - ADDefaultACOffset;        					// A3 result
	GetRealValue.f32IInv = GeneralADbuffer.f32IInv * ADGain.f32IInv * ADCorrection.f32IInv - ADChannelOffset.f32IInv;
	GeneralADbuffer.f32VInv = (float32)AdcMirror.ADCRESULT10 - ADDefaultACOffset;	                        // A5 result
	GetRealValue.f32VInv = GeneralADbuffer.f32VInv * ADGain.f32VInv * ADCorrection.f32VInv * InvVoltConReg.f32Drop_Coeff - ADChannelOffset.f32VInv;
	GeneralADbuffer.f32VOut = (float32)AdcMirror.ADCRESULT8 - ADDefaultACOffset;	                        // A4 result
	GetRealValue.f32VOut = -(GeneralADbuffer.f32VOut * ADGain.f32VOut * ADCorrection.f32VOut - ADChannelOffset.f32VOut);

	GeneralADbuffer.f32TempInv = (float32)AdcMirror.ADCRESULT11;											// B4 result
	GetRealValue.f32TempInv = ADCorrection.f32TempInv * GeneralADbuffer.f32TempInv;
}
/*=============================================================================*
 * FUNCTION:	void ADAccGridCalc(void)
 *
 * PURPOSE:	Accumulation of PFC sample data including Grid voltage and current, Bus voltage, temperature
 *						and frequency every Grid voltage period
 *
 * CALLED BY:	void ADC_INT_PFC_Control(void)
 *============================================================================*/
void ADAccGridCalc(void)
{ 	
	AD_Acc.f32VBusP += GetRealValue.f32VBusP;
	AD_Acc.f32VBusN += GetRealValue.f32VBusN;
	AD_Acc.f32TempPFC += GetRealValue.f32TempPFC;

	if (1 == g_StateCheck.bit.GridAD_initial)
	{
		AD_Acc.f32VGrid_ave += GetRealValue.f32VGrid;
		AD_Acc.f32IGrid_ave += GetRealValue.f32IGrid;
	}
	AD_Acc.f32IGrid_rms += GetRealValue.f32IGrid * GetRealValue.f32IGrid;
	AD_Acc.f32VGrid_rms += GetRealValue.f32VGrid * GetRealValue.f32VGrid;
    AD_Acc.f32GridFreq += (CoffStepToFre * GridPLLConReg.f32Theta_Step);
   
	AD_Acc.i16GridCounter++;

	if ( 1== g_StateCheck.bit.Grid_Zero_Crossing_Flag)
	{
		g_StateCheck.bit.Grid_Zero_Crossing_Flag = 0;

		AD_Sum.i16GridCounter = AD_Acc.i16GridCounter;
		AD_Sum.f32TempPFC = AD_Acc.f32TempPFC;
		AD_Sum.f32VBusP = AD_Acc.f32VBusP;
		AD_Sum.f32VBusN = AD_Acc.f32VBusN;
		AD_Sum.f32GridFreq = AD_Acc.f32GridFreq;

		if (1 == g_StateCheck.bit.GridAD_initial)
		{
			AD_Sum.f32IGrid_ave = AD_Acc.f32IGrid_ave;
			AD_Sum.f32VGrid_ave = AD_Acc.f32VGrid_ave;
		}

		AD_Sum.f32IGrid_rms = AD_Acc.f32IGrid_rms;
		AD_Sum.f32VGrid_rms = AD_Acc.f32VGrid_rms;

		AD_Acc.i16GridCounter = 0;
		AD_Acc.f32VBusP = 0;
		AD_Acc.f32VBusN = 0;

		AD_Acc.f32GridFreq = 0;
		AD_Acc.f32TempPFC = 0;

		if (1 == g_StateCheck.bit.GridAD_initial)
		{
			AD_Acc.f32IGrid_ave = 0;
			AD_Acc.f32VGrid_ave = 0;
		}

		AD_Acc.f32IGrid_rms = 0;
		AD_Acc.f32VGrid_rms = 0;

		SEM_post(&SEM_GridPeriod);
	}// end of if zero crossing flag
}

/*=============================================================================*
 * FUNCTION:	void ADAccInvCalc(void)
 *
 * PURPOSE:	Accumulation of INV sample data including voltage, current and temperature of INV ,
 * 						load voltage of INV and frequency every output period.
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void ADAccInvCalc(void)
{
	AD_Acc.f32TempInv += GetRealValue.f32TempInv;

	if (1 == g_StateCheck.bit.InvAD_initial)
	{
		AD_Acc.f32IInv_ave += GetRealValue.f32IInv;
		AD_Acc.f32VInv_ave += GetRealValue.f32VInv;
		AD_Acc.f32VOut_ave += GetRealValue.f32VOut;
		AD_Acc.f32VDutyD_ave += InvCurrConReg.f32InvDuty;//WF 2018.07.26
	}

	AD_Acc.f32IInv_rms += GetRealValue.f32IInv * GetRealValue.f32IInv;
	AD_Acc.f32VInv_rms += GetRealValue.f32VInv * GetRealValue.f32VInv;
	AD_Acc.f32VOut_rms += GetRealValue.f32VOut * GetRealValue.f32VOut;

    AD_Acc.f32VOutFreq += (CoffStepToFre * OutPLLConReg.f32Theta_Step);
    AD_Acc.f32VOutLdFreq += (CoffStepToFre * VOutPLLConReg.f32Theta_Step);
	AD_Acc.i16InvCounter++;

	if ( 1== g_StateCheck.bit.Inv_Zero_Crossing_Flag)
	{
		g_StateCheck.bit.Inv_Zero_Crossing_Flag = 0;

		AD_Sum.i16InvCounter = AD_Acc.i16InvCounter;
		AD_Sum.f32TempInv = AD_Acc.f32TempInv;
		AD_Sum.f32VOutFreq = AD_Acc.f32VOutFreq;
		AD_Sum.f32VOutLdFreq = AD_Acc.f32VOutLdFreq;

		if (1 == g_StateCheck.bit.InvAD_initial)
		{
			AD_Sum.f32IInv_ave = AD_Acc.f32IInv_ave;
			AD_Sum.f32VInv_ave = AD_Acc.f32VInv_ave;
			AD_Sum.f32VOut_ave = AD_Acc.f32VOut_ave;
			AD_Sum.f32VDutyD_ave = AD_Acc.f32VDutyD_ave; //WF 2018.07.26
		}

		AD_Sum.f32IInv_rms = AD_Acc.f32IInv_rms;
		AD_Sum.f32VInv_rms = AD_Acc.f32VInv_rms;
		AD_Sum.f32VOut_rms = AD_Acc.f32VOut_rms;

		AD_Acc.i16InvCounter = 0;
		AD_Acc.f32TempInv = 0;
		AD_Acc.f32VOutFreq = 0;
		AD_Acc.f32VOutLdFreq = 0;

		if (1 == g_StateCheck.bit.InvAD_initial)
		{
			AD_Acc.f32IInv_ave = 0;
			AD_Acc.f32VInv_ave = 0;
			AD_Acc.f32VOut_ave = 0;
			AD_Acc.f32VDutyD_ave = 0;//WF 2018.07.26
		}

		AD_Acc.f32IInv_rms = 0;
		AD_Acc.f32VInv_rms = 0;
		AD_Acc.f32VOut_rms = 0;

		SEM_post(&SEM_InvPeriod);
	}// end of if zero crossing flag
}

/*=============================================================================*
 * FUNCTION:	void TSK_GridPeriod(void)
 *
 * PURPOSE:	Calculate RMS and run software protection every Grid voltage period
 *
 * CALLED BY:	void ADAccGridCalc(void) using semaphore 'SEM_GridPeriod'
 *============================================================================*/
void TSK_GridPeriod(void)
{
	while(1)
	{ // while loop start
		if (SEM_pend(&SEM_GridPeriod, SYS_FOREVER) == TRUE )
		{	
			if (AD_Sum.i16GridCounter > 0)
			{
				f32SumCounterReci = 1/ (float32)AD_Sum.i16GridCounter;

				/*
				* ADC module of DSP is checked by checking average value of voltage and current
				* through 'ADOffsetCheck()'
				*/
				if (1 == g_StateCheck.bit.GridAD_initial)
				{
					GridCurrentsAveCalc();
					GridVoltsAveCalc();
					i16Cnt_SysParaTemp++;
					if (i16Cnt_SysParaTemp == 25)  //20ms * 25 = 500ms
					{		
						g_StateCheck.bit.GridAD_initial = 0;
						ADOffsetCheck();
						i16Cnt_SysParaTemp = 0;
					}
				}
				/*RMS Calculating function*/
				GridCurrentsRMSCalc();
				GridVoltsRMSCalc();
				BusVoltsCalc();

				PFCTempCalc();
				GridFrequencyCalc();

				/*Software protection function*/
			    GridVoltCheck();
				GridFreqCheck();
			    GridCurrentCheck();
			    BusVoltCheck();
			    BusBalanceCheck();

				/*Controller related function*/
		        GridCurrentPI();
				GridIRefAmp();
				BusVoltSlowup();
			}     
		}                   
	}//end of while (1) 
}

/*=============================================================================*
 * FUNCTION:	void TSK_InvVoltPeriod(void)
 *
 * PURPOSE:	Calculate RMS and run software protection every Grid voltage period
 *
 * CALLED BY:	void ADAccInvCalc(void) using semaphore 'SEM_InvPeriod'
 *============================================================================*/
void TSK_InvVoltPeriod(void)
{
	while(1)
	{ // while loop start
		if (SEM_pend(&SEM_InvPeriod, SYS_FOREVER) == TRUE )
		{
			if (AD_Sum.i16InvCounter > 0)
			{
				f32SumCounterInv = 1/ (float32)AD_Sum.i16InvCounter;

				if (1 == g_StateCheck.bit.InvAD_initial)
				{
					GridVoltsAveCalc();
					//InvVoltsAveCalc();  //Why
					OutVoltsAveCalc();
					InvVoltsDutyAveCalc();
				}
				/*RMS Calculating function*/
			    InvCurrentsRMSCalc();
				InvVoltsRMSCalc();
				OutVoltsRMSCalc();
				InvTempCalc();
				OutFrequencyCalc();

				/*Software protection function*/
			    InvCurrentCheck();
				InvVoltCheck();
			    InvFreqCheck();
          		InputPowerLimit();
          		InvSyncCheck();
          	    //OverTemperatureLimit();
         		OutputCurrentLimit();
				SyncLogic_Control();
				CurrentProtectionIdentify();

				InvVoltSlowup();
				InvRestartCheck();
			}
		}
	}//end of while (1)
}

/*=============================================================================*
 * FUNCTION:	void GridCurrentsRMSCalc(void)
 *
 * PURPOSE:	Calculate RMS
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridCurrentsRMSCalc(void)
{
	float32		f32temp1;
	Uint8		u8temp1 = 0;
	float32		f32temp3 = 0;
	static float32	f32temp2[100];
	static Uint8		u8_count = 0;
	static Uint8    u8_count2 = 0;

	if (u8_count2 == 0)
	{
		for (u8_count2 = 0; u8_count2 < 100; u8_count2++)
			f32temp2[u8_count2] = 0;
		u8_count2 = 1;
	}

	f32temp1 = sqrt(AD_Sum.f32IGrid_rms * f32SumCounterReci);
	f32temp1 = (Calc_Result.f32IGrid_rms * 0.4f) + (f32temp1 * 0.6f);	//Low pass filter algorithm

	Calc_Result.f32IGrid_rms = f32temp1;

	f32temp2[u8_count] = Calc_Result.f32IGrid_rms;
	u8_count++;
	if (u8_count >= 100)
		u8_count = 0;

	for (u8temp1 = 0; u8temp1 < 100; u8temp1++)
		f32temp3 += f32temp2[u8temp1];

	Calc_Result.f32IGrid_rms_ave = f32temp3 * 0.01;
}

/*=============================================================================*
 * FUNCTION:	void GridVoltsRMSCalc(void)
 *
 * PURPOSE:	Calculate RMS
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridVoltsRMSCalc(void)
{
	float32 f32temp1;
	Calc_Result.f32VGrid_rms_previous = sqrt(AD_Sum.f32VGrid_rms * f32SumCounterReci);
	f32temp1 = Calc_Result.f32VGrid_rms_previous;
	f32temp1 = (Calc_Result.f32VGrid_rms_shadow * 0.4f) + (f32temp1 * 0.6f);
	Calc_Result.f32VGrid_rms_shadow = f32temp1;

	/*
	 * Make the Grid voltage protection more difficult to protect. Enhance robustness.
	 */
	if (Calc_Result.f32VGrid_rms_shadow <= 170)
		Calc_Result.f32VGrid_rms = Calc_Result.f32VGrid_rms_shadow + 2;
	else if(Calc_Result.f32VGrid_rms_shadow >= 280)
		Calc_Result.f32VGrid_rms = Calc_Result.f32VGrid_rms_shadow - 2;
	else
		Calc_Result.f32VGrid_rms = Calc_Result.f32VGrid_rms_shadow;
}
/*=============================================================================*
 * FUNCTION:	void GridCurrentPI(void)
 *
 * PURPOSE:	Change PFC current loop PI parameter according to Grid voltage amplitude.
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridCurrentPI(void)
{
    if(Calc_Result.f32VGrid_rms <= 220)
	{
		CurrConReg.f32Kp = 10.0f;
		CurrConReg.f32Ki = 3.0f;
	}
	else if(Calc_Result.f32VGrid_rms > 220 && Calc_Result.f32VGrid_rms <= 225)
	{
		CurrConReg.f32Kp = CurrConReg.f32Kp;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if(Calc_Result.f32VGrid_rms >225 && Calc_Result.f32VGrid_rms <= 260)
	{
		CurrConReg.f32Kp = 30.0f;
		CurrConReg.f32Ki = 3.0f;
	}
	else if(Calc_Result.f32VGrid_rms >260 && Calc_Result.f32VGrid_rms <= 265)
	{
		CurrConReg.f32Kp = CurrConReg.f32Kp;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if ( Calc_Result.f32VGrid_rms >265 && Calc_Result.f32VGrid_rms <= 270)
	{
		CurrConReg.f32Kp = 25.0f;
		CurrConReg.f32Ki = 3.0f;
	}
	else if (Calc_Result.f32VGrid_rms > 270 && Calc_Result.f32VGrid_rms <= 280)
	{
		CurrConReg.f32Kp = 20.0f;
		CurrConReg.f32Ki = 3.0f;
	}
	else
	{
		CurrConReg.f32Kp = 20.0f;
		CurrConReg.f32Ki = 3.0f;
	}
}

/*=============================================================================*
 * FUNCTION:	void GridIRefAmp(void)
 *
 * PURPOSE:	Change PFC voltage loop PI output restriction according to Grid voltage amplitude.
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridIRefAmp(void)
{
	if(Calc_Result.f32VGrid_rms <= 195)
		BusCon_Reg.f32IGrid_RefAmp_Restrict = 38;//WF 2018.08.15  34initial          170V,1.3load protection
	else if(Calc_Result.f32VGrid_rms > 195 && Calc_Result.f32VGrid_rms <= 230)
		BusCon_Reg.f32IGrid_RefAmp_Restrict = 38;
	else if(Calc_Result.f32VGrid_rms > 230)
		BusCon_Reg.f32IGrid_RefAmp_Restrict = 34;
	else
		BusCon_Reg.f32IGrid_RefAmp_Restrict = 34;
}
/*=============================================================================*
 * FUNCTION:	void PFCTempCalc(void)
 *
 * PURPOSE:	Get temperature of PFC by table look-up scheme
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void PFCTempCalc(void)
{
	float32 f32temp1;
	float32 f32temp2;

	f32temp1 = (AD_Sum.f32TempPFC * f32SumCounterReci);
	f32temp2 = f32temp1 * 0.073242187f;

	f32temp2 = (Calc_Result.f32TempPFC * 0.7f) + (f32temp2 * 0.3f);
	Calc_Result.f32TempPFC = f32temp2;  //Why?
}

/*=============================================================================*
 * FUNCTION:	void GridCurrentsAveCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridCurrentsAveCalc(void)
{
    float32	f32temp1;

	f32temp1 = (AD_Sum.f32IGrid_ave  * f32SumCounterReci);
	f32temp1 = (Calc_Result.f32IGrid_ave * 0.7f) + (f32temp1 * 0.3f);

	Calc_Result.f32IGrid_ave = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void GridVoltsAveCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridVoltsAveCalc(void)
{
    float32	f32temp1;

	f32temp1 = (AD_Sum.f32VGrid_ave  * f32SumCounterReci);
	f32temp1 = (Calc_Result.f32VGrid_ave * 0.3f) + (f32temp1 * 0.7f);

	Calc_Result.f32VGrid_ave = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void BusVoltsCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void BusVoltsCalc(void)
{
    float32	f32temp1, f32temp2, f32temp3;

	f32temp1 = (AD_Sum.f32VBusP * f32SumCounterReci);
	f32temp2 = (AD_Sum.f32VBusN * f32SumCounterReci);

	f32temp1 = (Calc_Result.f32VBusP * 0.4f) + (f32temp1 * 0.6f);
	f32temp2 = (Calc_Result.f32VBusN * 0.4f) + (f32temp2 * 0.6f);

	Calc_Result.f32VBusP = f32temp1;
	Calc_Result.f32VBusN = f32temp2;
	Calc_Result.f32VBus = Calc_Result.f32VBusP + Calc_Result.f32VBusN;

	/*
	 * 'Calc_Result.Coff_Dforward' is related to the forward control of PFC current loop
	 * 'Calc_Result.Coff_Dforward2' was used to accelerate the inverter regulation speed in hold-up mode,
	 *	which is abandoned now.
	 */
	if(Calc_Result.f32VBus > 200)
	{
		f32temp3 = 1 / Calc_Result.f32VBus;
		Calc_Result.Coff_Dforward = 2 * PWM_HALF_PERIOD * f32temp3;
		//Calc_Result.Coff_Dforward2 = BusCon_Reg.f32BusVolt_Ref * f32temp3;
		//if (Calc_Result.Coff_Dforward2 < 1)
			//Calc_Result.Coff_Dforward2 = 1;
	}
	else
	{
		Calc_Result.Coff_Dforward = 0;
	}
}
/*=============================================================================*
 * FUNCTION:	void GridFrequencyCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridFrequencyCalc(void)
{
    float32	f32temp1;
	f32temp1 = (AD_Sum.f32GridFreq  * f32SumCounterReci);
	f32temp1 = (Calc_Result.f32GridFreq * 0.6f) + (f32temp1 * 0.4f);
	Calc_Result.f32GridFreq = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void InvCurrentsRMSCalc(void)
 *
 * PURPOSE:	Calculate RMS
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void InvCurrentsRMSCalc(void)
{
	float32 f32temp1;

	Calc_Result.f32IInv_rms_previous = sqrt(AD_Sum.f32IInv_rms * f32SumCounterInv);
	f32temp1 = (Calc_Result.f32IInv_rms * 0.4f) + (Calc_Result.f32IInv_rms_previous * 0.6f);	//Low pass filter algorithm
	Calc_Result.f32IInv_rms = f32temp1;

	Calc_Result.f32IOut_rms = sqrt((Calc_Result.f32IInv_rms + 2.5) * (Calc_Result.f32IInv_rms - 2.5)) * ADCorrection.f32IOut;
}

/*=============================================================================*
 * FUNCTION:	void OutVoltsRMSCalc(void)
 *
 * PURPOSE:	Calculate RMS
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void OutVoltsRMSCalc(void)
{
	float32  f32temp1;

	f32temp1 = sqrt(AD_Sum.f32VOut_rms * f32SumCounterInv);
	f32temp1 = (Calc_Result.f32VOut_rms * 0.4f) + (f32temp1 * 0.6f);

	Calc_Result.f32VOut_rms = f32temp1;
}

/*=============================================================================*
 * FUNCTION:	void InvVoltsRMSCalc(void)
 *
 * PURPOSE:	Calculate RMS
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void InvVoltsRMSCalc(void)
{
		Calc_Result.f32VInv_rms_previous = sqrt(AD_Sum.f32VInv_rms * f32SumCounterInv);
		Calc_Result.f32VInv_rms = (Calc_Result.f32VInv_rms * 0.4f) + (Calc_Result.f32VInv_rms_previous * 0.6f);
}
/*=============================================================================*
 * FUNCTION:	void InvTempCalc(void)
 *
 * PURPOSE:	Get temperature of INVH(220V) and INVL(110V) by table lookup scheme
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void InvTempCalc(void)
{
	float32 f32temp1;
	float32 f32temp2;

	f32temp1 = (AD_Sum.f32TempInv * f32SumCounterInv);
	f32temp2 = f32temp1 * 0.073242187f;

	f32temp2 = (Calc_Result.f32TempInv * 0.7f) + (f32temp2 * 0.3f);
	Calc_Result.f32TempInv = f32temp2;
}
/*=============================================================================*
 * FUNCTION:	void InvCurrentsAveCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void InvCurrentsAveCalc(void)
{
    float32	f32temp1;

	f32temp1 = (AD_Sum.f32IInv_ave  * f32SumCounterInv);
	f32temp1 = (Calc_Result.f32IInv_ave * 0.7f) + (f32temp1 * 0.3f);

	Calc_Result.f32IInv_ave = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void InvVoltsAveCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void InvVoltsAveCalc(void)
{
    float32	f32temp1;

	f32temp1 = (AD_Sum.f32VInv_ave  * f32SumCounterInv);
	f32temp1 = (Calc_Result.f32VInv_ave * 0.7f) + (f32temp1 * 0.3f);

	Calc_Result.f32VInv_ave = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void OutVoltsAveCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void OutVoltsAveCalc(void)
{
    float32	f32temp1;

	f32temp1 = (AD_Sum.f32VOut_ave  * f32SumCounterInv);
	f32temp1 = (Calc_Result.f32VOut_ave * 0.7f) + (f32temp1 * 0.3f);

	Calc_Result.f32VOut_ave = f32temp1;
}

/*=============================================================================*
 * FUNCTION:	void InvVoltsDutyAveCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void InvVoltsDutyAveCalc(void)
{
    float32	f32temp1;

	f32temp1 = (AD_Sum.f32VDutyD_ave  * f32SumCounterInv);
	f32temp1 = (Calc_Result.f32VDutyD_ave * 0.7f) + (f32temp1 * 0.3f);

	Calc_Result.f32VDutyD_ave = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void OutFrequencyCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void OutFrequencyCalc(void)
{
    float32	f32temp1, f32temp2;
	f32temp1 = (AD_Sum.f32VOutFreq  * f32SumCounterInv );
	f32temp1 = (Calc_Result.f32VOutFreq * 0.6f) + (f32temp1 * 0.4f);
	Calc_Result.f32VOutFreq = f32temp1;

	f32temp2 = (AD_Sum.f32VOutLdFreq  * f32SumCounterInv);
	f32temp2 = (Calc_Result.f32VOutLdFreq * 0.6f) + (f32temp2 * 0.4f);
	Calc_Result.f32VOutLdFreq = f32temp2;
}

/*=============================================================================*
 * FUNCTION:	void DcFanSpeedSense(void)
 *
 * PURPOSE:	Detect whether the fan is blocked
 *
 * CALLED BY:	void FanCntl(void)
 *============================================================================*/
void DcFanSpeedSense(void)
{
	static Uint16  s_u16Cnt_DcFan1_Hi_Level = 0 ;
	static Uint16  s_u16Cnt_DcFan1_Low_Level = 0;
	static Uint16  s_u16Cnt_DcFan2_Hi_Level = 0;
	static Uint16  s_u16Cnt_DcFan2_Low_Level = 0;
	static Uint16  s_u16Cnt_FanCtrl_Hi_Level = 0;
	static Uint16  s_u16Cnt_FanCtrl_Low_Level = 0;
    static Uint16  temp = 0;

    if (temp > 90)
    {

    	if (s_u16Cnt_DcFan1_Hi_Level <= 19 || s_u16Cnt_DcFan1_Low_Level <= 19)
    		g_StateCheck.bit.DcFan1Fault = 1;
    	else
    		g_StateCheck.bit.DcFan1Fault = 0;

    	if (s_u16Cnt_DcFan2_Hi_Level <= 19 || s_u16Cnt_DcFan2_Low_Level <= 19)
    		g_StateCheck.bit.DcFan2Fault = 1;
    	else
    		g_StateCheck.bit.DcFan2Fault = 0;

    	s_u16Cnt_DcFan1_Hi_Level = 0 ;
    	s_u16Cnt_DcFan1_Low_Level = 0;
    	s_u16Cnt_DcFan2_Hi_Level = 0;
    	s_u16Cnt_DcFan2_Low_Level = 0;
    	s_u16Cnt_FanCtrl_Low_Level = 0;
    	s_u16Cnt_FanCtrl_Hi_Level = 0;
    	temp = 0;
    }
    else
    {
    	temp ++;
    	if (1 == DC_FAN1_FB_Level)                 // state  1:high lever  ,0:low lever
    		s_u16Cnt_DcFan1_Hi_Level++;
        else                                                // state  1:high lever  ,0:low lever
            s_u16Cnt_DcFan1_Low_Level++;

    	if (1 == DC_FAN2_FB_Level)                 // state  1:high lever  ,0:low lever
    		s_u16Cnt_DcFan2_Hi_Level++;
    	else                                                // state  1:high lever  ,0:low lever
    		s_u16Cnt_DcFan2_Low_Level++;

    	if (1 == DC_Fan_State)                 // state  1:high lever  ,0:low lever
    		s_u16Cnt_FanCtrl_Hi_Level++;
    	else                                                // state  1:high lever  ,0:low lever
    		s_u16Cnt_FanCtrl_Low_Level++;
    }
}

/*=============================================================================*
 * FUNCTION:	void HwInvOCPDetection (void)
 *
 * PURPOSE:	INV(220V) hardware over current protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwInvOCPDetection (void)
{
	static Uint16 s_u16Cnt_OCP_AC_Level = 0 ;
	static Uint16 s_u16Cnt_OCP_AC_Level_Back = 0 ;

	if(0 == g_StateCheck.bit.OCP_Inv && Calc_Result.f32VInv_rms_previous <= 30)
	{
		if(0 == INV_OCP_LEVEL)
		{
			s_u16Cnt_OCP_AC_Level ++;
			if (s_u16Cnt_OCP_AC_Level >= 1)	  // 1*2ms=2ms
			{
				s_u16Cnt_OCP_AC_Level = 0;
				g_StateCheck.bit.OCP_Inv = 1;
			}
		}
		else
			s_u16Cnt_OCP_AC_Level = 0;
	}
	else
	{
		if(1 == INV_OCP_LEVEL)
		{
			s_u16Cnt_OCP_AC_Level_Back ++;
			if (s_u16Cnt_OCP_AC_Level_Back > 500)	  // 500*2ms
			{
				s_u16Cnt_OCP_AC_Level_Back = 0;
				g_StateCheck.bit.OCP_Inv = 0;
			}
		}
		else
			s_u16Cnt_OCP_AC_Level_Back = 0;
	}
}
/*=============================================================================*
 * FUNCTION:	void HwInvOVPDetection (void)
 *
 * PURPOSE:	INV(220V) hardware over voltage protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwInvOVPDetection (void)
{
	static Uint16 s_u16Cnt_OVP_AC_Level = 0 ;
	static Uint16 s_u16Cnt_OVP_AC_Level_Back = 0 ;

	if(0 == g_StateCheck.bit.OVP_Inv)
	{
		if(0 == INV_OVP_LEVEL)
		{
			s_u16Cnt_OVP_AC_Level ++;
			if (s_u16Cnt_OVP_AC_Level > 1)	  // 2*2ms
			{
				s_u16Cnt_OVP_AC_Level = 0;
				g_StateCheck.bit.OVP_Inv = 1;
			}
		}
		else
			s_u16Cnt_OVP_AC_Level = 0;
	}
	else
	{
		if(1 == INV_OVP_LEVEL)
		{
			s_u16Cnt_OVP_AC_Level_Back ++;
			if (s_u16Cnt_OVP_AC_Level_Back > 500)	  // 500*2ms
			{
				s_u16Cnt_OVP_AC_Level_Back = 0;
				g_StateCheck.bit.OVP_Inv = 0;
			}
		}
		else
			s_u16Cnt_OVP_AC_Level_Back = 0;
	}
}

/*=============================================================================*
 * FUNCTION:	void HwBusOVPDetection (void)
 *
 * PURPOSE:	BUS hardware over voltage protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwBusOVPDetection (void)
{
	static Uint16 s_u16Cnt_Bus_OVP_Level = 0 ;
	static Uint16 s_u16Cnt_Bus_OVP_Fault_Back = 0;

	if(0 == g_StateCheck.bit.Bus_OVP_Fault)
	{
		if(0 == BUS_OVP_LEVEL)      //BUS_OVP_LEVEL=0 represents OVP error
		{
			s_u16Cnt_Bus_OVP_Level ++;                         //2017.4.17 GX    Johnny 2017.4.19
			if (s_u16Cnt_Bus_OVP_Level >= 1)	  		// 1*2ms
			{
				s_u16Cnt_Bus_OVP_Level = 0;
				g_StateCheck.bit.Bus_OVP_Fault = 1;
			}
		}
		else
			s_u16Cnt_Bus_OVP_Level = 0;
	}
	else
	{
		if(1 == BUS_OVP_LEVEL)
		{
			s_u16Cnt_Bus_OVP_Fault_Back ++;
			if (s_u16Cnt_Bus_OVP_Fault_Back >= 1)	  // 1500*2ms
			{
				s_u16Cnt_Bus_OVP_Fault_Back = 0;
				g_StateCheck.bit.Bus_OVP_Fault = 0;
			}
		}
		else
			s_u16Cnt_Bus_OVP_Fault_Back = 0;
	}
}
/*=============================================================================*
 * FUNCTION:	void HwGridOCPDetection (void)
 *
 * PURPOSE:	Input hardware over current protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwGridOCPDetection (void)
{
	static Uint16 s_u16Cnt_Grid_OCP_Level = 0 ;
	static Uint16 s_u16Cnt_Grid_OCP_Fault_Back = 0;

	if(0 == g_StateCheck.bit.OCP_AcGrid)
	{
		if(0 == GRID_OCP_LEVEL && g_StateCheck.bit.Input_dip_Disable_GridOCP == 0)      //BUS_OVP_LEVEL=0 represents OVP error
		{
			s_u16Cnt_Grid_OCP_Level ++;
			if (s_u16Cnt_Grid_OCP_Level >= 3)	  		// 2*2ms
			{
				s_u16Cnt_Grid_OCP_Level = 0;
				g_StateCheck.bit.OCP_AcGrid = 1;
			}
		}
		else
			s_u16Cnt_Grid_OCP_Level = 0;
	}
	else
	{
		if(1 == GRID_OCP_LEVEL)
		{
			s_u16Cnt_Grid_OCP_Fault_Back ++;
			if (s_u16Cnt_Grid_OCP_Fault_Back > 1500)	  // 1500*2ms
			{
				s_u16Cnt_Grid_OCP_Fault_Back = 0;
				g_StateCheck.bit.OCP_AcGrid = 0;
			}
		}
		else
			s_u16Cnt_Grid_OCP_Fault_Back = 0;
	}
}
/*=============================================================================*
 * FUNCTION:	void FanCntl(void)
 *
 * PURPOSE:	This function adjust the Fan drive signal duty ratio to change the Fan speed according to the temperature
 *
 * CALLED BY:	void TimeBase2msPRD(void)
 *============================================================================*/
void FanCntl(void)
{
	if(FanControl_Reg.u16Cnt_temp2<=9)
		FanControl_Reg.u16Cnt_temp2++;
	else
		FanControl_Reg.u16Cnt_temp2=1;

	if (Calc_Result.f32TempPFC > 50)
		FanControl_Reg.u16Cnt_temp1=10;
	else if(Calc_Result.f32TempPFC > 45)
		FanControl_Reg.u16Cnt_temp1=9;
	else if(Calc_Result.f32TempPFC > 40)
		FanControl_Reg.u16Cnt_temp1=8;
	else if(Calc_Result.f32TempPFC > 35)
		FanControl_Reg.u16Cnt_temp1=7;
	else if(Calc_Result.f32TempPFC > 30)
		FanControl_Reg.u16Cnt_temp1=6;
	else
		FanControl_Reg.u16Cnt_temp1=5;

  if(FanControl_Reg.u16Cnt_temp1>=FanControl_Reg.u16Cnt_temp2)
  {
	  DC_Fan_Enable;
	  DcFanSpeedSense();
  }
  else
	  DC_Fan_Disable;
  //end of while (1)
}

//--- end of file -----------------------------------------------------

