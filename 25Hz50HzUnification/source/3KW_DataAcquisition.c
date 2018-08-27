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
#pragma CODE_SECTION(Get_ADC_Result1, "ControlLoopInRAM")
#pragma CODE_SECTION(Get_ADC_Result2, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccGridCalc, "ControlLoopInRAM")
#pragma CODE_SECTION(ADAccInvCalc, "ControlLoopInRAM")

/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
struct	AD_Sample_Reg1		GeneralADbuffer, GetRealValue, ADGain,ADChannelOffset, ADCalibration;
struct	AD_ACC_Reg1			AD_Acc, AD_Sum, Calc_Result;
float32						f32SumCounterReci = 0;
float32						f32SumCounterInv = 0;
int16 						i16Cnt_SysParaTemp = 0;
extern	float32			f32TempTab[];

/*=============================================================================*
 * functions declaration
 *============================================================================*/
void Get_ADC_Result1();
void Get_ADC_Result2();
void ADAccGridCalc();
void ADAccInvCalc();
void TSK_GridPeriod();
void TSK_InvVoltPeriod();

void GridCurrentsAveCalc();
void InvCurrentsAveCalc();
void OutVoltsAveCalc();
void GridVoltsAveCalc();
void InvVoltsAveCalc();

void GridCurrentPIConfig();
void GridCurrentRefLimit();

/*RMS calculator function */
void GridCurrentsRMSCalc();
void GridCurrentsRMSAveCalc();
void InvCurrentsRMSCalc();
void OutCurrentsRMSCalc();
void GridVoltsRMSCalc();
void InvVoltsRMSCalc();
void OutVoltsRMSCalc();
void BusVoltsCalc();

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

	GeneralADbuffer.f32VGrid = (float32)AdcMirror.ADCRESULT1 - ADDefaultACOffset;		// B0 result
	GetRealValue.f32VGrid = GeneralADbuffer.f32VGrid * ADGain.f32VGrid * ADCalibration.f32VGrid - ADChannelOffset.f32VGrid;
	GeneralADbuffer.f32IGrid = (float32)AdcMirror.ADCRESULT0 - ADDefaultACOffset;	// A0 result
	GetRealValue.f32IGrid = -(GeneralADbuffer.f32IGrid * ADGain.f32IGrid * ADCalibration.f32IGrid) - ADChannelOffset.f32IGrid;

	GeneralADbuffer.f32VBusP = (float32)AdcMirror.ADCRESULT2;	// A1 result
	GetRealValue.f32VBusP = GeneralADbuffer.f32VBusP * ADGain.f32VBusP * ADCalibration.f32VBusP - ADChannelOffset.f32VBusP;
	GeneralADbuffer.f32VBusN = (float32)AdcMirror.ADCRESULT4;	 // A2 result
	GetRealValue.f32VBusN = GeneralADbuffer.f32VBusN * ADGain.f32VBusN * ADCalibration.f32VBusN - ADChannelOffset.f32VBusN;

	GeneralADbuffer.f32TempPFC = (float32)AdcMirror.ADCRESULT11;		// B5 result
	GetRealValue.f32TempPFC = GeneralADbuffer.f32TempPFC * ADCalibration.f32TempPFC *ADGain.f32TempPFC;

} // end of  Get_ADC_Result1

/*=============================================================================*
 * FUNCTION:	void Get_ADC_Result2(void)
 *
 * PURPOSE:	Get the INV sample data from the ADC module of DSP28335
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void Get_ADC_Result2(void)//ZJX changed
{
	// start of function

	GeneralADbuffer.f32IInvH = (float32)AdcMirror.ADCRESULT6 - ADDefaultACOffset;	// A3 result
	GetRealValue.f32IInvH = GeneralADbuffer.f32IInvH * ADGain.f32IInvH * ADCalibration.f32IInvH - ADChannelOffset.f32IInvH;
	GeneralADbuffer.f32VInvH = (float32)AdcMirror.ADCRESULT10 - ADDefaultACOffset;	 // A5 result
	GetRealValue.f32VInvH = GeneralADbuffer.f32VInvH * ADGain.f32VInvH * ADCalibration.f32VInvH * Parallel_Reg.f32VInvH_Comp_Coeff - ADChannelOffset.f32VInvH;
	GeneralADbuffer.f32VOutH = (float32)AdcMirror.ADCRESULT8 - ADDefaultACOffset;	// A4 result
	GetRealValue.f32VOutH = GeneralADbuffer.f32VOutH * ADGain.f32VOutH * ADCalibration.f32VOutH - ADChannelOffset.f32VOutH;

	GeneralADbuffer.f32IInvL = (float32)AdcMirror.ADCRESULT3 - ADDefaultACOffset;	// B1 result
	GetRealValue.f32IInvL = GeneralADbuffer.f32IInvL * ADGain.f32IInvL * ADCalibration.f32IInvL - ADChannelOffset.f32IInvL;
	GeneralADbuffer.f32VInvL = (float32)AdcMirror.ADCRESULT7 - ADDefaultACOffset;	// B3 result
	GetRealValue.f32VInvL = GeneralADbuffer.f32VInvL * ADGain.f32VInvL * ADCalibration.f32VInvL * Parallel_Reg.f32VInvL_Comp_Coeff - ADChannelOffset.f32VInvL;
	GeneralADbuffer.f32VOutL = (float32)AdcMirror.ADCRESULT5 - ADDefaultACOffset;	// B2 result
	GetRealValue.f32VOutL = GeneralADbuffer.f32VOutL * ADGain.f32VOutL * ADCalibration.f32VOutL - ADChannelOffset.f32VOutL;

	GeneralADbuffer.f32TempInvH = (float32)AdcMirror.ADCRESULT9;	// B4 result
	GetRealValue.f32TempInvH = GeneralADbuffer.f32TempInvH * ADCalibration.f32TempInvH * ADGain.f32TempInvH;
	GeneralADbuffer.f32TempInvL = (float32)AdcMirror.ADCRESULT13;	// B6 result
	GetRealValue.f32TempInvL = GeneralADbuffer.f32TempInvL * ADCalibration.f32TempInvL * ADGain.f32TempInvL;

} // end of  Get_ADC_Result2

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
	//start of ADAccGridCalc
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
	}// end of if Grid_Zero_Crossing_Flag

} // end of ADAccGridCalc

/*=============================================================================*
 * FUNCTION:	void ADAccInvCalc(void)
 *
 * PURPOSE:	Accumulation of INV sample data including voltage, current and temperature of INVH(220V) and INVL(110V) ,
 * 						load voltage of INVH(220V) and INVL(110V) and frequency every output period.
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void ADAccInvCalc(void)
{
	//start of ADAccInvCalc
	static float32 f32Phase_Diff_temp1 = 0;

	AD_Acc.f32TempInvH += GetRealValue.f32TempInvH;
	AD_Acc.f32TempInvL += GetRealValue.f32TempInvL;

	if (1 == g_StateCheck.bit.InvAD_initial)
	{
		AD_Acc.f32IInvH_ave += GetRealValue.f32IInvH;
		AD_Acc.f32VInvH_ave += GetRealValue.f32VInvH;
		AD_Acc.f32VOutH_ave += GetRealValue.f32VOutH;

		AD_Acc.f32IInvL_ave += GetRealValue.f32IInvL;
		AD_Acc.f32VInvL_ave += GetRealValue.f32VInvL;
		AD_Acc.f32VOutL_ave += GetRealValue.f32VOutL;

		/*
		 * 'AD_Acc.f32Phase_Diff_ave' is related to phase different between INVH(220V) and INVL(110V)
		 */
		f32Phase_Diff_temp1 = (VOutLPLLConReg.f32Theta - VOutHPLLConReg.f32Theta) * Value2Pi_Ratio * 360.0f;
		if (f32Phase_Diff_temp1 < 0)
			f32Phase_Diff_temp1 = f32Phase_Diff_temp1 + 360.0f;
		AD_Acc.f32Phase_Diff_ave += f32Phase_Diff_temp1;
	}

	AD_Acc.f32IInvH_rms += GetRealValue.f32IInvH * GetRealValue.f32IInvH;
	AD_Acc.f32VInvH_rms += GetRealValue.f32VInvH * GetRealValue.f32VInvH;
	AD_Acc.f32VOutH_rms += GetRealValue.f32VOutH * GetRealValue.f32VOutH;

	AD_Acc.f32IInvL_rms += GetRealValue.f32IInvL * GetRealValue.f32IInvL;
	AD_Acc.f32VInvL_rms += GetRealValue.f32VInvL * GetRealValue.f32VInvL;
	AD_Acc.f32VOutL_rms += GetRealValue.f32VOutL * GetRealValue.f32VOutL;

    AD_Acc.f32VOutFreq += (CoffStepToFre * OutPLLConReg.f32Theta_Step);
    AD_Acc.f32VOutHFreq += (CoffStepToFre * VOutHPLLConReg.f32Theta_Step);
    AD_Acc.f32VOutLFreq += (CoffStepToFre * VOutLPLLConReg.f32Theta_Step);

	AD_Acc.i16InvCounter++;

	if ( 1== g_StateCheck.bit.Inv_Zero_Crossing_Flag)
	{
		g_StateCheck.bit.Inv_Zero_Crossing_Flag = 0;

		AD_Sum.i16InvCounter = AD_Acc.i16InvCounter;
		AD_Sum.f32TempInvH = AD_Acc.f32TempInvH;
		AD_Sum.f32TempInvL = AD_Acc.f32TempInvL;

		AD_Sum.f32VOutFreq = AD_Acc.f32VOutFreq;
		AD_Sum.f32VOutHFreq = AD_Acc.f32VOutHFreq;
		AD_Sum.f32VOutLFreq = AD_Acc.f32VOutLFreq;

		if (1 == g_StateCheck.bit.InvAD_initial)
		{
			AD_Sum.f32IInvH_ave = AD_Acc.f32IInvH_ave;
			AD_Sum.f32VInvH_ave = AD_Acc.f32VInvH_ave;
			AD_Sum.f32VOutH_ave = AD_Acc.f32VOutH_ave;

			AD_Sum.f32IInvL_ave = AD_Acc.f32IInvL_ave;
			AD_Sum.f32VInvL_ave = AD_Acc.f32VInvL_ave;
			AD_Sum.f32VOutL_ave = AD_Acc.f32VOutL_ave;

			AD_Sum.f32Phase_Diff_ave = AD_Acc.f32Phase_Diff_ave;
		}
		AD_Sum.f32IInvH_rms = AD_Acc.f32IInvH_rms;
		AD_Sum.f32VInvH_rms = AD_Acc.f32VInvH_rms;
		AD_Sum.f32VOutH_rms = AD_Acc.f32VOutH_rms;

		AD_Sum.f32IInvL_rms = AD_Acc.f32IInvL_rms;
		AD_Sum.f32VInvL_rms = AD_Acc.f32VInvL_rms;
		AD_Sum.f32VOutL_rms = AD_Acc.f32VOutL_rms;

		AD_Acc.i16InvCounter = 0;
		AD_Acc.f32TempInvH = 0;
		AD_Acc.f32TempInvL = 0;

		AD_Acc.f32VOutFreq = 0;
		AD_Acc.f32VOutHFreq = 0;
		AD_Acc.f32VOutLFreq = 0;

		if (1 == g_StateCheck.bit.InvAD_initial)
		{
			AD_Acc.f32IInvH_ave = 0;
			AD_Acc.f32VInvH_ave = 0;
			AD_Acc.f32VOutH_ave = 0;

			AD_Acc.f32IInvL_ave = 0;
			AD_Acc.f32VInvL_ave = 0;
			AD_Acc.f32VOutL_ave = 0;

			AD_Acc.f32Phase_Diff_ave = 0;
		}
		AD_Acc.f32IInvH_rms = 0;
		AD_Acc.f32VInvH_rms = 0;
		AD_Acc.f32VOutH_rms = 0;

		AD_Acc.f32IInvL_rms = 0;
		AD_Acc.f32VInvL_rms = 0;
		AD_Acc.f32VOutL_rms = 0;

		SEM_post(&SEM_InvPeriod);

	}// end of if Inv_Zero_Crossing_Flag

}   //end of ADAccInvCalc

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
	{
		// while loop start
		if (SEM_pend(&SEM_GridPeriod, SYS_FOREVER) == TRUE )
		{	
			if (AD_Sum.i16GridCounter > 0)
			{
				f32SumCounterReci = 1/ (float32)AD_Sum.i16GridCounter;

				/*
				 * ADC module of DSP is checked by checking average value of voltage and current
				 * through 'ADOffsetCheck()'
				 */
				if (1 == g_StateCheck.bit.GridAD_initial )
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
				GridCurrentsRMSAveCalc();
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
				InputPowerLimit();

				/*Controller related function*/
				GridCurrentPIConfig();
				GridCurrentRefLimit();
				BusVoltSlowup();
			}     
		}                   
	}//end of while (1) 

}//end of TSK_GridPeriod()

/*=============================================================================*
 * FUNCTION:	void TSK_InvVoltPeriod(void)
 *
 * PURPOSE:	Calculate RMS and run software protection every Grid voltage period
 *
 * CALLED BY:	void ADAccInvCalc(void) using semaphore 'SEM_InvPeriod'
 *============================================================================*/
void TSK_InvVoltPeriod(void)
{
	//start of TSK_InvVoltPeriod()
	while(1)
	{
		// while loop start
		if (SEM_pend(&SEM_InvPeriod, SYS_FOREVER) == TRUE )
		{
			if (AD_Sum.i16InvCounter > 0)
			{
				f32SumCounterInv = 1/ (float32)AD_Sum.i16InvCounter;

				/*
				* ADC module of DSP is checked by checking average value of voltage and current
				* through 'ADOffsetCheck()'
				*/
				if (1 == g_StateCheck.bit.InvAD_initial)
				{
					GridVoltsAveCalc();
					InvVoltsAveCalc();
					OutVoltsAveCalc();
				}

				/*RMS Calculating function*/
				InvCurrentsRMSCalc();
				OutCurrentsRMSCalc();
				InvVoltsRMSCalc();
				OutVoltsRMSCalc();
				InvTempCalc();
				OutFrequencyCalc();

				/*Software protection function*/
				InvHCurrentCheck();
				InvLCurrentCheck();
				InvHVoltCheck();
				InvLVoltCheck();
			    InvHFreqCheck();
			    InvLFreqCheck();
          		OverTemperatureLimit();
         		OutputCurrentLimit();
         		InvSyncCheck();
         		InvHCurrentProtectionIdentify();
         		InvLCurrentProtectionIdentify();

				/*Controller related function*/
				SyncLogic_Control();
				InvVoltSlowup();
				InvRestartCheck();
			}
		}
	}//end of while (1)

}//end of TSK_InvVoltPeriod()

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

	f32temp1 = sqrt(AD_Sum.f32IGrid_rms * f32SumCounterReci);
	f32temp1 = (Calc_Result.f32IGrid_rms * 0.4f) + (f32temp1 * 0.6f);	//Low pass filter algorithm

	Calc_Result.f32IGrid_rms = f32temp1;
}
/*=============================================================================*
 * FUNCTION:	void GridCurrentsRMSAveCalc(void)
 *
 * PURPOSE:	Calculate RMS average
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridCurrentsRMSAveCalc(void)
{
	float32				f32temp1 = 0;
	static float32	f32temp2[100];
	static Uint8		u8_count = 0,	u8_count2 = 0;
	Uint8				u8temp1 = 0;

	if (u8_count2 == 0)
	{
		for (u8_count2 = 0; u8_count2 < 100; u8_count2++)
			f32temp2[u8_count2] = 0;
		u8_count2 = 1;
	}
	else
	{
		if (u8_count >= 100)
			u8_count = 0;

		f32temp2[u8_count] = Calc_Result.f32IGrid_rms;
		for (u8temp1 = 0; u8temp1 < 100; u8temp1++)
			f32temp1 += f32temp2[u8temp1];
		u8_count++;
	}

	Calc_Result.f32IGrid_rms_ave = f32temp1 * 0.01;
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
	static float32 f32temp2 = 0;
	Calc_Result.f32VGrid_rms_instant = sqrt(AD_Sum.f32VGrid_rms * f32SumCounterReci);
	f32temp1 = Calc_Result.f32VGrid_rms_instant;
	f32temp1 = (f32temp2 * 0.4f) + (f32temp1 * 0.6f);
	f32temp2 = f32temp1;

	/*
	 * Make the Grid voltage protection more difficult to protect. Enhance robustness.
	 */
	if (f32temp2 <= 170)
		Calc_Result.f32VGrid_rms = f32temp2 + 2;
	else if(f32temp2 >= 280)
		Calc_Result.f32VGrid_rms = f32temp2 - 2;
	else
		Calc_Result.f32VGrid_rms = f32temp2;
}

/*=============================================================================*
 * FUNCTION:	void GridCurrentPIConfig(void)
 *
 * PURPOSE:	Change PFC current loop PI parameter according to Grid voltage amplitude.
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridCurrentPIConfig(void)
{
	static Uint8		s_u8temp1 = 0;
	static float32 	s_f32VGrid_Volt_Subdivide1 = 0;
	static float32 	s_f32VGrid_Volt_Subdivide2 = 0;
	static float32 	s_f32VGrid_Volt_Subdivide3 = 0;
	static float32 	s_f32Hys_Width = 5;

	if(s_u8temp1 == 0)
	{
		if (Module_Type == LY25HZ)
		{
			s_f32VGrid_Volt_Subdivide1 = 185;
			s_f32VGrid_Volt_Subdivide2 = 210;
			s_f32VGrid_Volt_Subdivide3 = 265;
		}
		else
		{
			s_f32VGrid_Volt_Subdivide1 = 220;
			s_f32VGrid_Volt_Subdivide2 = 260;
			s_f32VGrid_Volt_Subdivide3 = 270;
		}
		s_u8temp1 = 1;
	}


	if(Calc_Result.f32VGrid_rms<=s_f32VGrid_Volt_Subdivide1) //185 //220
	{
		CurrConReg.f32Kp = CurrCon_Kp1;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if(Calc_Result.f32VGrid_rms > s_f32VGrid_Volt_Subdivide1 && \
			Calc_Result.f32VGrid_rms <= (s_f32VGrid_Volt_Subdivide1 + s_f32Hys_Width))  //185~190 //220~225
	{
		CurrConReg.f32Kp = CurrConReg.f32Kp;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if(Calc_Result.f32VGrid_rms > (s_f32VGrid_Volt_Subdivide1 + s_f32Hys_Width) && \
			Calc_Result.f32VGrid_rms <= s_f32VGrid_Volt_Subdivide2) //190~210 //225~260
	{
		CurrConReg.f32Kp = CurrCon_Kp2;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if(Calc_Result.f32VGrid_rms > s_f32VGrid_Volt_Subdivide2 && \
			Calc_Result.f32VGrid_rms <= (s_f32VGrid_Volt_Subdivide2 + s_f32Hys_Width)) //210~215 //260~265
	{
		CurrConReg.f32Kp = CurrConReg.f32Kp;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if(Calc_Result.f32VGrid_rms >(s_f32VGrid_Volt_Subdivide2 + s_f32Hys_Width) && \
			Calc_Result.f32VGrid_rms <= s_f32VGrid_Volt_Subdivide3)  //215~165 //265~270
	{
		CurrConReg.f32Kp = CurrCon_Kp3;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if(Calc_Result.f32VGrid_rms >s_f32VGrid_Volt_Subdivide3 &&
			Calc_Result.f32VGrid_rms <= (s_f32VGrid_Volt_Subdivide3 + s_f32Hys_Width)) //265~270 //270~275
	{
		CurrConReg.f32Kp = CurrConReg.f32Kp;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else if (Calc_Result.f32VGrid_rms > (s_f32VGrid_Volt_Subdivide3 + s_f32Hys_Width))//275
	{
		CurrConReg.f32Kp = CurrCon_Kp4;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
	else
	{
		CurrConReg.f32Kp = CurrCon_Kp5;
		CurrConReg.f32Ki = CurrConReg.f32Ki;
	}
}

/*=============================================================================*
 * FUNCTION:	void GridCurrentRefLimit(void)
 *
 * PURPOSE:	Change PFC voltage loop PI output restriction according to Grid voltage amplitude.
 *
 * CALLED BY:	void TSK_GridPeriod(void)
 *============================================================================*/
void GridCurrentRefLimit(void)
{
	static Uint8 s_u8temp1 = 0;
	static float32 s_f32VGrid_Volt_Subdivide1= 0;
	static float32 s_f32VGrid_Volt_Subdivide2= 0;

	if (s_u8temp1 == 0)
	{
		if (Module_Type == LY25HZ)
		{
			s_f32VGrid_Volt_Subdivide1= 185;
			s_f32VGrid_Volt_Subdivide2= 230;
		}
		else
		{
			s_f32VGrid_Volt_Subdivide1= 195;
			s_f32VGrid_Volt_Subdivide2= 230;
		}
		s_u8temp1 = 1;
	}
	if(Calc_Result.f32VGrid_rms <= s_f32VGrid_Volt_Subdivide1)
		BusCon_Reg.f32IGrid_RefAmp_Limit   = IGrid_RefAmp_Limit1;
	else if(Calc_Result.f32VGrid_rms > s_f32VGrid_Volt_Subdivide1 && Calc_Result.f32VGrid_rms <= s_f32VGrid_Volt_Subdivide2)
		BusCon_Reg.f32IGrid_RefAmp_Limit   = IGrid_RefAmp_Limit2;
	else if(Calc_Result.f32VGrid_rms > s_f32VGrid_Volt_Subdivide2)
		BusCon_Reg.f32IGrid_RefAmp_Limit   = IGrid_RefAmp_Limit3;
	else
		BusCon_Reg.f32IGrid_RefAmp_Limit   = IGrid_RefAmp_Limit3;
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
	float32 	f32temp1;
	float32 	f32temp2;
	Uint8 	u8PFCTempLowValue = 1;
	Uint8 	u8PFCTempHiValue = 125;
	int16 	u16cnt;
	int16 	u16length = 124;

	f32temp1 = (AD_Sum.f32TempPFC * f32SumCounterReci);

	if (Module_Type == LY25HZ)
	{
		if (f32temp1 <= f32TempTab[u16length])
			f32temp2 = u8PFCTempHiValue;
		else if (f32temp1 > f32TempTab[0])
			f32temp2 = u8PFCTempLowValue;
		else
		{
			for(u16cnt = u16length - 1; u16cnt >= 0; u16cnt --)
			{
				if((f32TempTab[u16cnt] > f32temp1) && (f32temp1 >= f32TempTab[u16cnt+1]))
				{
					f32temp2 = u16cnt + 1;
					break;
				}
			}
		}
	}
	else
		f32temp2 = f32temp1;

	f32temp2 = (Calc_Result.f32TempPFC * 0.7f) + (f32temp2 * 0.3f);
	Calc_Result.f32TempPFC = f32temp2;
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
    float32 f32temp1;

	f32temp1 = (AD_Sum.f32VGrid_ave  * f32SumCounterReci);
	f32temp1 = (Calc_Result.f32VGrid_ave * 0.7f) + (f32temp1 * 0.3f);

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
	 * 'Calc_Result.f32Coff_Dforward' is related to the forward control of PFC current loop
	 *	which is abandoned now.
	 */
	if(Calc_Result.f32VBus > 200)
	{
		f32temp3 = 1 / Calc_Result.f32VBus;
		Calc_Result.f32Coff_Dforward = 2 * PWM_HALF_PERIOD * f32temp3;
	}
	else
		Calc_Result.f32Coff_Dforward = 0;
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
	float32 f32temp1, f32temp2;

	Calc_Result.f32IInvH_rms_instant = sqrt(AD_Sum.f32IInvH_rms * f32SumCounterInv);
	Calc_Result.f32IInvL_rms_instant= sqrt(AD_Sum.f32IInvL_rms * f32SumCounterInv);

	f32temp1 = (Calc_Result.f32IInvH_rms * 0.4f) + (Calc_Result.f32IInvH_rms_instant * 0.6f);	//Low pass filter algorithm
	f32temp2 = (Calc_Result.f32IInvL_rms * 0.4f) + (Calc_Result.f32IInvL_rms_instant * 0.6f);	//Low pass filter algorithm

	Calc_Result.f32IInvH_rms = f32temp1;
	Calc_Result.f32IInvL_rms = f32temp2;
}

/*=============================================================================*
 * FUNCTION:	void OutCurrentsRMSCalc(void)
 *
 * PURPOSE:		Calculate RMS
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void OutCurrentsRMSCalc(void)
{
	Calc_Result.f32IOutH_rms = sqrt((Calc_Result.f32IInvH_rms + InvHNoLoadCurrent) * (Calc_Result.f32IInvH_rms - InvHNoLoadCurrent)) * ADCalibration.f32IOutH;
	Calc_Result.f32IOutL_rms = sqrt((Calc_Result.f32IInvL_rms + InvLNoLoadCurrent) * (Calc_Result.f32IInvL_rms - InvLNoLoadCurrent)) * ADCalibration.f32IOutL;
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
	float32  f32temp1, f32temp2;

	f32temp1 = sqrt(AD_Sum.f32VOutH_rms * f32SumCounterInv);
	f32temp2 = sqrt(AD_Sum.f32VOutL_rms * f32SumCounterInv);

	f32temp1 = (Calc_Result.f32VOutH_rms * 0.4f) + (f32temp1 * 0.6f);	//Low pass filter algorithm
	f32temp2 = (Calc_Result.f32VOutL_rms * 0.4f) + (f32temp2 * 0.6f);	//Low pass filter algorithm

	Calc_Result.f32VOutH_rms = f32temp1;
	Calc_Result.f32VOutL_rms = f32temp2;
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
	float32 f32temp1, f32temp2;

	Calc_Result.f32VInvH_rms_instant = sqrt(AD_Sum.f32VInvH_rms * f32SumCounterInv);
	Calc_Result.f32VInvL_rms_instant = sqrt(AD_Sum.f32VInvL_rms * f32SumCounterInv);

	f32temp1 = (Calc_Result.f32VInvH_rms * 0.4f) + (Calc_Result.f32VInvH_rms_instant * 0.6f);
	f32temp2 = (Calc_Result.f32VInvL_rms * 0.4f) + (Calc_Result.f32VInvL_rms_instant * 0.6f);

	Calc_Result.f32VInvH_rms = f32temp1;
	Calc_Result.f32VInvL_rms = f32temp2;
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
	float32 f32temp_invh1, f32temp_invh2;
	float32 f32temp_invl1, f32temp_invl2;
	float32 f32InvTempLowValue = 1;
	float32 f32InvTempHiValue = 125;
	int16 u16invh_cnt, u16invl_cnt;
	int16 u16length = 124;

	f32temp_invh1 = (AD_Sum.f32TempInvH * f32SumCounterInv);
	f32temp_invl1 = (AD_Sum.f32TempInvL * f32SumCounterInv);

	if (Module_Type == LY25HZ)
	{
		if (f32temp_invh1 <= f32TempTab[u16length])
			f32temp_invh2 = f32InvTempHiValue;
		else if (f32temp_invh1 > f32TempTab[0])
			f32temp_invh2 = f32InvTempLowValue;
		else
		{
			for(u16invh_cnt = u16length - 1; u16invh_cnt >= 0; u16invh_cnt --)
			{
				if((f32TempTab[u16invh_cnt] > f32temp_invh1) && (f32temp_invh1 >= f32TempTab[u16invh_cnt+1]))
				{
					f32temp_invh2 = u16invh_cnt + 1;
					break;
				}
			}
		}

		if (f32temp_invl1 <= f32TempTab[u16length])
			f32temp_invl2 = f32InvTempHiValue;
		else if (f32temp_invl1 > f32TempTab[0])
			f32temp_invl2 =  f32InvTempLowValue;
		else
		{
			for(u16invl_cnt = u16length - 1; u16invl_cnt >= 0; u16invl_cnt--)
			{
				if((f32TempTab[u16invl_cnt] > f32temp_invl1) && (f32temp_invl1 >= f32TempTab[u16invl_cnt+1]))
				{
					f32temp_invl2 = u16invl_cnt + 1;
					break;
				}
			}
		}
	}
	else
	{
		f32temp_invh2 = f32temp_invh1;
		f32temp_invl2 = f32temp_invh1;
	}

	f32temp_invh2 = (Calc_Result.f32TempInvH * 0.7f) + (f32temp_invh2 * 0.3f);
	f32temp_invl2 = (Calc_Result.f32TempInvL * 0.7f) + (f32temp_invl2 * 0.3f);

	Calc_Result.f32TempInvH = f32temp_invh2;
	Calc_Result.f32TempInvL = f32temp_invl2;

	if(Calc_Result.f32TempInvH > Calc_Result.f32TempInvL)
		Calc_Result.f32TempInvMax = Calc_Result.f32TempInvH;
	else
		Calc_Result.f32TempInvMax = Calc_Result.f32TempInvL;
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
    float32	f32temp1, f32temp2;

	f32temp1 = (AD_Sum.f32IInvH_ave  * f32SumCounterInv);
	f32temp2 = (AD_Sum.f32IInvL_ave  * f32SumCounterInv);

	f32temp1 = (Calc_Result.f32IInvH_ave * 0.7f) + (f32temp1 * 0.3f);
	f32temp2 = (Calc_Result.f32IInvL_ave * 0.7f) + (f32temp2 * 0.3f);

	Calc_Result.f32IInvH_ave = f32temp1;
	Calc_Result.f32IInvL_ave = f32temp2;
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
    float32	f32temp1, f32temp2;

	f32temp1 = (AD_Sum.f32VInvH_ave  * f32SumCounterInv);
	f32temp2 = (AD_Sum.f32VInvL_ave  * f32SumCounterInv);

	f32temp1 = (Calc_Result.f32VInvH_ave * 0.7f) + (f32temp1 * 0.3f);
	f32temp2 = (Calc_Result.f32VInvL_ave * 0.7f) + (f32temp2 * 0.3f);

	Calc_Result.f32VInvH_ave = f32temp1;
	Calc_Result.f32VInvL_ave = f32temp2;
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
    float32	f32temp1, f32temp2;

	f32temp1 = (AD_Sum.f32VOutH_ave  * f32SumCounterInv);
	f32temp2 = (AD_Sum.f32VOutL_ave  * f32SumCounterInv);

	f32temp1 = (Calc_Result.f32VOutH_ave * 0.7f) + (f32temp1 * 0.3f);
	f32temp2 = (Calc_Result.f32VOutL_ave * 0.7f) + (f32temp2 * 0.3f);

	Calc_Result.f32VOutH_ave = f32temp1;
	Calc_Result.f32VOutL_ave = f32temp2;
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
    float32	f32temp1, f32tempH, f32tempL;

	f32temp1 = (AD_Sum.f32VOutFreq  * f32SumCounterInv );
	f32temp1 = (Calc_Result.f32VOutFreq * 0.6f) + (f32temp1 * 0.4f);
	Calc_Result.f32VOutFreq = f32temp1;

	f32tempH = (AD_Sum.f32VOutHFreq  * f32SumCounterInv);
	f32tempH = (Calc_Result.f32VOutHFreq * 0.6f) + (f32tempH * 0.4f);
	Calc_Result.f32VOutHFreq = f32tempH;

	f32tempL = (AD_Sum.f32VOutLFreq  * f32SumCounterInv);
	f32tempL = (Calc_Result.f32VOutLFreq * 0.6f) + (f32tempL * 0.4f);
	Calc_Result.f32VOutLFreq = f32tempL;
}
/*=============================================================================*
 * FUNCTION:	void OutPhaseDiffCalc(void)
 *
 * PURPOSE:	Calculate average value
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void)
 *============================================================================*/
void OutPhaseDiffCalc(void)
{
    float32	f32temp1;
    static float32 f32temp2 = 0;

	f32temp1 = (AD_Sum.f32Phase_Diff_ave  * f32SumCounterInv );
	f32temp1 = f32temp2 * 0.6f + f32temp1 * 0.4f;
	f32temp2 = f32temp1;

	/*
	 * 'Calc_Result.f32Phase_Diff_ave'  is send to LCD to show the phase different
	 */
	if (g_Sys_Current_State == NormalState)
		Calc_Result.f32Phase_Diff_ave = f32temp2;
	else
		Calc_Result.f32Phase_Diff_ave = 0;
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
	static Uint16  	s_u16Cnt_DcFan1_Hi_Level = 0 ;
	static Uint16  	s_u16Cnt_DcFan1_Low_Level = 0;
	static Uint16  	s_u16Cnt_DcFan2_Hi_Level = 0;
	static Uint16  	s_u16Cnt_DcFan2_Low_Level = 0;
    static Uint8  	s_u8period_cnt = 0;

    if (s_u8period_cnt > Fan_Period_Cnt)	//'s_u8period_cnt > 90' contains 18~9 Fan period
    {
    	if (s_u16Cnt_DcFan1_Hi_Level <= Fan_Cnt_Limit || s_u16Cnt_DcFan1_Low_Level <= Fan_Cnt_Limit) //if high level or low level count is smaller than 19, the fan is broken.
    		g_StateCheck.bit.DcFan1Fault = 1;
    	else
    		g_StateCheck.bit.DcFan1Fault = 0;

    	if (s_u16Cnt_DcFan2_Hi_Level <= Fan_Cnt_Limit || s_u16Cnt_DcFan2_Low_Level <= Fan_Cnt_Limit)
    		g_StateCheck.bit.DcFan2Fault = 1;
    	else
    		g_StateCheck.bit.DcFan2Fault = 0;

    	s_u16Cnt_DcFan1_Hi_Level = 0 ;
    	s_u16Cnt_DcFan1_Low_Level = 0;
    	s_u16Cnt_DcFan2_Hi_Level = 0;
    	s_u16Cnt_DcFan2_Low_Level = 0;
    	s_u8period_cnt = 0;
    }
    else
    {
    	s_u8period_cnt ++;
    	if (1 == DC_FAN1_FB_Level)	 					// state  1:high lever  ,0:low lever
    		s_u16Cnt_DcFan1_Hi_Level++;
        else                                               				 // state  1:high lever  ,0:low lever
            s_u16Cnt_DcFan1_Low_Level++;

    	if (1 == DC_FAN2_FB_Level)                		 // state  1:high lever  ,0:low lever
    		s_u16Cnt_DcFan2_Hi_Level++;
    	else                                                				// state  1:high lever  ,0:low lever
    		s_u16Cnt_DcFan2_Low_Level++;
    }
}

/*=============================================================================*
 * FUNCTION:	void HwInvHOCPDetection (void)
 *
 * PURPOSE:	INVH(220V) hardware over current protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwInvHOCPDetection (void)
{
	static Uint16 s_u16Cnt_OCP_AC_Level = 0 ;
	static Uint16 s_u16Cnt_OCP_AC_Level_Back = 0 ;

	if (0 == g_StateCheck.bit.OCP_InvH && Calc_Result.f32VInvH_rms_instant <= 30)
	{
		if (0 == INVH_OCP_LEVEL)
		{
			s_u16Cnt_OCP_AC_Level ++;
			if (s_u16Cnt_OCP_AC_Level >= 1)	// 1 * 2ms = 2ms
			{
				s_u16Cnt_OCP_AC_Level = 0;
				g_StateCheck.bit.OCP_InvH = 1;
			}
		}
		else
			s_u16Cnt_OCP_AC_Level = 0;
	}
	else
	{
		if (1 == INVH_OCP_LEVEL)
		{
			s_u16Cnt_OCP_AC_Level_Back ++;
			if (s_u16Cnt_OCP_AC_Level_Back > 500)	  // 500 * 2ms = 1s
			{
				s_u16Cnt_OCP_AC_Level_Back = 0;
				g_StateCheck.bit.OCP_InvH = 0;
			}
		}
		else
			s_u16Cnt_OCP_AC_Level_Back = 0;
	}
}

/*=============================================================================*
 * FUNCTION:	void HwInvLOCPDetection (void)
 *
 * PURPOSE:	INVL(110V) hardware over current protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwInvLOCPDetection (void)
{
	static Uint16 s_u16Cnt_OCP_AC_Level = 0 ;
	static Uint16 s_u16Cnt_OCP_AC_Level_Back = 0 ;

	if (0 == g_StateCheck.bit.OCP_InvL && Calc_Result.f32VInvL_rms_instant <= 20)
	{
		if (0 == INVL_OCP_LEVEL)
		{
			s_u16Cnt_OCP_AC_Level ++;
			if (s_u16Cnt_OCP_AC_Level >= 1)	  // 1 * 2ms = 2ms
			{
				s_u16Cnt_OCP_AC_Level = 0;
				g_StateCheck.bit.OCP_InvL = 1;
			}
		}
		else
			s_u16Cnt_OCP_AC_Level = 0;
	}
	else
	{
		if (1 == INVL_OCP_LEVEL)
		{
			s_u16Cnt_OCP_AC_Level_Back ++;
			if (s_u16Cnt_OCP_AC_Level_Back > 500)	  // 500 * 2ms = 1s
			{
				s_u16Cnt_OCP_AC_Level_Back = 0;
				g_StateCheck.bit.OCP_InvL = 0;
			}
		}
		else
			s_u16Cnt_OCP_AC_Level_Back = 0;
	}
}

/*=============================================================================*
 * FUNCTION:	void HwInvHOVPDetection (void)
 *
 * PURPOSE:	INVH(220V) hardware over voltage protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwInvHOVPDetection (void)
{
	static Uint16 s_u16Cnt_OVP_AC_Level = 0 ;
	static Uint16 s_u16Cnt_OVP_AC_Level_Back = 0 ;

	if (0 == g_StateCheck.bit.OVP_InvH)
	{
		if (0 == INVH_OVP_LEVEL)
		{
			s_u16Cnt_OVP_AC_Level ++;
			if (s_u16Cnt_OVP_AC_Level > 1)	  // 1 * 2ms = 2ms
			{
				s_u16Cnt_OVP_AC_Level = 0;
				g_StateCheck.bit.OVP_InvH = 1;
			}
		}
		else
			s_u16Cnt_OVP_AC_Level = 0;
	}
	else
	{
		if (1 == INVH_OVP_LEVEL)
		{
			s_u16Cnt_OVP_AC_Level_Back ++;
			if (s_u16Cnt_OVP_AC_Level_Back > 500)	  // 500 * 2ms = 1s
			{
				s_u16Cnt_OVP_AC_Level_Back = 0;
				g_StateCheck.bit.OVP_InvH = 0;
			}
		}
		else
			s_u16Cnt_OVP_AC_Level_Back = 0;
	}
}

/*=============================================================================*
 * FUNCTION:	void HwInvLOVPDetection (void)
 *
 * PURPOSE:	INVL(110V) hardware over voltage protection IO signal detect
 *
 * CALLED BY:	void DigitalIODetect()
 *============================================================================*/
void HwInvLOVPDetection (void)
{
	static Uint16 s_u16Cnt_OVP_AC_Level = 0 ;
	static Uint16 s_u16Cnt_OVP_AC_Level_Back = 0 ;

	if (0 == g_StateCheck.bit.OVP_InvL)
	{
		if (0 == INVL_OVP_LEVEL)
		{
			s_u16Cnt_OVP_AC_Level ++;
			if (s_u16Cnt_OVP_AC_Level > 1)	  // 1 * 2ms = 2ms
			{
				s_u16Cnt_OVP_AC_Level = 0;
				g_StateCheck.bit.OVP_InvL = 1;
			}
		}
		else
			s_u16Cnt_OVP_AC_Level = 0;
	}
	else
	{
		if (1 == INVL_OVP_LEVEL)
		{
			s_u16Cnt_OVP_AC_Level_Back ++;
			if (s_u16Cnt_OVP_AC_Level_Back > 500)	  // 500 * 2ms = 1s
			{
				s_u16Cnt_OVP_AC_Level_Back = 0;
				g_StateCheck.bit.OVP_InvL = 0;
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

	if (0 == g_StateCheck.bit.Bus_OVP_Fault)
	{
		if (0 == BUS_OVP_LEVEL)
		{
			s_u16Cnt_Bus_OVP_Level ++;
			if (s_u16Cnt_Bus_OVP_Level >= 1)	// 1 * 2ms = 2ms
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
		if (1 == BUS_OVP_LEVEL)
		{
			s_u16Cnt_Bus_OVP_Fault_Back ++;
			if (s_u16Cnt_Bus_OVP_Fault_Back > 1500)	  // 1500 * 2ms = 3s
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

	if (0 == g_StateCheck.bit.OCP_AcGrid)
	{
		/*
		 * Input current protection is a Cycle-by-cycle hardware protection, which is need to be
		 * ignored during the Grid voltage dip period.
		 */
		if (0 == GRID_OCP_LEVEL && g_StateCheck.bit.VGridDip_Disable_GridOCP == 0)
		{
			s_u16Cnt_Grid_OCP_Level ++;
			if (s_u16Cnt_Grid_OCP_Level >= 3)					// 3 * 2ms = 6ms
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
			if (s_u16Cnt_Grid_OCP_Fault_Back > 1500)	  // 1500 * 2ms = 3s
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
	static Uint8 u8Fan_Temp_Limit = 50;
	static Uint8 u8Fan_Temp_Limit_Step = 5;
	static Uint8 u8Fan_Cnt_Period = 0;
	static Uint8 u8Fan_Cnt_OnTime = 5;

	if(u8Fan_Cnt_Period<=9)
		u8Fan_Cnt_Period++;
   else
	   u8Fan_Cnt_Period=1;

	if (Calc_Result.f32TempPFC > u8Fan_Temp_Limit) //50
		u8Fan_Cnt_OnTime=10;
	else if(Calc_Result.f32TempPFC > (u8Fan_Temp_Limit - u8Fan_Temp_Limit_Step))//45
		u8Fan_Cnt_OnTime=9;
	else if(Calc_Result.f32TempPFC > (u8Fan_Temp_Limit - 2 * u8Fan_Temp_Limit_Step))//40
		u8Fan_Cnt_OnTime=8;
	else if(Calc_Result.f32TempPFC > (u8Fan_Temp_Limit - 3 * u8Fan_Temp_Limit_Step))//35
		u8Fan_Cnt_OnTime=7;
	else if(Calc_Result.f32TempPFC > (u8Fan_Temp_Limit - 4 * u8Fan_Temp_Limit_Step))//30
		u8Fan_Cnt_OnTime=6;
	else
		u8Fan_Cnt_OnTime=5;

	if(u8Fan_Cnt_OnTime>=u8Fan_Cnt_Period)
	{
		DC_Fan_Enable;
		DcFanSpeedSense();
	}
	else
		DC_Fan_Disable;
}

//--- end of file -----------------------------------------------------

