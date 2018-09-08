/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_DataAcquisition.c 
 *
 *  PURPOSE  : Data acquisition file of 28335 for 5KW PV Inverter.
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


#include "DSP2803x_Device.h"				// Peripheral address definitions
#include "F28035_example.h"					// Main include file

struct	AD_Sample_Reg1	GetRealValue, ADGain ,ADChannelOffset, ADCalibration;
struct	AD_Sample_Reg0	GeneralADbuffer;

struct	AD_ACC_Reg1	 Calc_Result;		  	// IQ20
struct	AD_ACC_Reg2  AD_Acc, AD_Sum;     	// IQ10

_iq20		i32SumCounterBypass;
Uint16 	i16TempTab[];
//#pragma CODE_SECTION(Get_ADC_Result, "ControlLoopInRAM")
//#pragma CODE_SECTION(ADAccCalc, "ControlLoopInRAM")

void Get_ADC_Result();
void ADAccCalc();
void TSK_GridPeriod();

//local function
void GridCurrentsRMSCalc();
void GridVoltsRMSCalc();
void OutVoltsRMSCalc();
void TempAmbCalc();
void GridFrequencyCalc();
void AveCalc();
void DitherEliminate();

void DcFanSpeedSense();
/*=============================================================================*
 * FUNCTION: SVPWM_Model()
 * PURPOSE : use U/I ADC result to decide Ta,Tb,Tc by SVPWM_Model, and
 *			 dqPLL and PID Current Controller included.
 * INPUT: 
 *     Vuv, Vvw, Vwu, Ia, Ib, and Ic
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *     void
 *
 * CALLED BY: 
 *     void SVPWMADCswi(void); 
 * 
 *============================================================================*/

void Get_ADC_Result (void)
{
	 // start of function	   GeneralADbuffer-
	GeneralADbuffer.i32VGrid = (int32) AdcResult.ADCRESULT6;
	GeneralADbuffer.i32VOut = (int32) AdcResult.ADCRESULT14;
	GeneralADbuffer.i32TempAmb = (int32) AdcResult.ADCRESULT2;

	GetRealValue.iq20VGrid = _IQmpy(_IQ20mpyI32(ADGain.iq20VGrid, GeneralADbuffer.i32VGrid), ADCalibration.iq20VGrid) - ADChannelOffset.iq20VGrid;
	GetRealValue.iq20VOut = _IQmpy(_IQ20mpyI32(ADGain.iq20VOut, GeneralADbuffer.i32VOut), ADCalibration.iq20VOut) - ADChannelOffset.iq20VOut;
	GetRealValue.iq20TempAmb = _IQmpy(_IQ20mpyI32(ADGain.iq20TempAmb, GeneralADbuffer.i32TempAmb), ADCalibration.iq20TempAmb) - ADChannelOffset.iq20TempAmb;
} // end
/*=============================================================================================================

 * FUNCTION: ADAccCalc()
 * PURPOSE :// ------------------------------------------------
	// --- Accumulate Value Calculation
	// --- Temperature of heatsink, Controlborad, Transformer
	// --- PV panel voltage and current
	// --- Insulation voltage
	// ------------------------------------------------	
 *		
 *	  
 * INPUT: 
 *     *	   *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *
 * CALLED BY: 
 *       ADC_INT_Control() * 	
===============================================================================================================*/
void ADAccCalc(void)
{ 	
	// ------------------------------------------------
	// --- Accumulate Value Calculation
	// --- Temperature of heatsink, Controlborad, Transformer
	// --- PV panel voltage and current
	// --- Insulation voltage
	// ------------------------------------------------

	AD_Acc.iq2VOut_RMS += _IQ2mpyIQX(GetRealValue.iq20VOut,20, GetRealValue.iq20VOut,20);
	AD_Acc.iq10TempAmb += _IQtoIQ10(GetRealValue.iq20TempAmb);

	AD_Acc.iq2VGrid_RMS += _IQ2mpyIQX(GetRealValue.iq20VGrid,20, GetRealValue.iq20VGrid,20);
	AD_Acc.iq10GridFreq +=  _IQ10mpyIQX(CoffStepToFre, 15, GridPLLReg.iq20Delta_Theta, 20);

	if (1 == g_StateCheck.bit.AD_initial)
	{
		AD_Acc.iq20VGrid_ave += GetRealValue.iq20VGrid;
		AD_Acc.iq20VOut_ave += GetRealValue.iq20VOut;
	}
	// ------------------------------------------------
	// ---  Accumulate of Input power calculation  
	// ------------------------------------------------
	AD_Acc.i16Counter++;
	
	// ------------------------------------------------
	if (1== g_StateCheck.bit.Zero_Crossing_Flag)  // 20000Hz / 50Hz = 400
	{
		g_StateCheck.bit.Zero_Crossing_Flag = 0;

		AD_Sum.i16Counter = AD_Acc.i16Counter;

		AD_Sum.iq2VOut_RMS = AD_Acc.iq2VOut_RMS;
		AD_Sum.iq10TempAmb = AD_Acc.iq10TempAmb;

		AD_Sum.iq2VGrid_RMS = AD_Acc.iq2VGrid_RMS;
		AD_Sum.iq10GridFreq = AD_Acc.iq10GridFreq;

		if (1 == g_StateCheck.bit.AD_initial)
		{
			AD_Sum.iq20VOut_ave = AD_Acc.iq20VOut_ave;
			AD_Sum.iq20VGrid_ave = AD_Acc.iq20VGrid_ave;
			AD_Acc.iq20VOut_ave = 0;
			AD_Acc.iq20VGrid_ave = 0;
		}

		AD_Acc.i16Counter = 0;	
		AD_Acc.iq10TempAmb = 0;

		AD_Acc.iq10GridFreq = 0;
		AD_Acc.iq2VOut_RMS = 0;
		AD_Acc.iq2VGrid_RMS = 0;

		SEM_post(&SEM_GridPeriod);	//notify TASK EnergyCalcLowPrio to calc
	}// end of if zero crossing flag
} // end of EnergyAccCalc

/*=============================================================================*
 * FUNCTION: EnergyCalcLowPrio
 * PURPOSE : 
 *		Task function. Average and RMS value is calculated in EnergyCalcLowPrio.
 *		The task sem is posted in  function EnergyAccCalc() every grid period.
 * INPUT: 
 *     Varables struct ECalc_Sum
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     Calculate functions and check functions
 *				Caluculate functions:
 *					GridCurrentsRMSCalc(), GridVoltsRMSCalc(); 
 *					InductorCurrentsRMSCalc(); Temperature();
 *					PVPanelInformation(); PowerCalculate();	KwHourCalculate();
 *				Check functions:
 *					StandbusVoltCal(); GridVoltCheck();	GridCurrentCheck();
 *					GridFreqCheck(); PvVoltCheck();	PvIsolationCheck();
 * CALLED BY: 
 *     BIOS 
 *
 *============================================================================*/
void TSK_GridPeriod(void)     
{
	static Uint16 i16Cnt_SysParaTemp = 0;

	while(1)
	{ // while loop start
		if (SEM_pend(&SEM_GridPeriod, SYS_FOREVER) == TRUE )
		{	
			if (AD_Sum.i16Counter > 0)
			{
				i32SumCounterBypass = _IQdiv( _IQ(1), _IQ(AD_Sum.i16Counter) );

				if (1 == g_StateCheck.bit.AD_initial )
				{
					AveCalc();
					i16Cnt_SysParaTemp++;
					if (i16Cnt_SysParaTemp == 25)  //20ms * 25 = 500ms
					{
						g_StateCheck.bit.AD_initial = 0;
						ADOffsetCheck();
						i16Cnt_SysParaTemp = 0;
					}
				}

	 			GridVoltsRMSCalc();			
				OutVoltsRMSCalc();	
				TempAmbCalc();	
				GridFrequencyCalc();

				TemperatureCheck();
			    GridVoltCheck();
				GridFreqCheck();
				SCRCheck();
			}     
		}                   
	}//end of while (1) 
}//end of EnergyCalcLowPrio task function



/*=============================================================================*
 * FUNCTION: GridCurrentsRMSCalc()     
 * PURPOSE : 
 *		Calculate grid current RMS value,caculated value is ten times of real 
 *		value, for example, 502 means 50.2 Ampere.--------##########################???????????????
 * INPUT: 
 *     ECalc_Sum.i32GridCurrentUSqrt,ECalc_Sum.i32GridCurrentVSqrt,
 *	
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *
 * CALLED BY: 
 *                    TSK_GridPeriod(void)     
 * 
 *============================================================================*/
/*=============================================================================*
 * FUNCTION: GridVoltsRMSCalc
 * PURPOSE : 
 *		Calculate grid voltage RMS value
 *		
 * INPUT: 
 *    
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *
 * CALLED BY:  *        
                        TSK_GridPeriod(void)       chuok 
 * 
 *============================================================================*/
void GridVoltsRMSCalc(void)
{
	_iq20	i32temp1;

	i32temp1 = _IQ10toIQ( _IQ10sqrt( _IQ10mpyIQX( AD_Sum.iq2VGrid_RMS, 2, i32SumCounterBypass, 20) ) );
	i32temp1 = _IQmpy(Calc_Result.iq20VGrid_RMS, _IQ(0.2f)) + _IQmpy(i32temp1, _IQ(0.8f));

	Calc_Result.iq20VGrid_RMS = i32temp1;
}

void OutVoltsRMSCalc(void)
{
	_iq20	i32temp1;

	i32temp1 = _IQ10toIQ( _IQ10sqrt( _IQ10mpyIQX(AD_Sum.iq2VOut_RMS, 10, i32SumCounterBypass, 20) ) );
	i32temp1 = _IQmpy(Calc_Result.iq20VOut_RMS, _IQ(0.2f)) + _IQmpy(i32temp1, _IQ(0.8f));	        

	Calc_Result.iq20VOut_RMS = i32temp1;
}

/*=============================================================================*
 * FUNCTION: TempAmbCalc
 * PURPOSE : 
 *		Temperature Calculation value
 *		
 * INPUT: 
 *    
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *
 * CALLED BY:  *        
                        TSK_GridPeriod(void)       chuok 
 * 
 *============================================================================*/
void TempAmbCalc(void)
{
    _iq20	i32temp1;

	Uint32 i32SCRTempLowValue = 1;
	Uint32 i32SCRTempHiValue = 125;
	int16 i16count = 1;
	int16 i16length = 124;

	i32temp1 = _IQmpyIQX(AD_Sum.iq10TempAmb,10, i32SumCounterBypass, 20);
	
	if (i32temp1 <= i16TempTab[i16length])
		Calc_Result.iq20TempAmb = i32SCRTempHiValue;
	else if (i32temp1 > i16TempTab[0])
		Calc_Result.iq20TempAmb = i32SCRTempLowValue;
	else
	{
		for(i16count = i16length - 1; i16count >= 0; i16count--)
		{
			if( (i16TempTab[i16count] > i32temp1) && (i32temp1 >= i16TempTab[i16count + 1]) )
			{
				Calc_Result.iq20TempAmb = i16count + 1;
				break;
			}
		}
	}

	i32temp1 = Calc_Result.iq20TempAmb;
	i32temp1 = _IQmpy(Calc_Result.iq20TempAmb, _IQ(0.4f)) + _IQmpy(i32temp1, _IQ(0.6f));

	Calc_Result.iq20TempAmb = i32temp1;
}

void GridFrequencyCalc(void)
{
    _iq	i32temp1;
    i32temp1 = _IQmpyIQX(AD_Sum.iq10GridFreq, 10, i32SumCounterBypass, 20);
    i32temp1 = _IQmpy(Calc_Result.iq20GridFreq, _IQ(0.4f)) + _IQmpy(i32temp1, _IQ(0.6f));

	Calc_Result.iq20GridFreq = i32temp1;
}

void AveCalc(void)
{
	 _iq	i32temp1, i32temp2;

	i32temp1 = _IQmpy(AD_Sum.iq20VGrid_ave, i32SumCounterBypass);
	i32temp2 = _IQmpy(AD_Sum.iq20VOut_ave, i32SumCounterBypass);

	i32temp1 = _IQmpy(Calc_Result.iq20VGrid_ave, _IQ(0.7f)) + _IQmpy(i32temp1, _IQ( 0.3f));
	i32temp2 = _IQmpy(Calc_Result.iq20VOut_ave, _IQ(0.7f)) + _IQmpy(i32temp2, _IQ(0.3f));

	Calc_Result.iq20VGrid_ave = i32temp1;
	Calc_Result.iq20VOut_ave = i32temp2;
}
//Digital I/O information
//fan test I/O
void DcFanSpeedSense(void)
{
	static Uint16  s_i16Cnt_DcFan1_Hi_Level = 0 ;
	static Uint16  s_i16Cnt_DcFan1_Low_Level = 0;
	static Uint16  s_i16Cnt_DcFan2_Hi_Level = 0;
	static Uint16  s_i16Cnt_DcFan2_Low_Level = 0;
    static Uint16  i16temp = 0;

    if (i16temp > 90)
    {

    	if (s_i16Cnt_DcFan1_Hi_Level <= 19 || s_i16Cnt_DcFan1_Low_Level <= 19)
    		g_StateCheck.bit.DcFan1Fault = 1;
    	else
    		g_StateCheck.bit.DcFan1Fault = 0;

    	if (s_i16Cnt_DcFan2_Hi_Level <= 19 || s_i16Cnt_DcFan2_Low_Level <= 19)
    		g_StateCheck.bit.DcFan2Fault = 1;
    	else
    		g_StateCheck.bit.DcFan2Fault = 0;

    	s_i16Cnt_DcFan1_Hi_Level = 0 ;
    	s_i16Cnt_DcFan1_Low_Level = 0;
    	s_i16Cnt_DcFan2_Hi_Level = 0;
    	s_i16Cnt_DcFan2_Low_Level = 0;
    	i16temp = 0;
    }
    else
    {
    	i16temp ++;
    	if (1 == DC_FAN1_FB_Level)                 // state  1:high lever  ,0:low lever
    		s_i16Cnt_DcFan1_Hi_Level++;
        else                                                // state  1:high lever  ,0:low lever
            s_i16Cnt_DcFan1_Low_Level++;

    	if (1 == DC_FAN2_FB_Level)                 // state  1:high lever  ,0:low lever
    		s_i16Cnt_DcFan2_Hi_Level++;
    	else                                                // state  1:high lever  ,0:low lever
    		s_i16Cnt_DcFan2_Low_Level++;
    }
}

void DitherEliminate(void)
{
	static Uint8 s_u8templockold = 0;
	static Uint8 s_u8templockstart = 0;
	static Uint8 s_u8templocktimes = 0;
	static Uint8 s_u8tempswitchold  = 0;
	static Uint8 s_u8tempswitchstart = 0;
	static Uint8 s_u8tempswitchtimes = 0;
	static Uint8 s_u8temp = 0;

	if (s_u8temp == 0)
	{
		s_u8templockold = MODULE_LOCK;
		s_u8tempswitchold = SWITCH_MODE;
		s_u8temp = 1;
	}

	if (s_u8templockstart >= 1)
	{
		s_u8templocktimes ++;
		if (s_u8templocktimes >= 1)
		{
			if (MODULE_LOCK != s_u8templockold)
			{
				if (MODULE_LOCK == 1)
					g_StateCheck.bit.Module_Lock = 1;
				else
					g_StateCheck.bit.Module_Lock = 0;

				s_u8templockold = MODULE_LOCK;
			}
			s_u8templockstart = 0;
			s_u8templocktimes = 0;
		}
	}
	else
	{
		if (MODULE_LOCK != s_u8templockold)
			s_u8templockstart = 1;
	}

	if (s_u8tempswitchstart >= 1)
	{
		s_u8tempswitchtimes ++;
		if (s_u8tempswitchtimes >= 1)
		{
			if (SWITCH_MODE != s_u8tempswitchold)
			{
				if (SWITCH_MODE == 1)
					g_StateCheck.bit.Force_Switch = 1;
				else
					g_StateCheck.bit.Force_Switch = 0;

				s_u8tempswitchold = SWITCH_MODE;
			}
			s_u8tempswitchstart = 0;
			s_u8tempswitchtimes = 0;
		}
	}
	else
	{
		if (SWITCH_MODE != s_u8tempswitchold)
			s_u8tempswitchstart = 1;
	}
}
//--- end of file -----------------------------------------------------

