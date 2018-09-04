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

struct	AD_Sample_Reg1	GetRealValue, ADGain ,ADChannelOffset, ADCorrection;
struct	AD_Sample_Reg0	GeneralADbuffer;

struct	AD_ACC_Reg1	 Calc_Result;		  	// IQ20
struct	AD_ACC_Reg2  AD_Acc, AD_Sum;     	// IQ10

int32   	g_i32GridFrequencyPriodTemp;
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

	GetRealValue.i32VGrid = _IQmpy(_IQ20mpyI32(ADGain.i32VGrid, GeneralADbuffer.i32VGrid), ADCorrection.i32VGrid) - ADChannelOffset.i32VGrid;
	GetRealValue.i32VOut = _IQmpy(_IQ20mpyI32(ADGain.i32VOut, GeneralADbuffer.i32VOut), ADCorrection.i32VOut) - ADChannelOffset.i32VOut;
	GetRealValue.i32TempAmb = _IQmpy(_IQ20mpyI32(ADGain.i32TempAmb, GeneralADbuffer.i32TempAmb), ADCorrection.i32TempAmb) - ADChannelOffset.i32TempAmb;
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

	AD_Acc.i32VOut_RMS += _IQ2mpyIQX(GetRealValue.i32VOut,20, GetRealValue.i32VOut,20);
	AD_Acc.i32TempAmb += _IQtoIQ10(GetRealValue.i32TempAmb);

	AD_Acc.i32VGrid_RMS += _IQ2mpyIQX(GetRealValue.i32VGrid,20, GetRealValue.i32VGrid,20);
	AD_Acc.i32GridFreq +=  _IQ10mpyIQX(CoffStepToFre, 15, GridPLLReg.i32Delta_Theta, 20);

	if (1 == g_StateCheck.bit.AD_initial)
	{
		AD_Acc.i32VGrid_ave += GetRealValue.i32VGrid;
		AD_Acc.i32VOut_ave += GetRealValue.i32VOut;
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

		AD_Sum.i32VOut_RMS = AD_Acc.i32VOut_RMS;
		AD_Sum.i32TempAmb = AD_Acc.i32TempAmb;

		AD_Sum.i32VGrid_RMS = AD_Acc.i32VGrid_RMS;
		AD_Sum.i32GridFreq = AD_Acc.i32GridFreq;

		if (1 == g_StateCheck.bit.AD_initial)
		{
			AD_Sum.i32VOut_ave = AD_Acc.i32VOut_ave;
			AD_Sum.i32VGrid_ave = AD_Acc.i32VGrid_ave;
			AD_Acc.i32VOut_ave = 0;
			AD_Acc.i32VGrid_ave = 0;
		}

		AD_Acc.i16Counter = 0;	
		AD_Acc.i32TempAmb = 0;

		AD_Acc.i32GridFreq = 0;
		AD_Acc.i32VOut_RMS = 0;
		AD_Acc.i32VGrid_RMS = 0;

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
				GridVoltsRMSCalc();
				GridFrequencyCalc();

				TemperatureCheck();
			    GridVoltCheck();
				GridFreqCheck();
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

	i32temp1 = _IQ10toIQ( _IQ10sqrt( _IQ10mpyIQX( AD_Sum.i32VGrid_RMS, 2, i32SumCounterBypass, 20) ) );
	i32temp1 = _IQmpy(Calc_Result.i32VGrid_RMS, _IQ(0.2f)) + _IQmpy(i32temp1, _IQ(0.8f));	        

	Calc_Result.i32VGrid_RMS = i32temp1;
}

void OutVoltsRMSCalc(void)
{
	_iq20	i32temp1;

	i32temp1 = _IQ10toIQ( _IQ10sqrt( _IQ10mpyIQX(AD_Sum.i32VOut_RMS, 10, i32SumCounterBypass, 20) ) );
	i32temp1 = _IQmpy(Calc_Result.i32VOut_RMS, _IQ(0.2f)) + _IQmpy(i32temp1, _IQ(0.8f));	        

	Calc_Result.i32VOut_RMS = i32temp1;
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

	i32temp1 = _IQmpyIQX(AD_Sum.i32TempAmb,10, i32SumCounterBypass, 20);
	
	if (i32temp1 <= i16TempTab[i16length])
		Calc_Result.i32TempAmb = i32SCRTempHiValue;
	else if (i32temp1 > i16TempTab[0])
		Calc_Result.i32TempAmb = i32SCRTempLowValue;
	else
	{
		for(i16count = i16length - 1; i16count >= 0; i16count--)
		{
			if( (i16TempTab[i16count] > i32temp1) && (i32temp1 >= i16TempTab[i16count + 1]) )
			{
				Calc_Result.i32TempAmb = i16count + 1;
				break;
			}
		}
	}

	i32temp1 = Calc_Result.i32TempAmb;
	i32temp1 = _IQmpy(Calc_Result.i32TempAmb, _IQ(0.4f)) + _IQmpy(i32temp1, _IQ(0.6f));

	Calc_Result.i32TempAmb = i32temp1;
}

void GridFrequencyCalc(void)
{
    _iq	i32temp1;
    i32temp1 = _IQmpyIQX(AD_Sum.i32GridFreq, 10, i32SumCounterBypass, 20);
    i32temp1 = _IQmpy(Calc_Result.i32GridFreq, _IQ(0.4f)) + _IQmpy(i32temp1, _IQ(0.6f));

	Calc_Result.i32GridFreq = i32temp1;
}

void AveCalc(void)
{
	 _iq	i32temp1, i32temp2;

	i32temp1 = _IQmpy(AD_Sum.i32VGrid_ave, i32SumCounterBypass);
	i32temp2 = _IQmpy(AD_Sum.i32VOut_ave, i32SumCounterBypass);

	i32temp1 = _IQmpy(Calc_Result.i32VGrid_ave, _IQ(0.7f)) + _IQmpy(i32temp1, _IQ( 0.3f));
	i32temp2 = _IQmpy(Calc_Result.i32VOut_ave, _IQ(0.7f)) + _IQmpy(i32temp2, _IQ(0.3f));

	Calc_Result.i32VGrid_ave = i32temp1;
	Calc_Result.i32VOut_ave = i32temp2;
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

void HwSwitchCheck(void)
{
    static Uint16 s_u16Cnt_Switch = 0;

	if(1 == g_StateCheck.bit.OVP_InvL)
	{
		s_u16Cnt_InvOVP++;
		if(s_u16Cnt_InvOVP >= 1)
		{
			g_SysFaultMessage.bit.unrecoverHW_InvL_OVP = 1;
			s_u16Cnt_InvOVP = 0;
		}
		g_StateCheck.bit.OVP_InvL = 0;
	}
}
//--- end of file -----------------------------------------------------

