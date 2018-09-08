/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_ProtectionLogic.c 
 *
 *  PURPOSE  : Calculate electrical energy related measurments values, 
 *			       and do some checks according to calculated values. 
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-19      V0.1           Ken      	    Created
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *   g_ECalc_Result     					struct, calculation results 

 *   NAME								 	 DESCRIPTION
 *	g_i16ECalc_Grid_Zero_Crossing_Flag	 flag of every grid period*      
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *  EnergyAccCalc    				swi function.This function only performes 
 *									the basic data accumulate preprocess	*    
 *============================================================================*/

#include "DSP2803x_Device.h"			// Peripheral address definitions
#include "F28035_example.h"					// Main include file

SAFETY_PARAMETER_REG	SafetyReg;

//------------------
void GridVoltCheck(void);
void GridFreqCheck(void);
void TemperatureCheck(void);
void DCFanCheck(void);
void ADOffsetCheck(void);
void SCRCheck(void);
//--- end of local functions

/**********************************************************************
* FUNCION :  Grid Volt Check
* PURPOSE :  Three phase grid voltage range Check  for  safety rugulation
*                   
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:   void TSK_GridPeriod(void)
* 
**********************************************************************/
void GridVoltCheck(void)
{ 
    static  Uint16  s_u16Cnt_GridVolt_Fault_Back = 0;
	if(g_SysFaultMessage.bit.VGridOverRating == 0 && g_SysFaultMessage.bit.VGridUnderRating == 0)
	{
	    if(Calc_Result.iq20VGrid_RMS > SafetyReg.iq20VGrid_HiLimit)
	    {
			g_SysFaultMessage.bit.VGridOverRating = 1;
			s_u16Cnt_GridVolt_Fault_Back = 0;
		}
		else if (Calc_Result.iq20VGrid_RMS < SafetyReg.iq20VGrid_LowLimit)
		{
			g_SysFaultMessage.bit.VGridUnderRating = 1;
			s_u16Cnt_GridVolt_Fault_Back = 0;
		}
	}

	if((Calc_Result.iq20VGrid_RMS > SafetyReg.iq20VGrid_LowLimitBack) && (Calc_Result.iq20VGrid_RMS < SafetyReg.iq20VGrid_HiLimitBack))
	{       
		s_u16Cnt_GridVolt_Fault_Back++;
		if(s_u16Cnt_GridVolt_Fault_Back >= 300)     //300*20ms =6s
		{
			g_SysFaultMessage.bit.VGridOverRating = 0;
			g_SysFaultMessage.bit.VGridUnderRating = 0;
			s_u16Cnt_GridVolt_Fault_Back = 0;
		}
	}
	else
		s_u16Cnt_GridVolt_Fault_Back = 0;
}
/**********************************************************************
* FUNCION :  Grid  frequency Check
* PURPOSE :  grid frequency range checked  for  safety regulation
*                   
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  void TSK_GridPeriod(void)
* 
**********************************************************************/
void GridFreqCheck(void) 
{ 
	static Uint16 s_u16Cnt_GridFreq_High_Fault = 0;
	static Uint16 s_u16Cnt_GridFreq_Low_Fault = 0;
	static Uint16 s_u16Cnt_GridFreq_Fault_Back = 0;

	//----------------------------------------------------R----------------------------
	if ( (0 == g_SysFaultMessage.bit.FreGridOverRating) && (0 == g_SysFaultMessage.bit.FreGridUnderRating) )
	{
	    if(Calc_Result.iq20GridFreq > SafetyReg.i32FreGrid_HiLimit)
	    {
	        s_u16Cnt_GridFreq_High_Fault++;
	        if(s_u16Cnt_GridFreq_High_Fault >= SafetyReg.u16FreGrid_ProtectionTime)
	        {
	            s_u16Cnt_GridFreq_High_Fault = 0;
	            g_SysFaultMessage.bit.FreGridOverRating = 1;
	        }
	    }
	    else if(Calc_Result.iq20GridFreq < SafetyReg.i32FreGrid_LowLimit)
	    {
	        s_u16Cnt_GridFreq_Low_Fault++;
	        if (s_u16Cnt_GridFreq_Low_Fault >= SafetyReg.u16FreGrid_ProtectionTime)
	        {
	            s_u16Cnt_GridFreq_Low_Fault = 0;
	            g_SysFaultMessage.bit.FreGridUnderRating = 1;
	        }
	    }
	    else
	    {
	        s_u16Cnt_GridFreq_High_Fault = 0;
	        s_u16Cnt_GridFreq_Low_Fault = 0;
	    }
	}
	else
	{
	    if((Calc_Result.iq20GridFreq < SafetyReg.i32FreGrid_HiLimit) && (Calc_Result.iq20GridFreq > SafetyReg.i32FreGrid_LowLimit))
	    {
	        s_u16Cnt_GridFreq_Fault_Back++;
	        if (s_u16Cnt_GridFreq_Fault_Back > 300)
	        {
	            s_u16Cnt_GridFreq_Fault_Back = 0;
	            g_SysFaultMessage.bit.FreGridOverRating = 0;
	            g_SysFaultMessage.bit.FreGridUnderRating = 0;
	        }
	    }
	    else
	        s_u16Cnt_GridFreq_Fault_Back = 0;
	}
}  

void TemperatureCheck(void)
{
	static Uint8 s_u8Cnt_HeatsinkTemp_High_Fault = 0;
	static Uint8 s_u8Cnt_HeatsinkTemp_Fault_Back = 0;

	if(0 == g_SysFaultMessage.bit.OverTempFault)
	{
		if(Calc_Result.iq20TempAmb >= TempAmbHiLimit)//85
		{
			s_u8Cnt_HeatsinkTemp_High_Fault++;
			if(s_u8Cnt_HeatsinkTemp_High_Fault > 5) //40ms*5=200ms
			{
				g_SysFaultMessage.bit.OverTempFault = 1;
				s_u8Cnt_HeatsinkTemp_High_Fault = 0;
				s_u8Cnt_HeatsinkTemp_Fault_Back = 0;
			}
		}
		else
			s_u8Cnt_HeatsinkTemp_High_Fault = 0;
	}
	else
	{
		if(Calc_Result.iq20TempAmb < (TempAmbHiLimit - 30))//55
		{
			s_u8Cnt_HeatsinkTemp_Fault_Back++;
			if(s_u8Cnt_HeatsinkTemp_Fault_Back > 50)//40ms*50=2s
			{
				g_SysFaultMessage.bit.OverTempFault = 0;
				s_u8Cnt_HeatsinkTemp_Fault_Back = 0;
				s_u8Cnt_HeatsinkTemp_High_Fault = 0;
			}
		}
		else
			s_u8Cnt_HeatsinkTemp_Fault_Back = 0;
	}
}

void ADOffsetCheck(void)
{
	// Grid Voltage channel
	if ((_IQabs(Calc_Result.iq20VGrid_ave) > AD_Channel_Offset_VGridLimit))
		g_SysFaultMessage.bit.HWADFault_VGrid = 1;
	else
		ADChannelOffset.iq20VGrid = Calc_Result.iq20VGrid_ave;

	// Out voltage channel
	if (_IQabs(Calc_Result.iq20VOut_ave) > AD_Channel_Offset_VOutLimit)
		g_SysFaultMessage.bit.HWADFault_VOut = 1;
	else
		ADChannelOffset.iq20VOut = Calc_Result.iq20VOut_ave;
}

void DCFanCheck(void)
{
	if(1 == g_StateCheck.bit.DcFan1Fault)
		g_SysWarningMessage.bit.Fan1Block = 1;
	else
		g_SysWarningMessage.bit.Fan1Block = 0;

	if(1 == g_StateCheck.bit.DcFan2Fault)
		g_SysWarningMessage.bit.Fan2Block = 1;
	else
		g_SysWarningMessage.bit.Fan2Block = 0;

	if((1 == g_StateCheck.bit.DcFan1Fault) && (1 == g_StateCheck.bit.DcFan2Fault))
		g_SysWarningMessage.bit.Fan_Fault = 1;
}

void SCRCheck(void)
{
	if ( Calc_Result.iq20VOut_RMS <= 10 )
	{
		if ( g_Sys_State == NormalState )
			g_SysFaultMessage.bit.Bypass_SCR_Fault = 1;
		else if ( g_Sys_State == Standby )
			g_SysFaultMessage.bit.Inv_SCR_Fault = 1;
		else
			;
	}
}
