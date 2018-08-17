/*=============================================================================*
 *         Copyright(c)
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_ProtectionLogic.c
 *
 *  PURPOSE  : Calculate electrical energy related measurements values,
 *			       and do some checks according to calculated values. 
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
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
 *  EnergyAccCalc    				swi function.This function only performs
 *									the basic data accumulate preprocess	*    
 *============================================================================*/


#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file

/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
SAFETY_PARAMETER_REG		SafetyReg;
VOLTAGE_REVISE_REG 			Output_VoltRe_Reg;
struct POWER_DERATE_REG  PowerDerate_Reg;
struct POWER_CON_REG  PowerCon_Reg;
struct ShortCheck_REG  ShortCheck_Reg;

/*=============================================================================*
 * 	functions declaration
 *============================================================================*/
void GridVoltCheck(void);
void GridCurrentCheck(void);
void GridFreqCheck(void);
void InvFreqCheck(void) ;
void InvVoltCheck(void);
void InvCurrentCheck(void);
void InvParallelCurCheck(void);

void BusVoltCheck(void);
void BusBalanceCheck(void);

void OverTemperatureLimit(void);
void FaultCodeRefresh(void);

void OutputCurrentLimit(void);
void InputPowerLimit(void);

void ShortCheck(void);
void CurrentProtectionIdentify(void);
//--- end of local functions

/*******************************************************************************************
* FUNCION :  Grid Volt Check
* PURPOSE :  Three phase grid voltage range Check  for  safety regulation
*******************************************************************************************/
void GridVoltCheck(void)
{ 
	static  Uint16  s_u16Cnt_GridVolt_High_Fault = 0;
	static  Uint16  s_u16Cnt_GridVolt_Low_Fault = 0;
	static  Uint16  s_u16Cnt_GridVolt_Low_Fault2 = 0;
	static  Uint16  s_u16Cnt_GridVolt_Fault_Back = 0;
	static  Uint8    s_u8Cnt_Griddip_Back = 0;

    if(g_SysFaultMessage.bit.VGridOverRating == 0 && g_SysFaultMessage.bit.VGridUnderRating == 0 && g_Sys_Current_State == NormalState)
    {
		if(Calc_Result.f32VGrid_rms > SafetyReg.f32VGrid_HiLimit)  //289V
		{
			s_u16Cnt_GridVolt_High_Fault++;
            if(s_u16Cnt_GridVolt_High_Fault >= SafetyReg.f32VGrid_HighProtectionTime)//6*20ms
            {
				g_SysFaultMessage.bit.VGridOverRating = 1;
				s_u16Cnt_GridVolt_High_Fault = 0;
				s_u16Cnt_GridVolt_Fault_Back = 0;
            }
        }
		//掉电保持
		else if(Calc_Result.f32VGrid_rms < 100 && \
				((Calc_Result.f32VBusP + Calc_Result.f32VBusN) < 660))
		{
			s_u16Cnt_GridVolt_Low_Fault++;
            if(s_u16Cnt_GridVolt_Low_Fault >= 1)
            {
            		SYNC_COM2_OFF;
                	g_SysFaultMessage.bit.VGridUnderRating = 1;
    				s_u16Cnt_GridVolt_Low_Fault = 0;
    				s_u16Cnt_GridVolt_Fault_Back = 0;
    				GRID_RELY_OFF;
            }
        }
		//输入欠压
		else if(Calc_Result.f32VGrid_rms < SafetyReg.f32VGrid_LowLimit) //149V
		{
			s_u16Cnt_GridVolt_Low_Fault2++;
            if(s_u16Cnt_GridVolt_Low_Fault2 >= SafetyReg.f32VGrid_LowProtectionTime)
            {
                    g_SysFaultMessage.bit.VGridUnderRating = 1;
    				s_u16Cnt_GridVolt_Low_Fault2 = 0;
    				s_u16Cnt_GridVolt_Fault_Back = 0;
    				GRID_RELY_OFF;
            }
        }
        else
        {
            s_u16Cnt_GridVolt_Low_Fault = 0;
	     	s_u16Cnt_GridVolt_High_Fault = 0;
	     	s_u16Cnt_GridVolt_Low_Fault2 = 0;
        }	
	}
    else
    {
		if((Calc_Result.f32VGrid_rms > SafetyReg.f32VGrid_LowLimitBack) && (Calc_Result.f32VGrid_rms < SafetyReg.f32VGrid_HiLimitBack))
		{	
			s_u16Cnt_GridVolt_Fault_Back++;
			if(s_u16Cnt_GridVolt_Fault_Back >= 150)     // 150*20ms =3s  279V 159V
			{
				g_SysFaultMessage.bit.VGridOverRating = 0;
				g_SysFaultMessage.bit.VGridUnderRating = 0;
				s_u16Cnt_GridVolt_Fault_Back = 0;
			}
		}
		else
			s_u16Cnt_GridVolt_Fault_Back = 0;
    }
    //在掉电100ms再恢复时，保证恢复好了之后，GridOCP再置0，置0之后输入硬件过流保护会再恢复
    if (Calc_Result.f32VGrid_rms >= SafetyReg.f32VGrid_LowLimit && g_StateCheck.bit.Input_dip_Disable_GridOCP == 1)
    {
    	s_u8Cnt_Griddip_Back ++;
    	if(s_u8Cnt_Griddip_Back >= 3 )
    	{
    		g_StateCheck.bit.Input_dip_Disable_GridOCP = 0;
    		s_u8Cnt_Griddip_Back = 0;
    	}
    }
}

/**********************************************************************
* FUNCION :  Grid  frequency Check
* PURPOSE :  grid frequency range checked  for  safety regulation
**********************************************************************/
void GridFreqCheck(void) 
{ 
    static Uint16 s_u16Cnt_GridFreq_High_Fault = 0;
    static Uint16 s_u16Cnt_GridFreq_Low_Fault = 0;
    static Uint16 s_u16Cnt_GridFreq_Fault_Back = 0;


    if ( (0 == g_SysFaultMessage.bit.FreGridOverRating) && (0 == g_SysFaultMessage.bit.FreGridUnderRating) )
    {
		if(Calc_Result.f32GridFreq > SafetyReg.f32FreGrid_HiLimit)//65.2Hz
        {
            s_u16Cnt_GridFreq_High_Fault++;
            if(s_u16Cnt_GridFreq_High_Fault >= SafetyReg.f32FreGrid_ProtectionTime) //  10*20ms
            {
                g_SysFaultMessage.bit.FreGridOverRating = 1;
                s_u16Cnt_GridFreq_High_Fault = 0;
                s_u16Cnt_GridFreq_Fault_Back = 0;
            }
        }
        else if(Calc_Result.f32GridFreq < SafetyReg.f32FreGrid_LowLimit) //44.8Hz
        {
            s_u16Cnt_GridFreq_Low_Fault++;
            if (s_u16Cnt_GridFreq_Low_Fault >= 20) //20*20ms     10*20msSafetyReg.f32FreGrid_ProtectionTime
            {
               g_SysFaultMessage.bit.FreGridUnderRating = 1;
               s_u16Cnt_GridFreq_Low_Fault = 0;
               s_u16Cnt_GridFreq_Fault_Back = 0;
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
        if((Calc_Result.f32GridFreq < SafetyReg.f32FreGrid_HiLimit) && (Calc_Result.f32GridFreq > SafetyReg.f32FreGrid_LowLimit))
        {
            s_u16Cnt_GridFreq_Fault_Back++;
            if (s_u16Cnt_GridFreq_Fault_Back > 300)      //300*20ms=6s
            {
                s_u16Cnt_GridFreq_Fault_Back = 0;
                g_SysFaultMessage.bit.FreGridOverRating = 0;
                g_SysFaultMessage.bit.FreGridUnderRating = 0;	  		  
            }
        }
        else
        {
             s_u16Cnt_GridFreq_Fault_Back = 0;
        }
    }
}

/**********************************************************************
* FUNCION :  Inv  frequency Check
* PURPOSE :  Inv frequency range checked  for  safety rugulation
**********************************************************************/
void InvFreqCheck(void)
{
    static Uint16 s_u16Cnt_InvFreq_High_Fault = 0;
    static Uint16 s_u16Cnt_InvFreq_Low_Fault = 0;

    if ((0 == g_SysFaultMessage.bit.FreInvOverRating) && (0 == g_SysFaultMessage.bit.FreInvUnderRating) \
    		 && SYNC_COM2_LEVEL == 0)
    {
		if(Calc_Result.f32VOutFreq > 50.5)//50.5Hz
        {
            s_u16Cnt_InvFreq_High_Fault ++;
            if(s_u16Cnt_InvFreq_High_Fault  >= 10) //10*40ms
            {
                g_SysFaultMessage.bit.FreInvOverRating = 1;
                s_u16Cnt_InvFreq_High_Fault = 0;
            }
        }
        else if(Calc_Result.f32VOutFreq < 49.5) //49.5Hz
        {
        	s_u16Cnt_InvFreq_Low_Fault++;
            if (s_u16Cnt_InvFreq_Low_Fault >= 10) //10*40ms
            {
            	g_SysFaultMessage.bit.FreInvUnderRating = 1;
            	s_u16Cnt_InvFreq_Low_Fault = 0;
            }
        }
        else
        {
        	s_u16Cnt_InvFreq_High_Fault = 0;
            s_u16Cnt_InvFreq_Low_Fault = 0;
        }
    }

}
/*******************************************************************************************
* FUNCION :  ADOffsetCheck
* PURPOSE :  Check the ADC module of the DSP
*******************************************************************************************/
void ADOffsetCheck(void)
{
	// Grid Current channel
	if ((fabs(Calc_Result.f32IGrid_ave) > AD_Channel_Offset_IGridLimit))
	{
		g_StateCheck.bit.HWADFault_IGrid_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
		ADChannelOffset.f32IGrid = Calc_Result.f32IGrid_ave;

	// Inv Current channel
	if (fabs(Calc_Result.f32IInv_ave) > AD_Channel_Offset_IInvLimit)
	{
		g_StateCheck.bit.HWADFault_IInv_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
		ADChannelOffset.f32IInv = Calc_Result.f32IInv_ave;

	// Grid Voltage channel
	if ((fabs(Calc_Result.f32VGrid_ave) > AD_Channel_Offset_VGridLimit))
	{
		g_StateCheck.bit.HWADFault_VGrid_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
		ADChannelOffset.f32VGrid = Calc_Result.f32VGrid_ave;

	// INV voltage channel
	if (fabs(Calc_Result.f32VInv_ave) > AD_Channel_Offset_VInvLimit)
	{
		g_StateCheck.bit.HWADFault_VInv_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
		ADChannelOffset.f32VInv = Calc_Result.f32VInv_ave;

	// Out voltage channel
	if (fabs(Calc_Result.f32VOut_ave) > AD_Channel_Offset_VInvLimit)
	{
		g_StateCheck.bit.HWADFault_VOut_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
		ADChannelOffset.f32VOut = Calc_Result.f32VOut_ave;
}

/****************************************************************************************
* FUNCION :  Grid Current Check
* PURPOSE :  Check the Grid current for safety regulation
****************************************************************************************/
void GridCurrentCheck(void)
{
    static  Uint16  s_u16Cnt_Ovr_AcCurrent_Delay= 0;
    static  Uint16  s_u16Cnt_Ovr_AcCurrent_FaultBack= 0;
    
	if(0 == g_SysFaultMessage.bit.OCP_AC_RMS)
	{    
		if(Calc_Result.f32IGrid_rms > OverRated_InputCurrentRms )// 19*1.3
	    {
	        s_u16Cnt_Ovr_AcCurrent_Delay++;
	        if(s_u16Cnt_Ovr_AcCurrent_Delay >= 160) // 20ms*160 = 3.2s  130%负载运行超过3s
	        {
	            s_u16Cnt_Ovr_AcCurrent_Delay = 0;
	            g_SysFaultMessage.bit.OCP_AC_RMS = 1;			
	        }
	    }
	    else
	    {
	        s_u16Cnt_Ovr_AcCurrent_Delay = 0;
	    }
	}
	else
	{
		if(Calc_Result.f32IGrid_rms < Rated_InputCurrentRms )//19
		{
	        s_u16Cnt_Ovr_AcCurrent_FaultBack++;
	        if(s_u16Cnt_Ovr_AcCurrent_FaultBack >= 150) // 3.2s
	        {
	            s_u16Cnt_Ovr_AcCurrent_FaultBack = 0;
	            g_SysFaultMessage.bit.OCP_AC_RMS = 0;
	        }
	    }
	    else
	    {
	        s_u16Cnt_Ovr_AcCurrent_FaultBack = 0;
	    }
	}
}

/**************************************************************************************************
* FUNCION :  Inverter Current Check
* PURPOSE :  Single phase Inverter Current range Check  for safety regulation
*************************************************************************************************/
void InvCurrentCheck(void)
{
    static  Uint16  s_u16Cnt_InvCurrent_High1_Fault = 0;
    static  Uint16  s_u16Cnt_InvCurrent_High2_Fault = 0;
    static  Uint16  s_u16Cnt_InvCurrent_High3_Fault = 0;
    static  Uint16  s_u16Cnt_InvCurrent_High4_Fault = 0;
    static  Uint16  s_u16Cnt_InvCurrent_High_Fault_Back = 0;

    if(g_SysFaultMessage.bit.Inv_OverLoad == 0)
    {
    	if(Calc_Result.f32IOut_rms >=  SafetyReg.f32IInv_Hi4Limit )//2过载   13*2A
    	{
    		s_u16Cnt_InvCurrent_High4_Fault++;
    	    if(s_u16Cnt_InvCurrent_High4_Fault >= SafetyReg.f32IInv_Hi3ProtectionTime)// 20ms*3
    	    {
    	    	g_SysFaultMessage.bit.Inv_OverLoad = 1;
    	    	PowerCon_Reg.f32OutputWatt2 = Calc_Result.f32IOut_rms;//WF 2018.08.14
    	    	s_u16Cnt_InvCurrent_High4_Fault = 0;
    	    	s_u16Cnt_InvCurrent_High_Fault_Back = 0;
    	    }
    	}
    	else if(Calc_Result.f32IOut_rms >=  SafetyReg.f32IInv_Hi3Limit )//1.5过载  13*1.5A
    	{
    		s_u16Cnt_InvCurrent_High3_Fault++;
    	    if(s_u16Cnt_InvCurrent_High3_Fault >= 10)//10*20ms
    	    	g_SysWarningMessage.bit.Inv_OverLoad = 1;
    	    if(s_u16Cnt_InvCurrent_High3_Fault >= 7 )// 1s   SafetyReg.f32IInv_Hi2ProtectionTime
    	    {
    	    	g_SysFaultMessage.bit.Inv_OverLoad = 1;
    	    	PowerCon_Reg.f32OutputWatt1 = Calc_Result.f32IOut_rms;//WF 2018.08.14
    	    	s_u16Cnt_InvCurrent_High3_Fault = 0;
    	    	s_u16Cnt_InvCurrent_High_Fault_Back = 0;
    	    }
    	}
    	else if(Calc_Result.f32IOut_rms >= SafetyReg.f32IInv_Hi2Limit)//1.3倍过载     13*1.3A
		{
			s_u16Cnt_InvCurrent_High2_Fault++;
			if(s_u16Cnt_InvCurrent_High2_Fault >= 10)   //10*20ms
				g_SysWarningMessage.bit.Inv_OverLoad = 1;
			if(s_u16Cnt_InvCurrent_High2_Fault >= 3 * SafetyReg.f32IInv_Hi2ProtectionTime)	// 3*50*20ms=3s
			{
				g_SysFaultMessage.bit.Inv_OverLoad = 1;
				s_u16Cnt_InvCurrent_High2_Fault = 0;
				s_u16Cnt_InvCurrent_High_Fault_Back = 0;
			}
		}
		else if(Calc_Result.f32IOut_rms >= SafetyReg.f32IInv_Hi1Limit)//1.2倍过载      13*1.2A
		{
			s_u16Cnt_InvCurrent_High1_Fault++;
			if(s_u16Cnt_InvCurrent_High1_Fault >= 10)//10*20ms
				g_SysWarningMessage.bit.Inv_OverLoad = 1;
			if(s_u16Cnt_InvCurrent_High1_Fault >= SafetyReg.f32IInv_Hi1ProtectionTime )	//10min
			{
				g_SysFaultMessage.bit.Inv_OverLoad = 1;
				s_u16Cnt_InvCurrent_High1_Fault = 0;
				s_u16Cnt_InvCurrent_High_Fault_Back = 0;
			}
		}
		else
		{
			g_SysWarningMessage.bit.Inv_OverLoad = 0;
			s_u16Cnt_InvCurrent_High1_Fault = 0;
			s_u16Cnt_InvCurrent_High2_Fault = 0;
			s_u16Cnt_InvCurrent_High3_Fault = 0;
			s_u16Cnt_InvCurrent_High4_Fault = 0;
		}
	}
	else
    {
    	if(Calc_Result.f32IInv_rms < ( SafetyReg.f32IInv_Hi1Limit - 2))//比1.2倍过载小2A
    	{
			s_u16Cnt_InvCurrent_High_Fault_Back++;
			if(s_u16Cnt_InvCurrent_High_Fault_Back >= SafetyReg.f32IInv_HiLimitBackTime  ) //1分钟
			{
				g_SysFaultMessage.bit.Inv_OverLoad = 0;
				g_SysWarningMessage.bit.Inv_OverLoad = 0;
				s_u16Cnt_InvCurrent_High_Fault_Back = 0;
			}
    	}
    	else
    	{
    		s_u16Cnt_InvCurrent_High_Fault_Back = 0;
    	}
    }
}

/**************************************************************************************************
* FUNCION :  InvParallelCurCheck
* PURPOSE :  Parallel current check
*************************************************************************************************/
void InvParallelCurCheck(void)
{
	static Uint8 u8temp1 = 0;

	if(0 == g_SysFaultMessage.bit.unrecoverInvCurrSharFault && g_Sys_Current_State == NormalState)
	{
		if(Calc_Result.f32IOut_rms - Calc_Result.f32IInv_para_aver  > 6.5 || \
				Calc_Result.f32IInv_para_aver - Calc_Result.f32IOut_rms > 6.5)
		{
			u8temp1++;
			if (u8temp1 == 3)
			{
				g_SysFaultMessage.bit.unrecoverInvCurrSharFault = 1;
				u8temp1 = 0;
			}
		}
		else
			u8temp1 = 0;
	}
}

/************************************************************************************************
* FUNCION :  BusVolt Check
* PURPOSE :   check  bus voltage for safety regulation
***********************************************************************************************/
void  BusVoltCheck(void)
{
	static Uint16 s_u16Cnt_BusVolt_Ovr_Fault = 0;
	static Uint16 s_u16Cnt_BusVolt_Low_Fault = 0;

	if ((g_SysFaultMessage.bit.recoverSW_Bus_UVP == 0) && (g_StateCheck.bit.PfcSoftStart == 1))		// Johnny 2017.4.19
	{
		if ((Calc_Result.f32VBusP + Calc_Result.f32VBusN) < SafetyReg.f32VBus_LowLimit)  //660V
		{
			s_u16Cnt_BusVolt_Low_Fault++;
			if (s_u16Cnt_BusVolt_Low_Fault >= SafetyReg.f32VBus_ProtectionTime)      //100*0.02s = 2s  //2018.1.23 GX
			{
				s_u16Cnt_BusVolt_Low_Fault = 0;
				g_SysFaultMessage.bit.recoverSW_Bus_UVP = 1;
			}
		}
		else
			s_u16Cnt_BusVolt_Low_Fault = 0;
	}
	else
		s_u16Cnt_BusVolt_Low_Fault = 0;

	if (g_SysFaultMessage.bit.unrecoverSW_Bus_OVP == 0)
	{
		if ((Calc_Result.f32VBusP + Calc_Result.f32VBusN) >= 950 ) //900VSafetyReg.f32VBus_HiLimit
		{
			s_u16Cnt_BusVolt_Ovr_Fault++;
			if (s_u16Cnt_BusVolt_Ovr_Fault > SafetyReg.f32VBus_ProtectionTime) //100*0.02s = 2s
			{
				s_u16Cnt_BusVolt_Ovr_Fault = 0;
				g_SysFaultMessage.bit.unrecoverSW_Bus_OVP = 1;
			}
		}
		else
			s_u16Cnt_BusVolt_Ovr_Fault = 0;
	}
}

/**********************************************************************
* FUNCION :  BusBalance Check
* PURPOSE :  check  bus voltage for safety regulation
**********************************************************************/
void  BusBalanceCheck(void)
{
	static Uint16 s_u16Cnt_Bus_Unba_Fault = 0;

	if ((g_SysFaultMessage.bit.BusVoltUnbalanceFault == 0) && (g_StateCheck.bit.PfcSoftStart == 1))
	{
		if (abs(Calc_Result.f32VBusP - Calc_Result.f32VBusN) > 30)
		{
			s_u16Cnt_Bus_Unba_Fault++;
			if (s_u16Cnt_Bus_Unba_Fault >= 100)      //50*20ms=1s
			{
				s_u16Cnt_Bus_Unba_Fault = 0;
				g_SysFaultMessage.bit.BusVoltUnbalanceFault = 1;
			}
		}
		else
			s_u16Cnt_Bus_Unba_Fault = 0;
	}
	else
		s_u16Cnt_Bus_Unba_Fault = 0;
}

/**********************************************************************
* FUNCION :  InvVolt Check
* PURPOSE :  check inverter voltage
**********************************************************************/
void  InvVoltCheck(void)
{
	static Uint16 s_u16Cnt_InvVolt_Ovr_Fault = 0;
	static Uint16 s_u16Cnt_InvVolt_Low_Fault = 0;
	static Uint16 s_u16Cnt_InvVolt_Low_Fault_Back = 0;

	if (g_SysFaultMessage.bit.unrecoverSW_Inv_OVP == 0)
	{
		if (Calc_Result.f32VInv_rms > SafetyReg.f32VInv_HiLimit ) //242V
		{
			s_u16Cnt_InvVolt_Ovr_Fault++;
			if (s_u16Cnt_InvVolt_Ovr_Fault >= 5)      //100ms
			{
				s_u16Cnt_InvVolt_Ovr_Fault = 0;
				g_SysFaultMessage.bit.unrecoverSW_Inv_OVP = 1;
			}
		}
		else
			s_u16Cnt_InvVolt_Ovr_Fault = 0;
	}

	if (g_StateCheck.bit.VInvUnderRating == 0)
	{
		if (Calc_Result.f32VInv_rms < SafetyReg.f32VInv_LowLimit && (1 == g_StateCheck.bit.Inv_SoftStart)) //198V
		{
			s_u16Cnt_InvVolt_Low_Fault++;
			if (s_u16Cnt_InvVolt_Low_Fault > 10)//200ms
			{
				s_u16Cnt_InvVolt_Low_Fault = 0;
				g_StateCheck.bit.VInvUnderRating = 1;
				g_SysWarningMessage.bit.VInvUnderRating = 1;
			}
		}
		else
			s_u16Cnt_InvVolt_Low_Fault = 0;
		s_u16Cnt_InvVolt_Low_Fault_Back = 0;
	}
	else
	{
		if(Calc_Result.f32VInv_rms > SafetyReg.f32VInv_LowLimitBack) // 203V
		{
			s_u16Cnt_InvVolt_Low_Fault_Back++;
			if (s_u16Cnt_InvVolt_Low_Fault_Back >300)// 20ms*300=6s
			{
				s_u16Cnt_InvVolt_Low_Fault_Back = 0;
				g_StateCheck.bit.VInvUnderRating = 0;
				g_SysWarningMessage.bit.VInvUnderRating = 0;
			}
		}
		else
			s_u16Cnt_InvVolt_Low_Fault_Back = 0;
		s_u16Cnt_InvVolt_Low_Fault = 0;
	}
}
/**********************************************************************************
* FUNCION :  	InvSyncCheck()
* PURPOSE :		check if the synchronization line is broken when modules are
* 						already in parallel condition
***********************************************************************************/
void InvSyncCheck(void)
{
	static Uint8 u8temp1 = 0;
	static Uint8 u8temp2 = 0;
	static float32 f32phase_diff = 0.014 * 6.283185307;

	if  ( 1 == g_StateCheck.bit.Inv_SoftStart && Calc_Result.f32VOut_rms>= 190 )
	{
		 // when relay picks up, there will be several period for PLL function to lock right phase of load voltage
		if (u8temp1 >= 120)  //40 * 20ms = 800ms
		{
			if(fabs(OutPLLConReg.f32Theta - VOutPLLReg.f32Theta) >=  f32phase_diff && \
					g_ParaLogic_State.bit.SyncPhase_Flag == 1)
				u8temp2 ++;
			else
				u8temp2 = 0;
			if(u8temp2 >= 3)
			{
				g_SysFaultMessage.bit.unrecoverHW_SynLine_cut = 1;
				g_StateCheck.bit.Sync_Fault2 = 1;
				u8temp2 = 0;
			}
		}
		else
			u8temp1 ++;
	}
	else
	{
		u8temp1 = 0;
		u8temp2 = 0;
	}
}
/**********************************************************************
* FUNCION :  InputPowerLimit()
* PURPOSE :  limit input power when Grid voltage is under 165V
**********************************************************************/
void InputPowerLimit(void)
{
	if(g_Sys_Current_State == NormalState)
	{
		if(Calc_Result.f32VGrid_rms < VGridLowLimit2)  //165V
			PowerDerate_Reg.f32ACPowerDerating_VRate = ACPowerDerating_VoltageRate - (VGridLowLimit2 - Calc_Result.f32VGrid_rms);
		else
			PowerDerate_Reg.f32ACPowerDerating_VRate = ACPowerDerating_VoltageRate;
	}
	else
	{
		    PowerDerate_Reg.f32ACPowerDerating_VRate = ACPowerDerating_VoltageRate;
	}
}
/********************************************************************************
* FUNCION :  OutputCurrentLimit()
* PURPOSE :  limit output current according to fans condition
********************************************************************************/
void OutputCurrentLimit(void)  //以下限制均考虑到了120%~130%额定负载
{
	float32	f32CurrLimitTemp1;
	float32 f32CurrLimitTemp2;
	float32 f32CurrLimitTemp3;
	float32 f32CurrLimitTemp4;

	if(1 == g_StateCheck.bit.DcFanFault)
	{
		SafetyReg.f32IInv_Hi2Limit = Rated_Inv_OutputCurrentRms * 0.65f;
		SafetyReg.f32IInv_Hi1Limit = Rated_Inv_OutputCurrentRms * 0.6f;
	}
	else
	{
		f32CurrLimitTemp1 = PowerDerate_Reg.f32ACPowerDerating_HTRate;
		f32CurrLimitTemp2 = PowerDerate_Reg.f32ACPowerDerating_VRate;

		if(f32CurrLimitTemp1 <= f32CurrLimitTemp2)
		{
			f32CurrLimitTemp3 = f32CurrLimitTemp1 * 0.013f;
			f32CurrLimitTemp4 = f32CurrLimitTemp1 * 0.012f;
		}
		else
		{
			f32CurrLimitTemp3 = f32CurrLimitTemp2 * 0.013f;
			f32CurrLimitTemp4 = f32CurrLimitTemp2 * 0.012f;
		}
		SafetyReg.f32IInv_Hi2Limit = Rated_Inv_OutputCurrentRms * f32CurrLimitTemp3;
		SafetyReg.f32IInv_Hi1Limit = Rated_Inv_OutputCurrentRms * f32CurrLimitTemp4;
	}
}

/**********************************************************************
* FUNCION :  Over Temperature Limit
* PURPOSE :  Over Temperature load Limited
**********************************************************************/
void OverTemperatureLimit(void)
{
    static	Uint16 s_u16Cnt_HeatsinkTemp_High_Fault = 0;
    static  Uint16 s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
    float32 f32tempMax;

    if(Calc_Result.f32TempInv > Calc_Result.f32TempPFC)
    {
    	f32tempMax = Calc_Result.f32TempInv;
    }
    else
    {
    	f32tempMax = Calc_Result.f32TempPFC;
    }

	if((0 == g_SysFaultMessage.bit.InvTempOverLimit) && (0 == g_SysFaultMessage.bit.PfcOverTempFault))
	 {
		if(f32tempMax >= PowerDerate_Reg.f32Heatsink_OverTemperature_Limit)//85度
		{
			s_u16Cnt_HeatsinkTemp_High_Fault++;
			if(s_u16Cnt_HeatsinkTemp_High_Fault > 5) //40ms*5=200ms
			{
				 if(Calc_Result.f32TempInv > Calc_Result.f32TempPFC)
				 {
					 g_SysFaultMessage.bit.InvTempOverLimit = 1;
					 PowerCon_Reg.f32OutputWatt = Calc_Result.f32TempInv;
				 }
				 else
				 {
					 g_SysFaultMessage.bit.PfcOverTempFault = 1;
					 PowerCon_Reg.f32OutputWatt = Calc_Result.f32TempPFC;
				 }

				g_SysWarningMessage.bit.OverTemp = 1;
				s_u16Cnt_HeatsinkTemp_High_Fault = 0;
				s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
			}
		}
		else if(f32tempMax >= PowerDerate_Reg.f32Heatsink_DeratingTemperature_Limit)//75度
		{
			s_u16Cnt_HeatsinkTemp_High_Fault = 0;
			g_SysWarningMessage.bit.OverTemp = 1;
			PowerDerate_Reg.f32ACPowerDerating_HTRate = ACPowerDerating_HeathinkTempRate - (f32tempMax - PowerDerate_Reg.f32Heatsink_DeratingTemperature_Limit) * 5;
		}
		else
		{
			s_u16Cnt_HeatsinkTemp_High_Fault = 0;
			g_SysWarningMessage.bit.OverTemp = 0;
			PowerDerate_Reg.f32ACPowerDerating_HTRate = ACPowerDerating_HeathinkTempRate;
		}
	}
	else
	{
		if(f32tempMax < (PowerDerate_Reg.f32Heatsink_OverTemperature_Limit - 30))//55度
		{
			s_u16Cnt_HeatsinkTemp_Fault_Back++;
			if(s_u16Cnt_HeatsinkTemp_Fault_Back > 50)//40ms*50=2s
			{
				g_SysFaultMessage.bit.InvTempOverLimit = 0;
				g_SysFaultMessage.bit.PfcOverTempFault = 0;
				g_SysWarningMessage.bit.OverTemp = 0;
				s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
				s_u16Cnt_HeatsinkTemp_High_Fault = 0;
			}
		}
		else
		{
			s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
		}
	}
}

/******************************************************************************************************************
* FUNCION :  	DcPreCharCheck
* PURPOSE :  	Bus voltage previous charge check when module launches.
* 						If the Bus voltage is too low, the previous charge resistance may be broken or ADC fault happens
********************************************************************************************************************/
void DcPreCharCheck(void)
{
	if(0 == g_StateCheck.bit.DcPreCharCheckOver)
	{
		if((Calc_Result.f32VBusP + Calc_Result.f32VBusN) >= (Calc_Result.f32VGrid_rms * 2 * 1.22))
		{
			g_StateCheck.bit.DcPreCharCheckOver = 1;
			GRID_RELY_ON;
		}
		else
		    g_SysFaultMessage.bit.DcFuseFault = 1;
	}
}

/*******************************************************************************************************
* FUNCION :  DCFanCheck
* PURPOSE :  Check Fan
******************************************************************************************************/
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
	{
		g_StateCheck.bit.DcFanFault = 1;
		g_SysFaultMessage.bit.DcFanFault = 1;
	}
	else if ((0 == g_StateCheck.bit.DcFan1Fault) && (0 == g_StateCheck.bit.DcFan2Fault))
		g_StateCheck.bit.DcFanFault = 0;
	else
		g_StateCheck.bit.DcFanFault = 1;
}

// 220V Output Inverter Over Current Hardware Protection
void HwInvOCPCheck(void)
{
    static Uint16 s_u16Cnt_InvOCP = 0;
	if(1 == g_StateCheck.bit.OCP_Inv)
	{
		s_u16Cnt_InvOCP++;
		if(s_u16Cnt_InvOCP >= 1)
		{
		    g_SysFaultMessage.bit.recoverHW_Inv_OCP = 1;
			s_u16Cnt_InvOCP = 0;
		}
		g_StateCheck.bit.OCP_Inv = 0;
	}
}

// 220V Output Inverter Over Voltage Hardware Protection
void HwInvOVPCheck(void)
{
    static Uint16 s_u16Cnt_InvOVP = 0;
	if(1 == g_StateCheck.bit.OVP_Inv)
	{
		s_u16Cnt_InvOVP++;
		if(s_u16Cnt_InvOVP >= 1)
		{
			g_SysFaultMessage.bit.unrecoverHW_Inv_OVP = 1;
			s_u16Cnt_InvOVP = 0;
		}
		g_StateCheck.bit.OVP_Inv = 0;
	}
}

// Bus over voltage protection
void HwBusOVPCheck(void)
{
	static Uint16 s_u16Cnt_BusOVP = 0;
    if(1 == g_StateCheck.bit.Bus_OVP_Fault)
	{
    	s_u16Cnt_BusOVP ++;  //2017.3.26 GX
		if(s_u16Cnt_BusOVP >= 1)
		{
			g_SysFaultMessage.bit.unrecoverHW_Bus_OVP = 1;
			s_u16Cnt_BusOVP = 0;
		}
		 g_StateCheck.bit.Bus_OVP_Fault = 0;
	}
}


// Ac Grid Over Current Hardware Protection
void HwGridOCPCheck(void)
{
    static Uint16 s_u16Cnt_GridOCP = 0;

    if(1 == g_StateCheck.bit.OCP_AcGrid)
	{
    	s_u16Cnt_GridOCP ++;
		if(s_u16Cnt_GridOCP >= 1)
		{
		    g_SysFaultMessage.bit.unrecoverHW_OCP_AC = 1;
			s_u16Cnt_GridOCP = 0;
		}
		g_StateCheck.bit.OCP_AcGrid = 0;
	}
}

/*****************************************************************************************************************
* FUNCION :  	CurrentProtectionIdentify
* PURPOSE :  	1,The CPLD runs cycle-by-cycle protection to deal with output short condition
* 							When CPLD protects 400 times within two output period, it will send a signal to DSP,
* 							if DSP get the signal, the PWM will be disabled.
* 							This function is to guarantee DSP disable the PWM, when CPLD's protection times smaller
* 							than 400.
* 						2, In extremely few cases parallel control will face some problem, which lead to large current
* 							and nearly normal voltage. This function can protect the module when this situation happens.
*****************************************************************************************************************/
void CurrentProtectionIdentify(void)
{
	static Uint8 u8temp1_fault = 0;
	static Uint8 u8temp2_fault = 0;

	if (g_Sys_Current_State == NormalState)
	{
		if(Calc_Result.f32IInv_rms_previous>= 33)
		{
			//If voltage is very low while the current is very large and the voltage is not very low , it will be treated as over current
			if (Calc_Result.f32VInv_rms_previous > 30)
			{
				u8temp1_fault ++;
				if (u8temp1_fault == 2)
				{
					g_SysFaultMessage.bit.unrecoverHW_Inv_OCP  = 1;
					g_StateCheck.bit.ParallelInvOCP = 1;
					PowerCon_Reg.f32OutputWatt4 = Calc_Result.f32IInv_rms_previous;//WF 2018.08.14
					u8temp1_fault = 0;
				}
			}
			else
			{
				//If voltage is very low while the current is very large and the voltage is low , it will be treated as a short condition
				u8temp2_fault ++;
				if (u8temp2_fault == 2)
				{
					g_StateCheck.bit.OCP_Inv = 1;
					PowerCon_Reg.f32OutputWatt3 = Calc_Result.f32IInv_rms_previous;//WF 2018.08.14
					u8temp2_fault = 0;
				}
			}
		}
		else
		{
			u8temp1_fault = 0;
			u8temp2_fault = 0;
		}
	}
}

/*******************************************************************************************************
* FUNCION :  ShortRecover
* PURPOSE :  ShortRecover
******************************************************************************************************/
void ShortCheck(void)//ZJX 2017.11.06
{
	if((1 == g_SysFaultMessage.bit.recoverHW_Inv_OCP))
	{
		ShortCheck_Reg.Restart_time_interval++;
		if(ShortCheck_Reg.Restart_times<=2)
		{
			if(ShortCheck_Reg.Restart_time_interval==5000)
			{
				g_SysFaultMessage.bit.recoverHW_Inv_OCP = 0;
				DRV_RST_ON;
				ShortCheck_Reg.Restart_time_interval=0;
				ShortCheck_Reg.Restart_times++;
				SafetyReg.f32Inv_VoltRms_Ref = 0.1 * Inv_VoltRef;
			}
		}
		else
		{
			ShortCheck_Reg.Restart_time_interval = 0;
			ShortCheck_Reg.Restart_times = 0;
			g_SysFaultMessage.bit.unrecoverHW_Inv_OCP = 1;
			g_StateCheck.bit.ShortInv = 1;
		}
	}
	else
	{
		DRV_RST_OFF;
		Output_VoltRe_Reg.Inv_Light_Flag = 0;
		Output_VoltRe_Reg.Inv_Middle_Flag = 0;
		Output_VoltRe_Reg.Inv_Heavy_Flag = 0;
	}
}

