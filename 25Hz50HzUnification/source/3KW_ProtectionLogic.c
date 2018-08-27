/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_DataAcquisition.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/
			// Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file
#include "DSP2833x_Device.h"
/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
SAFETY_PARAMETER_REG			SafetyReg;
struct POWER_DERATE_REG  	PowerDerate_Reg;
/*=============================================================================*
 * 	functions declaration
 *============================================================================*/
void GridVoltCheck(void);
void GridCurrentCheck(void);
void GridFreqCheck(void);

void InvHFreqCheck(void) ;
void InvLFreqCheck(void) ;
void InvHVoltCheck(void);
void InvLVoltCheck(void);
void InvHCurrentCheck(void);
void InvLCurrentCheck(void);
void InvHParallelCurCheck(void);
void InvLParallelCurCheck(void);
void InvSyncCheck(void);

void BusVoltCheck(void);
void BusBalanceCheck(void);

void OverTemperatureLimit(void);

void OutputCurrentLimit(void);
void InputPowerLimit(void);

void ShortRecover(void);
void InvHCurrentProtectionIdentify(void);
void InvLCurrentProtectionIdentify(void);
//--- end of local functions

/*******************************************************************************************
* FUNCION :  Grid Volt Check
* PURPOSE :  Three phase grid voltage range Check  for  safety regulation
*******************************************************************************************/
void GridVoltCheck(void)
{ 
	static  Uint16  s_u16Cnt_GridVolt_High_Fault = 0;
    static  Uint16  s_u16Cnt_GridVolt_Low_Fault1 = 0;
    static  Uint16  s_u16Cnt_GridVolt_Low_Fault2=0;
    static  Uint16  s_u16Cnt_GridVolt_Fault_Back = 0;
    static  Uint8  s_u8Cnt_Griddip_Back = 0;

    if(g_SysFaultMessage.bit.VGridOverRating == 0 && g_SysFaultMessage.bit.VGridUnderRating == 0)
    {
		if(Calc_Result.f32VGrid_rms > SafetyReg.f32VGrid_HiLimit)  //289V
		{
			s_u16Cnt_GridVolt_High_Fault++;
            if(s_u16Cnt_GridVolt_High_Fault >= SafetyReg.u16VGrid_HighProtectionTime)//6*20ms
            {
				g_SysFaultMessage.bit.VGridOverRating = 1;
				s_u16Cnt_GridVolt_High_Fault = 0;
				s_u16Cnt_GridVolt_Fault_Back = 0;
            }
        }
		//   for  short voltage dip
		else if(Calc_Result.f32VGrid_rms < SafetyReg.f32VGridDipLimit&& \
				((Calc_Result.f32VBus) < SafetyReg.f32VGridDip_BusVoltLimit))
		{
			s_u16Cnt_GridVolt_Low_Fault1++;
            if(s_u16Cnt_GridVolt_Low_Fault1 >= 1)
            {
            		SYNC_COM2_OFF;
                	g_SysFaultMessage.bit.VGridUnderRating = 1;
    				s_u16Cnt_GridVolt_Low_Fault1 = 0;
    				s_u16Cnt_GridVolt_Fault_Back = 0;
    				GRID_RELY_OFF;
            }
        }
		//  for   under voltage  protection
		else if(Calc_Result.f32VGrid_rms < SafetyReg.f32VGrid_LowLimit )
		{
			s_u16Cnt_GridVolt_Low_Fault2++;
			if(s_u16Cnt_GridVolt_Low_Fault2 >= SafetyReg.u16VGrid_LowProtectionTime)//10*20ms
			{
				   	g_SysFaultMessage.bit.VGridUnderRating = 1;
			    	s_u16Cnt_GridVolt_Low_Fault2 = 0;
			    	s_u16Cnt_GridVolt_Fault_Back = 0;
			    	GRID_RELY_OFF;
			}
		}
        else
        {
            s_u16Cnt_GridVolt_Low_Fault1 = 0;
            s_u16Cnt_GridVolt_Low_Fault2 = 0;
	     	s_u16Cnt_GridVolt_High_Fault = 0;
        }	
	}
    else
    {
		if((Calc_Result.f32VGrid_rms > SafetyReg.f32VGrid_LowLimitBack) && (Calc_Result.f32VGrid_rms < SafetyReg.f32VGrid_HiLimitBack))
		{	
			s_u16Cnt_GridVolt_Fault_Back++;
			if(s_u16Cnt_GridVolt_Fault_Back >= 150 )     // 150*20ms =3s  279V 159V
			{
				g_SysFaultMessage.bit.VGridOverRating = 0;
				g_SysFaultMessage.bit.VGridUnderRating = 0;
				s_u16Cnt_GridVolt_Fault_Back = 0;
			}
		}
		else
			s_u16Cnt_GridVolt_Fault_Back = 0;
    }
    /*
     * In Grid voltage dip time, GridOCP is set to disable input current hardware protection,
     * when voltage recovers, GridOCP clears.
     */
    if (Calc_Result.f32VGrid_rms >= SafetyReg.f32VGrid_LowLimitBack && g_StateCheck.bit.VGridDip_Disable_GridOCP == 1)
    {
    	s_u8Cnt_Griddip_Back ++;
    	if(s_u8Cnt_Griddip_Back >= 3 )
    	{
    		g_StateCheck.bit.VGridDip_Disable_GridOCP = 0;
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
            if(s_u16Cnt_GridFreq_High_Fault >= SafetyReg.u16FreGrid_ProtectionTime) //  10*20ms
            {
                g_SysFaultMessage.bit.FreGridOverRating = 1;
                s_u16Cnt_GridFreq_High_Fault = 0;
                s_u16Cnt_GridFreq_Fault_Back = 0;
            }
        }
        else if(Calc_Result.f32GridFreq < SafetyReg.f32FreGrid_LowLimit) //44.8Hz
        {
            s_u16Cnt_GridFreq_Low_Fault++;
            if (s_u16Cnt_GridFreq_Low_Fault >= SafetyReg.u16FreGrid_ProtectionTime) //  10*20ms
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
* FUNCION :  InvH  frequency Check
* PURPOSE :  InvH frequency range checked  for  safety rugulation
**********************************************************************/
void InvHFreqCheck(void)
{
    static Uint16 s_u16Cnt_InvHFreq_High_Fault = 0;
    static Uint16 s_u16Cnt_InvHFreq_Low_Fault = 0;

    if ((0 == g_SysFaultMessage.bit.FreInvOverRating) && (0 == g_SysFaultMessage.bit.FreInvUnderRating) )
    {
		if(Calc_Result.f32VOutHFreq > SafetyReg.f32FreVOut_HiLimit)//25.4Hz
        {
            s_u16Cnt_InvHFreq_High_Fault ++;
            if(s_u16Cnt_InvHFreq_High_Fault  >= 10) //10*40ms
            {
                g_SysFaultMessage.bit.FreInvOverRating = 1;
                g_StateCheck.bit.FreVOutH_Fault = 1;
                s_u16Cnt_InvHFreq_High_Fault = 0;
            }
        }
        else if(Calc_Result.f32VOutHFreq < SafetyReg.f32FreVOut_LowLimit) //24.6Hz
        {
        	s_u16Cnt_InvHFreq_Low_Fault++;
            if (s_u16Cnt_InvHFreq_Low_Fault >= 10) //10*40ms
            {
            	g_SysFaultMessage.bit.FreInvUnderRating = 1;
            	g_StateCheck.bit.FreVOutH_Fault = 1;
            	s_u16Cnt_InvHFreq_Low_Fault = 0;
            }
        }
        else
        {
        	s_u16Cnt_InvHFreq_High_Fault = 0;
            s_u16Cnt_InvHFreq_Low_Fault = 0;
        }
    }
}
/**********************************************************************
* FUNCION :  InvL  frequency Check
* PURPOSE :  InvL frequency range checked  for  safety rugulation
**********************************************************************/
void InvLFreqCheck(void)
{
    static Uint16 s_u16Cnt_InvLFreq_High_Fault = 0;
    static Uint16 s_u16Cnt_InvLFreq_Low_Fault = 0;

    if ((0 == g_SysFaultMessage.bit.FreInvOverRating) && (0 == g_SysFaultMessage.bit.FreInvUnderRating) )
    {
		if(Calc_Result.f32VOutLFreq > SafetyReg.f32FreVOut_HiLimit)//25.4Hz
        {
            s_u16Cnt_InvLFreq_High_Fault ++;
            if(s_u16Cnt_InvLFreq_High_Fault  >= 10) //10*40ms
            {
                g_SysFaultMessage.bit.FreInvOverRating = 1;
                g_StateCheck.bit.FreVOutL_Fault = 1;
                s_u16Cnt_InvLFreq_High_Fault = 0;
            }
        }
        else if(Calc_Result.f32VOutLFreq < SafetyReg.f32FreVOut_LowLimit) //24.6Hz
        {
        	s_u16Cnt_InvLFreq_Low_Fault++;
            if (s_u16Cnt_InvLFreq_Low_Fault >= 10) //10*40ms
            {
            	g_SysFaultMessage.bit.FreInvUnderRating = 1;
            	g_StateCheck.bit.FreVOutL_Fault = 1;
            	s_u16Cnt_InvLFreq_Low_Fault = 0;
            }
        }
        else
        {
        	s_u16Cnt_InvLFreq_High_Fault = 0;
            s_u16Cnt_InvLFreq_Low_Fault = 0;
        }
    }
}
/*******************************************************************************************
* FUNCION :  ADOffsetCheck
* PURPOSE :  Check the ADC module of the DSP
*******************************************************************************************/
// AD channel check
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
	if (fabs(Calc_Result.f32IInvH_ave) > AD_Channel_Offset_IInvLimit)
	{
		g_StateCheck.bit.HWADFault_IInvH_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else if (fabs(Calc_Result.f32IInvL_ave) > AD_Channel_Offset_IInvLimit)
	{
		g_StateCheck.bit.HWADFault_IInvL_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
	{
		ADChannelOffset.f32IInvH = Calc_Result.f32IInvH_ave;
		ADChannelOffset.f32IInvL = Calc_Result.f32IInvL_ave;
	}

	// Grid Voltage channel
	if ((fabs(Calc_Result.f32VGrid_ave) > AD_Channel_Offset_VGridLimit))
	{
		g_StateCheck.bit.HWADFault_VGrid_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
		ADChannelOffset.f32VGrid = Calc_Result.f32VGrid_ave;

	// INV voltage channel
	if (fabs(Calc_Result.f32VInvH_ave) > AD_Channel_Offset_VInvLimit)
	{
		g_StateCheck.bit.HWADFault_VInvH_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else if (fabs(Calc_Result.f32VInvL_ave) > AD_Channel_Offset_VInvLimit)
	{
		g_StateCheck.bit.HWADFault_VInvL_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
	{
		ADChannelOffset.f32VInvH = Calc_Result.f32VInvH_ave;
		ADChannelOffset.f32VInvL = Calc_Result.f32VInvL_ave;
	}

	// Out voltage channel
	if (fabs(Calc_Result.f32VOutH_ave) > AD_Channel_Offset_VInvLimit)
	{
		g_StateCheck.bit.HWADFault_VOutH_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else if (fabs(Calc_Result.f32VOutL_ave) > AD_Channel_Offset_VInvLimit)
	{
		g_StateCheck.bit.HWADFault_VOutL_ave = 1;
		g_SysFaultMessage.bit.HWADFault = 1;
	}
	else
	{
		ADChannelOffset.f32VOutH = Calc_Result.f32VOutH_ave;
		ADChannelOffset.f32VOutL = Calc_Result.f32VOutL_ave;
	}
}

/****************************************************************************************
* FUNCION :  Grid Current Check
* PURPOSE :  Check the Grid current for safety regulation
****************************************************************************************/
void GridCurrentCheck(void)
{
    static  Uint16  s_u16Cnt_Ovr_AcCurrent_Fault= 0;
    static  Uint16  s_u16Cnt_Ovr_AcCurrent_FaultBack= 0;
    
	if(0 == g_SysFaultMessage.bit.OCP_AC_RMS)
	{    
		if(Calc_Result.f32IGrid_rms > OverRated_InputCurrentRms )// 13.5*1.3
	    {
	        s_u16Cnt_Ovr_AcCurrent_Fault++;
	        if(s_u16Cnt_Ovr_AcCurrent_Fault >= 160) // 20ms*160 = 3.2s  130%over load 3s protect
	        {
	            s_u16Cnt_Ovr_AcCurrent_Fault = 0;
	            g_SysFaultMessage.bit.OCP_AC_RMS = 1;			
	        }
	    }
	    else
	        s_u16Cnt_Ovr_AcCurrent_Fault = 0;
	}
	else
	{
		if(Calc_Result.f32IGrid_rms < Rated_InputCurrentRms )//13.5
		{
	        s_u16Cnt_Ovr_AcCurrent_FaultBack++;
	        if(s_u16Cnt_Ovr_AcCurrent_FaultBack >= 150) // 3.2s
	        {
	            s_u16Cnt_Ovr_AcCurrent_FaultBack = 0;
	            g_SysFaultMessage.bit.OCP_AC_RMS = 0;
	        }
	    }
	    else
	        s_u16Cnt_Ovr_AcCurrent_FaultBack = 0;
	}
}

/**************************************************************************************************
* FUNCION :  Inverter Current Check
* PURPOSE :  Single phase Inverter Current range Check  for safety regulation
*************************************************************************************************/
void InvHCurrentCheck(void)
{
    static  Uint16  s_u16Cnt_InvHCurrent_High1_Fault = 0;
    static  Uint16  s_u16Cnt_InvHCurrent_High2_Fault = 0;
    static  Uint16  s_u16Cnt_InvHCurrent_High3_Fault = 0;
    static  Uint16  s_u16Cnt_InvHCurrent_High4_Fault = 0;
    static  Uint16  s_u16Cnt_InvHCurrent_High_Fault_Back = 0;

    if(g_SysFaultMessage.bit.InvH_OverLoad == 0)
    {
    	 if(Calc_Result.f32IOutH_rms >=  SafetyReg.f32IInvH_Hi4Limit )//200% over load  5.5*2A
    	 {
    		 s_u16Cnt_InvHCurrent_High4_Fault++;
    		 if(s_u16Cnt_InvHCurrent_High4_Fault >= SafetyReg.u16IInvH_Hi4ProtectionTime)// 40ms*3
    	     {
    			 g_SysFaultMessage.bit.InvH_OverLoad = 1;
    			 s_u16Cnt_InvHCurrent_High4_Fault = 0;
    			 s_u16Cnt_InvHCurrent_High_Fault_Back = 0;
    	     }
    	 }
    	 else if(Calc_Result.f32IOutH_rms >=  SafetyReg.f32IInvH_Hi3Limit )//150% over load  5.5*1.5A
    	 {
    		 s_u16Cnt_InvHCurrent_High3_Fault++;
    		 if(s_u16Cnt_InvHCurrent_High3_Fault >= SafetyReg.u16IInv_WarningTime)//10*40ms
    			 g_SysWarningMessage.bit.InvH_OverLoad = 1;
    		 if(s_u16Cnt_InvHCurrent_High3_Fault >= SafetyReg.u16IInvH_Hi3ProtectionTime)// 1s
    		 {
    			 g_SysFaultMessage.bit.InvH_OverLoad = 1;
    			 s_u16Cnt_InvHCurrent_High3_Fault = 0;
    			 s_u16Cnt_InvHCurrent_High_Fault_Back = 0;
    		 }
    	 }
    	else if(Calc_Result.f32IOutH_rms >=SafetyReg.f32IInvH_Hi2Limit )//130% over load  5.5*1.3A
		{
    		s_u16Cnt_InvHCurrent_High2_Fault++;
			if(s_u16Cnt_InvHCurrent_High2_Fault >= SafetyReg.u16IInv_WarningTime)   //10*40ms
				g_SysWarningMessage.bit.InvH_OverLoad = 1;
			if(s_u16Cnt_InvHCurrent_High2_Fault >= SafetyReg.u16IInvH_Hi2ProtectionTime)// 75*40ms=3s
			{
				g_SysFaultMessage.bit.InvH_OverLoad = 1;
				s_u16Cnt_InvHCurrent_High2_Fault = 0;
				s_u16Cnt_InvHCurrent_High_Fault_Back = 0;
			}
		}
		else if(Calc_Result.f32IOutH_rms >=  SafetyReg.f32IInvH_Hi1Limit )//120% over load  5.5*1.2A
		{
			s_u16Cnt_InvHCurrent_High1_Fault++;
			if(s_u16Cnt_InvHCurrent_High1_Fault >= SafetyReg.u16IInv_WarningTime)//10*40ms
				g_SysWarningMessage.bit.InvH_OverLoad = 1;
			if(s_u16Cnt_InvHCurrent_High1_Fault >= SafetyReg.u16IInvH_Hi1ProtectionTime)// 10min
			{
				g_SysFaultMessage.bit.InvH_OverLoad = 1;
				s_u16Cnt_InvHCurrent_High1_Fault = 0;
				s_u16Cnt_InvHCurrent_High_Fault_Back = 0;
			}
		}
		else
		{
			g_SysWarningMessage.bit.InvH_OverLoad = 0;
			s_u16Cnt_InvHCurrent_High1_Fault = 0;
			s_u16Cnt_InvHCurrent_High2_Fault = 0;
			s_u16Cnt_InvHCurrent_High3_Fault = 0;
			s_u16Cnt_InvHCurrent_High4_Fault = 0;
		}
	}
	else
    {
    	if(Calc_Result.f32IOutH_rms < ( SafetyReg.f32IInvH_Hi1Limit - 2))//5.5*1.2A - 2
    	{
			s_u16Cnt_InvHCurrent_High_Fault_Back++;
			if(s_u16Cnt_InvHCurrent_High_Fault_Back >=  SafetyReg.u16IInvH_HiLimitBackTime ) //1500 * 40ms = 60s
			{
				g_SysFaultMessage.bit.InvH_OverLoad = 0;
				g_SysWarningMessage.bit.InvH_OverLoad = 0;
				s_u16Cnt_InvHCurrent_High_Fault_Back = 0;
			}
    	}
    	else
    		s_u16Cnt_InvHCurrent_High_Fault_Back = 0;
    }
}
/**************************************************************************************************
* FUNCION :  Inverter Current Check
* PURPOSE :  Single phase Inverter Current range Check  for safety regulation
*************************************************************************************************/
void InvLCurrentCheck(void)
{
    static  Uint16  s_u16Cnt_InvLCurrent_High1_Fault = 0;
    static  Uint16  s_u16Cnt_InvLCurrent_High2_Fault = 0;
    static  Uint16  s_u16Cnt_InvLCurrent_High3_Fault = 0;
    static  Uint16  s_u16Cnt_InvLCurrent_High4_Fault = 0;
    static  Uint16  s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
	// 110V Output Inverter
	if(g_SysFaultMessage.bit.InvL_OverLoad == 0)
	{
		if(Calc_Result.f32IOutL_rms >=  SafetyReg.f32IInvL_Hi4Limit )//200% over load 7.3*2A
		{
			s_u16Cnt_InvLCurrent_High4_Fault++;
			if(s_u16Cnt_InvLCurrent_High4_Fault >= SafetyReg.u16IInvL_Hi4ProtectionTime )// 40ms*3
			{
				g_SysFaultMessage.bit.InvL_OverLoad = 1;
				s_u16Cnt_InvLCurrent_High4_Fault = 0;
				s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
			}
		}
		else if(Calc_Result.f32IOutL_rms >=  SafetyReg.f32IInvL_Hi3Limit )//150% over load 7.3*1.5A
		{
			s_u16Cnt_InvLCurrent_High3_Fault++;
			if(s_u16Cnt_InvLCurrent_High3_Fault >= SafetyReg.u16IInv_WarningTime)//10*40ms
				 g_SysWarningMessage.bit.InvL_OverLoad = 1;
			if(s_u16Cnt_InvLCurrent_High3_Fault >= SafetyReg.u16IInvL_Hi3ProtectionTime )// 1s
			{
				 g_SysFaultMessage.bit.InvL_OverLoad = 1;
				 s_u16Cnt_InvLCurrent_High3_Fault = 0;
				 s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
			}
		}
		else if(Calc_Result.f32IOutL_rms >=  SafetyReg.f32IInvL_Hi2Limit )//130% over load 7.3*1.3A
		{
			s_u16Cnt_InvLCurrent_High2_Fault++;
			if(s_u16Cnt_InvLCurrent_High2_Fault >= SafetyReg.u16IInv_WarningTime)//10*40ms
				g_SysWarningMessage.bit.InvL_OverLoad = 1;
			if(s_u16Cnt_InvLCurrent_High2_Fault >=SafetyReg.u16IInvL_Hi2ProtectionTime)// 3s
			{
				g_SysFaultMessage.bit.InvL_OverLoad = 1;
				s_u16Cnt_InvLCurrent_High2_Fault = 0;
				s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
			}
		}
		else if(Calc_Result.f32IOutL_rms >=  SafetyReg.f32IInvL_Hi1Limit )//120% over load 7.3*1.2A
		{
			s_u16Cnt_InvLCurrent_High1_Fault++;
			if(s_u16Cnt_InvLCurrent_High1_Fault >= SafetyReg.u16IInv_WarningTime)//10*40ms
				g_SysWarningMessage.bit.InvL_OverLoad = 1;
			if(s_u16Cnt_InvLCurrent_High1_Fault >= SafetyReg.u16IInvL_Hi1ProtectionTime )// 10min
			{
				g_SysFaultMessage.bit.InvL_OverLoad = 1;
				s_u16Cnt_InvLCurrent_High1_Fault = 0;
				s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
			}
		}
		else
		{
			g_SysWarningMessage.bit.InvL_OverLoad = 0;
			s_u16Cnt_InvLCurrent_High1_Fault = 0;
			s_u16Cnt_InvLCurrent_High2_Fault = 0;
			s_u16Cnt_InvLCurrent_High3_Fault = 0;
			s_u16Cnt_InvLCurrent_High4_Fault = 0;
		}
	}
	else
	{
		if(Calc_Result.f32IOutL_rms < ( SafetyReg.f32IInvL_Hi1Limit - 2))// 7.3*1.2A -2
		{
			s_u16Cnt_InvLCurrent_High_Fault_Back++;
			if(s_u16Cnt_InvLCurrent_High_Fault_Back >= SafetyReg.u16IInvL_HiLimitBackTime ) //1min
			{
				g_SysFaultMessage.bit.InvL_OverLoad = 0;
				g_SysWarningMessage.bit.InvL_OverLoad = 0;//11.1
				s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
			}
		}
		else
			s_u16Cnt_InvLCurrent_High_Fault_Back = 0;
	}
}

/**************************************************************************************************
* FUNCION :  InvHParallelCurCheck
* PURPOSE :  Parallel current check
*************************************************************************************************/
void InvHParallelCurCheck(void)
{
	static Uint8 u8para_ave_faultH = 0;

	if(0 == g_SysFaultMessage.bit.unrecoverInvHCurrSharFault && g_Sys_Current_State == NormalState)
	{
		if(fabs(Calc_Result.f32IOutH_rms - Parallel_Reg.f32IInvH_para_ave) > SafetyReg.f32InvHParaCurDeviationLimit )
		{
			u8para_ave_faultH++;
			if (u8para_ave_faultH == SafetyReg.u16Para_Ave_ProtectionTime)
			{
				g_SysFaultMessage.bit.unrecoverInvHCurrSharFault = 1;
				u8para_ave_faultH = 0;
			}
		}
		else
			u8para_ave_faultH = 0;
	}
}
/**************************************************************************************************
* FUNCION :  InvLParallelCurCheck
* PURPOSE :  Parallel current check
*************************************************************************************************/
void InvLParallelCurCheck(void)
{
	static Uint8 u8para_ave_faultL = 0;

	if(0 == g_SysFaultMessage.bit.unrecoverInvLCurrSharFault && g_Sys_Current_State == NormalState)
	{
		if(fabs(Calc_Result.f32IOutL_rms - Parallel_Reg.f32IInvL_para_ave) > SafetyReg.f32InvLParaCurDeviationLimit)
		{
			u8para_ave_faultL++;
			if (u8para_ave_faultL == SafetyReg.u16Para_Ave_ProtectionTime)
			{
				g_SysFaultMessage.bit.unrecoverInvLCurrSharFault = 1;
				u8para_ave_faultL = 0;
			}
		}
		else
			u8para_ave_faultL = 0;
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

	if ((g_SysFaultMessage.bit.recoverSW_Bus_UVP == 0) && (g_StateCheck.bit.PfcSoftStart == 1))
	{
		if ((Calc_Result.f32VBus) < SafetyReg.f32VBus_LowLimit)  //660V
		{
			s_u16Cnt_BusVolt_Low_Fault++;
			if (s_u16Cnt_BusVolt_Low_Fault >= SafetyReg.u16VBusProtectionTime)      //100*0.02s = 2s
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
		if ((Calc_Result.f32VBus) >= SafetyReg.f32VBus_HiLimit) //900V
		{
			s_u16Cnt_BusVolt_Ovr_Fault++;
			if (s_u16Cnt_BusVolt_Ovr_Fault > SafetyReg.u16VBusProtectionTime) //100*0.02s = 2s
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
	static Uint16 s_u16Cnt_Bus_Unbal_Fault = 0;

	if ((g_SysFaultMessage.bit.BusVoltUnbalanceFault == 0) && (g_StateCheck.bit.PfcSoftStart == 1))
	{
		if (fabs(Calc_Result.f32VBusP - Calc_Result.f32VBusN) > 30)
		{
			s_u16Cnt_Bus_Unbal_Fault++;
			if (s_u16Cnt_Bus_Unbal_Fault >= 100)      //50*20ms=1s
			{
				s_u16Cnt_Bus_Unbal_Fault = 0;
				g_SysFaultMessage.bit.BusVoltUnbalanceFault = 1;
			}
		}
		else
			s_u16Cnt_Bus_Unbal_Fault = 0;
	}
	else
		s_u16Cnt_Bus_Unbal_Fault = 0;
}

/**********************************************************************
* FUNCION :  InvVolt Check
* PURPOSE :  check inverter voltage
**********************************************************************/
void  InvHVoltCheck(void)
{
	static Uint16 s_u16Cnt_InvVoltH_Ovr_Fault = 0;
	static Uint16 s_u16Cnt_InvVoltH_Low_Fault = 0;
	static Uint16 s_u16Cnt_InvVoltH_Low_Fault_Back = 0;

	if (g_SysFaultMessage.bit.unrecoverSW_InvH_OVP == 0 && g_Sys_Current_State == NormalState)
	{
		if (Calc_Result.f32VInvH_rms > SafetyReg.f32VInvH_HiLimit ) //242V
		{
			s_u16Cnt_InvVoltH_Ovr_Fault++;
			if (s_u16Cnt_InvVoltH_Ovr_Fault >= 5)      //200ms
			{
				s_u16Cnt_InvVoltH_Ovr_Fault = 0;
				g_SysFaultMessage.bit.unrecoverSW_InvH_OVP = 1;
			}
		}
    else
    	s_u16Cnt_InvVoltH_Ovr_Fault = 0;
	}

	if (g_StateCheck.bit.VInvHUnderRating == 0)
	{
		if (Calc_Result.f32VInvH_rms < SafetyReg.f32VInvH_LowLimit && (1 == g_StateCheck.bit.Inv_SoftStart)) //202V
		{
			s_u16Cnt_InvVoltH_Low_Fault++;
			if (s_u16Cnt_InvVoltH_Low_Fault > 5)//200ms
			{
				s_u16Cnt_InvVoltH_Low_Fault = 0;
				g_StateCheck.bit.VInvHUnderRating = 1;
				g_SysWarningMessage.bit.VInvHUnderRating = 1;
			}
		}
		else
		{
			s_u16Cnt_InvVoltH_Low_Fault = 0;
		}
		s_u16Cnt_InvVoltH_Low_Fault_Back = 0;
	}
	else
	{
		if(Calc_Result.f32VInvH_rms > SafetyReg.f32VInvH_LowLimitBack) // 210V
		{
			s_u16Cnt_InvVoltH_Low_Fault_Back++;
			if (s_u16Cnt_InvVoltH_Low_Fault_Back >150)//40ms*150=6s
			{
				s_u16Cnt_InvVoltH_Low_Fault_Back = 0;
				g_StateCheck.bit.VInvHUnderRating = 0;
				g_SysWarningMessage.bit.VInvHUnderRating = 0;
			}
		}
		else
		{
			s_u16Cnt_InvVoltH_Low_Fault_Back = 0;
		}
		s_u16Cnt_InvVoltH_Low_Fault = 0;
	}
}
/**********************************************************************
* FUNCION :  InvVolt Check
* PURPOSE :  check inverter voltage
**********************************************************************/
void  InvLVoltCheck(void)
{
	static Uint16 s_u16Cnt_InvVoltL_Ovr_Fault = 0;
	static Uint16 s_u16Cnt_InvVoltL_Low_Fault = 0;
	static Uint16 s_u16Cnt_InvVoltL_Low_Fault_Back = 0;

	if (g_SysFaultMessage.bit.unrecoverSW_InvL_OVP == 0 && g_Sys_Current_State == NormalState)
	{
		if (Calc_Result.f32VInvL_rms > SafetyReg.f32VInvL_HiLimit) //121V
		{
			s_u16Cnt_InvVoltL_Ovr_Fault++;
			if (s_u16Cnt_InvVoltL_Ovr_Fault >= 5 )      //200ms
			{
				s_u16Cnt_InvVoltL_Ovr_Fault = 0;
				g_SysFaultMessage.bit.unrecoverSW_InvL_OVP = 1;
			}
		}
		else
			s_u16Cnt_InvVoltL_Ovr_Fault = 0;
	}

	if (g_StateCheck.bit.VInvLUnderRating == 0)
	{
		if (Calc_Result.f32VInvL_rms < SafetyReg.f32VInvL_LowLimit && (1 == g_StateCheck.bit.Inv_SoftStart))//101V
		{
			s_u16Cnt_InvVoltL_Low_Fault++;
			if (s_u16Cnt_InvVoltL_Low_Fault > 5)//200ms
			{
				s_u16Cnt_InvVoltL_Low_Fault = 0;
				g_StateCheck.bit.VInvLUnderRating = 1;
				g_SysWarningMessage.bit.VInvLUnderRating = 1;
			}
		}
		else
		{
			s_u16Cnt_InvVoltL_Low_Fault = 0;
		}
		s_u16Cnt_InvVoltL_Low_Fault_Back = 0;
	}
	else
	{
		if(Calc_Result.f32VInvL_rms > SafetyReg.f32VInvL_LowLimitBack)//105V
		{
			s_u16Cnt_InvVoltL_Low_Fault_Back++;
			if (s_u16Cnt_InvVoltL_Low_Fault_Back >150)//6s
			{
				s_u16Cnt_InvVoltL_Low_Fault_Back = 0;
				g_StateCheck.bit.VInvLUnderRating = 0;
				g_SysWarningMessage.bit.VInvLUnderRating = 0;
			}
		}
		else
		{
			s_u16Cnt_InvVoltL_Low_Fault_Back = 0;
		}
		s_u16Cnt_InvVoltL_Low_Fault = 0;
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

	if  ( 1 == g_StateCheck.bit.Inv_SoftStart && Calc_Result.f32VOutH_rms>= 190 )
	{
		 // when relay picks up, there will be several period for PLL function to lock right phase of load voltage
		if (u8temp1 >= 20)  //20 * 40ms = 800ms
		{
			if(fabs(OutPLLConReg.f32Theta - VOutHPLLConReg.f32Theta) >=  f32phase_diff && \
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
	if (g_Sys_Current_State == NormalState)
	{
		if(Calc_Result.f32VGrid_rms < VGridLowLimit2 )
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
void OutputCurrentLimit(void)
{
	float32	f32CurrLimitTemp1;
	float32 f32CurrLimitTemp2;
	float32 f32CurrLimitTemp3;
	float32 f32CurrLimitTemp4;

	if(1 == g_StateCheck.bit.DcFanFault)
	{
		SafetyReg.f32IInvH_Hi2Limit = Rated_InvH_OutputCurrentRms * 0.65f;
		SafetyReg.f32IInvH_Hi1Limit = Rated_InvH_OutputCurrentRms * 0.6f;
		SafetyReg.f32IInvL_Hi2Limit = Rated_InvL_OutputCurrentRms * 0.65f;
		SafetyReg.f32IInvL_Hi1Limit = Rated_InvL_OutputCurrentRms * 0.6f;
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

		SafetyReg.f32IInvH_Hi2Limit = Rated_InvH_OutputCurrentRms * f32CurrLimitTemp3;
		SafetyReg.f32IInvH_Hi1Limit = Rated_InvH_OutputCurrentRms * f32CurrLimitTemp4;
		SafetyReg.f32IInvL_Hi2Limit = Rated_InvL_OutputCurrentRms * f32CurrLimitTemp3;
		SafetyReg.f32IInvL_Hi1Limit = Rated_InvL_OutputCurrentRms * f32CurrLimitTemp4;
	}
}

/**********************************************************************
* FUNCION :  Over Temperature Limit
* PURPOSE :  Over Temperature load Limited
**********************************************************************/
void OverTemperatureLimit(void)
{
    static	Uint16 s_u16Cnt_HeatsinkTemp_High_Fault = 0;
    static  	Uint16 s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
    float32 f32tempMax;

    if(Calc_Result.f32TempInvMax > Calc_Result.f32TempPFC)
    	f32tempMax = Calc_Result.f32TempInvMax;
    else
    	f32tempMax = Calc_Result.f32TempPFC;

	if((0 == g_SysFaultMessage.bit.InvTempOverLimit) && (0 == g_SysFaultMessage.bit.PfcOverTempFault))
	 {
		if(f32tempMax >= PowerDerate_Reg.f32Heatsink_OverTemperature_Limit)//85
		{
			s_u16Cnt_HeatsinkTemp_High_Fault++;
			if(s_u16Cnt_HeatsinkTemp_High_Fault > 5) //40ms*5=200ms
			{
				 if(Calc_Result.f32TempInvMax > Calc_Result.f32TempPFC)
					 g_SysFaultMessage.bit.InvTempOverLimit = 1;
				 else
					 g_SysFaultMessage.bit.PfcOverTempFault = 1;

				g_SysWarningMessage.bit.OverTemp = 1;
				s_u16Cnt_HeatsinkTemp_High_Fault = 0;
				s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
			}
		}
		else if(f32tempMax >= PowerDerate_Reg.f32Heatsink_DeratingTemperature_Limit)//75
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
		if(f32tempMax < (PowerDerate_Reg.f32Heatsink_OverTemperature_Limit - 30))//55
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
			s_u16Cnt_HeatsinkTemp_Fault_Back = 0;
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
		if((Calc_Result.f32VBus) >= (Calc_Result.f32VGrid_rms * 2 * 1.13))
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
void HwInvHOCPCheck(void)
{
    static Uint16 s_u16Cnt_InvOCP = 0;
	if(1 == g_StateCheck.bit.OCP_InvH)
	{
		s_u16Cnt_InvOCP++;
		if(s_u16Cnt_InvOCP >= 1)
		{
			g_SysFaultMessage.bit.recoverHW_InvH_OCP = 1;
			s_u16Cnt_InvOCP = 0;
		}
		g_StateCheck.bit.OCP_InvH = 0;
	}
}

// 110V Output Inverter Over Current Hardware Protection
void HwInvLOCPCheck(void)
{
    static Uint16 s_u16Cnt_InvOCP = 0;
	if(1 == g_StateCheck.bit.OCP_InvL)
	{
		s_u16Cnt_InvOCP++;
		if(s_u16Cnt_InvOCP >= 1)
		{
			g_SysFaultMessage.bit.recoverHW_InvL_OCP = 1;
			s_u16Cnt_InvOCP = 0;
		}
		g_StateCheck.bit.OCP_InvL = 0;
	}
}

// 220V Output Inverter Over Voltage Hardware Protection
void HwInvHOVPCheck(void)
{
    static Uint16 s_u16Cnt_InvOVP = 0;
	if(1 == g_StateCheck.bit.OVP_InvH)
	{
		s_u16Cnt_InvOVP++;
		if(s_u16Cnt_InvOVP >= 1)
		{
			g_SysFaultMessage.bit.unrecoverHW_InvH_OVP = 1;
			s_u16Cnt_InvOVP = 0;
		}
		g_StateCheck.bit.OVP_InvH = 0;
	}
}

// 110V Output Inverter Over Voltage Hardware Protection
void HwInvLOVPCheck(void)
{
    static Uint16 s_u16Cnt_InvOVP = 0;
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

// Bus over voltage protection
void HwBusOVPCheck(void)
{
	static Uint16 s_u16Cnt_BusOVP = 0;
    if(1 == g_StateCheck.bit.Bus_OVP_Fault)
	{
    	s_u16Cnt_BusOVP ++;
		if(s_u16Cnt_BusOVP >= 5)
		{
			g_SysWarningMessage.bit.HW_Bus_OVP = 1;
	        g_SysFaultMessage.bit.unrecoverSW_Bus_OVP = 1;
			s_u16Cnt_BusOVP = 0;
		}
		else
		{
			g_SysWarningMessage.bit.HW_Bus_OVP = 0;
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
* FUNCION :  	InvHCurrentProtectionIdentify
* PURPOSE :  	1,The CPLD runs cycle-by-cycle protection to deal with output short condition
* 							When CPLD protects 400 times within two output period, it will send a signal to DSP,
* 							if DSP get the signal, the PWM will be disabled.
* 							This function is to guarantee DSP disable the PWM, when CPLD's protection times smaller
* 							than 400.
* 						2, In extremely few cases parallel control will face some problem, which lead to large current
* 							and nearly normal voltage. This function can protect the module when this situation happens.
*****************************************************************************************************************/
void InvHCurrentProtectionIdentify(void)
{
	static Uint8 s_u8Cnt_InvH_Ovr_Current_Hi_fault1 = 0;
	static Uint8 s_u8Cnt_InvH_Ovr_Current_Hi_fault2 = 0;

	if (g_Sys_Current_State == NormalState)
	{
		if(Calc_Result.f32IInvH_rms_instant>= 20)
		{
			//If voltage is very low while the current is very large and the voltage is not very low , it will be treated as over current
			if (Calc_Result.f32VInvH_rms_instant > 30)
			{
				s_u8Cnt_InvH_Ovr_Current_Hi_fault1 ++;
				if (s_u8Cnt_InvH_Ovr_Current_Hi_fault1 == 2)
				{
					g_SysFaultMessage.bit.unrecoverHW_InvH_OCP  = 1;
					g_StateCheck.bit.ParallelInvHOCP = 1;
					s_u8Cnt_InvH_Ovr_Current_Hi_fault1 = 0;
				}
			}
			else
			{
				//If voltage is very low while the current is very large and the voltage is low , it will be treated as a short condition
				s_u8Cnt_InvH_Ovr_Current_Hi_fault2 ++;
				if (s_u8Cnt_InvH_Ovr_Current_Hi_fault2 == 2)
				{
					g_StateCheck.bit.OCP_InvH = 1;
					s_u8Cnt_InvH_Ovr_Current_Hi_fault2 = 0;
				}
			}
		}
		else
		{
			s_u8Cnt_InvH_Ovr_Current_Hi_fault1 = 0;
			s_u8Cnt_InvH_Ovr_Current_Hi_fault2 = 0;
		}
	}
}

void InvLCurrentProtectionIdentify(void)
{
	static Uint8 s_u8Cnt_InvL_Ovr_Current_Hi_fault1 = 0;
	static Uint8 s_u8Cnt_InvL_Ovr_Current_Hi_fault2 = 0;

	if (g_Sys_Current_State == NormalState)
	{
		if(Calc_Result.f32IInvL_rms_instant >= 25)
		{
			//If voltage is very low while the current is very large and the voltage is not very low , it will be treated as over current
			if (Calc_Result.f32VInvL_rms_instant > 20)
			{
				s_u8Cnt_InvL_Ovr_Current_Hi_fault1 ++;
				if (s_u8Cnt_InvL_Ovr_Current_Hi_fault1 == 2)
				{
					g_SysFaultMessage.bit.unrecoverHW_InvL_OCP  = 1;
					g_StateCheck.bit.ParallelInvLOCP = 1;
					s_u8Cnt_InvL_Ovr_Current_Hi_fault1 = 0;
				}
			}
			else
			{
				//If voltage is very low while the current is very large and the voltage is low , it will be treated as a short condition
				s_u8Cnt_InvL_Ovr_Current_Hi_fault2 ++;
				if (s_u8Cnt_InvL_Ovr_Current_Hi_fault2 == 2)
				{
					g_StateCheck.bit.OCP_InvL = 1;
					s_u8Cnt_InvL_Ovr_Current_Hi_fault2 = 0;
				}
			}
		}
		else
		{
			s_u8Cnt_InvL_Ovr_Current_Hi_fault1 = 0;
			s_u8Cnt_InvL_Ovr_Current_Hi_fault2 = 0;
		}
	}
}

/*******************************************************************************************************
* FUNCION :  ShortRecover
* PURPOSE :  ShortRecover
******************************************************************************************************/
void ShortRecover(void)
{
	static Uint16 u16Restart_time_interval = 0;
	if((1 == g_SysFaultMessage.bit.recoverHW_InvH_OCP)||(1 == g_SysFaultMessage.bit.recoverHW_InvL_OCP))
	{
		u16Restart_time_interval ++;
		if(SafetyReg.u16Short_Restart_times<=2)
		{
			//After short protection, the module will restart with 22V and 16.5V output voltage to test
			//whether the output is still shorted
			if(u16Restart_time_interval==5000)  //2ms * 5000 = 10s
			{
				g_SysFaultMessage.bit.recoverHW_InvH_OCP = 0;
				g_SysFaultMessage.bit.recoverHW_InvL_OCP = 0;
				DRV_RST_ON;
				u16Restart_time_interval = 0;
				SafetyReg.u16Short_Restart_times ++;
				SafetyReg.f32InvH_VoltRms_Ref = 0.1 * InvH_RatedVolt_Ref;
				SafetyReg.f32InvL_VoltRms_Ref = 0.15 * InvL_RatedVolt_Ref;
				//InvHVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;
				//InvLVoltConReg.f32VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;
			}
		}
		else
		{
			u16Restart_time_interval = 0;
			SafetyReg.u16Short_Restart_times = 0;
			g_SysFaultMessage.bit.unrecoverHW_InvH_OCP = 1;
			g_SysFaultMessage.bit.unrecoverHW_InvL_OCP = 1;
			g_StateCheck.bit.ShortInv = 1;
		}
	}
	else
	{
		DRV_RST_OFF;
		Output_VoltRe_Reg.u8InvL_Light_Flag = 0;
		Output_VoltRe_Reg.u8InvL_Middle_Flag = 0;
		Output_VoltRe_Reg.u8InvL_Heavy_Flag = 0;
		Output_VoltRe_Reg.u8InvH_Light_Flag = 0;
		Output_VoltRe_Reg.u8InvH_Middle_Flag = 0;
		Output_VoltRe_Reg.u8InvH_Heavy_Flag = 0;
	}
}
