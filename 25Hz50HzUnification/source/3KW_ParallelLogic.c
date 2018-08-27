/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_ParallelLogic.c
 *   
 *  PURPOSE  : Definition of parallel function and synchronization function
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2        001             		Li Zhang
 *    												Xun Gao
 *    												Jianxi Zhu
 *============================================================================*/


#include "DSP2833x_Device.h"				// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file


/* Configure the DATA/CODE section running in RAM from flash */
#pragma CODE_SECTION(ECAP1_INT_SyncPhase_Control, "ControlLoopInRAM")
#pragma CODE_SECTION(InvParallel_Control, "ControlLoopInRAM")

/*=============================================================================*
 * 	Variables declaration
 *============================================================================*/
struct  PARALLEL_REG  Parallel_Reg;
union	PARALLEL_STATE  g_ParaLogic_State;

/*=============================================================================*
 * 	functions declaration
 *============================================================================*/
void SyncLogic_Control(void);
void InvParallel_Control(void);
void InvRelay_Control(void);
void ECAP1_INT_SyncPhase_Control(void);

/*=============================================================================*
 * FUNCTION:	ECAP1_INT_SyncPhase_Control()
 *
 * PURPOSE:	Make the phase of output sine voltage stay the same with the  synchronizing signal
 * 						Hardware-related
 * 
 * CALLED BY:	void ECAP1_INT_ISR(void)
 *============================================================================*/
void ECAP1_INT_SyncPhase_Control(void)
{
	// start of ECAP2_INT_SyncPhase_Control

	static  Uint16  s_u16Cnt_SyncPhase_Fault = 0;
	static  Uint16  s_u16Cnt_SyncPhase_Fault_Back = 0;

	Parallel_Reg.u16Cnt_COM1_Receive = 0;
	g_ParaLogic_State.bit.InvSoftStart_EN = 1;

	if (g_Sys_Current_State != PermanentState && g_Sys_Current_State != FaultState)
	{
		if (1==SYNC_COM2_LEVEL)//COM2 low voltage (hardware-related)
			SYNC_COM2_OFF;
	}

	/*
	 * the function will be executed when synchronizing signal reaches pi
	 * if native phase does not reaches pi, it need to be adjusted
	 */
	if(OutPLLConReg.f32Theta <= SyncPhase_Low1Limit)
		OutPLLConReg.f32Theta = OutPLLConReg.f32Theta + SyncPhase_ReguStep2;
	else if(OutPLLConReg.f32Theta >= SyncPhase_Hi1Limit)
		OutPLLConReg.f32Theta = OutPLLConReg.f32Theta - SyncPhase_ReguStep2;
	else if(OutPLLConReg.f32Theta <= SyncPhase_Low4Limit)
		OutPLLConReg.f32Theta = OutPLLConReg.f32Theta + SyncPhase_ReguStep1;
	else if(OutPLLConReg.f32Theta >= SyncPhase_Hi4Limit)
		OutPLLConReg.f32Theta = OutPLLConReg.f32Theta - SyncPhase_ReguStep1;
	else
		;

	/*
	 * The native phase are estimated.
	 * It should be adjusted in a narrow range between 'SyncPhase_Low2Limit' and 'SyncPhase_Hi2Limit'
	 * If it is out of the range, defined by 'SyncPhase_Low3Limit' and 'SyncPhase_Hi3Limit', it is out of sync.
	 * If it is in sync, 'g_ParaLogic_State.bit.SelfPhaseOut_EN' will be set and the native phase signal will be
	 * send to sync bus.
	 */
	if(0 == g_ParaLogic_State.bit.SyncPhase_Flag)
	{
		if((OutPLLConReg.f32Theta >= SyncPhase_Low2Limit) && (OutPLLConReg.f32Theta <= SyncPhase_Hi2Limit))
		{
			s_u16Cnt_SyncPhase_Fault_Back++;
			if(s_u16Cnt_SyncPhase_Fault_Back > 4)
			{
				g_ParaLogic_State.bit.SelfPhaseOut_EN = 1;
				g_ParaLogic_State.bit.SyncPhase_Flag = 1;
				g_SysWarningMessage.bit.InvAsyn = 0;
				s_u16Cnt_SyncPhase_Fault_Back = 0;
				s_u16Cnt_SyncPhase_Fault = 0;
			}
		}
		else
			s_u16Cnt_SyncPhase_Fault_Back = 0;
	}
	else
	{
		if((OutPLLConReg.f32Theta <= SyncPhase_Low3Limit) || (OutPLLConReg.f32Theta >= SyncPhase_Hi3Limit))
		{
			s_u16Cnt_SyncPhase_Fault++;
			if(s_u16Cnt_SyncPhase_Fault > 20)
			{
				g_ParaLogic_State.bit.SelfPhaseOut_EN = 0;
				g_ParaLogic_State.bit.SyncPhase_Flag = 0;
				g_SysWarningMessage.bit.InvAsyn = 1;
				s_u16Cnt_SyncPhase_Fault = 0;
				s_u16Cnt_SyncPhase_Fault_Back = 0;
			}
		}
		else
			s_u16Cnt_SyncPhase_Fault = 0;
	}
} // end of ECAP1_INT_SyncPhase_Control


/*=============================================================================*
 * FUNCTION:	SyncLogic_Control()
 *
 * PURPOSE:	This function is used to decide whether the module is the first one in bus
 *
 * CALLED BY:	void TSK_InvVoltPeriod(void).
 * 						Called every 40ms.
 *============================================================================*/
void SyncLogic_Control(void)
{
	// start of SyncLogic_Control

	Parallel_Reg.u16Cnt_COM1_Receive ++;

	if((Parallel_Reg.u16Cnt_COM1_Receive >= 5))	// 5 * 40ms = 200ms
	{
		//if there are already output voltages or the module is slave one, the sync line may be broken
		if(0 == g_ParaLogic_State.bit.SelfPhaseOut_EN)
		{
			// The module is the first one in bus, so it can send the native phase signal out
			g_ParaLogic_State.bit.SelfPhaseOut_EN = 1;
			SYNC_COM2_OFF;
			Parallel_Reg.u16Cnt_COM1_Receive = 0;
		}
		else
		{
			if ( (Calc_Result.f32VOutH_rms >= 200 || Calc_Result.f32VOutL_rms >= 100 ))
			{
				if(g_Sys_Current_State != FaultState  && g_Sys_Current_State != PermanentState)
				{
					g_StateCheck.bit.Sync_Fault1 = 1;
					g_SysFaultMessage.bit.unrecoverHW_SynLine_cut = 1;
					Parallel_Reg.u16Cnt_COM1_Receive = 0;
				}
			}
			else
			{
				g_ParaLogic_State.bit. SyncProblem_Flag= 1;
				g_ParaLogic_State.bit.SelfPhaseOut_EN = 0;
				Parallel_Reg.u16Cnt_COM1_Receive = 0;
				OutPLLConReg.f32Theta = 0;
				SYNC_COM1_ON;
			}
		}
	}
} // end of SyncLogic_Control

/*=============================================================================*
 * FUNCTION:	InvParallel_Control()
 *
 * PURPOSE:	This function controls the set trigger time of the relay and SCR
 *
 * CALLED BY:	void ADC_INT_INV_Control(void)
 *============================================================================*/
void InvParallel_Control(void)
{
	// start of InvParallel_Control

	if(1 == g_ParaLogic_State.bit.SyncPhase_Flag)
	{
		if(1 == g_StateCheck.bit.Inv_SoftStart)
		{
			SYNC_COM2_ON;
			if(0==SYNC_COM2_LEVEL)	//COM2 high voltage (hardware-related)
				InvRelay_Control();
		}
	}
	else
	{
		RelaysOFF();
		Parallel_Reg.u16Cnt_SCR_ON = 0;
	}
	/*
	 * In hold up time, The first protection module will pull the COM2 down. Other modules will protect,
	 * when they discover the low voltage on COM2 bus.
	 * This logic aims to prevent the slow protection module to bear all the load.
	 */
	if (1 == g_StateCheck.bit.Inv_SoftStart && Calc_Result.f32VGrid_rms < SafetyReg.f32VGrid_LowLimit && \
			1==SYNC_COM2_LEVEL)
	{
		g_SysFaultMessage.bit.VGridUnderRating = 1;
	}
} // end of InvParallel_Control

/*=============================================================================*
 * FUNCTION:	void InvRelay_Control(void)
 *
 * PURPOSE:	This function controls the set trigger time of the relay and SCR
 *
 * CALLED BY:	void InvParallel_Control(void)
 *============================================================================*/
void InvRelay_Control(void)
{
	if(Parallel_Reg.u16Cnt_SCR_ON < 2400)
	{
		Parallel_Reg.u16Cnt_SCR_ON ++;
		if(Parallel_Reg.u16Cnt_SCR_ON > 240)  //440
		{
			INVL_RELY_ON;
			if(Parallel_Reg.u16Cnt_SCR_ON > 400)  //600
			{
				InvL_CurrShare_ON;
				if(Parallel_Reg.u16Cnt_SCR_ON > 430)  //630
				{
					INVH_RELY_ON;
					if(Parallel_Reg.u16Cnt_SCR_ON > 600)  //800
						InvH_CurrShare_ON;
				}
			}
		}
		else
		{
			INVH_SCR_OFF;
			INVL_SCR_OFF;
			INVL_RELY_OFF;
			INVH_RELY_OFF;
		}
	}
} // end of InvRelay_Control

//--- end of file -----------------------------------------------------

