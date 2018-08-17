/*=============================================================================*
 *         Copyright(c) 2009-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : 50KW_main.c 
 *   
 *  PURPOSE  : This document  include  initializtion of CPU system , ad sampling, three phase pwm output,Peripheral Interrupt Expansion equal,
 *                        system default parameter  defined , state change funtion,sofeware period  interrupt.
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *
 * 	Software Development Environment:
 *	-- CCS 5.4
 * 	-- BIOS 5.42
 *	--  
 *    
 *============================================================================*/


//Main including header files
#include "DSP2833x_Device.h"			       // Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file
 

//--- Global variables
Uint16 g_u16DelayTimeSys = 0;
Uint8 u_cnt = 0;
Uint32 hosttemp = 0;


//--- Global functions
void SysParamDefault(); 

/* EEPROM related function */
void Ref_ReadFromEEPROM();
void Ref_WriteToEEPROM();
void Times_ReadFromEEPROM();
void Times_WriteToEEPROM();
void Sample_ReadFromEEPROM();
void Sample_WriteToEEPROM();


/********************************************************************************************************
* FUNCION : 	void main(void)
* PURPOSE : 	Initializtion of CPU system , AD sampling, three phase pwm output, Peripheral
* 						Interrupt Expansion equal,and  system default parameter  defined
********************************************************************************************************/
void main(void)
{
    //--- CPU Initialization HWADFault
    InitSysCtrl();						// Initialize the CPU
    InitPieCtrl();		 				// Initialize and enable the PIE
    InitWatchdog();						// Initialize the Watchdog Timer
    InitGpio();							// Initialize the shared GPIO pins
    //GpioDataRegs.GPASET.bit.GPIO1 = 1;
	InitScib(TEST_COMM_BAUDRATE, MODE_INT);
	InitScic(TEST_COMM_BAUDRATE, MODE_INT);								// MASTER_COMM_BAUDRATE
	//	InitScia(COMMUNICATION_BAUDRATE, MODE_INT);

	// Initialize the Flash and OTP, reserved for SCIa boot and on-line flash
	memcpy(	&g_u16InitFlash_InRAM_runstart,
			&g_u16InitFlash_InRAM_loadstart,
			&g_u16InitFlash_InRAM_loadend - &g_u16InitFlash_InRAM_loadstart);

	InitFlash();						// Initialize the Flash
	
	memcpy(	&g_u16ControlLoopInRAM_runstart,
			&g_u16ControlLoopInRAM_loadstart,
			&g_u16ControlLoopInRAM_loadend - &g_u16ControlLoopInRAM_loadstart);

	MemMoveForFlashProgramming();
	Eeprom_Gpio_Init();           // Initialize the I2C.
	SysParamDefault();

    InitAdc();							// Initialize necessary ADC module, directly for SVPWM.
    InitEPwm();							// Initialize the PWM, Note, ALL PWM output must be disabled.
    InitECap();							// Initialize the Capture module, to measure Grid frequency.
  	InitECanGpio();                     // Initialize the ECan GPIO.  2017.10.26 GX
  	InitECan();							// Initialize the ECan.  2017.10.26 GX
 	InitECana();						// Initialize the ECana.  2017.10.26 GX

    SetDBGIER(IER | 0x6000);							// Enable everything in IER, plus INT14 and DLOGINT
    *(volatile unsigned int *)0x00000C14 |= 0x0C00;		// Set TIMER2 FREE=SOFT=1

#ifdef DebugVersion
    LOG_printf(&trace, " End of Main Initialization. ");
#endif
  

} // end of main()

/***************************************************************************************************************
* FUNCION : System Default parameter 
* PURPOSE : Check safety regulation, set initializtion parameter
***************************************************************************************************************/
void SysParamDefault(void)
{
 
	PfcPWMOutputsDisable();
	InvPWMOutputsDisable();

	i16Cnt_SysParaTemp = 0;

	ADGain.f32IGrid = IGridMeasureGain;	//2017.8.16 GX IGridMeasureGain changed
	ADGain.f32VGrid = VGridMeasureGain;


	ADGain.f32IInv = IInvMeasureGain;
	ADGain.f32VInv = VInvMeasureGain;
	ADGain.f32VOut = VInvMeasureGain;

	ADGain.f32VBusP = VBusMeasureGain;
	ADGain.f32VBusN = VBusMeasureGain;

	ADCorrection.f32IGrid = 1;
	ADCorrection.f32VGrid = 1;
	ADCorrection.f32IOut =1;

	ADCorrection.f32IInv = 1.0f;
	ADCorrection.f32VInv = 1.0f;
	ADCorrection.f32VOut = 1;

	ADCorrection.f32VBusP= 1;
	ADCorrection.f32VBusN = 1;
	ADCorrection.f32TempPFC = 1;
	ADCorrection.f32TempInv = 1;

	PowerDerate_Reg.f32Ambient_OverTemperature_Limit = DeratingAmbientTemperature;
	PowerDerate_Reg.f32ACPowerDerating_VRate = ACPowerDerating_VoltageRate;
	PowerDerate_Reg.f32ACPowerDerating_HTRate = ACPowerDerating_HeathinkTempRate;
	PowerDerate_Reg.f32Heatsink_DeratingTemperature_Limit = DeratingHeatsinkTemperatureLimit;
	PowerDerate_Reg.f32Heatsink_OverTemperature_Limit = ShutHeatsinkTemperatureLimit;
	PowerCon_Reg.f32OutputWatt = 0;
	PowerCon_Reg.f32OutputWatt1 = 0;
	PowerCon_Reg.f32OutputWatt2 = 0;
	PowerCon_Reg.f32OutputWatt3 = 0;
	PowerCon_Reg.f32OutputWatt4 = 0;

	SafetyReg.f32FreGrid_HiLimit = FreGridHiLimit;
	SafetyReg.f32FreGrid_LowLimit = FreGridLowLimit;
	SafetyReg.f32FreGrid_ProtectionTime = FreGridProtectionTime;

	SafetyReg.f32ReConnetionTime = SafetyReConnetionTime;

	SafetyReg.f32VGrid_HiLimit = VGridHiLimit;
	SafetyReg.f32VGrid_LowLimit = VGridLowLimit;
	SafetyReg.f32VGrid_HiLimitBack = VGridHiLimitBack;
	SafetyReg.f32VGrid_LowLimitBack = VGridLowLimitBack;
	SafetyReg.f32VGrid_HighProtectionTime = VGridHighProtectionTime;
	SafetyReg.f32VGrid_LowProtectionTime = VGridLowProtectionTime;

	SafetyReg.f32Inv_VoltRms_Ref_LCD = Inv_VoltRef;
	SafetyReg.f32Inv_VoltRms_Ref = SafetyReg.f32Inv_VoltRms_Ref_LCD;

	SafetyReg.f32VInv_HiLimit = VInvHiLimit;
	SafetyReg.f32VInv_LowLimit = VInvLowLimit;
	SafetyReg.f32VInv_LowLimitBack = VInvLowLimitBack;

	SafetyReg.f32VBus_HiLimit = Bus_Over_Volt_Limit;
	SafetyReg.f32VBus_LowLimit = Bus_Under_Volt_Limit;
	SafetyReg.f32VBus_ProtectionTime = BusVoltProtectionTime;

	SafetyReg.f32IInv_Hi2Limit = Rated_Inv_OutputCurrentRms * 1.3;
	SafetyReg.f32IInv_Hi1Limit = Rated_Inv_OutputCurrentRms * 1.2;
	SafetyReg.f32IInv_Hi3Limit = Rated_Inv_OutputCurrentRms * 1.5;
	SafetyReg.f32IInv_Hi4Limit = Rated_Inv_OutputCurrentRms * 2;
	SafetyReg.f32IInv_Hi2ProtectionTime = IInvHi2ProtectionTime;
	SafetyReg.f32IInv_Hi1ProtectionTime = IInvHi1ProtectionTime;
	SafetyReg.f32IInv_Hi3ProtectionTime = IInvHi3ProtectionTime;
	SafetyReg.f32IInv_HiLimitBackTime = IInvHiLimitBackTime;

	SafetyReg.f32IGrid_HiLimit = OverRated_InputCurrentPeak;

	SafetyReg.f32InvTemp_HiLimit = ShutInvTemperatureLimit;
	SafetyReg.f32AcFanEnableTemp_Limit = AcFanEnableTemperatureLimit;
	SafetyReg.f32AcFanDisableTemp_Limit = AcFanDisableTemperatureLimit;

	SafetyReg.i16SafeCountry = 1;
	SafetyReg.u16EnergyAcquisitionPeriod = EnergyAcquisitionPeriod;

	Output_VoltRe_Reg.Inv_Heavy_Flag = 0;
	Output_VoltRe_Reg.Inv_Light_Flag = 0;
	Output_VoltRe_Reg.Inv_Middle_Flag = 0;


	ADChannelOffset.f32VBusP = 1.0f;
	ADChannelOffset.f32VBusN = 1.0f;

	ADChannelOffset.f32BTS = 0;

	ADChannelOffset.f32IGrid = 0;
	ADChannelOffset.f32VGrid = 0;

	ADChannelOffset.f32IInv = 0;
	ADChannelOffset.f32VInv = 0;
	ADChannelOffset.f32VOut = 0;

	ADChannelOffset.f32TempPFC = 0;
	ADChannelOffset.f32TempInv = 0;

	GeneralADbuffer.i16ADSelectCounter = 0;


	// before enter normal mode, bus voltage controller,current controller should be initiated
	BusCon_Reg.f32BusVolt_Ref = 850;
	BusCon_Reg.f32BusFliter = 0;
	BusCon_Reg.f32BusVolt_Cmd = 0;
	BusCon_Reg.f32BusVolt_Fdb = 0;
	BusCon_Reg.f32BusVoltErr_Old = 0;
	BusCon_Reg.f32BusVoltErr_New = 0;
	BusCon_Reg.f32IGridAmp_Ref = 0;
	BusCon_Reg.PWMen_or_dis = 0;
	BusCon_Reg.f32IGridAmp_Ref = 40;

	CurrConReg.f32IGrid_Ref = 0;
	CurrConReg.f32IGrid_Fdb = 0;
	CurrConReg.f32IGridErr_Old = 0;
	CurrConReg.f32IGridErr_New = 0;
	CurrConReg.f32PfcDuty_Con = 0;
	CurrConReg.Driveopen = 0;
	CurrConReg.f32PfcDuty_ff_factor = 1;
	CurrConReg.f32Kp = 40;
	CurrConReg.f32Ki = 3;

 	GridPLLConReg.f32Theta = 0;
	GridPLLConReg.f32Vd = 0;
	GridPLLConReg.f32Vq = 0;
	GridPLLConReg.f32VqErr_Old = 0;
	GridPLLConReg.f32VqErr_New = 0;
	GridPLLConReg.f32Theta_Step = GridTheta_StepRated;
	GridPLLConReg.f32Vd_Filter = 0;
	GridPLLConReg.f32PLL_fail = 0;
	GridPLLConReg.f32Theta_Offset = 0;
	GridPLLConReg.f32real_Sin_Theta = 0;

	GridPLLReg.Input[0] = 0;
	GridPLLReg.Input[1] = 0;
	GridPLLReg.Input[2] = 0;
	GridPLLReg.Output[0] = 0;
	GridPLLReg.Output[1] = 0;
	GridPLLReg.Output[2] = 0;
	GridPLLReg.MAC = 0;
	GridPLLReg.f32Refer = 0;
	GridPLLReg.f32PIDErr_Old = 0;
	GridPLLReg.f32PIDErr_New = 0;
	GridPLLReg.f32PID_Output = 0;
	GridPLLReg.f32Delta_Theta = 0;
	GridPLLReg.f32Frequency = 0;
	GridPLLReg.f32Theta = 0;
	//2017.4.10 GX

	VOutPLLConReg.f32Theta = 0;
	VOutPLLConReg.f32Vd = 0;
	VOutPLLConReg.f32Vq = 0;
	VOutPLLConReg.f32VqErr_Old = 0;
	VOutPLLConReg.f32VqErr_New = 0;
	VOutPLLConReg.f32Theta_Step = VOutTheta_StepRated;
	VOutPLLConReg.f32Vd_Filter = 0;
	VOutPLLConReg.f32PLL_fail = 0;
	VOutPLLConReg.f32Theta_Offset = 0;
	VOutPLLConReg.f32real_Sin_Theta = 0;

	VOutPLLReg.Input[0] = 0;
	VOutPLLReg.Input[1] = 0;
	VOutPLLReg.Input[2] = 0;
	VOutPLLReg.Output[0] = 0;
	VOutPLLReg.Output[1] = 0;
	VOutPLLReg.Output[2] = 0;
	VOutPLLReg.MAC = 0;
	VOutPLLReg.f32Refer = 0;
	VOutPLLReg.f32PIDErr_Old = 0;
	VOutPLLReg.f32PIDErr_New = 0;
	VOutPLLReg.f32PID_Output = 0;
	VOutPLLReg.f32Delta_Theta = 0;
	VOutPLLReg.f32Frequency = 0;
	VOutPLLReg.f32Theta = 0;
    //2017.7.24 WF



	InvVoltConReg.f32VoltRms_Ref = Inv_VoltRef;
	InvVoltConReg.f32VoltRms_Fdb = 0;
	InvVoltConReg.f32VoltInst_Ref = 0;
	InvVoltConReg.f32VoltInst_Fdb = 0;
	InvVoltConReg.f32VoltInst_OutMax  = Rated_Inv_OutputCurrentPeak * 1.2f;
	InvVoltConReg.f32VoltInst_OutMin  = - (Rated_Inv_OutputCurrentPeak * 1.2f);
	InvCurrConReg.f32InvCurr_Ref = 0;
	InvCurrConReg.f32InvCurr_Fdb = 0;
	InvVoltConReg.Input[0] = 0;
	InvVoltConReg.Input[1] = 0;
	InvVoltConReg.Input[2] = 0;
	InvVoltConReg.Output[0] = 0;
	InvVoltConReg.Output[1] = 0;
	InvVoltConReg.Output[2] = 0;
	InvVoltConReg.MAC = 0;
	InvVoltConReg.f32Drop_Coeff = 1;

 	OutPLLConReg.f32Theta = 0;
  	OutPLLConReg.f32Theta_Step = InvTheta_StepRated;
  	OutPLLConReg.Period = 0;
  	InvCurrConReg.f32InvDuty = 0;
	AD_Acc.i16GridCounter = 400;
	AD_Acc.i16InvCounter = PWM_FREQ / RatedInvFrequency / 2;

	AD_Acc.f32VBusP = 0;
	AD_Acc.f32VBusN = 0;

	AD_Acc.f32BTS = 0;

	AD_Acc.f32TempPFC = 0;
	AD_Acc.f32TempInv = 0;

	AD_Acc.f32GridFreq = 0;
	AD_Acc.f32VOutFreq = 0;

	AD_Acc.f32IGrid_ave = 0;
	AD_Acc.f32VGrid_ave = 0;

	AD_Acc.f32IInv_ave = 0;
	AD_Acc.f32VInv_ave = 0;
	AD_Acc.f32VOut_ave = 0;
	AD_Acc.f32VDutyD_ave=0;//WF 2018.07.26

	AD_Acc.f32IGrid_rms = 0;
	AD_Acc.f32VGrid_rms = 0;

	AD_Acc.f32IInv_rms = 0;
	AD_Acc.f32VInv_rms = 0;
	AD_Acc.f32VOut_rms = 0;

	AD_Sum.i16GridCounter = 400;
	AD_Sum.i16InvCounter = 400;//PWM_FREQ / RatedInvFrequency/2

	AD_Sum.f32VBusP = 0;
	AD_Sum.f32VBusN = 0;

	AD_Sum.f32BTS = 0;

	AD_Sum.f32TempPFC = 0;
	AD_Sum.f32TempInv = 0;

	AD_Sum.f32GridFreq = 0;
	AD_Sum.f32VOutFreq = 0;
	AD_Sum.f32VOutLdFreq = 0;

	AD_Sum.f32IGrid_ave = 0;
	AD_Sum.f32VGrid_ave = 0;

	AD_Sum.f32IInv_ave = 0;
	AD_Sum.f32VInv_ave = 0;
	AD_Sum.f32VOut_ave = 0;
	AD_Sum.f32VDutyD_ave=0;

	AD_Sum.f32IGrid_rms = 0;
	AD_Sum.f32VGrid_rms = 0;

	AD_Sum.f32IInv_rms = 0;
	AD_Sum.f32VInv_rms = 0;
	AD_Sum.f32VOut_rms = 0;

	Calc_Result.f32VBusP = 0;
	Calc_Result.f32VBusN = 0;
	Calc_Result.f32VBus = 0;

	Calc_Result.f32BTS = 0;
	
	Calc_Result.f32TempPFC = 0;
	Calc_Result.f32TempInv = 0;

	Calc_Result.f32GridFreq = 0;
	Calc_Result.f32VOutFreq = 0;

	Calc_Result.f32IGrid_ave = 0;
	Calc_Result.f32VGrid_ave = 0;

	Calc_Result.f32IInv_ave = 0;
	Calc_Result.f32VInv_ave = 0;
	Calc_Result.f32VOut_ave = 0;
	Calc_Result.f32VDutyD_ave = 0;

	Calc_Result.f32IGrid_rms = 0;
	Calc_Result.f32IGrid_rms_ave = 0;
	Calc_Result.f32VGrid_rms = 0;
	Calc_Result.f32VGrid_rms_previous = 0;
	Calc_Result.f32VGrid_rms_shadow = 0;

	Calc_Result.f32IInv_rms = 0;
	Calc_Result.f32IOut_rms = 0;
	Calc_Result.f32IInv_rms_previous = 0;
	Calc_Result.f32VInv_rms = 0;
	Calc_Result.f32VOut_rms = 0;
	Calc_Result.f32VInv_rms_previous = 0;

	Calc_Result.Coff_Dforward2 = 0;
	Calc_Result.f32IInv_para_aver = 0;

	FanControl_Reg.u16Cnt_temp1=0;
	FanControl_Reg.u16Cnt_temp2=0;

	g_SysWarningMessage.Word.byte0 = 0;
	g_SysWarningMessage.Word.byte1 = 0;

	g_SysFaultMessage.Word.byte0 = 0;
	g_SysFaultMessage.Word.byte1 = 0;
	g_SysFaultMessage.Word.byte2 = 0;
	g_SysFaultMessage.Word.byte3 = 0;
	g_SysFaultMessage.Word.byte4 = 0;
	g_SysFaultMessage.Word.unrecover1 = 0;
	g_SysFaultMessage.Word.unrecover2 = 0;
	g_SysFaultMessage.Word.unrecover3 = 0;

	g_StateCheck.Word.byte0 = 0;
	g_StateCheck.Word.byte1 = 0;
	g_StateCheck.Word.byte2 = 0;
	g_StateCheck.Word.byte3 = 0;
	g_StateCheck.Word.byte4 = 0;
	g_StateCheck.Word.byte5 = 0;
	g_StateCheck.Word.byte6 = 0;

	g_StateCheck.bit.GridAD_initial = 1;
	g_StateCheck.bit.InvAD_initial = 1;

	g_ParaLogic_State.Word.byte0 = 0;
	g_ParaLogic_State.Word.byte1 = 0;

	Parallel_Reg.u16Cnt_COM1_Receive = 0;
	Parallel_Reg.u16Cnt_COM1_Receive_max = 0;
	Parallel_Reg.u16Cnt_SCR_ON = 0;
//Controller parameter

//Power limit parameter

	ShortCheck_Reg.Restart_time_interval= 0;
	ShortCheck_Reg.Restart_times = 0;
	InvRestartCheckReg.temp1 = 0;


    ComFlag.SciRcvStart=0;
	ComFlag.Arrival_40ms=0;
	ComFlag.OverTimeFlag=0;
	ComFlag.OverTimeDelay=0;
	ComFlag.OverTimeStartFlag=0;

	ComCnt.SaveFactorySettings = 0;
	ComCnt.RestoreFactorySettings = 0;
	ComCnt.SaveFactorySettingsStart = 0;
	ComCnt.RestoreFactorySettingsStart = 0;
	ComCnt.Cnt_Control_WriteToEEPROM = 0;
    ComCnt.Cnt_Control_ReadFromEEPROM = 0;
   

    //ECan 中间变量
    Ecan_ModuleData.Input_Volt_Rms = 0X0000;
    Ecan_ModuleData.Input_Curr_Rms = 0X0000;
    Ecan_ModuleData.BusP_Volt = 0X0000;

    Ecan_ModuleData.BusN_Volt = 0X0000;
    Ecan_ModuleData.Out_Volt_Rms = 0X0000;
    Ecan_ModuleData.Inv_Volt_Rms = 0X0000;

    Ecan_ModuleData.Inv_Cur_Rms = 0X0000;
    Ecan_ModuleData.PFC_Temp = 0X0000;
    Ecan_ModuleData.Inv_Temp = 0X0000;

    Ecan_ModuleData.Inv_Freq = 0X0000;
    Ecan_ModuleData.RunTimeHour_L = 0X0000;
    Ecan_ModuleData.RunTimeHour_H = 0X0000;

    Ecan_ModuleData.Error.Byte.Alert = 0X00;
    Ecan_ModuleData.Error.Byte.Fault1 = 0X00;
    Ecan_ModuleData.Error.Byte.Fault2 = 0X0000;
    Ecan_ModuleData.Error.Byte.Fault3 = 0X0000;
    Ecan_ModuleData.Check = 0X00;

    //ECan 上位机修正中间变量
    Ecan_SytemREVISED.Input_Volt_Rms = 0;
    Ecan_SytemREVISED.Bus_P = 0;
    Ecan_SytemREVISED.Bus_N = 0;

    Ecan_SytemREVISED.Inv_Volt_Rms = 0;
    Ecan_SytemREVISED.Inv_Cur_Rms = 0;
    Ecan_SytemREVISED.Inv_OutV_Rms = 0;

    Ecan_SytemREVISED.PFC_Temp = 0;
    Ecan_SytemREVISED.Inv_Temp = 0;
    Ecan_SytemREVISED.Input_Cur_Rms = 0;

    Ecan_SytemREVISED.Aver_Curr_Inv = 0;
    Ecan_SytemREVISED.RestartOverTimes = 0;
    Ecan_SytemREVISED.INV_Volt_Ref = 0;
    Ecan_SytemREVISED.INV_Drop_Coeff = 0;


    //ECan 上位命令
    Ecan_SytemOrder.rsvr0 = 0;
    Ecan_SytemOrder.Defaluts = 0xFFFF;
    Ecan_SytemOrder.Output_Enable = 0;
    Ecan_SytemOrder.rsvr1 = 0;

    g_StateCheck.bit.ECANPWMEnable = 0;

    Ecan_Error.u8Upload_Trans_Error = 0;
    Ecan_Error.u8Broadcast_Trans_Error = 0;

   	RunningTime.Hour_H = 0;
   	RunningTime.Hour_L = 0;
   	RunningTime.Minute = 0;
   	RunningTime.Second = 0;
   	RunningTime.OverFlow = 0;
   	RunningTime.TimeCheck = 0;

	#ifdef NORMAL_EEPROM
 		Times_ReadFromEEPROM();
 		Ref_ReadFromEEPROM();
 		Sample_ReadFromEEPROM();
	#endif

	#ifdef RESET_EEPROM
 		Times_WriteToEEPROM();
 		Ref_WriteToEEPROM();
 		Sample_WriteToEEPROM();
	#endif
}     



/**********************************************************************
* FUNCION : User Init
* PURPOSE : This is the user initialization file to be specified in the DSP/BIOS configuration file, System - Global Settings.
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY: DSP/BIOS configuration file-Global Settings 
* 
**********************************************************************/

void UserInit(void)
{
					// SVPWM2009_FLASH, if defined, is in CCS project options

// Section .trcdata is generated by DSP/BIOS.
// It must be copied from its load to its run address BEFORE main().
// It should be removed in release version if not necessary.
	memcpy(	&trcdata_runstart, 
			    &trcdata_loadstart, 
			    &trcdata_loadend - &trcdata_loadstart);
} 

/**********************************************************************
* FUNCION :  SEM post  when periodic   timerBase of 2ms   is ready
* PURPOSE :  SEM post  for  state changed
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  DSP/BIOS kernel 2ms periodic  call 
* 
**********************************************************************/
void TimeBase2msPRD(void)
{
	FanCntl(); //ZJX 2017.11.06
	SEM_post(&TimeBase2msReady);
}

/**********************************************************************
* FUNCION :  SEM post  when  periodic  timerBase of 10ms  is ready
* PURPOSE :  SEM post  for  MPPT or anti islanding detection
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  DSP/BIOS kernel 10ms periodic  call 
* PRD_TimeBase500ms
**********************************************************************/
void TimeBase500msPRD(void)
{		
	static Uint16 s_count = 0;

	SEM_post(&SEM_TimeBase500ms);
	SEM_post(&SEM_ECAN);
	
	if (ModuleAdd != 0XFF)//0 == SYNC_COM2_LEVEL && 2018.4.9 GX
	{
		s_count++;
		if (s_count == 4)
		{
			if (hosttemp == u8_hostdrop && g_Mod_Status != Master)
			{
				if (u_cnt >= 2 && u_cnt < 5)
					g_Mod_Status = idle;
				else if (u_cnt >= 5)
				{
					g_Mod_Status = Master;
					u_cnt = 1;
				}
				u_cnt ++ ;
			}
			else
				u_cnt = 1;
			hosttemp = u8_hostdrop;
			s_count = 0;
		}
	}
} // end of TimerBase10msPRD()


/**********************************************************************
* FUNCION :  
* PURPOSE :
* INPUT :
*        void
* RETURN :
*        void
* CALLS:
*        void
*
* CALLED BY:  DSP/BIOS kernel
**********************************************************************/
void TimeBase20msPRD(void)
{		
   static Uint16 scia_cnt=0;
   //static Uint8 scib_reset=0;
   //SEM_post(&SEM_MasterCommRx);
   SEM_post(&SEM_MasterCommTx);  

    scia_cnt++;
	if(2 == scia_cnt)
	{
		SEM_post(&SEM_SCIAComm);
		scia_cnt = 0;
		if(ScibRegs.SCICTL1.bit.SWRESET == 0)
		{
			ScibRegs.SCICTL1.bit.SWRESET = 1;
			//scib_reset ++;
		}
		/*if(scib_reset >= 100)
		{
			scib_reset = 0;
			g_SysWarningMessage.bit.LCD_Comm_Error = 1;
		}*/
	}
	   
} // end of TimerBase20msPRD()

void TimeBase100msPRD(void)
{		
	SEM_post(&SEM_Time_Record);
}
