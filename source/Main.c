/*=============================================================================*
 *	Copyright(c) 2009-2011
 *          ALL RIGHTS RESERVED
 *
 *  FILENAME : Main.c
 *   
 *  PURPOSE  : This document  include  initializtion of CPU system , ad sampling, three phase pwm output,Peripheral Interrupt Expansion equal,
 *                      system default parameter  defined , state change funtion, sofeware period  interrupt.
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					Li Zhang
 *    												Xun Gao
 *    												Jianxi Zhu
 *============================================================================*/


#include "DSP2833x_Device.h"			       		// Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file

//--- variables
Uint16 g_u16DelayTimeSys = 0;
Uint8 	u_cnt = 1;
Uint32 hosttemp = 0;

//--- functions
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
    InitWatchdog();					// Initialize the Watchdog Timer
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

	ADGain.f32IGrid = IGridMeasureGain;
	ADGain.f32VGrid = VGridMeasureGain;

	ADGain.f32IInvH = IInvHMeasureGain;
	ADGain.f32VInvH = VInvHMeasureGain;
	ADGain.f32VOutH = VInvHMeasureGain;

	ADGain.f32IInvL = IInvLMeasureGain;
	ADGain.f32VInvL = VInvLMeasureGain;
	ADGain.f32VOutL = VInvLMeasureGain;

	ADGain.f32VBusP = VBusMeasureGain;
	ADGain.f32VBusN = VBusMeasureGain;

	ADGain.f32TempPFC = TempMeasureGain;
	ADGain.f32TempInvH = TempMeasureGain;
	ADGain.f32TempInvL = TempMeasureGain;

	ADCalibration.f32IGrid = 1;
	ADCalibration.f32VGrid = 1.0f;

	ADCalibration.f32IInvH = 1.0f;
	ADCalibration.f32VInvH = 1.0f;
	ADCalibration.f32VOutH = 1;
	ADCalibration.f32IOutH= 1.0f;

	ADCalibration.f32IInvL = 1.0f;
	ADCalibration.f32VInvL = 1.0f;
	ADCalibration.f32VOutL = 1;
	ADCalibration.f32IOutL = 1.0f;

	ADCalibration.f32VBusP= 1;
	ADCalibration.f32VBusN = 1;
	ADCalibration.f32TempPFC = 1;
	ADCalibration.f32TempInvH = 1;
	ADCalibration.f32TempInvL = 1;

	PowerDerate_Reg.f32ACPowerDerating_VRate = ACPowerDerating_VoltageRate;
	PowerDerate_Reg.f32ACPowerDerating_HTRate = ACPowerDerating_HeathinkTempRate;
	PowerDerate_Reg.f32Heatsink_DeratingTemperature_Limit = DeratingHeatsinkTemperatureLimit;
	PowerDerate_Reg.f32Heatsink_OverTemperature_Limit = ShutHeatsinkTemperatureLimit;

	SafetyReg.f32FreGrid_HiLimit = FreGridHiLimit;
	SafetyReg.f32FreGrid_LowLimit = FreGridLowLimit;
	SafetyReg.f32FreGrid_ProtectionTime = FreGridProtectionTime;

	SafetyReg.f32VGrid_HiLimit = VGridHiLimit;
	SafetyReg.f32VGrid_LowLimit = VGridLowLimit;
	SafetyReg.f32VGrid_HiLimitBack = VGridHiLimitBack;
	SafetyReg.f32VGrid_LowLimitBack = VGridLowLimitBack;
	SafetyReg.f32VGrid_HighProtectionTime = VGridHighProtectionTime;
	SafetyReg.f32VGrid_LowProtectionTime = VGridLowProtectionTime;

	SafetyReg.f32InvH_VoltRms_Ref_LCD = InvH_RatedVolt_Ref;
	SafetyReg.f32InvL_VoltRms_Ref_LCD= InvL_RatedVolt_Ref;
	SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD;
	SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD;

	SafetyReg.f32VInvH_HiLimit = VInvHHiLimit;
	SafetyReg.f32VInvL_HiLimit = VInvLHiLimit;
	SafetyReg.f32VInvH_LowLimit = VInvHLowLimit;
	SafetyReg.f32VInvL_LowLimit = VInvLLowLimit;
	SafetyReg.f32VInvH_LowLimitBack = VInvHLowLimitBack;
	SafetyReg.f32VInvL_LowLimitBack = VInvLLowLimitBack;

	SafetyReg.f32VBus_HiLimit = Bus_Over_Volt_Limit;
	SafetyReg.f32VBus_LowLimit = Bus_Under_Volt_Limit;
	SafetyReg.f32VBusProtectionTime = BusVoltProtectionTime;

	SafetyReg.f32IInvH_Hi2Limit = Rated_InvH_OutputCurrentRms * 1.3;
	SafetyReg.f32IInvH_Hi1Limit = Rated_InvH_OutputCurrentRms * 1.2;
	SafetyReg.f32IInvH_Hi3Limit = Rated_InvH_OutputCurrentRms * 1.5;
	SafetyReg.f32IInvH_Hi4Limit = Rated_InvH_OutputCurrentRms * 2;
	SafetyReg.f32IInvH_Hi2ProtectionTime = IInvHi2ProtectionTime;
	SafetyReg.f32IInvH_Hi1ProtectionTime = IInvHi1ProtectionTime;
	SafetyReg.f32IInvH_Hi3ProtectionTime = IInvHi3ProtectionTime;
	SafetyReg.f32IInvH_HiLimitBackTime = IInvHiLimitBackTime;

	SafetyReg.f32IInvL_Hi2Limit = Rated_InvL_OutputCurrentRms * 1.3;
	SafetyReg.f32IInvL_Hi1Limit = Rated_InvL_OutputCurrentRms * 1.2;
	SafetyReg.f32IInvL_Hi3Limit = Rated_InvL_OutputCurrentRms * 1.5;
	SafetyReg.f32IInvL_Hi4Limit = Rated_InvL_OutputCurrentRms * 2;
	SafetyReg.f32IInvL_Hi2ProtectionTime = IInvHi2ProtectionTime;
	SafetyReg.f32IInvL_Hi1ProtectionTime = IInvHi1ProtectionTime;
	SafetyReg.f32IInvL_Hi3ProtectionTime = IInvHi3ProtectionTime;
	SafetyReg.f32IInvL_HiLimitBackTime = IInvHiLimitBackTime;

	SafetyReg.f32IGrid_HiLimit = OverRated_InputCurrentPeak;

	SafetyReg.f32InvTemp_HiLimit = ShutInvTemperatureLimit;

	Output_VoltRe_Reg.u8InvH_Heavy_Flag = 0;
	Output_VoltRe_Reg.u8InvH_Light_Flag = 0;
	Output_VoltRe_Reg.u8InvH_Middle_Flag = 0;

	Output_VoltRe_Reg.u8InvL_Heavy_Flag = 0;
	Output_VoltRe_Reg.u8InvL_Light_Flag = 0;
	Output_VoltRe_Reg.u8InvL_Middle_Flag = 0;

	ADChannelOffset.f32VBusP = 1.0f;
	ADChannelOffset.f32VBusN = 1.0f;

	ADChannelOffset.f32IGrid = 0;
	ADChannelOffset.f32VGrid = 0;

	ADChannelOffset.f32IInvH = 0;
	ADChannelOffset.f32VInvH = 0;
	ADChannelOffset.f32VOutH = 0;

	ADChannelOffset.f32IInvL = 0;
	ADChannelOffset.f32VInvL = 0;
	ADChannelOffset.f32VOutL = 0;

	ADChannelOffset.f32TempPFC = 0;
	ADChannelOffset.f32TempInvH = 0;
	ADChannelOffset.f32TempInvL = 0;

	// before enter normal mode, bus voltage controller,current controller should be initiated
	BusCon_Reg.f32BusVolt_Ref = 850;
	BusCon_Reg.f32BusFliter = 0;
	BusCon_Reg.f32BusVolt_Cmd = 0;
	BusCon_Reg.f32BusVolt_Fdb = 0;
	BusCon_Reg.f32BusVoltErr_Old = 0;
	BusCon_Reg.f32BusVoltErr_New = 0;
	BusCon_Reg.f32IGridAmp_Ref = 0;
	BusCon_Reg.PWMen_or_dis = 0;
	BusCon_Reg.f32IGrid_RefAmp_Limit   = 28;

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

	VOutHPLLReg.Input[0] = 0;
	VOutHPLLReg.Input[1] = 0;
	VOutHPLLReg.Input[2] = 0;
	VOutHPLLReg.Output[0] = 0;
	VOutHPLLReg.Output[1] = 0;
	VOutHPLLReg.Output[2] = 0;
	VOutHPLLReg.MAC = 0;
	VOutHPLLReg.f32Refer = 0;
	VOutHPLLReg.f32PIDErr_Old = 0;
	VOutHPLLReg.f32PIDErr_New = 0;
	VOutHPLLReg.f32PID_Output = 0;
	VOutHPLLReg.f32Delta_Theta = 0;
	VOutHPLLReg.f32Frequency = 0;
	VOutHPLLReg.f32Theta = 0;

	VOutLPLLReg.Input[0] = 0;
	VOutLPLLReg.Input[1] = 0;
	VOutLPLLReg.Input[2] = 0;
	VOutLPLLReg.Output[0] = 0;
	VOutLPLLReg.Output[1] = 0;
	VOutLPLLReg.Output[2] = 0;
	VOutLPLLReg.MAC = 0;
	VOutLPLLReg.f32Refer = 0;
	VOutLPLLReg.f32PIDErr_Old = 0;
	VOutLPLLReg.f32PIDErr_New = 0;
	VOutLPLLReg.f32PID_Output = 0;
	VOutLPLLReg.f32Delta_Theta = 0;
	VOutLPLLReg.f32Frequency = 0;
	VOutLPLLReg.f32Theta = 0;

	InvHVoltConReg.f32VoltRms_Ref = InvH_RatedVolt_Ref;
	InvHVoltConReg.f32VoltRms_Fdb = 0;
	InvHVoltConReg.f32VoltInst_Ref = 0;
	InvHVoltConReg.f32VoltInst_Fdb = 0;
	InvHCurrConReg.f32InvCurr_Ref = 0;
	InvHCurrConReg.f32InvCurr_Fdb = 0;
	InvHVoltConReg.Input[0] = 0;
	InvHVoltConReg.Input[1] = 0;
	InvHVoltConReg.Input[2] = 0;
	InvHVoltConReg.Output[0] = 0;
	InvHVoltConReg.Output[1] = 0;
	InvHVoltConReg.Output[2] = 0;
	InvHVoltConReg.MAC = 0;
	InvHVoltConReg.f32Drop_Coeff = 0;

	InvLVoltConReg.f32VoltRms_Ref =  InvL_RatedVolt_Ref;
	InvLVoltConReg.f32VoltRms_Fdb = 0;
	InvLVoltConReg.f32VoltInst_Ref = 0;
	InvLVoltConReg.f32VoltInst_Fdb = 0;
	InvLCurrConReg.f32InvCurr_Ref = 0;
    InvLCurrConReg.f32InvCurr_Fdb = 0;
    InvLVoltConReg.Input[0] = 0;
    InvLVoltConReg.Input[1] = 0;
    InvLVoltConReg.Input[2] = 0;
    InvLVoltConReg.Output[0] = 0;
    InvLVoltConReg.Output[1] = 0;
    InvLVoltConReg.Output[2] = 0;
    InvLVoltConReg.MAC = 0;
	InvLVoltConReg.f32Drop_Coeff = 0;

 	OutPLLConReg.f32Theta = 0;
  	OutPLLConReg.f32Theta_Step = InvTheta_StepRated;
  	OutPLLConReg.Period = 0;
  	VOutHPLLConReg.f32Valpha = 0;
  	VOutHPLLConReg.f32Vbeta = 0;
  	VOutHPLLConReg.f32Vd = 0;
  	VOutHPLLConReg.f32Vq = 0;
  	VOutHPLLConReg.f32VqErr_Old = 0;
  	VOutHPLLConReg.f32VqErr_New = 0;
  	VOutHPLLConReg.f32Theta_Step = InvTheta_StepRated;
  	VOutLPLLConReg.f32Valpha = 0;
  	VOutLPLLConReg.f32Vbeta = 0;
  	VOutLPLLConReg.f32Vd = 0;
  	VOutLPLLConReg.f32Vq = 0;
  	VOutLPLLConReg.f32VqErr_Old = 0;
  	VOutLPLLConReg.f32VqErr_New = 0;
  	VOutLPLLConReg.f32Theta_Step = InvTheta_StepRated;


  	InvLCurrConReg.f32InvDuty = 0;
  	InvHCurrConReg.f32InvDuty = 0;
	AD_Acc.i16GridCounter = 400;
	AD_Acc.i16InvCounter = PWM_FREQ / RatedInvFrequency / 2;

	AD_Acc.f32VBusP = 0;
	AD_Acc.f32VBusN = 0;


	AD_Acc.f32TempPFC = 0;
	AD_Acc.f32TempInvH = 0;
	AD_Acc.f32TempInvL = 0;

	AD_Acc.f32GridFreq = 0;
	AD_Acc.f32VOutFreq = 0;
	AD_Acc.f32VOutHFreq = 0;
	AD_Acc.f32VOutLFreq = 0;

	AD_Acc.f32IGrid_ave = 0;
	AD_Acc.f32VGrid_ave = 0;

	AD_Acc.f32IInvH_ave = 0;
	AD_Acc.f32VInvH_ave = 0;
	AD_Acc.f32VOutH_ave = 0;

	AD_Acc.f32IInvL_ave = 0;
	AD_Acc.f32VInvL_ave = 0;
	AD_Acc.f32VOutL_ave = 0;

	AD_Acc.f32IGrid_rms = 0;
	AD_Acc.f32VGrid_rms = 0;

	AD_Acc.f32IInvH_rms = 0;
	AD_Acc.f32VInvH_rms = 0;
	AD_Acc.f32VOutH_rms = 0;

	AD_Acc.f32IInvL_rms = 0;
	AD_Acc.f32VInvL_rms = 0;
	AD_Acc.f32VOutL_rms = 0;

	AD_Acc.f32Phase_Diff_ave = 0;

	AD_Sum.i16GridCounter = 400;
	AD_Sum.i16InvCounter = PWM_FREQ / RatedInvFrequency/2;

	AD_Sum.f32VBusP = 0;
	AD_Sum.f32VBusN = 0;

	AD_Sum.f32TempPFC = 0;
	AD_Sum.f32TempInvH = 0;
	AD_Sum.f32TempInvL = 0;

	AD_Sum.f32GridFreq = 0;
	AD_Sum.f32VOutFreq = 0;
	AD_Sum.f32VOutHFreq = 0;
	AD_Sum.f32VOutLFreq = 0;

	AD_Sum.f32IGrid_ave = 0;
	AD_Sum.f32VGrid_ave = 0;

	AD_Sum.f32IInvH_ave = 0;
	AD_Sum.f32VInvH_ave = 0;
	AD_Sum.f32VOutH_ave = 0;

	AD_Sum.f32IInvL_ave = 0;
	AD_Sum.f32VInvL_ave = 0;
	AD_Sum.f32VOutL_ave = 0;

	AD_Sum.f32IGrid_rms = 0;
	AD_Sum.f32VGrid_rms = 0;

	AD_Sum.f32IInvH_rms = 0;
	AD_Sum.f32VInvH_rms = 0;
	AD_Sum.f32VOutH_rms = 0;

	AD_Sum.f32IInvL_rms = 0;
	AD_Sum.f32VInvL_rms = 0;
	AD_Sum.f32VOutL_rms = 0;

	AD_Sum.f32Phase_Diff_ave = 0;

	Calc_Result.f32VBusP = 0;
	Calc_Result.f32VBusN = 0;
	Calc_Result.f32VBus = 0;

	
	Calc_Result.f32TempPFC = 0;
	Calc_Result.f32TempInvH = 0;
	Calc_Result.f32TempInvL = 0;

	Calc_Result.f32GridFreq = 0;
	Calc_Result.f32VOutFreq = 0;
	Calc_Result.f32VOutLFreq = 0;
	Calc_Result.f32VOutHFreq = 0;

	Calc_Result.f32IGrid_ave = 0;
	Calc_Result.f32VGrid_ave = 0;

	Calc_Result.f32IInvH_ave = 0;
	Calc_Result.f32VInvH_ave = 0;
	Calc_Result.f32VOutH_ave = 0;

	Calc_Result.f32IInvL_ave = 0;
	Calc_Result.f32VInvL_ave = 0;
	Calc_Result.f32VOutL_ave = 0;

	Calc_Result.f32IGrid_rms = 0;
	Calc_Result.f32VGrid_rms = 0;
	Calc_Result.f32VGrid_rms_instant = 0;

	Calc_Result.f32IInvH_rms = 0;
	Calc_Result.f32IOutH_rms = 0;
	Calc_Result.f32VInvH_rms = 0;
	Calc_Result.f32VOutH_rms = 0;
	Calc_Result.f32VInvH_rms_instant = 0;
	Calc_Result.f32IInvH_rms_instant = 0;

	Calc_Result.f32IInvL_rms = 0;
	Calc_Result.f32VInvL_rms = 0;
	Calc_Result.f32VOutL_rms = 0;
	Calc_Result.f32IOutL_rms = 0;
	Calc_Result.f32VInvL_rms_instant = 0;
	Calc_Result.f32IInvL_rms_instant = 0;

	Calc_Result.f32Coff_Dforward = 0;
	Calc_Result.f32Phase_Diff_ave= 0;


	FanCntl_Reg.u16Cnt_OnTime=0;
	FanCntl_Reg.u16Cnt_Period=0;

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
	Parallel_Reg.u16Cnt_SCR_ON = 0;
	Parallel_Reg.f32IInvH_para_ave = 0;
	Parallel_Reg.f32IInvL_para_ave = 0;
	Parallel_Reg.f32VInvH_Comp_Coeff = 0;
	Parallel_Reg.f32VInvL_Comp_Coeff = 0;
//Controller parameter

//Power limit parameter
	ShortCheck_Reg.Restart_time_interval = 0;
	ShortCheck_Reg.Restart_times = 0;

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
   

    //ECan
    Ecan_ModuleData.u16VGrid_rms = 0X0000;
    Ecan_ModuleData.u16IGrid_rms = 0X0000;
    Ecan_ModuleData.u16VBusP = 0X0000;

    Ecan_ModuleData.u16VBusN = 0X0000;
    Ecan_ModuleData.u16VOutH_rms = 0X0000;
    Ecan_ModuleData.u16VOutL_rms = 0X0000;

    Ecan_ModuleData.u16VInvH_rms = 0X0000;
    Ecan_ModuleData.u16VInvL_rms = 0X0000;
    Ecan_ModuleData.u16IInvH_rms = 0X0000;

    Ecan_ModuleData.u16IInvL_rms = 0X0000;
    Ecan_ModuleData.u16Temp_PFC = 0X0000;
    Ecan_ModuleData.u16Temp_InvH = 0X0000;

    Ecan_ModuleData.u16Temp_InvL = 0X0000;
    Ecan_ModuleData.u16VOutH_Freq = 0X0000;
    Ecan_ModuleData.u16VOutL_Freq = 0X0000;

    Ecan_ModuleData.u16Phase_Diff = 0X0000;
    Ecan_ModuleData.u16RunTimeHour_L = 0X0000;
    Ecan_ModuleData.u16RunTimeHour_H = 0X0000;

    Ecan_ModuleData.Alert.Byte.Alert = 0X00;
    Ecan_ModuleData.Fault.Byte.Fault1 = 0X00;
    Ecan_ModuleData.Fault.Byte.Fault2 = 0X00;
    Ecan_ModuleData.Fault.Byte.Fault3 = 0X00;
    Ecan_ModuleData.Check = 0X00;

    //ECan parameter revise
    Ecan_SysParaCalibration.u16VGrid_rms = 0;
    Ecan_SysParaCalibration.u16VBusP = 0;
    Ecan_SysParaCalibration.u16VBusN = 0;

    Ecan_SysParaCalibration.u16VInvH_rms = 0;
    Ecan_SysParaCalibration.u16VInvL_rms = 0;
    Ecan_SysParaCalibration.u16IInvH_rms = 0;

    Ecan_SysParaCalibration.u16IInvL_rms = 0;
	Ecan_SysParaCalibration.u16VOutH_rms = 0;
	Ecan_SysParaCalibration.u16VOutL_rms = 0;

    Ecan_SysParaCalibration.u16Temp_PFC = 0;
    Ecan_SysParaCalibration.u16Temp_InvH = 0;
    Ecan_SysParaCalibration.u16Temp_InvL = 0;

    Ecan_SysParaCalibration.u16IGrid_rms = 0;
    Ecan_SysParaCalibration.u16IInvH_para_ave = 0;
    Ecan_SysParaCalibration.u16IInvL_para_ave = 0;

    Ecan_SysParaCalibration.u16RestartTimes = 0;
    Ecan_SysParaCalibration.u16InvH_VoltRms_Ref = 0;
    Ecan_SysParaCalibration.u16InvL_VoltRms_Ref = 0;

    //ECan upper computer order
    Ecan_SytemOrder.rsvr0 = 0;
    Ecan_SytemOrder.Defaluts = 0xFFFF;
    Ecan_SytemOrder.Output_Enable = 0;
    Ecan_SytemOrder.rsvr1 = 0;

    EcanP2A_Tx.P2AMail_id.all = 0xC0000007;
    EcanP2A_Tx.P2AMail_data.DWord.CANH_Bytes = 0x00000000;
    EcanP2A_Tx.P2AMail_data.DWord.CANL_Bytes = 0x00000000;

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
	FanCntl();
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
	
	if (ModuleAdd != 0X00 && g_Sys_Current_State != FaultState  && g_Sys_Current_State != PermanentState )//0 == SYNC_COM2_LEVEL && 2018.4.9 GX
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
}

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

   //SEM_post(&SEM_MasterCommTx);

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

//----------------end file-----------------------
