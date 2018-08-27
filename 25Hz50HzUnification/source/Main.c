/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_ProtectionLogic.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/

#include "DSP2833x_Device.h"			       		// Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file

//--- variables
Uint8 	u8ecan_cnt = 1;
Uint8 	u8hosttemp = 0;

//--- functions
void SysParamDefault(); 

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

	SysParamDefault();

    InitAdc();							// Initialize necessary ADC module, directly for SVPWM.
    InitEPwm();							// Initialize the PWM, Note, ALL PWM output must be disabled.
    InitECap();							// Initialize the Capture module, to measure Grid frequency.
  	InitECan();							// Initialize the ECan.
 	InitECana();						// Initialize the ECana.
 	EEPROMParamDefault();	// Initialize the EEprom

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

	HWI_disable();
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

	ADCalibration.f32IGrid = Default_Calibra_Coeff;
	ADCalibration.f32VGrid = Default_Calibra_Coeff;

	ADCalibration.f32IInvH = Default_Calibra_Coeff;
	ADCalibration.f32VInvH = Default_Calibra_Coeff;
	ADCalibration.f32VOutH = Default_Calibra_Coeff;
	ADCalibration.f32IOutH= Default_Calibra_Coeff;

	ADCalibration.f32IInvL = Default_Calibra_Coeff;
	ADCalibration.f32VInvL = Default_Calibra_Coeff;
	ADCalibration.f32VOutL = Default_Calibra_Coeff;
	ADCalibration.f32IOutL = Default_Calibra_Coeff;

	ADCalibration.f32VBusP= Default_Calibra_Coeff;
	ADCalibration.f32VBusN = Default_Calibra_Coeff;
	ADCalibration.f32TempPFC = Default_Calibra_Coeff;
	ADCalibration.f32TempInvH = Default_Calibra_Coeff;
	ADCalibration.f32TempInvL = Default_Calibra_Coeff;

	ADChannelOffset.f32VBusP = Default_Offset;
	ADChannelOffset.f32VBusN = Default_Offset;

	ADChannelOffset.f32IGrid = Default_Offset;
	ADChannelOffset.f32VGrid = Default_Offset;

	ADChannelOffset.f32IInvH = Default_Offset;
	ADChannelOffset.f32VInvH = Default_Offset;
	ADChannelOffset.f32VOutH = Default_Offset;

	ADChannelOffset.f32IInvL = Default_Offset;
	ADChannelOffset.f32VInvL = Default_Offset;
	ADChannelOffset.f32VOutL = Default_Offset;

	ADChannelOffset.f32TempPFC = Default_Offset;
	ADChannelOffset.f32TempInvH = Default_Offset;
	ADChannelOffset.f32TempInvL = Default_Offset;

	AD_Acc.i16GridCounter = GridCounter;
	AD_Acc.i16InvCounter = InvCounter;

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

	AD_Sum.i16GridCounter = GridCounter;
	AD_Sum.i16InvCounter = InvCounter;

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
	Calc_Result.f32TempInvMax = 0;

	Calc_Result.f32GridFreq = 0;
	Calc_Result.f32VOutFreq = 0;
	Calc_Result.f32VOutLFreq = 0;
	Calc_Result.f32VOutHFreq = 0;

	Calc_Result.f32IGrid_ave = 0;
	Calc_Result.f32VGrid_ave = 0;
	Calc_Result.f32IGrid_rms_ave = 0;

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

	PowerDerate_Reg.f32ACPowerDerating_VRate = ACPowerDerating_VoltageRate;
	PowerDerate_Reg.f32ACPowerDerating_HTRate = ACPowerDerating_HeathinkTempRate;
	PowerDerate_Reg.f32Heatsink_DeratingTemperature_Limit = DeratingHeatsinkTemperatureLimit;
	PowerDerate_Reg.f32Heatsink_OverTemperature_Limit = ShutHeatsinkTemperatureLimit;

	SafetyReg.f32FreGrid_HiLimit = FreGridHiLimit;
	SafetyReg.f32FreGrid_LowLimit = FreGridLowLimit;
	SafetyReg.u16FreGrid_ProtectionTime = FreGridProtectionTime;

	SafetyReg.f32VGrid_HiLimit = VGridHiLimit;
	SafetyReg.f32VGrid_LowLimit = VGridLowLimit;
	SafetyReg.f32VGrid_HiLimitBack = VGridHiLimitBack;
	SafetyReg.f32VGrid_LowLimitBack = VGridLowLimitBack;
	SafetyReg.u16VGrid_HighProtectionTime = VGridHighProtectionTime;
	SafetyReg.u16VGrid_LowProtectionTime = VGridLowProtectionTime;
	SafetyReg.f32VGridDipLimit = VGridDipLimit;
	SafetyReg.f32VGridDip_BusVoltLimit = VGridDipBusVoltLimit;

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
	SafetyReg.u16VBusProtectionTime = BusVoltProtectionTime;

	SafetyReg.f32IInvH_Hi1Limit = Rated_InvH_OutputCurrentRms * 1.2;
	SafetyReg.f32IInvH_Hi2Limit = Rated_InvH_OutputCurrentRms * 1.3;
	SafetyReg.f32IInvH_Hi3Limit = Rated_InvH_OutputCurrentRms * 1.5;
	SafetyReg.f32IInvH_Hi4Limit = Rated_InvH_OutputCurrentRms * 2;
	SafetyReg.u16IInvH_Hi1ProtectionTime = IInvHi1ProtectionTime;
	SafetyReg.u16IInvH_Hi2ProtectionTime = IInvHi2ProtectionTime;
	SafetyReg.u16IInvH_Hi3ProtectionTime = IInvHi3ProtectionTime;
	SafetyReg.u16IInvH_Hi4ProtectionTime = IInvHi4ProtectionTime;
	SafetyReg.u16IInvH_HiLimitBackTime = IInvHiLimitBackTime;
	SafetyReg.f32IInvH_Hi1LimitBack = SafetyReg.f32IInvH_Hi1Limit - 2;

	SafetyReg.f32IInvL_Hi1Limit = Rated_InvL_OutputCurrentRms * 1.2;
	SafetyReg.f32IInvL_Hi2Limit = Rated_InvL_OutputCurrentRms * 1.3;
	SafetyReg.f32IInvL_Hi3Limit = Rated_InvL_OutputCurrentRms * 1.5;
	SafetyReg.f32IInvL_Hi4Limit = Rated_InvL_OutputCurrentRms * 2;
	SafetyReg.u16IInvL_Hi1ProtectionTime = IInvHi1ProtectionTime;
	SafetyReg.u16IInvL_Hi2ProtectionTime = IInvHi2ProtectionTime;
	SafetyReg.u16IInvL_Hi3ProtectionTime = IInvHi3ProtectionTime;
	SafetyReg.u16IInvL_Hi4ProtectionTime = IInvHi4ProtectionTime;
	SafetyReg.u16IInvL_HiLimitBackTime = IInvHiLimitBackTime;
	SafetyReg.u16IInv_WarningTime = IInvWarningTime;
	SafetyReg.f32IInvL_Hi1LimitBack = SafetyReg.f32IInvL_Hi1Limit - 2;

	SafetyReg.f32IGrid_HiLimit = OverRated_InputCurrentPeak;
	SafetyReg.f32InvTemp_HiLimit = ShutInvTemperatureLimit;
	SafetyReg.u16Short_Restart_times = 0;

	SafetyReg.f32FreVOut_LowLimit = InvFreq_Low_Limit;
	SafetyReg.f32FreVOut_HiLimit = InvFreq_High_Limit;

	SafetyReg.u16Para_Ave_ProtectionTime = ParaAveProtectionTime;
	SafetyReg.f32InvHParaCurDeviationLimit = InvHParaCurDeviationLimit;
	SafetyReg.f32InvLParaCurDeviationLimit = InvLParaCurDeviationLimit;

	Output_VoltRe_Reg.u8InvH_Heavy_Flag = 0;
	Output_VoltRe_Reg.u8InvH_Light_Flag = 0;
	Output_VoltRe_Reg.u8InvH_Middle_Flag = 0;

	Output_VoltRe_Reg.u8InvL_Heavy_Flag = 0;
	Output_VoltRe_Reg.u8InvL_Light_Flag = 0;
	Output_VoltRe_Reg.u8InvL_Middle_Flag = 0;

	// before enter normal mode, bus voltage controller,current controller should be initiated
	BusCon_Reg.f32BusVolt_Ref = BusVoltRef;
	BusCon_Reg.f32BusFliter = 0;
	BusCon_Reg.f32BusVolt_Cmd = 0;
	BusCon_Reg.f32BusVolt_Fdb = 0;
	BusCon_Reg.f32BusVoltErr_Old = 0;
	BusCon_Reg.f32BusVoltErr_New = 0;
	BusCon_Reg.f32IGridAmp_Ref = 0;
	BusCon_Reg.f32IGrid_RefAmp_Limit  = IGrid_RefAmp_Limit2;
	BusCon_Reg.f32BusVoltDiffErr_New = 0;
	BusCon_Reg.f32BusVoltDiffErr_Old = 0;
	BusCon_Reg.f32BusVoltDiff_Out = 0;
	BusCon_Reg.f32Bus_Kp = BusCon_Kp;
	BusCon_Reg.f32Bus_Ki = BusCon_Ki;
	BusCon_Reg.f32BusBal_Kp = BusBal_Kp;
	BusCon_Reg.f32BusBal_Ki = BusBal_Ki;

	CurrConReg.f32IGrid_Ref = 0;
	CurrConReg.f32IGrid_Fdb = 0;
	CurrConReg.f32IGridErr_Old = 0;
	CurrConReg.f32IGridErr_New = 0;
	CurrConReg.f32PfcDuty_Con = 0;
	CurrConReg.u8Drive_Open = 0;
	CurrConReg.f32PfcDuty_ff_factor = 1.0f;
	CurrConReg.f32Kp = CurrCon_Kp3;
	CurrConReg.f32Ki = CurrCon_Ki;
	CurrConReg.f32PfcDuty = 0;
	CurrConReg.f32PfcDuty_ff = 0;

	GridPLLConReg.f32Input[0] = 0;
	GridPLLConReg.f32Input[1] = 0;
	GridPLLConReg.f32Input[2] = 0;
	GridPLLConReg.f32Output[0] = 0;
	GridPLLConReg.f32Output[1] = 0;
	GridPLLConReg.f32Output[2] = 0;
	GridPLLConReg.f32MAC = 0;
	GridPLLConReg.f32Refer = 0;
	GridPLLConReg.f32PIDErr_Old = 0;
	GridPLLConReg.f32PIDErr_New = 0;
	GridPLLConReg.f32PID_Output = 0;
	GridPLLConReg.f32Theta = 0;
	GridPLLConReg.f32Kp = PLL_Grid_Kp;
	GridPLLConReg.f32Ki = PLL_Grid_Ki;
	GridPLLConReg.f32Theta_Step = GridTheta_StepRated;
	GridPLLConReg.f32Valpha = 0;
	GridPLLConReg.f32Sin_Theta = 0;
	GridPLLConReg.f32Cos_Theta = 0;

	VOutHPLLConReg.f32Input[0] = 0;
	VOutHPLLConReg.f32Input[1] = 0;
	VOutHPLLConReg.f32Input[2] = 0;
	VOutHPLLConReg.f32Output[0] = 0;
	VOutHPLLConReg.f32Output[1] = 0;
	VOutHPLLConReg.f32Output[2] = 0;
	VOutHPLLConReg.f32MAC = 0;
	VOutHPLLConReg.f32Refer = 0;
	VOutHPLLConReg.f32PIDErr_Old = 0;
	VOutHPLLConReg.f32PIDErr_New = 0;
	VOutHPLLConReg.f32PID_Output = 0;
	VOutHPLLConReg.f32Theta = 0;
	VOutHPLLConReg.f32Kp = PLL_Inv_Kp;
	VOutHPLLConReg.f32Ki = PLL_Inv_Ki;
	VOutHPLLConReg.f32Theta_Step = DELTA_ANGLE_INV;
	VOutHPLLConReg.f32Valpha = 0;
	VOutHPLLConReg.f32Sin_Theta = 0;
	VOutHPLLConReg.f32Cos_Theta = 0;


	VOutLPLLConReg.f32Input[0] = 0;
	VOutLPLLConReg.f32Input[1] = 0;
	VOutLPLLConReg.f32Input[2] = 0;
	VOutLPLLConReg.f32Output[0] = 0;
	VOutLPLLConReg.f32Output[1] = 0;
	VOutLPLLConReg.f32Output[2] = 0;
	VOutLPLLConReg.f32MAC = 0;
	VOutLPLLConReg.f32Refer = 0;
	VOutLPLLConReg.f32PIDErr_Old = 0;
	VOutLPLLConReg.f32PIDErr_New = 0;
	VOutLPLLConReg.f32PID_Output = 0;
	VOutLPLLConReg.f32Theta = 0;
	VOutLPLLConReg.f32Kp = PLL_Inv_Kp;
	VOutLPLLConReg.f32Ki = PLL_Inv_Ki;
	VOutLPLLConReg.f32Theta_Step = DELTA_ANGLE_INV;
  	VOutLPLLConReg.f32Valpha = 0;
	VOutLPLLConReg.f32Sin_Theta = 0;
	VOutLPLLConReg.f32Cos_Theta = 0;

 	OutPLLConReg.f32Theta = 0;
  	OutPLLConReg.f32Theta_Step = InvTheta_StepRated;
  	OutPLLConReg.f32Cos_Theta = 0;
  	OutPLLConReg.f32Sin_Theta = 0;

	InvHVoltConReg.f32VoltRms_Ref = InvH_RatedVolt_Ref;
	InvHVoltConReg.f32VoltInst_Ref = 0;
	InvHVoltConReg.f32VoltInst_Fdb = 0;
	InvHVoltConReg.f32VoltInst_ErrNew = 0;
	InvHVoltConReg.f32VoltInst_ErrOld = 0;
	InvHVoltConReg.f32VoltInst_ErrOut = 0;
	InvHVoltConReg.f32Input[0] = 0;
	InvHVoltConReg.f32Input[1] = 0;
	InvHVoltConReg.f32Input[2] = 0;
	InvHVoltConReg.f32Output[0] = 0;
	InvHVoltConReg.f32Output[1] = 0;
	InvHVoltConReg.f32Output[2] = 0;
	InvHVoltConReg.f32MAC = 0;
	InvHVoltConReg.f32Kp = InvH_Volt_Kp;
	InvHVoltConReg.f32Kr = InvH_Volt_Kr;
	InvHVoltConReg.f32VoltDutyUpLimit = 0;
	InvHVoltConReg.f32VoltDutyLowLimit = 0;
  	InvHVoltConReg.f32InvDuty = 0;
  	InvHVoltConReg.f32VoltGain = 0;

	InvLVoltConReg.f32VoltRms_Ref =  InvL_RatedVolt_Ref;
	InvLVoltConReg.f32VoltInst_Ref = 0;
	InvLVoltConReg.f32VoltInst_Fdb = 0;
	InvLVoltConReg.f32VoltInst_ErrNew = 0;
	InvLVoltConReg.f32VoltInst_ErrOld = 0;
	InvLVoltConReg.f32VoltInst_ErrOut = 0;
    InvLVoltConReg.f32Input[0] = 0;
    InvLVoltConReg.f32Input[1] = 0;
    InvLVoltConReg.f32Input[2] = 0;
    InvLVoltConReg.f32Output[0] = 0;
    InvLVoltConReg.f32Output[1] = 0;
    InvLVoltConReg.f32Output[2] = 0;
    InvLVoltConReg.f32MAC = 0;
	InvLVoltConReg.f32Kp = InvL_Volt_Kp;
	InvLVoltConReg.f32Kr = InvL_Volt_Kr;
	InvLVoltConReg.f32VoltDutyUpLimit = 0;
	InvLVoltConReg.f32VoltDutyLowLimit = 0;
  	InvLVoltConReg.f32InvDuty = 0;
  	InvLVoltConReg.f32VoltGain = 0;

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
    ComFlag.SciRcvStart=0;
	ComFlag.OverTimeFlag=0;
	ComFlag.OverTimeDelay=0;
	ComCnt.SaveFactorySettings = 0;
	ComCnt.ComRecByteNum = 0;

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

    Ecan_Error.u8Upload_Trans_Error = 0;
    Ecan_Error.u8Broadcast_Trans_Error = 0;

	RunningTime.Hour_H = 0;
	RunningTime.Hour_L = 0;
	RunningTime.Minute = 0;
	RunningTime.Second = 0;
	RunningTime.OverFlow = 0;
	RunningTime.TimeCheck = 0;

	HWI_enable();
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
			if (u8hosttemp == u8_hostdrop && g_Mod_Status != Master)
			{
				if (u8ecan_cnt >= 2 && u8ecan_cnt < 5)
					g_Mod_Status = idle;
				else if (u8ecan_cnt >= 5)
				{
					g_Mod_Status = Master;
					u8ecan_cnt = 1;
				}
				u8ecan_cnt ++ ;
			}
			else
				u8ecan_cnt = 1;
			u8hosttemp = u8_hostdrop;
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

    scia_cnt++;
	if(2 == scia_cnt)
	{
		SEM_post(&SEM_SCIAComm);
		scia_cnt = 0;
		if(ScibRegs.SCICTL1.bit.SWRESET == 0)
		{
			ScibRegs.SCICTL1.bit.SWRESET = 1;
		}
	}
	   
} // end of TimerBase20msPRD()

void TimeBase100msPRD(void)
{
	SEM_post(&SEM_Time_Record);
}

//----------------end file-----------------------
