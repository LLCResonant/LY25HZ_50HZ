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
#ifndef 	PROTECTION_LOGIC_H
#define 	PROTECTION_LOGIC_H

//======================= Global constants========== ==================
#ifdef		LY25HZ
#define	Rated_InvH_OutputCurrentRms	5.5
#define 	Rated_InvL_OutputCurrentRms 	7.3
#define 	IInvHi1ProtectionTime       	15000
#define 	IInvHi2ProtectionTime       	75
#define 	IInvHi3ProtectionTime       	25
#define 	IInvHi4ProtectionTime       	3
#define 	IInvHiLimitBackTime				1500
#define 	Rated_InputCurrentRms		13.5
#define 	OverRated_InputCurrentRms		Rated_InputCurrentRms * 1.3
#define 	OverRated_InputCurrentPeak  		OverRated_InputCurrentRms * 1.414

#define 	InvTheta_StepRated				RatedInvFrequency*Value_2Pi/PWM_FREQ*2
#define 	InvTheta_Step_Hi_Limit			25*1.1*Value_2Pi/PWM_FREQ*2
#define 	InvTheta_Step_Low_Limit		25*0.9*Value_2Pi/PWM_FREQ*2
#define 	InvFreq_Low_Limit				24.6
#define 	InvFreq_High_Limit				25.4

#define 	VGridDipBusVoltLimit 			680
#define	InvHParaCurDeviationLimit  	2.7
#define	InvLParaCurDeviationLimit  	3.6
#endif

#ifdef		LY50HZ
#define	Rated_Inv_OutputCurrentRms		13
#define 	IInvHi1ProtectionTime       		30000
#define 	IInvHi2ProtectionTime       		150
#define 	IInvHi3ProtectionTime       		7
#define 	IInvHi4ProtectionTime       		3
#define 	IInvHiLimitBackTime				3000
#define 	Rated_InputCurrentRms			19
#define 	OverRated_InputCurrentRms	Rated_InputCurrentRms * 1.3
#define 	OverRated_InputCurrentPeak  OverRated_InputCurrentRms * 1.414

#define 	RatedInvFrequency 				50
#define 	InvTheta_StepRated				RatedInvFrequency*Value_2Pi/PWM_FREQ*2
#define 	InvTheta_Step_Hi_Limit			50*1.1*Value_2Pi/PWM_FREQ*2
#define 	InvTheta_Step_Low_Limit		50*0.9*Value_2Pi/PWM_FREQ*2
#define 	InvFreq_Low_Limit				49.6
#define 	InvFreq_High_Limit				50.4

#define 	VGridDipBusVoltLimit 				660
#define	InvHParaCurDeviationLimit  		6.5
#endif

//-------------------------------------------------------------------------------

#define VGridHiLimit						289
#define VGridHiLimitBack 				279
#define VGridLowLimit	 				149
#define VGridLowLimit2 					165
#define VGridLowLimitBack  			159
#define VGridHighProtectionTime	6
#define VGridLowProtectionTime	 	10
#define VGridDipLimit	 					100

#define FreGridHiLimit						65.2
#define FreGridLowLimit 					44.8
#define FreGridProtectionTime		10

#define VInvHHiLimit   					242
#define VInvHLowLimit 					202
#define VInvHLowLimitBack 			210
#define VInvLHiLimit   					121
#define VInvLLowLimit 					101
#define VInvLLowLimitBack 			105

#define VInvLightRevise   				0.985
#define VInvMiddleRevise 				1.0
#define VInvHeavyRevise 				1.015

#define 	Fan_Period_Cnt					90
#define 	Fan_Cnt_Limit					19

#define 	IInvWarningTime				10
#define 	ParaAveProtectionTime				10
//==============================================================================
//==============================================================================

#define AD_Channel_Offset_IGridLimit	5
#define AD_Channel_Offset_IInvLimit		5
#define AD_Channel_Offset_VGridLimit	30
#define AD_Channel_Offset_VInvLimit		30

// Bus voltage definition
#define Bus_Under_Volt_Limit		660
#define Bus_Over_Volt_Limit	    900
#define BusVoltProtectionTime	100		// 100 * 20ms = 2s

// Derating definition
#define	ACPowerDerating_AmbientTempRate 	100
#define	ACPowerDerating_HeathinkTempRate	100
#define 	ACPowerDerating_VoltageRate				100
#define	DCPowerDerating_HeathinkTempRate	500
#define	DCPowerDerating_VoltageRate				50

#define	DeratingAmbientTemperature 				60

#define	DeratingHeatsinkTemperatureLimit		75
#define	ShutHeatsinkTemperatureLimit				85
#define 	ShutInvTemperatureLimit             		85
//******************************************************************************

// AC Grid Relay Control
#define GRID_RELY_ON		 	    GpioDataRegs.GPASET.bit.GPIO21 = 1;
#define GRID_RELY_OFF				GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;

// DC Fan Control
#ifdef 	SECOND_EDITION
#define DC_Fan_Enable				GpioDataRegs.GPASET.bit.GPIO17 = 1;
#define DC_Fan_State					GpioDataRegs.GPADAT.bit.GPIO17
#define DC_Fan_Disable				GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
#endif

#ifdef 	THIRD_EDITION
#define DC_Fan_Enable				GpioDataRegs.GPASET.bit.GPIO12 = 1;
#define DC_Fan_State					GpioDataRegs.GPADAT.bit.GPIO12
#define DC_Fan_Disable				GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
#endif


//DryCrtl
#define DryCrtl_ON						GpioDataRegs.GPASET.bit.GPIO20 = 1;
#define DryCrtl_OFF					GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;

// Driver Reset Control
#define DRV_RST_ON					GpioDataRegs.GPCSET.bit.GPIO82 = 1;
#define DRV_RST_OFF					GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;

#define GRID_OCP_CBC_ON		GpioDataRegs.GPASET.bit.GPIO1 = 1;
#define GRID_OCP_CBC_OFF		GpioDataRegs.GPACLEAR.bit.GPIO1  = 1;

typedef struct
{
	float32	f32VGrid_HiLimit;
	float32	f32VGrid_LowLimit;
  	float32 f32VGrid_HiLimitBack;
	float32 f32VGrid_LowLimitBack;
	Uint16	u16VGrid_LowProtectionTime;
	Uint16	u16VGrid_HighProtectionTime;
	float32	f32VGridDipLimit;
	float32	f32VGridDip_BusVoltLimit;

	float32	f32FreGrid_HiLimit;
	float32	f32FreGrid_LowLimit;
	Uint16	u16FreGrid_ProtectionTime;

    float32	f32VInvH_HiLimit;
    float32	f32VInvH_LowLimit;
    float32 f32VInvL_HiLimit;
    float32 f32VInvL_LowLimit;
    float32 f32VInvH_LowLimitBack;
    float32 f32VInvL_LowLimitBack;

	float32 f32IInvH_Hi1Limit;
	float32 f32IInvH_Hi2Limit;
	float32 f32IInvH_Hi3Limit;
	float32 f32IInvH_Hi4Limit;
	Uint16 	u16IInvH_Hi1ProtectionTime;
	Uint16 	u16IInvH_Hi2ProtectionTime;
	Uint16 	u16IInvH_Hi3ProtectionTime;
	Uint16 	u16IInvH_Hi4ProtectionTime;
	Uint16 	u16IInvH_HiLimitBackTime;
	float32 f32IInvH_Hi1LimitBack;

	float32	f32IInvL_Hi1Limit;
	float32 f32IInvL_Hi2Limit;
	float32 f32IInvL_Hi3Limit;
	float32 f32IInvL_Hi4Limit;
	Uint16  u16IInvL_Hi1ProtectionTime;
	Uint16  u16IInvL_Hi2ProtectionTime;
	Uint16  u16IInvL_Hi3ProtectionTime;
	Uint16  u16IInvL_Hi4ProtectionTime;
	Uint16  u16IInvL_HiLimitBackTime;
	float32 f32IInvL_Hi1LimitBack;

	Uint16 	u16IInv_WarningTime;

	float32	f32FreVOut_HiLimit;
	float32	f32FreVOut_LowLimit;

	float32 f32IGrid_HiLimit;

	float32	f32VBus_HiLimit;
	float32 f32VBus_LowLimit;
	Uint16	u16VBusProtectionTime;

	float32 f32InvTemp_HiLimit;
	float32 f32InvH_VoltRms_Ref;
	float32 f32InvL_VoltRms_Ref;
	float32 f32InvH_VoltRms_Ref_LCD;
	float32 f32InvL_VoltRms_Ref_LCD;

	float32	f32InvHParaCurDeviationLimit;
	float32	f32InvLParaCurDeviationLimit;
	Uint16 	u16Para_Ave_ProtectionTime;
	Uint16 	u16Short_Restart_times;

}SAFETY_PARAMETER_REG;
extern SAFETY_PARAMETER_REG	SafetyReg;

struct POWER_DERATE_REG
{
    float32	  f32ACPowerDerating_VRate;
    float32	  f32ACPowerDerating_HTRate;

    float32	  f32Heatsink_OverTemperature_Limit;
    float32	  f32Heatsink_DeratingTemperature_Limit;
};
extern  struct POWER_DERATE_REG  PowerDerate_Reg;

//===================== Global functions==================================

extern void GridVoltCheck(void);
extern void GridCurrentCheck(void);
extern void GridFreqCheck(void);
extern void InvHFreqCheck(void);
extern void InvLFreqCheck(void);
extern void InvHVoltCheck(void);
extern void InvLVoltCheck(void);
extern void InvHCurrentCheck(void);
extern void InvLCurrentCheck(void);
extern void TemperatureCheck(void);
extern void InvHParallelCurCheck(void);
extern void InvLParallelCurCheck(void);

extern void BusVoltCheck(void);
extern void BusBalanceCheck(void);//2017.3.24 GX BusBalance

extern void ADOffsetCheck(void);

extern void DcPreCharCheck(void);
extern void RelayCheck(void);
extern void DCFanCheck(void);
extern void HwInvHOCPCheck(void);
extern void HwInvHOVPCheck(void);
extern void HwBusOVPCheck(void);
extern void HwGridOCPCheck(void);
extern void HwInvLOCPCheck(void);
extern void HwInvLOVPCheck(void);

extern void ShortRecover(void);
extern void InvHCurrentProtectionIdentify(void);
extern void InvLCurrentProtectionIdentify(void);

extern void InputPowerLimit(void);
extern void OutputCurrentLimit(void);
extern void OverTemperatureLimit(void);
extern void InvSyncCheck(void);


#endif
//--- end of file -----------------------------------------------------


