/*=============================================================================*
 *         
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 50KW_ProtectionLogic.h 

 *  PURPOSE  : header files 50KW_ProtectionLogic.c 
 *			       define constant, struct declaration, extern varibles 
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *   
 *
 ******************************************************************************/

#ifndef PROTECTION_LOGIC_H
#define PROTECTION_LOGIC_H

#define Production_3kW_SIDO

//======================= Global constants==========                                                                                                                                                                 ==================
#ifdef	Production_3kW_SIDO
#define Rated_OutputPower			2000
#define Rated_InputPower			2200
#define	Rated_InvH_OutputCurrentRms	5.5
#define Rated_InvL_OutputCurrentRms 7.3
#define	Rated_InvH_OutputCurrentPeak		Rated_InvH_OutputCurrentRms * 1.414
#define Rated_InvL_OutputCurrentPeak		Rated_InvL_OutputCurrentRms * 1.414
#define IInvHi2ProtectionTime       25
#define IInvHi1ProtectionTime       15000
#define IInvHi3ProtectionTime       3
#define IInvHiLimitBackTime			1500
#define Rated_InputCurrentRms	13.5
#define OverRated_InputCurrentRms	Rated_InputCurrentRms * 1.3   //2017.3.24 GX
#define OverRated_InputCurrentPeak  OverRated_InputCurrentRms * 1.414

#define RatedInvFrequency 			25
#define DetectMaxInvFrequency		RatedInvFrequency*1.01
#define DetectMinInvFrequency		RatedInvFrequency*0.99
#define InvTheta_StepRated			RatedInvFrequency*Value_2Pi/PWM_FREQ*2  //2017.5.10 GX
#define InvTheta_Step_Hi_Limit		25*1.1*Value_2Pi/PWM_FREQ*2//2017.5.10 GX
#define InvTheta_Step_Low_Limit		25*0.9*Value_2Pi/PWM_FREQ*2//2017.5.10 GX
#endif

#ifdef	Production_3kW_SISO
#define Rated_OutputPower		2000
#define Rated_InputPower		2200
#define	Rated_OutputCurrentRms	72
#define	Rated_OutputCurrentPeak	Rated_OutputCurrentRms * 1.414
#define	Half_Rated_OutputCurrentPeak	Rated_OutputCurrentPeak * 0.5
#define OverRated_OutputCurrent (Rated_OutputCurrentRms + 2)
#define Rated_InputCurrent	20
#define OverRated_InputCurrent	(Rated_inputCurrent + 2)

#define RatedInvFrequency 			50
#define DetectMaxInvFrequency		RatedInvFrequency*1.01
#define DetectMinInvFrequency		RatedInvFrequency*0.99
#define InvTheta_StepRated			RatedInvFrequency*Value_2Pi/PWM_FREQ*2  //2017.5.10 GX
#define InvTheta_Step_Hi_Limit		DetectMaxInvFrequency*Value_2Pi/PWM_FREQ*2 //2017.5.10 GX
#define InvTheta_Step_Low_Limit		DetectMinInvFrequency*Value_2Pi/PWM_FREQ*2 //2017.5.10 GX

#endif

//-------------------------------------------------------------------------------

//******************************************************************************
#define CHINA_SUN
//China_Sun=====================================================================
#ifdef CHINA_SUN
  #define VGridHiLimit	289
  #define VGridHiLimitBack 279
  #define VGridLowLimit	 149
  #define VGridLowLimit2 165
  #define VGridLowLimitBack  159
  #define VGridHighProtectionTime	6
  #define VGridLowProtectionTime	 10

  #define FreGridHiLimit		65.2			//2017.4.12 GX//50.2
  #define FreGridLowLimit 		44.8			//2017.4.12 GX//44.949.5 2017.3.24 GX
  #define FreGridProtectionTime	10               //2017.4.18 GX

  #define EnergyAcquisitionPeriod   1
  #define SafetyReConnetionTime		10

  #define VInvHHiLimit   242
  #define VInvHLowLimit 202
  #define VInvHLowLimitBack 210
  #define VInvLHiLimit   121
  #define VInvLLowLimit 101
  #define VInvLLowLimitBack 105

  #define VInvLightRevise   0.985
  #define VInvMiddleRevise 1.0
  #define VInvHeavyRevise 1.015

#endif
//==============================================================================

//TUV===========================================================================
#ifdef VDE4105
  #define VGridHi2Limit	264.1 //280
  #define VGridHi1Limit 	264
  #define VGridHiLimit	253
  #define VGridLow1Limit	195.1   //170//184//175//175//185
  #define VGridLow2Limit	195  //160//165
  #define VGridLowLimitBack    200.1
  #define VGridProtectionTime	5

  #define FreGridHiLimit		50.2			//50.2	
  #define FreGridHi2Limit		50.5			//50.5
  #define FreGridLow2Limit 		48				//48
  #define FreGridLowLimit 		49.5			//49.5

  #define SafetyReConnetionTime		10

#endif
//==============================================================================

#define DCILimit	800  //135 /800/120/400                       
#define DCIProtectionTime 5

#define AD_Channel_Offset_IGridLimit	5//0.5
#define AD_Channel_Offset_IInvLimit		5
#define AD_Channel_Offset_VGridLimit	30
#define AD_Channel_Offset_VInvLimit		30

// Bus voltage defination
#define Bus_Under_Volt_Limit	660  //200//245 //2018.1.25 GX
#define Bus_Over_Volt_Limit	    900
#define BusVoltProtectionTime	100		// 100 * 20ms = 2s

// Derating defination
#define	ACPowerDerating_AmbientTempRate 	100
#define	ACPowerDerating_HeathinkTempRate	100
#define ACPowerDerating_VoltageRate			100
#define	DCPowerDerating_HeathinkTempRate	500
#define	DCPowerDerating_VoltageRate	50

#define	DeratingAmbientTemperature 60       
#define ShutAmbientTemperature 65 //65

#define	DeratingHeatsinkTemperatureLimit	75  
#define	ShutHeatsinkTemperatureLimit		85
#define ShutInvTemperatureLimit             85


#define AcFanEnableTemperatureLimit 	60
#define AcFanDisableTemperatureLimit	40
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


// Dry Contactor Control
#define DRY_CONT_ON         		GpioDataRegs.GPASET.bit.GPIO20 = 1;
#define DRY_CONT_OFF         		GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;

// Driver Reset Control
#define DRV_RST_ON					GpioDataRegs.GPCSET.bit.GPIO82 = 1;
#define DRV_RST_OFF					GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;

#define GRID_OCP_CBC_ON					GpioDataRegs.GPASET.bit.GPIO1 = 1;
#define GRID_OCP_CBC_OFF				GpioDataRegs.GPACLEAR.bit.GPIO1  = 1;

typedef struct
{
	int16	i16SafeCountry;

	float32	f32VGrid_HiLimit;
	float32	f32VGrid_LowLimit;
  	float32 f32VGrid_HiLimitBack;
	float32 f32VGrid_LowLimitBack;
	float32	f32VGrid_LowProtectionTime;
	float32	f32VGrid_HighProtectionTime;

	float32	f32FreGrid_HiLimit;
	float32	f32FreGrid_LowLimit;
	float32	f32FreGrid_ProtectionTime;

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
	float32 f32IInvH_Hi1ProtectionTime;
	float32 f32IInvH_Hi2ProtectionTime;
	float32 f32IInvH_Hi3ProtectionTime;
	float32 f32IInvH_HiLimitBackTime;

	float32	f32IInvL_Hi1Limit;
	float32 f32IInvL_Hi2Limit;
	float32 f32IInvL_Hi3Limit;
	float32 f32IInvL_Hi4Limit;
	float32 f32IInvL_Hi1ProtectionTime;
	float32 f32IInvL_Hi2ProtectionTime;
	float32 f32IInvL_Hi3ProtectionTime;
	float32 f32IInvL_HiLimitBackTime;

	float32 f32IGrid_HiLimit;

	float32	f32ReConnetionTime;

	float32	f32VBus_HiLimit;
	float32 f32VBus_LowLimit;
	float32	f32VBus_ProtectionTime;

	float32 f32InvTemp_HiLimit;
	float32	f32AcFanEnableTemp_Limit;
	float32 f32AcFanDisableTemp_Limit;

	float32 f32InvH_VoltRms_Ref;
	float32 f32InvL_VoltRms_Ref;
	float32 f32InvH_VoltRms_Ref_LCD;
	float32 f32InvL_VoltRms_Ref_LCD;

	Uint16 	u16EnergyAcquisitionPeriod;

}SAFETY_PARAMETER_REG;

extern SAFETY_PARAMETER_REG	SafetyReg;


typedef struct
{
	Uint8 	InvH_Light_Flag;
	Uint8 	InvH_Middle_Flag;
	Uint8 	InvH_Heavy_Flag;
	Uint8 	InvL_Light_Flag;
	Uint8 	InvL_Middle_Flag;
	Uint8 	InvL_Heavy_Flag;
}VOLTAGE_REVISE_REG;
extern VOLTAGE_REVISE_REG Output_VoltRe_Reg;

struct POWER_CON_REG
{
	float32   f32OutputWatt;
  	float32   f32OutputQ;
  	float32   f32PowerFactor;
	float32   f32PhaseAngle;
	float32 	f32InvL_u8tempVL;
	float32 	f32AD;
	float32 	f32Count;
	float32 	f32InvL_u8tempVL2;
	float32 	f32Count2;
};
extern  struct POWER_CON_REG  PowerCon_Reg;


struct POWER_DERATE_REG
{
    float32   f32OutputPower_AmbientTLimit;
    float32	  f32OutputPower_HeatsinkTLimit;
    float32	  f32InputPower_HeatsinkTLimit;

    float32	  f32ACPowerDerating_VRate;
    float32	  f32ACPowerDerating_HTRate;

    float32	  f32Heatsink_OverTemperature_Limit;
    float32	  f32Heatsink_DeratingTemperature_Limit;

    float32	  f32Ambient_OverTemperature_Limit;
};

extern  struct POWER_DERATE_REG  PowerDerate_Reg;


struct ShortCheck_REG
{
	float32  Restart_time_interval;
	float32  Restart_times;
};
extern  struct ShortCheck_REG  ShortCheck_Reg;//ZJX 2017.11.06


//===================== Global functions==================================

extern void GridVoltCheck(void);
extern void GridCurrentCheck(void);
extern void GridFreqCheck(void);
extern void InvFreqCheck(void);
extern void InvVoltCheck(void);
extern void InvCurrentCheck(void);
extern void TemperatureCheck(void);
extern void InvParallelCurCheck(void);

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
extern void CurrentProtectionIdentify(void);

extern void InputPowerLimit(void);
extern void OutputCurrentLimit(void);
extern void OverTemperatureLimit(void);
extern void InvSyncCheck(void);
extern Uint16  Bruce_test_cnt;

//------------------
#endif




//--- end of file -----------------------------------------------------


