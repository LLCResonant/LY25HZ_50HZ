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
#ifndef 	DATA_ACQUISITION_H
#define 	DATA_ACQUISITION_H

//================= Global constants============================================
// -----------------------------------------------
// --- Grid Frequency Limits Specifications
// -----------------------------------------------
#define 	Value_2Pi				6.283185307
#define 	Value_Pi 				3.141592654
#define	Value2Pi_Ratio		1 / Value_2Pi

#define InvH_RatedVolt_Ref				220
#define InvL_RatedVolt_Ref			    110

//PLL limits
#define	Rated50HzGirdFrequecy		50			// Unit, Hz
#define	Rated60HzGirdFrequecy		60			// Unit, Hz

#define DetectMaxGridFrequency		Rated60HzGirdFrequecy*1.15			// 66 Hz
#define DetectMinGridFrequency		Rated50HzGirdFrequecy*0.85			// 45 Hz

#define GridTheta_StepRated			Rated50HzGirdFrequecy*Value_2Pi/PWM_FREQ*2
#define GridTheta_Step_Hi_Limit		DetectMaxGridFrequency*Value_2Pi/PWM_FREQ*2
#define GridTheta_Step_Low_Limit	DetectMinGridFrequency*Value_2Pi/PWM_FREQ*2
#define GridCounter						PWM_FREQ/2/Rated50HzGirdFrequecy

#define CoffStepToFre				PWM_FREQ/Value_2Pi/2
#define Default_Calibra_Coeff	1.0f
#define Default_Offset			0.0f
// -----------------------------------------------
// define measure gain, the calculated values are ten times of real values
// -----------------------------------------------
#ifdef 	LY25HZ
#define IGridMeasureGain		0.0264f
#define IInvHMeasureGain      	0.0147f
#define IInvLMeasureGain       	0.0184f
#define TempMeasureGain			1.0f
#define InvHNoLoadCurrent		1.1f
#define InvLNoLoadCurrent		0.6f

#define VGridMeasureGain		0.235f
#define VInvHMeasureGain		0.235f
#define VInvLMeasureGain		0.117f
#define VBusMeasureGain			0.143f
#define RatedInvFrequency 		25
#define InvCounter				PWM_FREQ/2/RatedInvFrequency
#endif

#ifdef 	LY50HZ
#define IGridMeasureGain		0.04f
#define IInvLMeasureGain     	0.0f
#define IInvHMeasureGain      	0.038f
#define TempMeasureGain			0.07324187f
#define InvLNoLoadCurrent	    0.0f
#define InvHNoLoadCurrent	    2.5f

#define VGridMeasureGain		0.235f
#define VInvLMeasureGain		0.0f
#define VInvHMeasureGain		0.235f
#define VBusMeasureGain			0.143f
#define RatedInvFrequency 		50
#define InvCounter				PWM_FREQ/2/RatedInvFrequency
#endif

// define AD samples AC components' DC offset
#define ADDefaultACOffset				2048.0		//define AD samples' DC offset

// 220V Output Inverter Over Current Protection Status pin
#define INVH_OCP_LEVEL			    GpioDataRegs.GPCDAT.bit.GPIO80

// 220V Output Inverter Over Voltage Protection Status pin
#define INVH_OVP_LEVEL				GpioDataRegs.GPCDAT.bit.GPIO81

// 110V Output Inverter Over Current Protection Status pin
#define INVL_OCP_LEVEL				GpioDataRegs.GPBDAT.bit.GPIO46

// 110V Output Inverter Over Voltage Protection Status pin
#define INVL_OVP_LEVEL				GpioDataRegs.GPBDAT.bit.GPIO47

// DC Bus Over Voltage Protection Status pin
#define BUS_OVP_LEVEL			    	GpioDataRegs.GPBDAT.bit.GPIO45

// AC Grid Input Over Current Protection Status pin
#define GRID_OCP_LEVEL				GpioDataRegs.GPBDAT.bit.GPIO44

// DC Fans Speed Status pin
#define DC_FAN1_FB_Level				GpioDataRegs.GPCDAT.bit.GPIO77
#define DC_FAN2_FB_Level				GpioDataRegs.GPCDAT.bit.GPIO76

// Single module indication pin
#define Single_Module_Level			GpioDataRegs.GPBDAT.bit.GPIO43

//Bypass indication pin
#define Bypass_Module_Level			GpioDataRegs.GPBDAT.bit.GPIO42

/*=================struct define====================*/

struct	AD_Sample_Reg1 
{
	float32		f32IGrid;				// AC Input Current
	float32    	f32IInvH;				// AC Inv Current 220V
	float32		f32IInvL;				// AC Inv Current 110V
	float32    	f32IOutH;				// AC Output Current 220V
	float32		f32IOutL;				// AC Output Current 110V

	float32 		f32VGrid;				// AC Input Voltage
	float32		f32VInvH;				// Inv Output Voltage 220V
	float32		f32VInvL;				// Inv Output Voltage 110V
	float32 		f32VBusP;				// DC Bus High Voltage
	float32		f32VBusN;			// DC Bus Low Voltage

	float32 		f32VOutH;			// AC Output Voltage 220V
	float32		f32VOutL;			// AC Output Voltage 110V

    float32		f32TempInvH;
	float32		f32TempInvL;
    float32		f32TempPFC;
};

struct	AD_ACC_Reg1
{
	int16		i16GridCounter;
	int16		i16InvCounter;

	float32		f32IGrid_ave;
	float32		f32IInvH_ave;
	float32		f32IInvL_ave;

	float32		f32VGrid_ave;
	float32		f32VInvH_ave;
	float32		f32VInvL_ave;

	float32		f32VOutH_ave;
	float32		f32VOutL_ave;

	float32		f32IGrid_rms;
	float32		f32IGrid_rms_ave;
	float32		f32IInvH_rms;
	float32		f32IInvL_rms;
	float32		f32IInvH_rms_instant;
	float32		f32IInvL_rms_instant;
	float32		f32IOutH_rms;
	float32		f32IOutL_rms;

	float32 		f32VGrid_rms;
	float32 		f32VGrid_rms_instant;
	float32 		f32VInvH_rms;
	float32 		f32VInvL_rms;
	float32		f32VInvH_rms_instant;
	float32		f32VInvL_rms_instant;

	float32 		f32VOutH_rms;
	float32 		f32VOutL_rms;

	float32		f32VBusP;
	float32		f32VBusN;
	float32 		f32VBus;

	float32		f32Coff_Dforward;

	float32		f32TempInvH;
	float32		f32TempInvL;
    float32		f32TempPFC;
    float32 		f32TempInvMax;

	float32		f32GridFreq;
	float32		f32VOutFreq;
	float32 		f32VOutHFreq;
	float32 		f32VOutLFreq;

	float32 		f32Phase_Diff_ave;
};
/*=================Global variables declaration====================*/
extern	struct	AD_Sample_Reg1	GeneralADbuffer,  GetRealValue,  ADGain,  ADChannelOffset,  ADCalibration;
extern	struct	AD_ACC_Reg1		AD_Acc, 	AD_Sum, 	Calc_Result;
extern	float32	f32SumCounterReci, 	f32SumCounterInv;
extern	int16 	i16Cnt_SysParaTemp;
/*=================end of global variables declaration====================*/

/*=================Global function declaration====================*/
extern void Get_ADC_Result1(void);		// functions called in ADC.c
extern void Get_ADC_Result2(void);		// functions called in ADC.c
extern void ADAccGridCalc(void);
extern void ADAccInvCalc(void);

extern void DcFanSpeedSense(void);

extern void	HwInvHOCPDetection(void);
extern void HwInvHOVPDetection(void);

extern void	HwBusOVPDetection(void);
extern void HwGridOCPDetection(void);

extern void HwInvLOCPDetection(void);
extern void HwInvLOVPDetection(void);

extern void FanCntl(void);
/*=================end of global function declaration====================*/
#endif
//--- end of file -----------------------------------------------------


