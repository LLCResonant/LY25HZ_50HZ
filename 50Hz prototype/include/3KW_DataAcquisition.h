/*=============================================================================*
 *  			Copyright(c) 2010-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_DataAcquisition.h
 *  PURPOSE  : header files 3KW_DataAcquisition.c
 *			   define constant, struct declaration, extern varibles
 *				 
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *        
 ******************************************************************************/

#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

//================= Global constants============================================
// -----------------------------------------------
// --- Grid Frequency Limits Specifications
// -----------------------------------------------
#define Value_2Pi	6.283185307
#define Value_Pi 	3.141592654

#define	Stabilized_BusVoltage     	600
#define	Rated50HzGirdFrequecy		50			// Unit, Hz
#define	Rated60HzGirdFrequecy		60			// Unit, Hz
#define	RatedDeviationofGridFreq	1			// Unit, Hz
#define Rated50HzVOutFrequecy       50

#define Inv_VoltRef				220

//PLL limits
#define DetectMaxGridFrequency		Rated60HzGirdFrequecy*1.15			// 66 Hz 2017.4.12 GX
#define DetectMinGridFrequency		Rated50HzGirdFrequecy*0.85			// 45 Hz 2017.4.12 GX

#define GridTheta_StepRated			Rated50HzGirdFrequecy*Value_2Pi/PWM_FREQ*2 //2017.5.10 GX
#define GridTheta_Step_Hi_Limit		DetectMaxGridFrequency*Value_2Pi/PWM_FREQ*2 //2017.5.10 GX
#define GridTheta_Step_Low_Limit	DetectMinGridFrequency*Value_2Pi/PWM_FREQ*2 //2017.5.10 GX

#define VOutTheta_StepRated			Rated50HzVOutFrequecy*Value_2Pi/PWM_FREQ*2

#define CoffStepToFre				PWM_FREQ/Value_2Pi/2 //2017.5.10 GX

#define AngleToRadian				0.002777778
// -----------------------------------------------
// define measure gain, the caculated values are ten times of real values 
// -----------------------------------------------
#define IGridMeasureGain		0.04f  //2017.9.16	GX
#define IInvMeasureGain         0.038f// 0.033       initial 0.04  2018.08.11 WF

#define VGridMeasureGain		0.235f	//2017.2.20	GX
#define VInvMeasureGain		0.235f//2017.11.2	GX
#define VBusMeasureGain			0.143f	//2017.2.20 GX

#define EnergyMeasureGain		2.7778e-4 * 8.3333e-8f //OutputWattMeasureGain*5.2083e-5f*2.7778e-4f	
									//cumulative frequency: 5e-5, ts: 1/3600=2.7778e-4, kw: 1e-3
#define Step2FreGain			200 * PWM_FREQ/Value_2Pi	
									// 50Hz-->764.3f;764.3*10,sample frequency is dobuled.

// define AD samples AC components' DC offset
#define ADDefaultACOffset			2048.0		//define AD samples' DC offset

// 220V Output Inverter Over Current Protection Status pin
#define INV_OCP_LEVEL			    GpioDataRegs.GPCDAT.bit.GPIO80

// 220V Output Inverter Over Voltage Protection Status pin
#define INV_OVP_LEVEL				GpioDataRegs.GPCDAT.bit.GPIO81

// DC Bus Over Voltage Protection Status pin
#define BUS_OVP_LEVEL			    GpioDataRegs.GPBDAT.bit.GPIO45

// AC Grid Input Over Current Protection Status pin
#define GRID_OCP_LEVEL				GpioDataRegs.GPBDAT.bit.GPIO44

// DC Fans Speed Status pin
#define DC_FAN1_FB_Level			GpioDataRegs.GPCDAT.bit.GPIO77
#define DC_FAN2_FB_Level			GpioDataRegs.GPCDAT.bit.GPIO76

/*=================struct define====================*/

struct	AD_Sample_Reg1 
{
	int16	i16ADSelectCounter;

	float32	f32IGrid;				// AC Input Current
	float32 f32IInv;				// AC Output Current 220V
	float32 f32IOut;				// AC Output Current 220V

	float32 f32VGrid;				// AC Input Voltage
	float32	f32VInv;				// Inv Output Voltage 220V
	float32 f32VBusP;				// DC Bus High Voltage
	float32	f32VBusN;				// DC Bus Low Voltage

	float32 f32VOut;				// AC Output Voltage 220V
	float32 f32BTS;

    float32	f32TempInv;
    float32	f32TempPFC;
     
};

struct	AD_ACC_Reg1
{
	int16	i16GridCounter;
	int16	i16InvCounter;

	float32	f32IGrid_ave;
	float32	f32IInv_ave;

	float32	f32VGrid_ave;
	float32	f32VInv_ave;

	float32	f32VOut_ave;

	float32	 f32IGrid_rms;
	float32	 f32IGrid_rms_ave;
	float32	 f32IInv_rms;
	float32	 f32IOut_rms;
	float32 f32IInv_rms_previous;

	float32	f32IInv_para_aver; //2017.12.1 GX

	float32 f32VGrid_rms;
	float32 f32VGrid_rms_shadow;
	float32 f32VGrid_rms_previous;
	float32 f32VInv_rms;
	float32 f32VInv_rms_previous;

	float32 f32VOut_rms;

	float32	f32VBusP;
	float32	f32VBusN;
	float32 f32VBus;

	float32 f32BTS;

	float32	Coff_Dforward;

	float32	f32TempInv;
    float32	f32TempPFC;

	float32	f32GridFreq;
	float32	f32VOutFreq;
	float32	f32VOutLdFreq;

	float32 f32VDutyD_ave;
	float32 Coff_Dforward2;
};

/*=================struct define====================*/

/*=================gloable varibles declaration====================*/
extern	struct	AD_Sample_Reg1	GeneralADbuffer, GetRealValue, ADGain, ADChannelOffset, ADCorrection;
extern	struct	AD_ACC_Reg1		AD_Acc, AD_Sum, Calc_Result;
extern	float32	f32SumCounterReci;
extern	int16 i16Cnt_SysParaTemp;

struct FanCntl_REG
{
	Uint16 	u16Cnt_temp1;
	Uint16  u16Cnt_temp2;
};

extern  struct FanCntl_REG  FanControl_Reg;//ZJX 2017.11.06


extern  Uint32   VgridFreqCnt;
extern  Uint16   VgridZero_Cnt;

/*=================end of gloable varibles declaration====================*/

/*=================gloable function declaration====================*/
extern void Get_ADC_Result1(void);		// functions called in ADC.c
extern void Get_ADC_Result2(void);		// functions called in ADC.c
extern void ADAccGridCalc(void);
extern void ADAccInvCalc(void);

extern void DcFanSpeedSense(void);

extern void	HwInvOCPDetection(void);
extern void HwInvOVPDetection(void);

extern void	HwBusOVPDetection(void);
extern void HwGridOCPDetection(void);

extern void FanCntl(void);

/*=================end of gloable function declaration====================*/


/*extern float32    s_f32InductionCurrentDCA_Sum , s_f32InductionCurrentDCB_Sum, s_f32InductionCurrentDCC_Sum;
extern float32	f32temp11, f32temp22, f32temp33; */


#endif	//end of ENERGY_CALC_H defination


//--- end of file -----------------------------------------------------


