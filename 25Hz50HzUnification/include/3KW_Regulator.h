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
#ifndef REGULATOR_H
#define REGULATOR_H

/* Macros for Constants and Parameter Definitions */
/* 288 sampling points in 20ms, 48*3 = 144(10ms), 144*2(20ms) = 288 */
/* 20KHz switching frequency for single-phase inverter power stage */
#define PWM_FREQ			40000
#define PWM_FREQ1			(1/PWM_FREQ)
/* (period/2) for SPWM with 75MHz TBCLK */		 
#define PWM_PERIOD		(HSPCLK_FREQ/PWM_FREQ)

/* (period/2) for SPWM with 75MHz TBCLK */		 
#define PWM_HALF_PERIOD		((HSPCLK_FREQ/PWM_FREQ)/2)
#define PWM_HALF_PERIOQ		((HSPCLK_FREQ/PWM_FREQ)/2)

/* 5us deadband with 75MHz TBCLK, 5*75 = 375 */	 
#define DeadBand_Duration	50				//250*2=5us,375-->7.5us

#define LPF_B0_GRID	0.00034605f		//0.000346050740714038155762;
#define LPF_B1_GRID	0.00069210f		//0.000692101481428076311525;
#define LPF_B2_GRID	0.00034605f		//0.000346050740714038155762;
#define LPF_A1_GRID	1.94721182f		//1.947211823488243620516869;
#define LPF_A2_GRID	0.94859602f		//0.948596026451099749721152;

#define PLL_Grid_Kp	5e-5f
#define PLL_Grid_Ki		2.5e-8f
#define DELTA_ANGLE_GRID  2*Value_Pi *55/(PWM_FREQ/2)

#define BusBal_Kp  0.03f;
#define BusBal_Ki   0.00002f;
#define BusVoltRef   850;

#define VoltRefDelta   0.6;

#ifdef 	LY25HZ
//PFC
#define	CurrCon_Kp1	10.0f
#define	CurrCon_Kp2  	15.0f;
#define	CurrCon_Kp3  	30.0f;
#define	CurrCon_Kp4  	25.0f;
#define	CurrCon_Kp5	20.0f;
#define	CurrCon_Ki	 	3.0f

#define BusCon_Kp	0.2f
#define BusCon_Ki	0.001f
#define IGrid_RefAmp_Limit1 28
#define IGrid_RefAmp_Limit2 26
#define IGrid_RefAmp_Limit3 22

//Inv
/*
 * PR controller in s domain: Gpr =
 * 				    kr * s
 *  kp + --------------
 *  		  s^2 + w0^2
 */
#define InvH_Volt_Kp				0.6f
#define InvH_Volt_Kr				400* 0.00005f
#define Inv_Volt_Coff_a2			0.00006168503f  // (w0 * Ts) * (w0 * Ts), where w0 is 2 * pi * 25Hz; Ts is the sample period 1 / 20000Hz
#define Inv_Volt_Coff_a3			0.2499961f   		// 1 / (4 + w0 * w0 * Ts * Ts)

#define InvL_Volt_Kp				1.5f
#define InvL_Volt_Kr				350* 0.00005f

#define LPF_B0_INV	0.00003913f
#define LPF_B1_INV	0.00007826f
#define LPF_B2_INV	0.00003913f
#define LPF_A1_INV	1.9822318f
#define LPF_A2_INV	0.98238832f

#define PLL_Inv_Kp	1e-5f
#define PLL_Inv_Ki	5e-9f
#define DELTA_ANGLE_INV 2*Value_Pi * 24/(PWM_FREQ/2)
#endif

#ifdef 	LY50HZ
//PFC
#define	CurrCon_Kp1	10.0f
#define	CurrCon_Kp2  	30.0f;
#define	CurrCon_Kp3  	25.0f;
#define	CurrCon_Kp4  	20.0f;
#define	CurrCon_Kp5	20.0f;
#define	CurrCon_Ki	 	3.0f

#define BusCon_Kp	0.3f
#define BusCon_Ki	0.0015f
#define IGrid_RefAmp_Limit1 34
#define IGrid_RefAmp_Limit2 38
#define IGrid_RefAmp_Limit3 34
//Inv
#define InvH_Volt_Kp				0.01f
#define InvH_Volt_Kr				800 * 0.00005f
#define Inv_Volt_Coff_a2			0.0002467401f  	// (w0 * Ts) * (w0 * Ts), where w0 is 2 * pi * 25Hz; Ts is the sample period 1 / 20000Hz
#define Inv_Volt_Coff_a3			0.2499846f   		// 1 / (4 + w0 * w0 * Ts * Ts)

#define LPF_B0_INV	0.00034605f
#define LPF_B1_INV	0.00069210f
#define LPF_B2_INV	0.00034605f
#define LPF_A1_INV	1.94721182f
#define LPF_A2_INV	0.94859602f

#define PLL_Inv_Kp	5e-5f
#define PLL_Inv_Ki	2.5e-8f
#define DELTA_ANGLE_INV 2*Value_Pi * 50/(PWM_FREQ/2)
#endif

/* General Purpose PID Controller, Data type */

// PID regulator with inner staturation, parameters definition
struct BUS_CON_REG
{
  float32   f32BusVolt_Ref;
  float32	f32BusVolt_Cmd;
  float32   f32BusVolt_Fdb;       		// bus voltage
  float32   f32BusVoltErr_New;       	// current bus voltage error
  float32   f32BusVoltErr_Old;        	// before bus voltage error
  float32   f32IGridAmp_Ref;            	// final Idref command
  float32   f32IGrid_RefAmp_Limit;
  float32	f32BusVoltDiffErr_New;
  float32	f32BusVoltDiffErr_Old;
  float32	f32BusVoltDiff_Out;
  float32   f32BusFliter;
  float32	f32Bus_Kp;
  float32	f32Bus_Ki;
  float32	f32BusBal_Kp;
  float32	f32BusBal_Ki;
};
extern  struct BUS_CON_REG  BusCon_Reg;

struct CURRENTCONTREG	
{  
	float32  f32IGridErr_Old;   		// Input: Reference input
	float32  f32IGridErr_New;   		// Input: Feedback input
	float32	 f32IGrid_Ref;
	float32	 f32IGrid_Fdb;
	float32  f32PfcDuty_Con;
	float32	 f32PfcDuty_ff;
	float32  f32PfcDuty;
	float32  f32PfcDuty_ff_factor;
	float32  f32Kp;
	float32  f32Ki;
	Uint8   	u8Drive_Open;
};	            
extern	struct	CURRENTCONTREG	CurrConReg;

struct INVVOLTCONTREG
{
	float32  f32VoltRms_Ref;
	float32  f32VoltInst_ErrOld;   		// Input: Reference input
	float32  f32VoltInst_ErrNew;   			// Input: Feedback input
	float32  f32VoltInst_ErrOut;
	float32	 f32VoltInst_Ref;
	float32	 f32VoltInst_Fdb;
	float32	 f32VoltGain;
	float32  f32VoltDutyUpLimit;
	float32  f32VoltDutyLowLimit;
	float32  f32Input[3];
	float32  f32Output[3];
	float32  f32MAC;
	float32  f32InvDuty;
	float32  f32Kp;
	float32  f32Kr;
};
extern	struct	INVVOLTCONTREG	InvHVoltConReg, InvLVoltConReg;

struct PLLCONTREG	
{  
	float32  f32Valpha;   		// Input: Reference input 

	float32  f32Theta;   			// Output: PID output
	float32  f32Theta_Step;
	float32  f32Sin_Theta;
	float32	 f32Cos_Theta;

	float32  f32Input[3];   		// LPF
	float32  f32Output[3];   	// LPF
	float32  f32MAC;

	float32  f32Refer;
	float32  f32Kp;
	float32  f32Ki;
	float32  f32PIDErr_Old;
	float32  f32PIDErr_New;
	float32  f32PID_Output;
};
extern	struct	PLLCONTREG	GridPLLConReg, OutPLLConReg, VOutLPLLConReg, VOutHPLLConReg;

struct VOLTAGE_REVISE_REG
{
	Uint8 	u8InvH_Light_Flag;
	Uint8 	u8InvH_Middle_Flag;
	Uint8 	u8InvH_Heavy_Flag;
	Uint8 	u8InvL_Light_Flag;
	Uint8 	u8InvL_Middle_Flag;
	Uint8 	u8InvL_Heavy_Flag;
};
extern struct VOLTAGE_REVISE_REG Output_VoltRe_Reg;
/*=============================================================================*
 * 	Extern functions declaration
 *============================================================================*/

/* Initialize the starting Duty and loop PID parameters */
extern void RectifierStage_Init(void);
extern void InverterStage_Init(void);
extern void InvVoltSlowup(void);
extern void InvRestartCheck(void);
extern void BusVoltSlowup(void);
extern void ADC_INT_PFC_Control(void);
extern void ADC_INT_INV_Control(void);

extern int16 swGetStartADIsrPoint(void);
extern int16 swGetEndADIsrPoint(void);
extern int16 swGetEndCONPoint(void);

#endif

//--- end of file -----------------------------------------------------
