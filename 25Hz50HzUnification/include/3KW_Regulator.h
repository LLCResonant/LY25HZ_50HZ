/*=============================================================================*
 *         Copyright(c) 2010-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : 50KW_Regulator.h 
 *  PURPOSE  : header files 50KW_Regulator.c 
 *			       define constant, struct declaration, extern varibles 
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *
 ******************************************************************************/

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

#define CurrCon_Kp	30.0f
#define CurrCon_Ki	 	3.0f

#define BusCon_Kp	0.2
#define BusCon_Ki	0.001

#define PLLCon_K_alpha	5.588e-4f
#define PLLCon_K_beta	5.571e-4f


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
	float32	 f32VoltRms_Ref_Delta;
	float32  f32VoltInst_ErrOld;   		// Input: Reference input
	float32  f32VoltInst_ErrNew;   			// Input: Feedback input
	float32  f32VoltInst_ErrOut;
	float32	 f32VoltInst_Ref;
	float32	 f32VoltInst_Fdb;
	float32	 f32VoltGain;
	float32  f32VoltDutyUpLimit;
	float32  f32VoltDutyLowLimit;
	float32  Input[3];
	float32  Output[3];
	float32  MAC;
	float32  f32InvDuty;
};
extern	struct	INVVOLTCONTREG	InvHVoltConReg, InvLVoltConReg;

struct PLLCONTREG	
{  
	float32  f32Valpha;   		// Input: Reference input 
	float32  f32Vbeta;   		// Input: Feedback input
	float32  f32Vd;
	float32  f32Vq;
	float32  f32VqErr_Old;
	float32  f32VqErr_New;
	float32  Delta_Vq;   //2017.3.31 GX

	float32  f32Theta;   		// Output: PID output
	float32  f32Theta_Step;
	float32  Sin_Theta;		// 
	float32  Cos_Theta;			//
	float32	 Sin_5Theta;
	float32	 Cos_5Theta; 
	float32	 f32DQ_PLL_Lockin;
	float32	 f32Fre_Delta_k;
	float32  f32Vd_Filter;  //2017.2.27 GX
	float32  f32PLL_fail;   //2017.2.27 GX
	float32  f32Theta_Offset; //2017.3.7 GX
	float32  f32real_Sin_Theta;   //2017.3.7 GX
	Uint32   Period;//2017.5.24 GX
};	            
extern	struct	PLLCONTREG	GridPLLConReg, OutPLLConReg, VOutLPLLConReg, VOutHPLLConReg;

struct PLL
{
	float32  Input[3];   		// LPF
	float32  Output[3];   		// LPF
	float32  MAC;

	float32  f32Refer;
	float32  f32PIDErr_Old;
	float32  f32PIDErr_New;
	float32  f32PID_Output;   //2017.3.31 GX

	float32  f32Delta_Theta;   		// Output: PID output
	float32  f32Frequency;
	float32	 f32Theta;
};
extern	struct	PLL	GridPLLReg,VOutLPLLReg, VOutHPLLReg;


struct PLLCONTPARAREG	
{  
	float32  f32K_alpha;   		// Input: Feedback input 
	float32  f32K_beta;		// Variable: Error 
	float32  f32FreShift_Step;
};	            
extern	struct	PLLCONTPARAREG	PLLConParaReg;

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
//extern int16 Test_Start_of_SEQISR;

#endif

//--- end of file -----------------------------------------------------
