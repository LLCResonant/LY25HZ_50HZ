/*=============================================================================*
 *         Copyright(c) 2010-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_Regulator.h 
 *  PURPOSE  : header files 5KW_Regulator.c 
 *			       define constant, struct declaration, extern varibles 
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-24      V0.1           Ken      	    Created
 *
 ******************************************************************************/
#include "IQmathLib.h"
#ifndef SWITCHLOGIC_H
#define SWITCHLOGIC_H

/*
 *#define SCR_BY_ON		GpioDataRegs.GPASET.bit.GPIO1 = 1;
#define SCR_BY_OFF		GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
#define SCR_INV_ON		GpioDataRegs.GPASET.bit.GPIO2 = 1;
#define SCR_INV_OFF		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
 */

#define COM4_LEVEL			GpioDataRegs.GPBDAT.bit.GPIO32

#define COM2_ON					GpioDataRegs.GPASET.bit.GPIO0 = 1
#define COM4_ON					GpioDataRegs.GPASET.bit.GPIO22 = 1
#define SYNC_COM1_ON		GpioDataRegs.GPASET.bit.GPIO1 = 1

#define COM2_OFF				GpioDataRegs.GPACLEAR.bit.GPIO0= 1
#define COM4_OFF				GpioDataRegs.GPACLEAR.bit.GPIO22 = 1
#define SYNC_COM1_OFF		GpioDataRegs.GPACLEAR.bit.GPIO1 = 1

#define COM2_LEVEL			GpioDataRegs.GPADAT.bit.GPIO11
#define MODULE_LOCK		GpioDataRegs.GPADAT.bit.GPIO19
#define SWITCH_MODE			GpioDataRegs.GPADAT.bit.GPIO16

#define GridTheta_StepRated                  _IQ20(50*6.283185307/20000)
#define GridTheta_Step_Hi_Limit				_IQ20(60*6.283185307/20000)
#define GridTheta_Step_Low_Limit			_IQ20(40*6.283185307/20000)

struct PLL
{
	_iq20	i32Input[3];   		// LPF
	_iq20	i32Output[3];   		// LPF
	_iq20	i32MAC;

	_iq20	i32Refer;
	_iq20	i32PIDErr_Old;
	_iq20	i32PIDErr_New;
	_iq20	i32PID_Output;

	_iq20	i32Delta_Theta;   		// Output: PID output
	_iq20	i32Frequency;
	_iq20	i32Theta;
	_iq20	i32Theta_Step;
	_iq20	i32Valpha;
	_iq20	i32Sin_Theta;
	_iq20	i32Cos_Theta;
};
extern	struct	PLL	GridPLLReg;

extern void ADC_INT_Control(void);
extern void HRCAP1_INT_SyncPhase_Control(void);
extern void Switch_Logic_Control(void);
extern void HRCAP2_By_to_Inv(void);
extern void GridPLLControl(void);

#endif

//--- end of file -----------------------------------------------------
