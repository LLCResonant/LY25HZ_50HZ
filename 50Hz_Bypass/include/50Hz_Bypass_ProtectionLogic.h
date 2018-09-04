/*=============================================================================*
 *         Copyright(c) 2010-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_ProtectionLogic.h 

 *  PURPOSE  : header files 5KW_ProtectionLogic.c 
 *			       define constant, struct declaration, extern varibles 
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-24      V0.1           Ken      	    Created
 *
 ******************************************************************************/

#ifndef PROTECTION_LOGIC_H
#define PROTECTION_LOGIC_H

//======================= Global constants============================  
//-------------------------------------------------------------------------------

//******************************************************************************
 #define VGridHi1Limit	            _IQ20(275)
 #define VGridLow1Limit        	_IQ20(165)
 #define VGridHi2Limit	            _IQ20(275)
 #define VGridLow2Limit        	_IQ20(165)
 #define VGridHi3Limit	            _IQ20(275)
 #define VGridLow3Limit        	_IQ20(165)
 #define VGridProtectionTime	5

 #define FreGridHiLimit	        _IQ10(55)
 #define FreGridLowLimit 	    _IQ10(45)
 #define FreGridProtectionTime	5
 #define AD_Channel_Offset_VGridLimit	_IQ(30)
 #define AD_Channel_Offset_VOutLimit	_IQ(30)
//==============================================================================

#define TempAmbHiLimit          80
#define TempProtecitonTime      5

#define ProtectionSet0					GpioDataRegs.GPADAT.bit.GPIO6
#define ProtectionSet1					GpioDataRegs.GPADAT.bit.GPIO7
//******************************************************************************
typedef struct
{
	_iq20	i32VGrid_HiLimit;
	_iq20	i32VGrid_LowLimit;
  	_iq20   i32VGrid_HiLimitBack;
	_iq20   i32VGrid_LowLimitBack;
	Uint16	i16VGrid_ProtectionTime;

	_iq10	i32FreGrid_HiLimit;
	_iq10	i32FreGrid_LowLimit;
	Uint16	i32FreGrid_ProtectionTime;

	int32	i32TempAmb_HiLimit;
	Uint16	i32TempAmb_ProtectionTime;

}SAFETY_PARAMETER_REG;

extern SAFETY_PARAMETER_REG	SafetyReg;


//===================== Global functions==================================

extern void GridVoltCheck(void);
extern void GridFreqCheck(void);
extern void TemperatureCheck(void);
extern void DCFanCheck(void);
extern void ADOffsetCheck(void);
//------------------
#endif




//--- end of file -----------------------------------------------------


