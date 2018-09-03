/*=============================================================================*
 *         Copyright(c) 2009-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : 50KW_Safety_EEprom.c 
 *
 *  PURPOSE  :  
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *      
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *   
 *============================================================================*/


#include "DSP2833x_Device.h"     	// DSP2833x Headerfile Include File
#include "3KW_MAINHEADER.h"	    // Main include file


extern void Times_ReadFromEEPROM();
extern void Times_WriteToEEPROM();
extern void Ref_ReadFromEEPROM();
extern void Ref_WriteToEEPROM();
extern void Sample_ReadFromEEPROM();
extern void Sample_WriteToEEPROM();

/*=============================================================================*
 * FUNCTION: 	ReadSafetyFromEEPROM()
 *
 * PURPOSE : 
 *
 * INPUT: 		Uint16 u16StartAddr----EEPROM address start to be read out.
 *				SAFETY_PARAMETER_REG* p_safety----Will be filled with the value read out.
 *		
 * RETURN: 		Uint16 type value:
 *				I2C_W_R_OK
 *				I2C_W_R_EXCEED_LIMIT
 *				I2C_W_R_DATA_AMOUNT_IS_ZERO
 *				I2C_R_FAILED
 *
 * CALLS: 		I2C_Read_InWord()
 *
 * CALLED BY:	Main.c  
 *
 * MORE INFO:
 *============================================================================*/

void TSK_Time_Record(void)
{
	static Uint16 temp = 0;
	while(1)
	{
		if(SEM_pend(&SEM_Time_Record, SYS_FOREVER) == 1)
		{
			temp ++;
			if (temp == 10)
			{
				RunningTime.Second ++;
				temp = 0;
			}
			if (RunningTime.Second == 60)
			{
				RunningTime.Minute++;
				RunningTime.Second = 0;
			}
			if (RunningTime.Minute == 60)
			{
				RunningTime.Hour_L++;
				RunningTime.Minute = 0;
			}
			if (RunningTime.Hour_L == 0xFFFF)
			{
				RunningTime.Hour_L = 0;
				RunningTime.Hour_H ++;
			}
			if (RunningTime.Hour_H == 0xFFFF)
			{
				RunningTime.Hour_H = 0;
				RunningTime.Hour_L = 0;
				RunningTime.Minute = 0;
				RunningTime.Second = 0;
				RunningTime.OverFlow = 1;
			}
			if (RunningTime.OverFlow == 1)
			{
				 g_StateCheck.bit.Time_Overflow = 1;
			}
			else
			{
				g_StateCheck.bit.Time_Overflow = 0;
			}
		}
	}
}


void Times_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 Check = 0;
	RunningTime.Second = AT24c512_ReadByte(0x0000);
	RunningTime.Minute = AT24c512_ReadByte(0x0001);
	AT24c512_ReadSeriseWord(0x0002, &RunningTime.Hour_L , 2);
	RunningTime.OverFlow = AT24c512_ReadByte(0x0006);
	RunningTime.TimeCheck = AT24c512_ReadWord(0x0007);
	Check ^= RunningTime.Second;
	Check ^= RunningTime.Minute;
	Check ^= RunningTime.Hour_L;
	Check ^= RunningTime.Hour_H;
	Check ^= RunningTime.OverFlow;
	if (Check != RunningTime.TimeCheck || RunningTime.Second == 65535)
	{
		RunningTime.Second = 0;
		RunningTime.Minute = 0;
		RunningTime.Hour_L = 0;
		RunningTime.Hour_H = 0;
		RunningTime.OverFlow = 0;
		Times_WriteToEEPROM();
	}
	HWI_enable();
}


/*=============================================================================*
 * FUNCTION: 	SaveSafetyInEEPROM()
 *
 * PURPOSE : 
 *
 * INPUT: 		Uint16 u16StartAddr----EEPROM address to be written in,
 *				SAFETY_PARAMETER_REG safety-------Safety instance to be written in
 *
 * RETURN: 		Uint16 type value:
 *				I2C_W_R_OK
 *				I2C_W_R_EXCEED_LIMIT
 *				I2C_W_R_DATA_AMOUNT_IS_ZERO
 *				I2C_R_FAILED
 *
 * CALLS: 		I2C_Write_InWord()
 *
 * CALLED BY:   
 *
 * MORE INFO:
 *============================================================================*/


void Times_WriteToEEPROM()
{
	HWI_disable();
	RunningTime.TimeCheck = 0;
	RunningTime.TimeCheck ^= RunningTime.Second;
	RunningTime.TimeCheck ^= RunningTime.Minute;
	RunningTime.TimeCheck ^= RunningTime.Hour_L;
	RunningTime.TimeCheck ^= RunningTime.Hour_H;
	RunningTime.TimeCheck ^= RunningTime.OverFlow;
	AT24c512_WriteByte(0x0000, RunningTime.Second);
	AT24c512_WriteByte(0x0001, RunningTime.Minute);
	AT24c512_WriteSeriseWord(0x0002, &RunningTime.Hour_L, 2);
	AT24c512_WriteByte(0x0006, RunningTime.OverFlow);
	AT24c512_WriteWord(0x0007, RunningTime.TimeCheck);
	HWI_enable();
}

void Ref_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 	read_ref_check1 = 0;
	Uint16 	ref_check1 = 0;
	Uint16 	inv_ref1= 0;

	inv_ref1 = AT24c512_ReadWord(0x0009);
	read_ref_check1 = AT24c512_ReadWord(0x000B);
	ref_check1 ^= inv_ref1;
	if (ref_check1 == read_ref_check1 && inv_ref1 != 65535)
		SafetyReg.f32Inv_VoltRms_Ref_LCD = inv_ref1 * 0.1;
	else
	{
		SafetyReg.f32Inv_VoltRms_Ref_LCD = 220;
		Ref_WriteToEEPROM();
	}
	HWI_enable();
}

void Ref_WriteToEEPROM()
{
	HWI_disable();
	Uint16 ref_check2= 0;
	Uint16 	inv_ref2= 0;

	inv_ref2 = (Uint16)(SafetyReg.f32Inv_VoltRms_Ref_LCD * 10);
	ref_check2 ^= inv_ref2;
	AT24c512_WriteWord(0x0009, inv_ref2);
	AT24c512_WriteWord(0x000B, ref_check2);
	HWI_enable();
}

void Sample_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 Read_Sample_Check1 = 0;
	Uint16 Sample_Correct_VInv1= 0;
	Uint16 Sample_Correct_IOut1= 0;
	Uint16 Sample_Correct_VGrid1= 0;
	Uint16 Sample_Correct_IGrid1= 0;
	Uint16 Sample_Correct_VBusP1= 0;
	Uint16 Sample_Correct_VBusN1= 0;
	Uint16 Sample_Correct_VOut1= 0;
	Uint16 Sample_Correct_TempPFC1= 0;
	Uint16 Sample_Correct_TempInv1= 0;
	Uint16 Sample_Cur_Coeff1= 0;
	Uint16  Sample_Check1 = 0;

	Sample_Correct_VInv1 = AT24c512_ReadWord(0x0010);
	Sample_Correct_IOut1 = AT24c512_ReadWord(0x0012);
	Sample_Correct_VGrid1 = AT24c512_ReadWord(0x0014);
	Sample_Correct_IGrid1 = AT24c512_ReadWord(0x0016);
	Sample_Correct_VBusP1 = AT24c512_ReadWord(0x0018);
	Sample_Correct_VBusN1 = AT24c512_ReadWord(0x001A);
	Sample_Correct_VOut1= AT24c512_ReadWord(0x001C) ;
	Sample_Correct_TempPFC1= AT24c512_ReadWord(0x001E);
	Sample_Correct_TempInv1= AT24c512_ReadWord(0x0020);
	Sample_Cur_Coeff1 = AT24c512_ReadWord(0x0022);
	Read_Sample_Check1 = AT24c512_ReadWord(0x0024);
	//**********×¢ÒâµØÖ·**************

	Sample_Check1 ^= Sample_Correct_VInv1;
	Sample_Check1 ^= Sample_Correct_IOut1;
	Sample_Check1 ^= Sample_Correct_VGrid1;
	Sample_Check1 ^= Sample_Correct_IGrid1;
	Sample_Check1 ^= Sample_Correct_VBusP1;
	Sample_Check1 ^= Sample_Correct_VBusN1;
	Sample_Check1 ^= Sample_Correct_VOut1;
	Sample_Check1 ^= Sample_Correct_TempPFC1;
	Sample_Check1 ^= Sample_Correct_TempInv1;
	Sample_Check1 ^= Sample_Cur_Coeff1;

	if (Sample_Check1 == Read_Sample_Check1 && Sample_Correct_VInv1 != 65535)
	{
		ADCorrection.f32VInv = Sample_Correct_VInv1 * 0.001f;
		ADCorrection.f32IOut = Sample_Correct_IOut1 * 0.001f;
		ADCorrection.f32VGrid = Sample_Correct_VGrid1 * 0.001f;
		ADCorrection.f32IGrid = Sample_Correct_IGrid1 * 0.001f;
		ADCorrection.f32VBusP = Sample_Correct_VBusP1 * 0.001f;
		ADCorrection.f32VBusN = Sample_Correct_VBusN1 * 0.001f;
		ADCorrection.f32VOut = Sample_Correct_VOut1 * 0.001f;
		ADCorrection.f32TempPFC = Sample_Correct_TempPFC1 * 0.001f;
		ADCorrection.f32TempInv = Sample_Correct_TempInv1 * 0.001f;
		InvVoltConReg.f32Drop_Coeff = Sample_Cur_Coeff1 * 0.0001f;
	}
	else
	{
		ADCorrection.f32VInv = 1.0f;
		ADCorrection.f32IOut = 1.0f;
		ADCorrection.f32VGrid = 1.0f;
		ADCorrection.f32IGrid = 1.0f;
		ADCorrection.f32VBusP  = 1.0f;
		ADCorrection.f32VBusN  = 1.0f;
		ADCorrection.f32VOut = 1.0f;
		ADCorrection.f32TempPFC = 1.0f;
		ADCorrection.f32TempInv = 1.0f;
		InvVoltConReg.f32Drop_Coeff = 1.0f;
		Sample_WriteToEEPROM();
	}
	HWI_enable();
}

void Sample_WriteToEEPROM()
{
	HWI_disable();
	Uint16 Sample_Correct_VInv2= 0;
	Uint16 Sample_Correct_IOut2= 0;
	Uint16 Sample_Correct_VGrid2= 0;
	Uint16 Sample_Correct_IGrid2= 0;
	Uint16 Sample_Correct_VBusP2= 0;
	Uint16 Sample_Correct_VBusN2= 0;
	Uint16 Sample_Correct_VOut2= 0;
	Uint16 Sample_Correct_TempPFC2= 0;
	Uint16 Sample_Correct_TempInv2= 0;
	Uint16 Sample_Cur_Coeff2 = 0;
	Uint16  Sample_Check2 = 0;

	Sample_Correct_VInv2 = (Uint16)(ADCorrection.f32VInv * 1000);
	Sample_Correct_IOut2 = (Uint16)(ADCorrection.f32IOut* 1000);
	Sample_Correct_VGrid2 = (Uint16)(ADCorrection.f32VGrid* 1000);
	Sample_Correct_IGrid2 = (Uint16)(ADCorrection.f32IGrid * 1000);
	Sample_Correct_VBusP2 = (Uint16)(ADCorrection.f32VBusP * 1000);
	Sample_Correct_VBusN2= (Uint16)(ADCorrection.f32VBusN * 1000 );
	Sample_Correct_VOut2= (Uint16)(ADCorrection.f32VOut * 1000);
	Sample_Correct_TempPFC2= (Uint16)(ADCorrection.f32TempPFC * 1000);
	Sample_Correct_TempInv2= (Uint16)(ADCorrection.f32TempInv * 1000);
	Sample_Cur_Coeff2 = (Uint16)(InvVoltConReg.f32Drop_Coeff * 10000);

	Sample_Check2 ^= Sample_Correct_VInv2;
	Sample_Check2 ^= Sample_Correct_IOut2;
	Sample_Check2 ^= Sample_Correct_VGrid2;
	Sample_Check2 ^= Sample_Correct_IGrid2;
	Sample_Check2 ^= Sample_Correct_VBusP2;
	Sample_Check2 ^= Sample_Correct_VBusN2;
	Sample_Check2 ^= Sample_Correct_VOut2;
	Sample_Check2 ^= Sample_Correct_TempPFC2;
	Sample_Check2 ^= Sample_Correct_TempInv2;
	Sample_Check2 ^= Sample_Cur_Coeff2;

	AT24c512_WriteWord(0x0010, Sample_Correct_VInv2);
	AT24c512_WriteWord(0x0012, Sample_Correct_IOut2);
	AT24c512_WriteWord(0x0014, Sample_Correct_VGrid2);
	AT24c512_WriteWord(0x0016, Sample_Correct_IGrid2);
	AT24c512_WriteWord(0x0018, Sample_Correct_VBusP2);
	AT24c512_WriteWord(0x001A, Sample_Correct_VBusN2);
	AT24c512_WriteWord(0x001C, Sample_Correct_VOut2);
	AT24c512_WriteWord(0x001E, Sample_Correct_TempPFC2);
	AT24c512_WriteWord(0x0020, Sample_Correct_TempInv2);
	AT24c512_WriteWord(0x0022, Sample_Cur_Coeff2);
	AT24c512_WriteWord(0x0024, Sample_Check2);
	HWI_enable();
}
