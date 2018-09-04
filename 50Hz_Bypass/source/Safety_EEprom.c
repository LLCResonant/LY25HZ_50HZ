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


#include "DSP2803x_Device.h"				// Peripheral address definitions
#include "F28035_example.h"					// Main include file


extern void Times_ReadFromEEPROM();
extern void Times_WriteToEEPROM();
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
	if (Check != RunningTime.TimeCheck)
	{
		RunningTime.Second = 0;
		RunningTime.Minute = 0;
		RunningTime.Hour_L = 0;
		RunningTime.Hour_H = 0;
		RunningTime.OverFlow = 0;
		Times_WriteToEEPROM();
	}
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
}

void Sample_ReadFromEEPROM()
{
	Uint16 Read_Sample_Check1 = 0;
	Uint16 Sample_Correct_IGrid1 = 0;
	Uint16 Sample_Correct_VGrid1 = 0;
	Uint16 Sample_Correct_VOut1 = 0;
	Uint16 Sample_Correct_TempAmb1 = 0;
	Uint16  Sample_Check1 = 0;

	Sample_Correct_IGrid1 = AT24c512_ReadWord(0x0010);
	Sample_Correct_VGrid1 = AT24c512_ReadWord(0x0012);
	Sample_Correct_VOut1 = AT24c512_ReadWord(0x0014);
	Sample_Correct_TempAmb1 = AT24c512_ReadWord(0x0016);
	Read_Sample_Check1 = AT24c512_ReadWord(0x0018);   //**********注意地址**************

	Sample_Check1 ^= Sample_Correct_IGrid1;
	Sample_Check1 ^= Sample_Correct_VGrid1;
	Sample_Check1 ^= Sample_Correct_VOut1;
	Sample_Check1 ^= Sample_Correct_TempAmb1;

	if (Sample_Check1 == Read_Sample_Check1)
	{
		ADChannelOffset.i32IGrid = Sample_Correct_IGrid1 * 0.001f;
		ADChannelOffset.i32VGrid = Sample_Correct_VGrid1 * 0.001f;
		ADChannelOffset.i32VOut = Sample_Correct_VOut1 * 0.001f;
		ADChannelOffset.i32TempAmb = Sample_Correct_TempAmb1 * 0.001f;
	}
	else
	{
		ADChannelOffset.i32IGrid = 1.0f;
		ADChannelOffset.i32VGrid = 1.0f;
		ADChannelOffset.i32VOut = 1.0f;
		ADChannelOffset.i32TempAmb = 1.0f;
		Sample_WriteToEEPROM();
	}
}

void Sample_WriteToEEPROM()
{
	Uint16 Sample_Correct_IGrid2 = 0;
	Uint16 Sample_Correct_VGrid2 = 0;
	Uint16 Sample_Correct_VOut2 = 0;
	Uint16 Sample_Correct_TempAmb2 = 0;
	Uint16  Sample_Check2 = 0;

	Sample_Correct_IGrid2 = (Uint16)(ADChannelOffset.i32IGrid * 1000);
	Sample_Correct_VGrid2 = (Uint16)(ADChannelOffset.i32VGrid * 1000);
	Sample_Correct_VOut2 = (Uint16)(ADChannelOffset.i32VOut * 1000);
	Sample_Correct_TempAmb2 = (Uint16)(ADChannelOffset.i32TempAmb * 1000);

	Sample_Check2 ^= Sample_Correct_IGrid2;
	Sample_Check2 ^= Sample_Correct_VGrid2;
	Sample_Check2 ^= Sample_Correct_VOut2;
	Sample_Check2 ^= Sample_Correct_TempAmb2;

	AT24c512_WriteWord(0x0010, Sample_Correct_IGrid2);
	AT24c512_WriteWord(0x0012, Sample_Correct_VGrid2);
	AT24c512_WriteWord(0x0014, Sample_Correct_VOut2);
	AT24c512_WriteWord(0x0016, Sample_Correct_TempAmb2);
	AT24c512_WriteWord(0x0018, Sample_Check2);
}
