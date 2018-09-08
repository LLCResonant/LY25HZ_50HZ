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

void Times_ReadFromEEPROM();
void Times_WriteToEEPROM();
void Sample_ReadFromEEPROM();
void Sample_WriteToEEPROM();
void EEPROMParamDefault();

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
	static Uint16 s_u16temp = 0;
	while(1)
	{
		if(SEM_pend(&SEM_Time_Record, SYS_FOREVER) == 1)
		{
			s_u16temp ++;
			if (s_u16temp == 10)
			{
				RunningTime.u16Second ++;
				s_u16temp = 0;
			}
			if (RunningTime.u16Second == 60)
			{
				RunningTime.u16Minute++;
				RunningTime.u16Second = 0;
			}
			if (RunningTime.u16Minute == 60)
			{
				RunningTime.u16Hour_L++;
				RunningTime.u16Minute = 0;
			}
			if (RunningTime.u16Hour_L == 0xFFFF)
			{
				RunningTime.u16Hour_L = 0;
				RunningTime.u16Hour_H ++;
			}
			if (RunningTime.u16Hour_H == 0xFFFF)
			{
				RunningTime.u16Hour_H = 0;
				RunningTime.u16Hour_L = 0;
				RunningTime.u16Minute = 0;
				RunningTime.u16Second = 0;
				RunningTime.u16OverFlow = 1;
			}
			if (RunningTime.u16OverFlow == 1)
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

void EEPROMParamDefault(void)
{
	Eeprom_Gpio_Init();           // Initialize the I2C.
	#ifdef NORMAL_EEPROM
 	Times_ReadFromEEPROM();
 	Sample_ReadFromEEPROM();
	#endif

	#ifdef RESET_EEPROM
 	Times_WriteToEEPROM();
 	Sample_WriteToEEPROM();
	#endif
}

void Times_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 u16Check = 0;
	RunningTime.u16Second = AT24c512_ReadByte(0x0000);
	RunningTime.u16Minute = AT24c512_ReadByte(0x0002);
	AT24c512_ReadSeriseWord(0x0004, &RunningTime.u16Hour_L , 2);
	RunningTime.u16OverFlow = AT24c512_ReadByte(0x0008);
	RunningTime.u16TimeCheck = AT24c512_ReadWord(0x000A);
	u16Check ^= RunningTime.u16Second;
	u16Check ^= RunningTime.u16Minute;
	u16Check ^= RunningTime.u16Hour_L;
	u16Check ^= RunningTime.u16Hour_H;
	u16Check ^= RunningTime.u16OverFlow;
	if (u16Check != RunningTime.u16TimeCheck || RunningTime.u16Second == 65535 )
	{
		RunningTime.u16Second = 0;
		RunningTime.u16Minute = 0;
		RunningTime.u16Hour_L = 0;
		RunningTime.u16Hour_H = 0;
		RunningTime.u16OverFlow = 0;
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
	RunningTime.u16TimeCheck = 0;
	RunningTime.u16TimeCheck ^= RunningTime.u16Second;
	RunningTime.u16TimeCheck ^= RunningTime.u16Minute;
	RunningTime.u16TimeCheck ^= RunningTime.u16Hour_L;
	RunningTime.u16TimeCheck ^= RunningTime.u16Hour_H;
	RunningTime.u16TimeCheck ^= RunningTime.u16OverFlow;
	AT24c512_WriteByte(0x0000, RunningTime.u16Second);
	AT24c512_WriteByte(0x0002, RunningTime.u16Minute);
	AT24c512_WriteSeriseWord(0x0004, &RunningTime.u16Hour_L, 2);
	AT24c512_WriteByte(0x0008, RunningTime.u16OverFlow);
	AT24c512_WriteWord(0x000A, RunningTime.u16TimeCheck);
	HWI_enable();
}

void Sample_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 u16Read_Sample_Check1 = 0;
	Uint16 u16Sample_Correct_IGrid1 = 0;
	Uint16 u16Sample_Correct_VGrid1 = 0;
	Uint16 u16Sample_Correct_VOut1 = 0;
	Uint16 u16Sample_Correct_TempAmb1 = 0;
	Uint16  u16Sample_Check1 = 0;

	u16Sample_Correct_IGrid1 = AT24c512_ReadWord(0x0010);
	u16Sample_Correct_VGrid1 = AT24c512_ReadWord(0x0012);
	u16Sample_Correct_VOut1 = AT24c512_ReadWord(0x0014);
	u16Sample_Correct_TempAmb1 = AT24c512_ReadWord(0x0016);
	u16Read_Sample_Check1 = AT24c512_ReadWord(0x0018);

	u16Sample_Check1 ^= u16Sample_Correct_IGrid1;
	u16Sample_Check1 ^= u16Sample_Correct_VGrid1;
	u16Sample_Check1 ^= u16Sample_Correct_VOut1;
	u16Sample_Check1 ^= u16Sample_Correct_TempAmb1;

	if (u16Sample_Check1 == u16Read_Sample_Check1 && u16Sample_Correct_IGrid1 != 65535)
	{
		ADChannelOffset.iq20IGrid = _IQ20mpyI32(_IQ(0.001f), u16Sample_Correct_IGrid1);
		ADChannelOffset.iq20VGrid = _IQ20mpyI32(_IQ(0.001f),u16Sample_Correct_VGrid1);
		ADChannelOffset.iq20VOut = _IQ20mpyI32(_IQ(0.001f), u16Sample_Correct_VOut1);
		ADChannelOffset.iq20TempAmb = _IQ20mpyI32(_IQ(0.001f), u16Sample_Correct_TempAmb1);
	}
	else
	{
		ADChannelOffset.iq20IGrid = _IQ(1.0f);
		ADChannelOffset.iq20VGrid = _IQ(1.0f);
		ADChannelOffset.iq20VOut = _IQ(1.0f);
		ADChannelOffset.iq20TempAmb = _IQ(1.0f);
		Sample_WriteToEEPROM();
	}
	HWI_enable();
}

void Sample_WriteToEEPROM()
{
	HWI_disable();
	Uint16 u16Sample_Correct_IGrid2 = 0;
	Uint16 u16Sample_Correct_VGrid2 = 0;
	Uint16 u16Sample_Correct_VOut2 = 0;
	Uint16 u16Sample_Correct_TempAmb2 = 0;
	Uint16  u16Sample_Check2 = 0;

	u16Sample_Correct_IGrid2 = _IQint(ADChannelOffset.iq20IGrid * 1000);
	u16Sample_Correct_VGrid2 = _IQint(ADChannelOffset.iq20VGrid * 1000);
	u16Sample_Correct_VOut2 = _IQint(ADChannelOffset.iq20VOut * 1000);
	u16Sample_Correct_TempAmb2 = _IQint(ADChannelOffset.iq20TempAmb * 1000);

	u16Sample_Check2 ^= u16Sample_Correct_IGrid2;
	u16Sample_Check2 ^= u16Sample_Correct_VGrid2;
	u16Sample_Check2 ^= u16Sample_Correct_VOut2;
	u16Sample_Check2 ^= u16Sample_Correct_TempAmb2;

	AT24c512_WriteWord(0x0010, u16Sample_Correct_IGrid2);
	AT24c512_WriteWord(0x0012, u16Sample_Correct_VGrid2);
	AT24c512_WriteWord(0x0014, u16Sample_Correct_VOut2);
	AT24c512_WriteWord(0x0016, u16Sample_Correct_TempAmb2);
	AT24c512_WriteWord(0x0018, u16Sample_Check2);
	HWI_enable();
}
