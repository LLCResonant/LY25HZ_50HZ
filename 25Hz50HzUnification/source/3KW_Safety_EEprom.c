/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_Safety_EEprom.c
 *
 *  PURPOSE  :
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/


#include "DSP2833x_Device.h"     	// DSP2833x Headerfile Include File
#include "3KW_MAINHEADER.h"	    // Main include file

struct Time  RunningTime;

void Times_ReadFromEEPROM();
void Times_WriteToEEPROM();
void VoltsRef_ReadFromEEPROM();
void VoltsRef_WriteToEEPROM();
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

void EEPROMParamDefault(void)
{
	Eeprom_Gpio_Init();           // Initialize the I2C.
	#ifdef NORMAL_EEPROM
 	Times_ReadFromEEPROM();
 	VoltsRef_ReadFromEEPROM();
 	Sample_ReadFromEEPROM();
	#endif

	#ifdef RESET_EEPROM
 	Times_WriteToEEPROM();
 	VoltsRef_WriteToEEPROM();
 	Sample_WriteToEEPROM();
	#endif
}

void Times_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 Check = 0;
	RunningTime.Second = AT24c512_ReadByte(0x0000);
	RunningTime.Minute = AT24c512_ReadByte(0x0002);
	AT24c512_ReadSeriesWord(0x0004, &RunningTime.Hour_L , 2);
	RunningTime.OverFlow = AT24c512_ReadByte(0x0008);
	RunningTime.TimeCheck = AT24c512_ReadWord(0x000A);
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

void Times_WriteToEEPROM()
{
	HWI_disable();
	RunningTime.TimeCheck = 0;
	RunningTime.TimeCheck ^= RunningTime.Second;
	RunningTime.TimeCheck ^= RunningTime.Minute;
	RunningTime.TimeCheck ^= RunningTime.Hour_L;
	RunningTime.TimeCheck ^= RunningTime.Hour_H;
	RunningTime.TimeCheck ^= RunningTime.OverFlow;
	AT24c512_WriteWord(0x0000, RunningTime.Second);
	AT24c512_WriteWord(0x0002, RunningTime.Minute);
	AT24c512_WriteSeriesWord(0x0004, &RunningTime.Hour_L, 2);
	AT24c512_WriteWord(0x0008, RunningTime.OverFlow);
	AT24c512_WriteWord(0x000A, RunningTime.TimeCheck);
	HWI_enable();
}

void VoltsRef_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 read_ref_check1 = 0;
	Uint16 ref_check1 = 0;
	Uint16 	invh_ref1= 0;
	Uint16  invl_ref1= 0;

	invh_ref1 = AT24c512_ReadWord(0x0010);
	invl_ref1 = AT24c512_ReadWord(0x0012);
	read_ref_check1 = AT24c512_ReadWord(0x0014);
	ref_check1 ^= invh_ref1;
	ref_check1 ^= invl_ref1;
	if (ref_check1 == read_ref_check1&& invh_ref1 != 65535)
	{
		SafetyReg.f32InvH_VoltRms_Ref_LCD = invh_ref1 * 0.1;
		SafetyReg.f32InvL_VoltRms_Ref_LCD = invl_ref1 * 0.1;
	}
	else
	{
		SafetyReg.f32InvH_VoltRms_Ref_LCD = 220;
		SafetyReg.f32InvL_VoltRms_Ref_LCD = 110;
		VoltsRef_WriteToEEPROM();
	}
	HWI_enable();
}

void VoltsRef_WriteToEEPROM()
{
	HWI_disable();
	Uint16 ref_check2= 0;
	Uint16 	invh_ref2= 0;
	Uint16  invl_ref2= 0;

	invh_ref2 = (Uint16)(SafetyReg.f32InvH_VoltRms_Ref_LCD * 10);
	invl_ref2 = (Uint16)(SafetyReg.f32InvL_VoltRms_Ref_LCD * 10);
	ref_check2 ^= invh_ref2;
	ref_check2 ^= invl_ref2;
	AT24c512_WriteWord(0x0010, invh_ref2);
	AT24c512_WriteWord(0x0012, invl_ref2);
	AT24c512_WriteWord(0x0014, ref_check2);
	HWI_enable();
}

void Sample_ReadFromEEPROM()
{
	HWI_disable();
	Uint16 u16Read_Sample_Check1 = 0;
	Uint16 u16Sample_Correct_VInvH1= 0;
	Uint16 u16Sample_Correct_VInvL1= 0;
	Uint16 u16Sample_Correct_IOutH1= 0;
	Uint16 u16Sample_Correct_IOutL1= 0;
	Uint16 u16Sample_Correct_VGrid1= 0;
	Uint16 u16Sample_Correct_IGrid1= 0;
	Uint16 u16Sample_Correct_VBusP1= 0;
	Uint16 u16Sample_Correct_VBusN1= 0;
	Uint16 u16Sample_Correct_VOutH1= 0;
	Uint16 u16Sample_Correct_VOutL1= 0;
	Uint16 u16Sample_Correct_TempPFC1= 0;
	Uint16 u16Sample_Correct_TempInvH1= 0;
	Uint16 u16Sample_Correct_TempInvL1= 0;
	Uint16 u16Sample_InvH_Comp_Coeff1= 0;
	Uint16 u16Sample_InvL_Comp_Coeff1= 0;
	Uint16  u16Sample_Check1 = 0;

	u16Sample_Correct_VInvH1 = AT24c512_ReadWord(0x0020);
	u16Sample_Correct_VInvL1 = AT24c512_ReadWord(0x0022);
	u16Sample_Correct_IOutH1 = AT24c512_ReadWord(0x0024);
	u16Sample_Correct_IOutL1 = AT24c512_ReadWord(0x0026);
	u16Sample_Correct_VGrid1 = AT24c512_ReadWord(0x0028);
	u16Sample_Correct_IGrid1 = AT24c512_ReadWord(0x002A);
	u16Sample_Correct_VBusP1 = AT24c512_ReadWord(0x002C);
	u16Sample_Correct_VBusN1 = AT24c512_ReadWord(0x002E);
	u16Sample_Correct_VOutH1= AT24c512_ReadWord(0x0030) ;
	u16Sample_Correct_VOutL1= AT24c512_ReadWord(0x0032);
	u16Sample_Correct_TempPFC1= AT24c512_ReadWord(0x0034);
	u16Sample_Correct_TempInvH1= AT24c512_ReadWord(0x0036);
	u16Sample_Correct_TempInvL1= AT24c512_ReadWord(0x0038);
	u16Sample_InvH_Comp_Coeff1 = AT24c512_ReadWord(0x003A);
	u16Sample_InvL_Comp_Coeff1 = AT24c512_ReadWord(0x003C);
	u16Read_Sample_Check1 = AT24c512_ReadWord(0x003E);

	u16Sample_Check1 ^= u16Sample_Correct_VInvH1;
	u16Sample_Check1 ^= u16Sample_Correct_VInvL1;
	u16Sample_Check1 ^= u16Sample_Correct_IOutH1;
	u16Sample_Check1 ^= u16Sample_Correct_IOutL1;
	u16Sample_Check1 ^= u16Sample_Correct_VGrid1;
	u16Sample_Check1 ^= u16Sample_Correct_IGrid1;
	u16Sample_Check1 ^= u16Sample_Correct_VBusP1;
	u16Sample_Check1 ^= u16Sample_Correct_VBusN1;
	u16Sample_Check1 ^= u16Sample_Correct_VOutH1;
	u16Sample_Check1 ^= u16Sample_Correct_VOutL1;
	u16Sample_Check1 ^= u16Sample_Correct_TempPFC1;
	u16Sample_Check1 ^= u16Sample_Correct_TempInvH1;
	u16Sample_Check1 ^= u16Sample_Correct_TempInvL1;
	u16Sample_Check1 ^= u16Sample_InvH_Comp_Coeff1;
	u16Sample_Check1 ^= u16Sample_InvL_Comp_Coeff1;

	if (u16Sample_Check1 == u16Read_Sample_Check1 && u16Sample_Correct_VInvH1 != 65535)
	{
		ADCalibration.f32VInvH = u16Sample_Correct_VInvH1 * 0.001f;
		ADCalibration.f32VInvL = u16Sample_Correct_VInvL1 * 0.001f;
		ADCalibration.f32IOutH = u16Sample_Correct_IOutH1 * 0.001f;
		ADCalibration.f32IOutL = u16Sample_Correct_IOutL1 * 0.001f;
		ADCalibration.f32VGrid = u16Sample_Correct_VGrid1 * 0.001f;
		ADCalibration.f32IGrid = u16Sample_Correct_IGrid1 * 0.001f;
		ADCalibration.f32VBusP = u16Sample_Correct_VBusP1 * 0.001f;
		ADCalibration.f32VBusN = u16Sample_Correct_VBusN1 * 0.001f;
		ADCalibration.f32VOutH = u16Sample_Correct_VOutH1 * 0.001f;
		ADCalibration.f32VOutL = u16Sample_Correct_VOutL1 * 0.001f;
		ADCalibration.f32TempPFC = u16Sample_Correct_TempPFC1 * 0.001f;
		ADCalibration.f32TempInvH = u16Sample_Correct_TempInvH1 * 0.001f;
		ADCalibration.f32TempInvL = u16Sample_Correct_TempInvL1 * 0.001f;
		Parallel_Reg.f32VInvH_Comp_Coeff = u16Sample_InvH_Comp_Coeff1 * 0.0001f;
		Parallel_Reg.f32VInvL_Comp_Coeff = u16Sample_InvL_Comp_Coeff1 * 0.0001f;
	}
	else
	{
		ADCalibration.f32VInvH = 1.0f;
		ADCalibration.f32VInvL= 1.0f;
		ADCalibration.f32IOutH = 1.0f;
		ADCalibration.f32IOutL = 1.0f;
		ADCalibration.f32VGrid = 1.0f;
		ADCalibration.f32IGrid = 1.0f;
		ADCalibration.f32VBusP  = 1.0f;
		ADCalibration.f32VBusN  = 1.0f;
		ADCalibration.f32VOutH = 1.0f;
		ADCalibration.f32VOutL = 1.0f;
		ADCalibration.f32TempPFC = 1.0f;
		ADCalibration.f32TempInvH = 1.0f;
		ADCalibration.f32TempInvL = 1.0f;
		Parallel_Reg.f32VInvH_Comp_Coeff = 1.0f;
		Parallel_Reg.f32VInvL_Comp_Coeff = 1.0f;
		Sample_WriteToEEPROM();
	}
	HWI_enable();
}

void Sample_WriteToEEPROM()
{
	HWI_disable();
	Uint16 u16Sample_Correct_VInvH2 = 0;
	Uint16 u16Sample_Correct_VInvL2 = 0;
	Uint16 u16Sample_Correct_IOutH2 = 0;
	Uint16 u16Sample_Correct_IOutL2 = 0;
	Uint16 u16Sample_Correct_VGrid2 = 0;
	Uint16 u16Sample_Correct_IGrid2 = 0;
	Uint16 u16Sample_Correct_VBusP2 = 0;
	Uint16 u16Sample_Correct_VBusN2 = 0;
	Uint16 u16Sample_Correct_VOutH2 = 0;
	Uint16 u16Sample_Correct_VOutL2 = 0;
	Uint16 u16Sample_Correct_TempPFC2 = 0;
	Uint16 u16Sample_Correct_TempInvH2 = 0;
	Uint16 u16Sample_Correct_TempInvL2 = 0;
	Uint16 u16Sample_InvH_Comp_Coeff2 =0;
	Uint16 u16Sample_InvL_Comp_Coeff2 = 0;
	Uint16  u16Sample_Check2 = 0;

	u16Sample_Correct_VInvH2 = (Uint16)(ADCalibration.f32VInvH * 1000);
	u16Sample_Correct_VInvL2 = (Uint16)(ADCalibration.f32VInvL * 1000);
	u16Sample_Correct_IOutH2 = (Uint16)(ADCalibration.f32IOutH * 1000);
	u16Sample_Correct_IOutL2 = (Uint16)(ADCalibration.f32IOutL * 1000);
	u16Sample_Correct_VGrid2 = (Uint16)(ADCalibration.f32VGrid* 1000);
	u16Sample_Correct_IGrid2 = (Uint16)(ADCalibration.f32IGrid * 1000);
	u16Sample_Correct_VBusP2 = (Uint16)(ADCalibration.f32VBusP * 1000);
	u16Sample_Correct_VBusN2= (Uint16)(ADCalibration.f32VBusN * 1000 );
	u16Sample_Correct_VOutH2= (Uint16)(ADCalibration.f32VOutH * 1000);
	u16Sample_Correct_VOutL2= (Uint16)(ADCalibration.f32VOutL * 1000);
	u16Sample_Correct_TempPFC2= (Uint16)(ADCalibration.f32TempPFC * 1000);
	u16Sample_Correct_TempInvH2= (Uint16)(ADCalibration.f32TempInvH * 1000);
	u16Sample_Correct_TempInvL2= (Uint16)(ADCalibration.f32TempInvL * 1000);
	u16Sample_InvH_Comp_Coeff2= (Uint16)(Parallel_Reg.f32VInvH_Comp_Coeff * 10000);
	u16Sample_InvL_Comp_Coeff2= (Uint16)(Parallel_Reg.f32VInvL_Comp_Coeff * 10000);

	u16Sample_Check2 ^= u16Sample_Correct_VInvH2;
	u16Sample_Check2 ^= u16Sample_Correct_VInvL2;
	u16Sample_Check2 ^= u16Sample_Correct_IOutH2;
	u16Sample_Check2 ^= u16Sample_Correct_IOutL2;
	u16Sample_Check2 ^= u16Sample_Correct_VGrid2;
	u16Sample_Check2 ^= u16Sample_Correct_IGrid2;
	u16Sample_Check2 ^= u16Sample_Correct_VBusP2;
	u16Sample_Check2 ^= u16Sample_Correct_VBusN2;
	u16Sample_Check2 ^= u16Sample_Correct_VOutH2;
	u16Sample_Check2 ^= u16Sample_Correct_VOutL2;
	u16Sample_Check2 ^= u16Sample_Correct_TempPFC2;
	u16Sample_Check2 ^= u16Sample_Correct_TempInvH2;
	u16Sample_Check2 ^= u16Sample_Correct_TempInvL2;
	u16Sample_Check2 ^= u16Sample_InvH_Comp_Coeff2;
	u16Sample_Check2 ^= u16Sample_InvL_Comp_Coeff2;

	AT24c512_WriteWord(0x0020, u16Sample_Correct_VInvH2);
	AT24c512_WriteWord(0x0022, u16Sample_Correct_VInvL2);
	AT24c512_WriteWord(0x0024, u16Sample_Correct_IOutH2);
	AT24c512_WriteWord(0x0026, u16Sample_Correct_IOutL2);
	AT24c512_WriteWord(0x0028, u16Sample_Correct_VGrid2);
	AT24c512_WriteWord(0x002A, u16Sample_Correct_IGrid2);
	AT24c512_WriteWord(0x002C, u16Sample_Correct_VBusP2);
	AT24c512_WriteWord(0x002E, u16Sample_Correct_VBusN2);
	AT24c512_WriteWord(0x030, u16Sample_Correct_VOutH2);
	AT24c512_WriteWord(0x0032, u16Sample_Correct_VOutL2);
	AT24c512_WriteWord(0x0034, u16Sample_Correct_TempPFC2);
	AT24c512_WriteWord(0x0036, u16Sample_Correct_TempInvH2);
	AT24c512_WriteWord(0x0038, u16Sample_Correct_TempInvL2);
	AT24c512_WriteWord(0x003A, u16Sample_InvH_Comp_Coeff2);
	AT24c512_WriteWord(0x003C, u16Sample_InvL_Comp_Coeff2);
	AT24c512_WriteWord(0x003E, u16Sample_Check2);
	HWI_enable();
}
