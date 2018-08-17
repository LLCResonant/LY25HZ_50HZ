/*=============================================================================*
 *         
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 20KW_Scic_Slave.c 
 *
 *  PURPOSE  : SCIc between Master CPU and slave CPU.
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *   
 *                    Get_s_Sys_FaultMessage to Get_s_Sys_Message  
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

/*---- Standard headers -------------------------------------------------------*/

#include "DSP2833x_Device.h"		// Peripheral address definitions
#include "3KW_MAINHEADER.h"				// Main include file

//static Uint8	szRxBuf[40];
//static Uint8	szTxBuf[40];
Uint16	szArmADValue[18];
//static Uint8 	u8CommLoseFaultCnt = 0;
extern volatile float32 g_aux_f32OutputEnergy_new;

//static void Scic_Parsing(void);
//static void GetValue(void);
//static void SetValue(void);
//static Uint16 CheckSum(Uint8* pBuf, Uint16 len);

/*=============================================================================*
 * FUNCTION: TSK_Scic(void)
 * PURPOSE :
 * INPUT: 
 *     void
 *
 * RETURN: 
 *     void
 *
 *============================================================================*/

void TSK_SCIC_Transmit(void)
{
/*	while(1)
	{
		if(SEM_pend(&SEM_MasterCommTx, SYS_FOREVER) == 1)
		{
			SetValue();
			SciWrite(ID_SCIC, szTxBuf, 32);
			if(u8CommLoseFaultCnt> 5)       // 5*20ms = 0.1s, 20*0.02=0.4s
			{
				u8CommLoseFaultCnt = 0;
//				g_SysFaultMessage.bit.CommLoseFault =1;
			}
			u8CommLoseFaultCnt++;
			SEM_post(&SEM_MasterCommRx);
		}
	}
	//return;
*/
}

void TSK_SCIC_Receive(void)
{
/*
	while(1)
	{
		if(SEM_pend(&SEM_MasterCommRx, SYS_FOREVER) == 1)
		{
			Scic_Parsing();
			if(u8CommLoseFaultCnt>=20)
			{
				u8CommLoseFaultCnt = 0;
				g_SysFaultMessage.bit.CommLoseFault =1;
			}
		}
	}
*/
}


/*
void Scic_Parsing(void)
{
	Uint8 u8Tmp;
	Uint8 u8Data;
	static Uint16 CommFaultBackCnt =0;
	static Uint8 u8Index = 0;
	while(1)
	{
		u8Tmp = SciRead(ID_SCIC, &u8Data);
		if(!u8Tmp)
		{
			switch(u8Index)
			{
			case 0:
				if(u8Data == 'S')
				{
					szRxBuf[u8Index] = u8Data;
					u8Index++;
				}
				break;
			case 1:
				if(u8Data <=64 )    //can cut down
				{
					szRxBuf[u8Index] = u8Data;
					u8Index++;
				}
				else
				{
					u8Index = 0;
				}
				break;
			default:
				szRxBuf[u8Index] = u8Data;
				u8Index++;
				if(u8Index == (szRxBuf[1] + 4))
				{
					if( CheckSum(szRxBuf, szRxBuf[1] + 2) == ((szRxBuf[u8Index-2] << 8) + szRxBuf[u8Index-1]) )
					{
						u8CommLoseFaultCnt = 0;
						GetValue();
						CommFaultBackCnt++;
						if(CommFaultBackCnt>=300)
						{
							g_SysFaultMessage.bit.CommLoseFault =0;
							CommFaultBackCnt = 0;
						}
						u8Index = 0;
					}
					else
					{
						u8Index = 0;
					}	
				}
				break;
			}
		}
		else
		{
			u8CommLoseFaultCnt++;
			break;
		}
	}
}
*/

/*
void GetValue(void)
{
	Uint8 i;
	szArmADValue[0] = 0;
	for(i=1;i<18;i++)
	{
		szArmADValue[i] = (szRxBuf[2*i] << 8) + szRxBuf[2*i+1];
	}
//	Get_s_Sys_Message();
}
*/

/*
void SetValue(void)
{
	Uint16 u16Ret;
	szTxBuf[0] = 'M';
	szTxBuf[1] = 28;
	szTxBuf[2] = 0;//GET_HBYTE_OF_WORD(g_Sys_Current_State);
	szTxBuf[3] = GET_LBYTE_OF_WORD(g_Sys_Current_State);
	szTxBuf[4] = g_SysFaultMessage.Word.byte0;
	szTxBuf[5] = g_SysFaultMessage.Word.byte1;
	szTxBuf[6] = g_SysFaultMessage.Word.byte2;
	szTxBuf[7] = g_SysFaultMessage.Word.byte3;
	szTxBuf[8] = g_SysFaultMessage.Word.byte4;
	szTxBuf[9] = g_SysFaultMessage.Word.unrecover1;
	szTxBuf[10] = g_SysFaultMessage.Word.unrecover2;
	szTxBuf[11] = g_SysFaultMessage.Word.unrecover3;
	szTxBuf[12] = GET_HBYTE_OF_WORD((Uint16)(Calc_Result.f32IGrid_rms*100));
	szTxBuf[13] = GET_LBYTE_OF_WORD((Uint16)(Calc_Result.f32IGrid_rms*100));
	if((StabilizedVoltLoop == MpptPara_Reg.CurrentWorkMode)||(CurLoop == MpptPara_Reg.CurrentWorkMode))
	{	
	}
	if(MpptLoop == MpptPara_Reg.CurrentWorkMode)
	{
	}	
	szTxBuf[16] = GET_HBYTE_OF_WORD((Uint16)(Calc_Result.f32GridFreq*100));
	szTxBuf[17] = GET_LBYTE_OF_WORD((Uint16)(Calc_Result.f32GridFreq*100));
	szTxBuf[18] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32OutputWattH);
	szTxBuf[19] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32OutputWattH);
	szTxBuf[20] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32OutputEnergy);
	szTxBuf[21] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32OutputEnergy);
	szTxBuf[22] = GET_HBYTE_OF_WORD((Uint16)g_aux_f32OutputEnergy_new);//modified by XXXXX  8.24
	szTxBuf[23] = GET_LBYTE_OF_WORD((Uint16)g_aux_f32OutputEnergy_new);//modified by XXXXX  8.24

	szTxBuf[26] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VGrid_rms);     //modified by XXXXX
	szTxBuf[27] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VGrid_rms);

  	
	szTxBuf[12] = 0;
	szTxBuf[13] = g_StateCheck.Word.byte6;	
	szTxBuf[14] = g_StateCheck.Word.byte5;
	szTxBuf[15] = g_StateCheck.Word.byte4;
	szTxBuf[16] = g_StateCheck.Word.byte3;
	szTxBuf[17] = g_StateCheck.Word.byte2;
	szTxBuf[18] = g_StateCheck.Word.byte1;
	szTxBuf[19] = g_StateCheck.Word.byte0;
	szTxBuf[20] = g_MPPT_State.Word.byte1;
	szTxBuf[21] = g_MPPT_State.Word.byte0;
	szTxBuf[22] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32Vbus);
	szTxBuf[23] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32Vbus);	
	szTxBuf[24] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VbusP);
	szTxBuf[25] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VbusP);

	szTxBuf[38] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VInv_Rrms);
	szTxBuf[39] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VInv_Rrms);	
	szTxBuf[40] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VInv_RSrms);
	szTxBuf[41] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VInv_RSrms);
	szTxBuf[42] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VInv_Srms);
	szTxBuf[43] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VInv_Srms);

	szTxBuf[50] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32Viso1);
	szTxBuf[51] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32Viso1);
	szTxBuf[52] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32Viso2);
	szTxBuf[53] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32Viso2);
	szTxBuf[54] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VPV);
	szTxBuf[55] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VPV);
	szTxBuf[56] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VPV1);
	szTxBuf[57] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VPV1);
	szTxBuf[58] = GET_HBYTE_OF_WORD((Uint16)Calc_Result.f32VPV2);
	szTxBuf[59] = GET_LBYTE_OF_WORD((Uint16)Calc_Result.f32VPV2);

	u16Ret = CheckSum(szTxBuf,30);
	szTxBuf[30] = GET_HBYTE_OF_WORD(u16Ret);
	szTxBuf[31] = GET_LBYTE_OF_WORD(u16Ret);
}

*/

/*

Uint16 CheckSum(Uint8* pBuf, Uint16 len)
{
	Uint16 i;
	Uint16 u16Sum;
	u16Sum = 0;
	for(i = 0;i < len;i++)
	{
		u16Sum = u16Sum + *(pBuf + i);
	}
	return u16Sum;
}
*/

/*
void Get_s_Sys_Message(void)
{

}
*/
