/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_Scib_PCOsc.c
 *
 *  PURPOSE  :
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/


#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"					// Main include file
                                         

#pragma CODE_SECTION(Scib_SnatchGraph, "ControlLoopInRAM") 
//#pragma DATA_SECTION(u8Scib_UserDataBuf0, "ConstantsInRAM")
//#pragma DATA_SECTION(u8Scib_RxBuffer0, "ConstantsInRAM")
//#pragma DATA_SECTION(u16Scib_TransmitDataBuff, "ConstantsInRAM")
//#pragma DATA_SECTION(u16Scib_GraphDataBuff, "ConstantsInRAM")

typedef int (*pFunc1)(void);

/******************************variable definition******************************/
Uint8  u8Scib_UserDataBuf0[200];
Uint8  u8Scib_RxBuffer0[50];
Uint8  u8Scib_CommandBuffer0[50];
Uint8  *pScib_CommandIn0;
Uint8  u8Scib_Temp = 0;
Uint16 u16Scib_CommandLength = 0;

// snatch graph part
Uint8  u8Scib_SnatchGraphEnable = 1;
Uint16 u16Scib_Interval = 0;
Uint16 u16Scib_Interval1 = 0;
Uint8  u8Scib_wTrigger = 0;
Uint8  u8Scib_wTriggerSource = 0;

Uint16 u16Scib_SnatchDataCnt = 0;
Uint16 u16Scib_SaveDataCnt = 0;
Uint16 u16Scib_TransmitCnt = 0;

Uint16 u16Scib_CompareVal = 0;
Uint8  u8Scib_Sign = 0;

Uint8  u8Scib_DataKind[4] = {0,0,0,0};

Uint16  u16Scib_GraphDataBuff[4][500];
Uint16  u16Scib_TransmitDataBuff[500];

Uint8 u8Scib_HighByte = 0;
Uint8 u8Scib_LowByte = 0;
Uint8 u8Scib_SendHighHalfByte = 1;
/******************************variable definition******************************/

/******************************funciton list******************************/
Uint8 Scib_SnatchGraph(void);
extern volatile Uint8	g_aux_u8Date;
extern volatile float32 g_aux_f32OutputEnergy;
extern volatile float32 g_aux_f32OutputEnergy_new;



extern int16	Test_Time_of_SEQISR;
extern Uint16	MAX_Test_Time_of_SEQISR;
extern Uint16	Test_Start_of_SEQISR;
extern Uint16	Test_End_of_SEQISR;
extern  Uint8   u8Cnt1;
extern  Uint8   u8Cnt2;

void Scib_Parsing(void);
void Scib_Q1Command(void);
void Scib_Q3Command(void);
void Scib_QDCommand(void);
int16 swGetRLineVolt(void);
int16 swGetSLineVolt(void);
int16 swGetTLineVolt(void);
int16 swGetRCurr(void);
int16 swGetFault0(void);
int16 swGetFault1(void);
int16 swGetFault2(void);
int16 swGetFault3(void);
int16 swGetFault4(void);
int16 swGetFault5(void);
int16 swGetFaultUnrecover1(void);
int16 swGetFaultUnrecover2(void);
Uint8 sbNumToAscii(Uint16 u16Number, int8 i8Exponent, Uint8 *pbBuffer);
void Scib_WriteBinary(Uint16 *pstart, Uint16 u16Length);
void Scib_Q1CommandIndex(void);  //2017.3.1 GX

/******************************fuciton list******************************/

pFunc1 GetDataSubArray[30] =
{
	swGetFault0,swGetFault0,swGetFault1,swGetFault2,//3
	swGetFault3,swGetFault4,swGetFault5,swGetFaultUnrecover1,//7
	swGetFaultUnrecover2,swGetRLineVolt,swGetSLineVolt,swGetTLineVolt,//11
	swGetRCurr,swGetStartADIsrPoint,swGetEndCONPoint,swGetEndADIsrPoint
};


/*pFunc1 GetDataSubArray[10] =
{
    swGetRLineVolt,
    swGetRLineVolt, swGetSLineVolt, swGetTLineVolt,
    swGetRCurr,     swGetRLineVolt, swGetSLineVolt,
    swGetTLineVolt, swGetRCurr,     swGetRLineVolt,
} ;*/

/*
pFunc1 GetDataSubArray[9] =
{
    swGetVoltvw,
    swGetVoltvw,swGetVoltuv,swGetThetaOut,
    swGetCurra,swGetCurrb,swGetIDRef,
    swGetIIDFeedback1,swGetIIQFeedback2
} ;*/
/*
pFunc1 GetDataSubArray[30] =
{
	swGetFault0,swGetFault0,swGetFault1,swGetFault2,//3
	swGetFault3,swGetFault4,swGetFault5,swGetFaultUnrecover1,//7
	swGetFaultUnrecover2,swGetRLineVolt,swGetSLineVolt,swGetTLineVolt,//11
	swGetRCurr,swGetStartADIsrPoint,swGetEndCONPoint,swGetEndADIsrPoint

	swGetFault0,Calc_Result.f32DCIR,Calc_Result.f32DCIS,Calc_Result.f32DCIT,//3
//	Calc_Result.f32GFCIave,Calc_Result.f32GFCIrms,Calc_Result.f32GridFreq,//6
//	Calc_Result.f32IGrid_Rave,Calc_Result.f32IGrid_Rrms,Calc_Result.f32IGrid_Save,//9
//	Calc_Result.f32IGrid_Srms,Calc_Result.f32IGrid_Tave,Calc_Result.f32IGrid_Trms,//12
	Calc_Result.f32Input1Watt,Calc_Result.f32Input2Watt,Calc_Result.f32IPV1,//6
	Calc_Result.f32IPV2,Calc_Result.f32OutputWatt,Calc_Result.f32Vbus,//9
	Calc_Result.f32VbusN,Calc_Result.f32VbusP,Calc_Result.f32VGrid_Rave,//12
	Calc_Result.f32VGrid_Rrms,Calc_Result.f32VGrid_RSrms,Calc_Result.f32VGrid_Save,//15
	Calc_Result.f32VGrid_Srms,Calc_Result.f32VGrid_STrms,Calc_Result.f32VGrid_Tave,//18
	Calc_Result.f32VGrid_Trms,Calc_Result.f32VGrid_TRrms,Calc_Result.f32VInv_Rave,//21
//	Calc_Result.f32VInv_Rrms,Calc_Result.f32VInv_RSrms,Calc_Result.f32VInv_Save,//
//	Calc_Result.f32VInv_Srms,Calc_Result.f32VInv_STrms,Calc_Result.f32VInv_Tave,
	Calc_Result.f32VGrid_Trms,Calc_Result.f32VInv_TRrms,Calc_Result.f32Viso1,//24
	Calc_Result.f32Viso2,Calc_Result.f32VPV,Calc_Result.f32VPV1,//27
	Calc_Result.f32VPV2//28

} ;
*/

/*=============================================================================*
 * FUNCTION: TSK_Scib(void)
 * PURPOSE : 100ms Task Schedule For Scib
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     //Scib_TaskInitial();
 *     //SEM_pend();
 *     //Scib_Read();
 *     //Scib_Parsing();
 *
 * CALLED BY: 
 *  //   INT32 DSP Bios
 * 
 *============================================================================*/
void TSK_SCI_B(void)
{   
  
    pScib_CommandIn0 = u8Scib_CommandBuffer0;

    while(1)
    {
        if(SEM_pend(&SEM_TimeBase500ms, SYS_FOREVER) == 1);
        {
		   while(1)
	        {
	            u8Scib_Temp = SciRead(ID_SCIC, pScib_CommandIn0);

				if(SCI_RX_EMPTY == u8Scib_Temp)
				{
					break;
				}
								
				if(u16Scib_CommandLength >= con_MAX_COMMAND_LENGTH)
				{
					pScib_CommandIn0 = u8Scib_CommandBuffer0;
					u16Scib_CommandLength = 0;
					
				}
				else if(0x5A == (*pScib_CommandIn0))
				{
					FlashProgramming_TSK();
				}
				else if((con_CHAR_ENTER == (*pScib_CommandIn0)) || (0x0A == (*pScib_CommandIn0)))  //Deal with command with 0x0a
				{
			        Scib_Parsing();
					pScib_CommandIn0 = u8Scib_CommandBuffer0;
					u16Scib_CommandLength = 0;
				}
				else
				{
					u16Scib_CommandLength++;
					pScib_CommandIn0++;
	          	} 
		    }
        }
    }
}

/*=============================================================================*
 * FUNCTION: Scib_Parsing(void)
 * PURPOSE : Scib Command Parse
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     //Scib_Q1Command();
 *     //Scib_Q3Command();
 *     //Scib_QDCommand();
 *     
 *
 * CALLED BY: 
 *     //INT32 TASK_Scib() 
 * 
 *============================================================================*/                
void Scib_Parsing(void)
{
	static Uint8 Command_Index = 0;
	switch(u8Scib_CommandBuffer0[0])
	{
    case 'Q':
        {   
            if('1' == u8Scib_CommandBuffer0[1])
            {
            	if (Command_Index > 9)      //2017.3.1 GX
            	{
            		Scib_Q1CommandIndex();
            		Command_Index=0;
            	}
            	else
            	{
            		Scib_Q1Command();
            		Command_Index = Command_Index + 1;
            	}
            	//Scib_Q1Command();2017.3.1 GX
            }
            if('3' == u8Scib_CommandBuffer0[1])
            {
                Scib_Q3Command();
            }
            if('D' == u8Scib_CommandBuffer0[1])
            {
                Scib_QDCommand();
            }
        }
        break;

    default:

        break;
	}
	
}
/*=============================================================================*
 * FUNCTION: Scib_Q1Command(void)
 * PURPOSE : Send Data to IPOMS For Q1 Command
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *     //sbNumToAscii(); 
 *     //Scib_Write();
 *     
 *     extern int16 swGetGridVoltRMS_uv(void)
extern int16 swGetGridVoltRMS_vw(void)
extern int16 swGetGridVoltRMS_wu(void)
extern int16 swGetOutputCurrRMS_u(void)
extern int16 swGetOutputCurrRMS_v(void)
extern int16 swGetOutputCurrRMS_w(void)
extern int16 swGetInductionCurrRMS_a(void)
extern int16 swGetInductionCurrRMS_b(void)
extern int16 swGetInductionCurrRMS_c(void)
extern int16 swGetInductionCurrDC_a(void)
extern int16 swGetInductionCurrDC_b(void)
extern int16 swGetInductionCurrDC_c(void)
extern int16 swGetTempHeatsink(void)
extern int16 swGetTempControlborad(void)
extern int16 swGetTempTransformer(void)
extern int16 swGetVoltPV(void)
extern int16 swGetCurrPV(void)
extern int16 swGetInsulationResistorP(void)
extern int16 swGetInsulationResistorN(void)
extern int16 swGetOutputWatt(void)
extern int16 swGetInputWatt(void)
extern int16 swGetGridFreq(void)

 *     
 *
 * CALLED BY: 
 *   //  INT32 Scib_Parsing() 
 * 
 *============================================================================*/
void Scib_Q1Command(void)                
{
	Uint8	bStrLen;
	Uint8	bStrLen1;
	Uint8   *pDataBuf;
	
	bStrLen1 = 0;
	bStrLen = 0;

	
	bStrLen = sbNumToAscii( g_Sys_Current_State, 0, u8Scib_UserDataBuf0); //1   g_Sys_Current_State
	u8Scib_UserDataBuf0[bStrLen++] = 32;	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];

	bStrLen1 = sbNumToAscii(Module_Type, 0, pDataBuf);    //2
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32VGrid_rms, 0, pDataBuf);    //3 SafetyReg.f32InvH_VoltRms_Ref * 10
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii( Calc_Result.f32IGrid_rms_ave * 100, 0, pDataBuf);    //4 SafetyReg.f32InvL_VoltRms_Ref * 10
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32VInvH_rms, 0, pDataBuf);    //5
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32VInvL_rms, 0, pDataBuf);    //6
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
 
    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32IOutH_rms * 100, 0, pDataBuf);    //7
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32IOutL_rms * 100, 0, pDataBuf);    //8
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32VBusP, 0, pDataBuf);    //9
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32; 	

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32VBusN, 0, pDataBuf);    //10
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
    bStrLen1 = sbNumToAscii(SafetyReg.f32InvL_VoltRms_Ref*10 , 0, pDataBuf);//2017.6.7 ZR sample test Calc_Result.f32TempInvH SafetyReg.f32InvL_VoltRms_Ref*10
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(SafetyReg.f32InvH_VoltRms_Ref *10, 0, pDataBuf);//2017.6.7 ZR sample test Calc_Result.f32TempInvL SafetyReg.f32InvH_VoltRms_Ref *10
	//bStrLen1 = sbNumToAscii(Calc_Result.f32TempInvL, 0, pDataBuf);//12
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(SafetyReg.f32InvL_VoltRms_Ref_LCD *10, 0, pDataBuf);//13Calc_Result.f32TempPFCSYNC_COM2_LEVEL
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(SafetyReg.f32InvH_VoltRms_Ref_LCD *10, 0, pDataBuf);//14 Calc_Result.f32VOutHFreq * 100
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32VOutLFreq * 100, 0, pDataBuf);//15
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Calc_Result.f32GridFreq * 100, 0, pDataBuf);//16
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii( Calc_Result.f32VGrid_ave, 0, pDataBuf);//17
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;



/////////////////////////////
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte0, 0, pDataBuf);  //18
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte1, 0, pDataBuf);   //19
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
    
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte2, 0, pDataBuf); //20
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];  
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte3, 0, pDataBuf); //21
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte4, 0, pDataBuf);   //22
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.unrecover1, 0, pDataBuf);    //23
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];  
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.unrecover2, 0, pDataBuf);//24
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.unrecover3, 0, pDataBuf);//25
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;


    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte0, 0, pDataBuf);//26 s_u16Cnt_DcFan1_Hi_Level
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte1, 0, pDataBuf);//27 s_u16Cnt_DcFan1_Low_Level
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte2, 0, pDataBuf);  //28 s_u16Cnt_DcFan2_Hi_Level
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte3, 0, pDataBuf);//29 ADCalibration.f32TempPFC * 100
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte4, 0, pDataBuf);//30 ADCalibration.f32TempPFC * 100
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte5, 0, pDataBuf);//31 ADCalibration.f32TempPFC * 100
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte6, 0, pDataBuf);//32 ADCalibration.f32TempPFC * 100
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_Mod_Status, 0, pDataBuf);//33 s_u16Cnt_DcFan2_Low_Levelg_StateCheck.Word.byte4 g_Mod_Statusg_StateCheck.Word.byte5
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(ModuleAdd, 0, pDataBuf);//34 s_u16Cnt_FanCtrl_Hi_Level g_StateCheck.Word.byte5
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_InvH_Load, 0, pDataBuf);//35 s_u16Cnt_FanCtrl_Hi_Level g_StateCheck.Word.byte5
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_InvL_Load, 0, pDataBuf);//36 s_u16Cnt_FanCtrl_Hi_Level g_StateCheck.Word.byte5
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(GetRealValue.f32VGrid * 10, 0, pDataBuf);//ADCalibration.f32VInvH * 1000 37 g_StateCheck.Word.byte6 s_u16Cnt_FanCtrl_Low_LevelHWADFault   //  swGetOutputCurrRMS_w() s_f32LoadPower_Transformer
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(GeneralADbuffer.f32VGrid, 0, pDataBuf);//ADCalibration.f32VInvL * 100038// 2017.3.7 GX  g_MPPT_State.Word.byte0
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(ADCalibration.f32IGrid * 1000, 0, pDataBuf);//39  g_StateCheck.Word.byte6 s_u16Cnt_FanCtrl_Low_LevelHWADFault   //  swGetOutputCurrRMS_w() s_f32LoadPower_Transformer
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32; 

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(ADCalibration.f32VGrid * 1000, 0, pDataBuf);//40// 2017.3.7 GX  g_MPPT_State.Word.byte0
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(ADCalibration.f32IOutH * 1000, 0, pDataBuf);//41// 2017.3.7 GX  g_MPPT_State.Word.byte0
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(ADCalibration.f32IOutL * 1000, 0, pDataBuf);//42// 2017.3.7 GX  g_MPPT_State.Word.byte0
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysWarningMessage.Word.byte0, 0, pDataBuf);//43// 2017.3.7 GX  g_MPPT_State.Word.byte0
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysWarningMessage.Word.byte1, 0, pDataBuf);//44  //2017.3.3 GX g_MPPT_State.Word.byte1
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_ParaLogic_State.Word.byte0, 0, pDataBuf);//45
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_ParaLogic_State.Word.byte1 , 0, pDataBuf);//46
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(fabs(OutPLLConReg.f32Theta - VOutHPLLConReg.f32Theta)*1000, 0, pDataBuf);//47
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(Parallel_Reg.u16Cnt_COM1_Receive, 0, pDataBuf);//48
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
//********************** 
///////////////////////////////////////////////////////////
	      	
	SciWrite(ID_SCIC, u8Scib_UserDataBuf0, bStrLen);
	//Scib_WriteBinary(u8Scib_UserDataBuf0,bStrLen);

}

//=========================================================================================
//void Scib_Q1CommandIndex(void) 2017.3.1 GX
//=================================================================================
void Scib_Q1CommandIndex(void)          //  进行数据准备   存储到 u8Scib_UserDataBuf0【200】 并发送
{    //  发送数据的长度 是变化的   如 123  就是 ascii 三位   12  就发送ascii 两位
	Uint8	bStrLen;
	Uint8	bStrLen1;
	Uint8   *pDataBuf;
	Uint16   i=0;
	bStrLen1 = 0;
	bStrLen = 0;

    for (i=1;i<=50;i++)
	{
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];     // 新数据段的起始地址
	bStrLen1 = sbNumToAscii(i, 0, pDataBuf);    //2   长度
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
    }

	SciWrite(ID_SCIC, u8Scib_UserDataBuf0, bStrLen);   //  原来是ID_SCIB   ###   CHU   发送数据
	//Scib_WriteBinary(u8Scib_UserDataBuf0,bStrLen);
}



/*=============================================================================*
 * FUNCTION: Scib_Q3Command(void)
 * PURPOSE : Pick up parameter for Snatch Graph fuction
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *    //Scib_Write
 *  
 *     
 *     
 *     
 *
 * CALLED BY: 
 * //    INT32 Scib_Parsing() 
 * 
 *============================================================================*/
void Scib_Q3Command(void)
{
	Uint16 u16SnatchDataCntTemp, u16IntervalTemp;
	Uint8 u8DataKindTemp1, u8DataKindTemp2, u8DataKindTemp3, u8DataKindTemp4;
	Uint8 u8TriggerSourceTemp, u8TriggerTemp, u8SignTemp;
	Uint16 i, u16LengthTemp, u16CompareValTemp;
	
	u16LengthTemp = u16Scib_CommandLength;
	
	for(i = 0;i < u16LengthTemp;i++)
	{
		u8Scib_UserDataBuf0[i] = u8Scib_CommandBuffer0[i];		
	}

	//Scib_Write(u8Scib_UserDataBuf0, u16LengthTemp);
	SciWrite(ID_SCIC, u8Scib_UserDataBuf0, u16LengthTemp);
	
	//aaa
	u16SnatchDataCntTemp = (u8Scib_CommandBuffer0[2] - 48) * 100 + (u8Scib_CommandBuffer0[3] - 48) * 10 + u8Scib_CommandBuffer0[4] - 48;
	//bbb
	u16IntervalTemp = (u8Scib_CommandBuffer0[6] - 48) * 100 + (u8Scib_CommandBuffer0[7] - 48) * 10 + u8Scib_CommandBuffer0[8] - 48;
	//cc
	u8DataKindTemp1 = (u8Scib_CommandBuffer0[10] - 48) * 10 + u8Scib_CommandBuffer0[11] - 48;
	//dd
	u8DataKindTemp2 = (u8Scib_CommandBuffer0[13] - 48) * 10 + u8Scib_CommandBuffer0[14] - 48;
	//ee
	u8DataKindTemp3 = (u8Scib_CommandBuffer0[16] - 48) * 10 + u8Scib_CommandBuffer0[17] - 48;
	//ff
	u8DataKindTemp4 = (u8Scib_CommandBuffer0[19] - 48) * 10 + u8Scib_CommandBuffer0[20] - 48;
	//gg
	u8TriggerSourceTemp = (u8Scib_CommandBuffer0[22] - 48) * 10 + u8Scib_CommandBuffer0[23] - 48;
	//h
	u8TriggerTemp = u8Scib_CommandBuffer0[25] - 48;
	//+/-
	u8SignTemp = u8Scib_CommandBuffer0[27] - 48;
	//iiiii
	u16CompareValTemp = (u8Scib_CommandBuffer0[29] - 48) * 10000 + (u8Scib_CommandBuffer0[30] - 48) * 1000 \
	 + (u8Scib_CommandBuffer0[31] - 48) * 100 + (u8Scib_CommandBuffer0[32] - 48) * 10 + u8Scib_CommandBuffer0[32] - 48;
	
	if((u16SnatchDataCntTemp > 500) ||(u16IntervalTemp > 500) ||(u8TriggerTemp > 4)) 
	    
	{
        return;
	}
	
	//OS_ENTER_CRITICAL();
	u16Scib_SaveDataCnt = 0;
	u16Scib_SnatchDataCnt = u16SnatchDataCntTemp;
	u16Scib_Interval = u16IntervalTemp;
	u16Scib_Interval1 = u16Scib_Interval;
	
	u8Scib_DataKind[0] = u8DataKindTemp1;
	u8Scib_DataKind[1] = u8DataKindTemp2;
	u8Scib_DataKind[2] = u8DataKindTemp3;
	u8Scib_DataKind[3] = u8DataKindTemp4;
	u8Scib_wTriggerSource = u8TriggerSourceTemp;
	u8Scib_wTrigger = u8TriggerTemp;
	u8Scib_Sign = u8SignTemp;
	
	u16Scib_CompareVal = u16CompareValTemp;
	u16Scib_TransmitCnt = 0;
	//OS_EXIT_CRITICAL();
}

 /*=============================================================================*
 * FUNCTION: Scib_QDCommand(void)
 * PURPOSE :  Send Data to IPOMS For QD Command
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *    
 *  //Scib_WriteBinary()
 *     
 *     
 *     
 *
 * CALLED BY: 
 *     //INT32 Scib_Parsing() 
 * 
 *============================================================================*/
void Scib_QDCommand(void)
{
	Uint8 u8Temp;
	int16 i;
	int32 i32CheckSum;
	
	u8Temp = u8Scib_CommandBuffer0[2] - 48;
	
	if((u8Temp >=4 ) || (0 == u16Scib_TransmitCnt))	
	{
	    return;
    }
	u16Scib_TransmitDataBuff[0] = 0x01;		//SOH
	u16Scib_TransmitDataBuff[1] = u16Scib_TransmitCnt;	//length
	
	i32CheckSum =(int32)(0x01 + u16Scib_TransmitCnt);
	for(i = 0;i < u16Scib_TransmitCnt;i++)
	{
		u16Scib_TransmitDataBuff[2 + i] = u16Scib_GraphDataBuff[u8Temp][i];
		i32CheckSum += u16Scib_GraphDataBuff[u8Temp][i];
	}	
	
	//u16Scib_TransmitDataBuff[2+u16Scib_TransmitCnt]=i32CheckSum>>16;
	u16Scib_TransmitDataBuff[2 + u16Scib_TransmitCnt] = (Uint16)(i32CheckSum & 0x0000FFFF);

	Scib_WriteBinary(u16Scib_TransmitDataBuff, u16Scib_TransmitCnt + 3);
	
}
 /*=============================================================================*
 * FUNCTION: Scib_SnatchGraph(void)
 * PURPOSE :  SnatchGraph Data in the switching interrupt
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *    
 *  
 *     
 *     
 *     
 *
 * CALLED BY: 
 *     //INT32 Scib_QDCommand() 
 * 
 *============================================================================*/
Uint8 Scib_SnatchGraph(void)
{
	int16 i,j;
	
	//trigger condition check
	if(0 == u8Scib_SnatchGraphEnable)
	{ 
	  return(false);
	}

	if(0 == u8Scib_wTrigger)
	{
	  return(false);
	}
	if(u16Scib_Interval1 > 0)
	{
		if(0 == (--u16Scib_Interval1))
		{
			u16Scib_Interval1 = u16Scib_Interval;
		}
		else
		{
			return(false);
		}
	}
	
	for(i = 0;i < 4;i++)
	{   
	    j=u8Scib_DataKind[i];
		if(0 == j)
		{		
		    continue;
		}
		u16Scib_GraphDataBuff[i][u16Scib_SaveDataCnt] = (Uint16)GetDataSubArray[j]();
	}
        ++u16Scib_SaveDataCnt;
	if(u16Scib_SaveDataCnt == u16Scib_SnatchDataCnt)
	{
		u16Scib_TransmitCnt = u16Scib_SnatchDataCnt;
		u16Scib_SnatchDataCnt = 0;
		u16Scib_SaveDataCnt = 0;
		u8Scib_wTrigger = 0;
		u8Scib_wTriggerSource = 0;
		u8Scib_SnatchGraphEnable = 1;
		return(true);	
	}
	return(false);
}


int16 swGetFault0(void)
{
  return (GetRealValue.f32VGrid);// 1 GetRealValue.f32IGrid BusCon_Reg.f32BusVoltErr_New * 100 VGrid_Clarke.f32Alpha; PLLConReg.f32Vd;//IGrid_Clarke.f32Alpha    //SinValue
}	


int16 swGetFault1(void)
{

  return (GetRealValue.f32IGrid * 10);//  2 CurrConReg.f32PfcDuty_ffBusCon_Reg.f32BusVoltDiff_Out * 10 CurrConReg.f32PfcDuty_ff VGrid_Clarke.f32Beta; IGrid_Clarke.f32Beta   //SinValueB
}

int16 swGetFault2(void)
{
 
  return (GetRealValue.f32IInvH * 10);// 3 //2017.2.21 GX
}
int16 swGetFault3(void)
{
  return (CurrConReg.f32IGrid_Ref *100);// 4   GridPLLConReg.f32real_Sin_Theta * 400 Calc_Result.f32VBusP GeneralADbuffer.f32IGrid CurrConReg.f32PfcDuty BusCon_Reg.f32IGridAmp_Ref * 100  2017.2.22 GX GetRealValue.f32IGrid//  4 VGrid_Clarke.f32BetaP return(PLLConReg.Cos_Theta * 800); //modified by XXXXX// 4
}

int16 swGetFault4(void)
{
	return (GetRealValue.f32IInvL * 10);//5  GridPLLConReg.f32real_Sin_Theta Calc_Result.f32VBusN * 1GetRealValue.f32VGrid*10CurrConReg.f32PfcDuty GridPLLConReg.Sin_Theta * 400 GX 2017.2.20 GetRealValue.f32IGrid //IGrid_Clarke.f32Beta * 40
}

int16 swGetFault5(void)
{
  return ( GetRealValue.f32VInvL); //6  CurrConReg.f32PfcDuty_Con * 1 GetRealValue.f32VGrid*10//BusCon_Reg.f32IGridAmp_Ref * IGrid_Clarke.f32Beta * 40
}


int16 swGetFaultUnrecover1(void)
{
	return(BusCon_Reg.f32IGridAmp_Ref *10); //7 GetRealValue.f32IGrid_R * 100
}
int16 swGetFaultUnrecover2(void)
{
	return(GetRealValue.f32VGrid); //8   GetRealValue.f32IGrid_S * 100
}
int16 swGetRLineVolt(void)
{
	return(ADChannelOffset.f32VGrid);//9
}
int16 swGetSLineVolt(void)
{
	return(Calc_Result.f32VGrid_ave*10);//10GetRealValue.f32VGrid_RT
}
int16 swGetTLineVolt(void)
{
	return( Ecan_SysParaCalibration.u16VInvH_rms);//11
}
int16 swGetRCurr(void)
{
	return(Ecan_SysParaCalibration.u16VInvL_rms);//12
}

/*=============================================================================*
 * FUNCTION: sbNumToAscii
 * PURPOSE :  Convert input u16Number into max. 8 digital numbers including
 *			  decimal represented byASCII code.
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *     
 *   
 *     
 *
 * CALLED BY: 
 *    
 * Scib_Q1Command   
 *    
 * 
 *============================================================================*/
Uint8 sbNumToAscii(Uint16 u16Number, int8 i8Exponent, Uint8 *pbBuffer)
{
	Uint8 u8No;
	int8 i,j;
    Uint8 bArrayTemp[8] = {0,0,0,0,0,0,0,0};
    Uint32 u32NumberTemp;
	j = 0;

	if((0 == u16Number) || ((u16Number != 0) && (i8Exponent < -6)))
	{
		*pbBuffer = '0';
		u8No = 1;
	}
	else
	{
		u32NumberTemp = (Uint32)u16Number;
		for(i = 0;i < i8Exponent;i++)
		{
			if(u32NumberTemp <= 9999999)
			{
			    u32NumberTemp = u32NumberTemp * 10;
			}
			else 
			{
				u32NumberTemp = 99999999;
			}
		}
		while ((u32NumberTemp > 0) || (i8Exponent < 0))
		{
			i = u32NumberTemp % 10;
			u32NumberTemp = u32NumberTemp / 10;
			bArrayTemp[j] = i + 0x30;
			j++;

			i8Exponent++;
			if(0 == i8Exponent)
			{
				bArrayTemp[j] = '.'; 
				j++;
				if(0 == u32NumberTemp)
				{   
					bArrayTemp[j] = '0'; 
					j++;
				}
			}
		}
		u8No = 0;
		j--;
		while (j >= 0)
		{
			*(pbBuffer + u8No) = bArrayTemp[j];
			u8No++;
			j--;
		}
	}/*	end of number !=0 */
	return(u8No);/* char length*/	   
}

/*=============================================================================*
 * FUNCTION: Scib_WriteBinary
 * PURPOSE :  Write a 16bit data to  Scib Tx Port
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *    // void sSplit()
 *     
 *   
 *     
 *
 * CALLED BY: 
 *     Scib_QDCommand()
 *    
 *    
 * 
 *============================================================================*/
void Scib_WriteBinary(Uint16 *pstart, Uint16 u16Length)
{
    int8 i;
	int8 j;
	Uint16 *pData;
	Uint8  u8Data;
	
	u8Scib_SendHighHalfByte = 1;	
    pData = pstart;
	for(i = 0;i < u16Length;i++)
	{
	    //split
        //sSplit(*pData);

	    for(j = 0;j < 2;j++)
	    {   
	        if(1 == u8Scib_SendHighHalfByte)
            {   //write high 8bit
				u8Data = GET_HBYTE_OF_WORD(*pData);
				while(SciWrite(ID_SCIC, &u8Data, 1));
                u8Scib_SendHighHalfByte = 0;    
            }
            else
	        {   //write low 8bit
	            u8Data = GET_LBYTE_OF_WORD(*pData);
				while(SciWrite(ID_SCIC, &u8Data, 1));
                u8Scib_SendHighHalfByte = 1; 
     	    }
	    }

        ++pData;
    }
}
