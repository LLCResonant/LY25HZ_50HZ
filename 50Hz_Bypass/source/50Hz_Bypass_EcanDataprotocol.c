/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_Scib_Interface.c
 *
 *  PURPOSE  : SCIb for IPOMS (Virtual PCOsci by Ken)
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-19      V0.1           Ken      	    Created
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *
 *
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *
 *   Scib_SnatchGraph(void);
 *============================================================================*/

#include "DSP2803x_Device.h"	// Peripheral address definitions
#include "F28035_example.h"		// Main include file

/******************************variable definition******************************/
MAIL  u8ECAN_UserDataBuf0[64];  						//64个待发送的缓冲数据
MAIL  u8ECAN_CommandBuffer0;							//单个指令出队后的中转变量
MAIL  *pECAN_CommandIn0;								//单个指令出队后中转变量的指针
Uint16  u8ECAN_Temp = 0;								//用于检测指令队列中是否有 数据

Uint8 Error_Flag = 0;
Uint32 IDTEST = 0;
Uint8 ModuleAdd = 0;

ECAN_ORDER Ecan_SysParaCalibration;
ECAN_MODULE_DATA Ecan_ModuleData;
ECAN_REVISED Ecan_SytemREVISED;
P2AMAIL EcanP2A_Tx;
P2AMAIL EcanP2A_Rx;
enum MODULE_STATUS g_Mod_Status = idle;

Uint8 u8_modaddr[8] = {0,0,0,0,0,0,0,0};

/******************************function definition******************************/
void ECAN_COMM_TEST( void );
void ECAN_Parsing(MAIL mailp);
void ECAN_Response(Uint8 Error, Uint8 SAddress);
Uint32 Get_ID(Uint8 DataLength, Uint8 SAddress);
void DataUpload();
void Para_Revised(MAIL mailp);
void Sys_Set(MAIL mailp);
Uint8 XOR(MAIL temp);
void TSK_ECAN(void);
void Data_Format_Conver(void);
void Sys_Set_Oper(void);
void Para_Revise_Oper(Uint8);
Uint8 ErrorCheck(MAIL temp);
void Output_Revise (void);
void Sample_WriteToEEPROM();

void ECAN_COMM_TEST( void )
{
	MAIL	test;
	test.Mailbox_id.all=0x860F8007;
	test.Mailbox_data.DWord.CANL_Bytes=0x00000000;
	test.Mailbox_data.DWord.CANH_Bytes=0x00000000;
	eCAN_Transmit(test);
}
/*=============================================================================*
 * FUNCTION: TSK_ECAN(void)
 * PURPOSE :
 *============================================================================*/
void TSK_ECAN(void)
{
	 //__asm (" ESTOP0");
	static Uint8 b_count = 0;

	while(1)
	{
		//__asm (" ESTOP0");
		if (SEM_pend(&SEM_TimeBase500ms, SYS_FOREVER) == TRUE )
		{
			//__asm (" ESTOP0");

			pECAN_CommandIn0 = &u8ECAN_CommandBuffer0;	        // eCAN buffer 的起始地址 。 只执行一次。修改到main函数中了
			while(1)
			{
				u8ECAN_Temp = ECANRead(pECAN_CommandIn0); 		//读取指令,将指令调到u8ECAN_CommandBuffer0指令队列中
				if(ECAN_RX_EMPTY == u8ECAN_Temp)
				{
					break;
				}																						// 如读取的指令队列空  就会返回  等待下一次发送调度
				else
				{
					ECAN_Parsing(u8ECAN_CommandBuffer0);
					break;
				}
			}
			b_count ++;
			if (b_count >= 1)
			{
				if (b_count >= 2)
				{
					DataUpload();
					b_count = 0;
				}
			}
			if(ECanaRegs.CANGIF0.bit.BOIF0 == 1)
			{
				InitECanGpio();                     // Initialize the ECan GPIO
				InitECan();							// Initialize the ECan
				InitECana();						// Initialize the ECana
			}
		}
	}
}

/*=============================================================================*
 * FUNCTION: ECAN_Parsing(MAIL mailp)
 * PURPOSE :
 *============================================================================*/
void ECAN_Parsing(MAIL mailp)
{
	Uint8	Order_Code;		//指令信息
	Uint8	Error_Code;		//错误信息
	Uint8 	temp;

	temp = mailp.Mailbox_data.Byte.Code;
	Order_Code = temp & 0x0F;

	Error_Code = ErrorCheck(mailp);

	if (0) //!= Error_Code)
		;//ECAN_Response(Error_Code, ModuleAdd);//Error_Code
	else
	{
		switch(Order_Code)
		{
		case DATA_UPLOAD:
			DataUpload();
			ECAN_Response(DATA_NORMAL, ModuleAdd);//DATA_NORMAL
			break;
		case SINGLEDATA_REVISED:
			Para_Revised(mailp);
			ECAN_Response(DATA_NORMAL, ModuleAdd);
			break;
		case MULTIDATA_REVISED:
			Para_Revised(mailp);
			ECAN_Response(DATA_NORMAL, ModuleAdd);
			break;
		case SINGLEDATA_SET:
			Sys_Set(mailp);
			ECAN_Response(DATA_NORMAL, ModuleAdd);
			break;
		case MULTIDATA_SET:
			Sys_Set(mailp);
			ECAN_Response(DATA_NORMAL, ModuleAdd);
			break;
		case SINGLEDATA_CONFIG:
			ECAN_Response(DATA_NORMAL, ModuleAdd);
			break;
		case MULTIDATA_CONFIG:
			ECAN_Response(DATA_NORMAL, ModuleAdd);
			break;
		default:
			break;
		}
	}
}

/*=============================================================================*
 * FUNCTION: 	DataUpload(MAIL mailp)
 * PURPOSE : 	模块信息上传
 *============================================================================*/
void DataUpload(MAIL mailp)
{
	Uint8	bStrLen = 0;
	Uint8   DataLength = 0x06;
	Uint32  id = 0;
	Uint8   check = 0x00;
	MAIL	Temp;

	Data_Format_Conver();

	id = Get_ID(DataLength, ModuleAdd);
	Temp.Mailbox_id.all = id;
	Temp.Mailbox_data.Byte.Current_frame = 0x01;//第一帧
	Temp.Mailbox_data.Byte.Frames_num = 0x07;//3帧
	Temp.Mailbox_data.Byte.Code = 0x00;

	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16VGrid_rms;
	Temp.Mailbox_data.Word.Data2 = Ecan_ModuleData.u16VGrid_Freq;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16IGrid_rms;
	check = XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x02;//第二帧

	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16VOut_rms;
	Temp.Mailbox_data.Word.Data2 = Ecan_ModuleData.u16Temperature;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16RunTimeu16Hour_L;
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	DataLength = 0x06;  //校验字节也是有效字节
	id = Get_ID(DataLength, ModuleAdd);
	Temp.Mailbox_id.all = id;
	Temp.Mailbox_data.Byte.Current_frame = 0x03;//the third frame
	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16RunTimeu16Hour_H;
	Temp.Mailbox_data.Byte.byte5 = Ecan_ModuleData.Alert.Byte.Alert;
	Temp.Mailbox_data.Byte.byte6 = Ecan_ModuleData.Fault.Byte.Fault1;
	Temp.Mailbox_data.Byte.byte7 = Ecan_ModuleData.Fault.Byte.Fault2;

	check ^= XOR(Temp);
	Temp.Mailbox_data.Byte.byte8 = check;

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	ECANWrite(u8ECAN_UserDataBuf0, bStrLen);
}

/*=============================================================================*
 * FUNCTION: 	Para_Revised(MAIL mailp)
 * PURPOSE : 	从ECAN通讯信息中提取数据
 *============================================================================*/
void Para_Revised(MAIL mailp)
{
	Uint8 Revised_Num = 0;

	Revised_Num = mailp.Mailbox_data.Byte.Current_frame;
	if (Revised_Num > mailp.Mailbox_data.Byte.Frames_num)
	{
		Error_Flag = 0x01;
		return;
	}
	switch (Revised_Num)
	{
	case 1:
		Ecan_SytemREVISED.u16VGrid_rms = mailp.Mailbox_data.Word.Data1;
		Ecan_SytemREVISED.u16IGrid_rms = mailp.Mailbox_data.Word.Data2;
		Ecan_SytemREVISED.u16VOut_rms = mailp.Mailbox_data.Word.Data3;
		break;
	case 2:
		Ecan_SytemREVISED.u16Temperature = mailp.Mailbox_data.Word.Data1;
		break;
	default:
		break;
	}
	Para_Revise_Oper(Revised_Num);
}

/*=============================================================================*
 * FUNCTION:  Sys_Set(MAIL mailp)
 * PURPOSE :    区分系统命令分类
 *============================================================================*/
void Sys_Set(MAIL mailp)
{
	Uint8 SysSet_Num = 0;
	SysSet_Num = mailp.Mailbox_data.Byte.Current_frame;

	if (SysSet_Num > mailp.Mailbox_data.Byte.Frames_num)
	{
		Error_Flag = 0x02;
		return;
	}
	switch (SysSet_Num)
	{
	case 1:
		Ecan_SysParaCalibration.Defaluts = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.Output_Enable = mailp.Mailbox_data.Word.Data3;
		break;
	case 2:
		Ecan_SysParaCalibration.Defaluts = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.Output_Enable = mailp.Mailbox_data.Word.Data3;
		break;
	default:
		break;
	}
	Sys_Set_Oper();
}

/*=============================================================================*
 * FUNCTION: 	ECAN_Response(Uint8 Error_Code, Uint8 SAddress)
 * PURPOSE : 	统一应答帧函数
 *============================================================================*/
void ECAN_Response(Uint8 Error_Code, Uint8 SAddress)
{
	Uint8 bStrLen = 0;
	Uint32 id = 0;
	Uint8 DataLength = 0x00;
	MAIL Temp;

	id = Get_ID(DataLength, SAddress);
	Temp.Mailbox_id.all = id;

	Temp.Mailbox_data.Byte.Current_frame = 0x01;//单帧填1
	Temp.Mailbox_data.Byte.Frames_num = 0x01;//单帧填1
	Temp.Mailbox_data.Byte.Code = Error_Code | DATA_UPLOAD;  //命令码为0，错误码来自外部


	Temp.Mailbox_data.Word.Data1 = 0x00;
	Temp.Mailbox_data.DWord.CANH_Bytes = 0x00;

	__asm ("ESTOP0");
	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	ECANWrite(u8ECAN_UserDataBuf0, bStrLen);   //  回送命令
}

/*=============================================================================*
 * FUNCTION: 	Get_ID (Uint8 DataLength, Uint8 SAddress)
 * PURPOSE : 	获取ECAN通讯的ID位
 *============================================================================*/
Uint32 Get_ID (Uint8 DataLength, Uint8 SAddress)
{

	Uint32 ID = 0;
	Uint32 u32_Add = 0;
	Uint32 u32_ModuleType = 0;
	Uint32 u32_ComAdd = 0;
	Uint32 u32_ComType = 0;

	u32_Add = SAddress;
	u32_ModuleType = MODULE_TYPE;
	u32_ComAdd = Computer_ADDRESS;
	u32_ComType = Computer_TYPE;

	ID = DataLength & 0x0000007;
	ID |= (u32_Add << 3) & 0x000001F8;
	ID |= (u32_ModuleType << 9) & 0x0000FE00;
	ID |= (u32_ComAdd << 16) & 0x003F0000;
	ID |= (u32_ComType << 22) & 0x1FC00000;
	ID |= 0XC0000000;

	IDTEST = ID;
	return ID;
}

/*=============================================================================*
 * FUNCTION: 	XOR(MAIL temp)
 * PURPOSE : 	校验位产生
 *============================================================================*/
Uint8 XOR(MAIL temp)
{
	Uint8 check = 0;
	Uint8 length = 0;

	if (temp.Mailbox_data.Byte.Current_frame == temp.Mailbox_data.Byte.Frames_num)//说明该帧还有校验字节
		length = temp.Mailbox_id.bit.byte_num - 1;
	else
		length = temp.Mailbox_id.bit.byte_num;
	if (length >= 1)
		check ^= temp.Mailbox_data.Byte.byte3;
	if (length >= 2)
		check ^= temp.Mailbox_data.Byte.byte4;
	if (length >= 3)
		check ^= temp.Mailbox_data.Byte.byte5;
	if (length >= 4)
		check ^= temp.Mailbox_data.Byte.byte6;
	if (length >= 5)
		check ^= temp.Mailbox_data.Byte.byte7;
	if (length == 6)
		check ^= temp.Mailbox_data.Byte.byte8;

	return check;
}

/*=============================================================================*
 * FUNCTION: 	Data_Format_Conver()
 * PURPOSE : 	上传数据格式转换
 *============================================================================*/
void Data_Format_Conver()
{

	Ecan_ModuleData.u16VGrid_rms = _IQint(Calc_Result.iq20VGrid_RMS * 100);
	Ecan_ModuleData.u16VGrid_Freq = _IQint(Calc_Result.iq20GridFreq * 10);
	Ecan_ModuleData.u16IGrid_rms = 0;

	Ecan_ModuleData.u16VOut_rms = _IQint(Calc_Result.iq20VOut_RMS * 100);
    Ecan_ModuleData.u16Temperature = _IQint(Calc_Result.iq20TempAmb * 10);
    Ecan_ModuleData.u16RunTimeu16Hour_L = RunningTime.u16Hour_L;

    Ecan_ModuleData.u16RunTimeu16Hour_H = RunningTime.u16Hour_H;
    //故障
    //温度超限告警A04
    Ecan_ModuleData.Alert.bit.OverTemp = g_SysWarningMessage.bit.OverTemp;
    //风扇1异常A05
    Ecan_ModuleData.Alert.bit.Fan1Block = g_SysWarningMessage.bit.Fan1Block;
    //风扇2异常A06
    Ecan_ModuleData.Alert.bit.Fan2Block = g_SysWarningMessage.bit.Fan2Block;

    Ecan_ModuleData.Alert.bit.Fan_Fault = g_SysWarningMessage.bit.Fan_Fault;



    //错误
    //输入欠压F01
    Ecan_ModuleData.Fault.bit.VGridUnderRating = g_SysFaultMessage.bit.VGridUnderRating;
    //输入过压F02
    Ecan_ModuleData.Fault.bit.VGridOverRating = g_SysFaultMessage.bit.VGridOverRating;

    //启动异常F07
	 Ecan_ModuleData.Fault.bit.Launch_Fault = g_SysFaultMessage.bit.HWADFault_VGrid | \
			 	 	 	 	 	 	 	 	 g_SysFaultMessage.bit.HWADFault_VOut;
    //输入频率异常F11
    Ecan_ModuleData.Fault.bit.FreGridFault = g_SysFaultMessage.bit.FreGridOverRating | \
    										   g_SysFaultMessage.bit.FreGridUnderRating;
    //过温关机F05
    Ecan_ModuleData.Fault.bit.OverTempFault = g_SysFaultMessage.bit.OverTempFault;

}

/*=============================================================================*
 * FUNCTION: 	Data_Format_Conver()
 * PURPOSE : 	参数校正执行程序
 *============================================================================*/
void Para_Revise_Oper(Uint8 temp)
{
	Uint8 Revised_Num;
	_iq20 iq20temp1 = 0;
	Revised_Num = temp;

	switch (Revised_Num)
	{
	case 1:
		if (Ecan_SytemREVISED.u16VGrid_rms > 800 && Ecan_SytemREVISED.u16VGrid_rms < 1200)
		{
			iq20temp1 = _IQmpy(ADCalibration.iq20VGrid, _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16VGrid_rms));
			if (iq20temp1 > _IQ(0.8f) && iq20temp1 < _IQ(0.8f))
				ADCalibration.iq20VGrid = _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16VGrid_rms);
		}

		if (Ecan_SytemREVISED.u16VOut_rms > 800 && Ecan_SytemREVISED.u16VOut_rms < 1200)
		{
			iq20temp1 = _IQmpy(ADCalibration.iq20VOut, _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16VOut_rms));
			if (iq20temp1 > _IQ(0.8f) && iq20temp1 < _IQ(0.8f))
				ADCalibration.iq20VOut = _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16VOut_rms);
		}

		if (Ecan_SytemREVISED.u16Temperature > 800 && Ecan_SytemREVISED.u16Temperature < 1200)
		{
			iq20temp1 = _IQmpy(ADCalibration.iq20TempAmb, _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16Temperature));
			if (iq20temp1 > _IQ(0.8f) && iq20temp1 < _IQ(0.8f))
				ADCalibration.iq20TempAmb= _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16Temperature);
		}

		break;
	case 2:
		if (Ecan_SytemREVISED.u16IGrid_rms > 800 && Ecan_SytemREVISED.u16IGrid_rms < 1200)
		{
			iq20temp1 = _IQmpy(ADCalibration.iq20IGrid, _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16IGrid_rms));
			if (iq20temp1 > _IQ(0.8f) && iq20temp1 < _IQ(0.8f))
				ADCalibration.iq20IGrid= _IQ20mpyI32(_IQ(0.001f), Ecan_SytemREVISED.u16IGrid_rms);
		}
		break;
	default:
		break;
	}
	Sample_WriteToEEPROM();
}

/*=============================================================================*
 * FUNCTION: 	void Sys_Set_Oper()
 * PURPOSE : 	上位机命令执行程序
 *============================================================================*/
void Sys_Set_Oper()
{
	if (0 == Ecan_SysParaCalibration.Defaluts)
	{
		ADChannelOffset.iq20IGrid = 1.0f;
		ADChannelOffset.iq20VGrid = 1.0f;
		ADChannelOffset.iq20VOut = 1.0f;
		ADChannelOffset.iq20TempAmb= 1.0f;
	};
	if (0x0055 == Ecan_SysParaCalibration.Output_Enable)
	{
		;
	}
	else if (0x00aa == Ecan_SysParaCalibration.Output_Enable)
	{
		;
	}
}

/*=============================================================================*
 * FUNCTION: 	Uint8 ErrorCheck(MAIL temp)
 * PURPOSE : 	Check error
 *============================================================================*/
Uint8 ErrorCheck(MAIL temp)
{
	Uint8 temp1;

	temp1 = XOR(temp);
	if (temp.Mailbox_data.Byte.byte7 != temp1)
		return CHECK_ERROR;
	else
		return DATA_NORMAL;
}
/*--------end of the file----------------*/
