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
#include "DSP2833x_Device.h"	// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file

//#pragma DATA_SECTION(u8ECAN_UserDataBuf0, "ConstantsInRAM")
//#pragma DATA_SECTION(u8ECAN_RxBuffer0, "ConstantsInRAM")
//#pragma DATA_SECTION(u16ECAN_TransmitDataBuff, "ConstantsInRAM")

/******************************variable definition******************************/
//#pragma DATA_SECTION(u8ECAN_UserDataBuf0,"SLOWDATA");
MAIL  u8ECAN_UserDataBuf0[64];  						//64个待发送的缓冲数据
MAIL  u8ECAN_CommandBuffer0;							//单个指令出队后的中转变量
MAIL  *pECAN_CommandIn0;								//单个指令出队后中转变量的指针
Uint16  u8ECAN_Temp = 0;								//用于检测指令队列中是否有 数据
Uint8 g_InvH_Load = 0;
Uint8 g_InvL_Load = 0;

Uint8 ModuleAdd = 0X00;
Uint8 Error_Flag = 0;
Uint32 IDTEST = 0;
Uint8  u8_hostdrop = 0;

ECAN_ORDER Ecan_SytemOrder;
ECAN_MODULE_DATA Ecan_ModuleData;
ECAN_REVISED Ecan_SysParaCalibration;
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
void Broadcast(void);
Uint8 ErrorCheck(MAIL temp);
void Arbitrate(P2AMAIL);
void Output_Revise (void);
void Coeff_Feedback();


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
		if (SEM_pend(&SEM_ECAN, SYS_FOREVER) == TRUE )
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
				if (ModuleAdd != 0X00 && g_Sys_Current_State != FaultState  && g_Sys_Current_State != PermanentState)
					Broadcast();
				if (b_count >= 2)
				{
					DataUpload();
					b_count = 0;
				}
			}
			if(ECanaRegs.CANGIF0.bit.BOIF0 == 1)
			{
				InitECanGpio();                     // Initialize the ECan GPIO.  2017.10.26 GX
				InitECan();							// Initialize the ECan.  2017.10.26 GX
				InitECana();						// Initialize the ECana.  2017.10.26 GX
			}
		}
	}
}

void Broadcast(void)
{
	EcanP2A_Tx.P2AMail_id.all = 0XDFFF0007;
	EcanP2A_Tx.P2AMail_id.bit.source_type = Module_Type;
	EcanP2A_Tx.P2AMail_id.bit.source_address = ModuleAdd;
	if (g_Mod_Status == Master)
	{
		EcanP2A_Tx.P2AMail_id.bit.bus = 0x0;
 		if ( SafetyReg.u16Short_Restart_times == 0 && 0 == SYNC_COM2_LEVEL && NormalState == g_Sys_Current_State)//  //2018.4.3 GX
 		{
 			Output_Revise(); //2018.4.3 GX
 			EcanP2A_Tx.P2AMail_data.Word.Word1= (Uint16)(SafetyReg.f32InvH_VoltRms_Ref_LCD * 10);
 			EcanP2A_Tx.P2AMail_data.Word.Word2 = (Uint16)(SafetyReg.f32InvL_VoltRms_Ref_LCD * 10);
 			EcanP2A_Tx.P2AMail_data.Word.Word3 = (Uint16)(g_InvH_Load);
 			EcanP2A_Tx.P2AMail_data.Word.Word4 = (Uint16)(g_InvL_Load);

 			eCAN_Broadcast(EcanP2A_Tx);   //  回送命令
 		}
 		else
 		{
 			EcanP2A_Tx.P2AMail_data.DWord.CANL_Bytes = ModuleAdd;
 			EcanP2A_Tx.P2AMail_data.DWord.CANH_Bytes = Module_Type;
 			eCAN_Broadcast(EcanP2A_Tx);
 		}
	}
	else if (g_Mod_Status == Slave)
		EcanP2A_Tx.P2AMail_id.bit.bus = 0x1;
	else
	{
		EcanP2A_Tx.P2AMail_id.bit.bus = 0x1;
		EcanP2A_Tx.P2AMail_data.DWord.CANL_Bytes = ModuleAdd;
		EcanP2A_Tx.P2AMail_data.DWord.CANH_Bytes = Module_Type;
		//__asm ("ESTOP0");
		eCAN_Broadcast(EcanP2A_Tx);   //  回送命令
	}
}


void Arbitrate(P2AMAIL P2A_RX)
{
	if ( ModuleAdd != 0X00 && g_Sys_Current_State != FaultState  && g_Sys_Current_State != PermanentState )  //	COM2 High Voltage
	{
		if (g_Mod_Status == idle)
		{
			if (P2A_RX.P2AMail_id.bit.bus == 0)
				g_Mod_Status = Slave;
			else if (ModuleAdd > P2A_RX.P2AMail_id.bit.source_address)
				g_Mod_Status = Slave;
			else
				g_Mod_Status = idle;
		}
		else if (g_Mod_Status == Slave)
		{
			if ( ( SafetyReg.u16Short_Restart_times == 0 ) && 0 == SYNC_COM2_LEVEL&& (NormalState == g_Sys_Current_State )  &&  P2A_RX.P2AMail_id.bit.bus == 0  )//
			{
				if(P2A_RX.P2AMail_data.Word.Word1 > 2110 && P2A_RX.P2AMail_data.Word.Word1 < 2290 \
					&& 	P2A_RX.P2AMail_data.Word.Word2> 1010 && P2A_RX.P2AMail_data.Word.Word2 < 1190)
				{
					SafetyReg.f32InvH_VoltRms_Ref_LCD = P2A_RX.P2AMail_data.Word.Word1 * 0.1;
					SafetyReg.f32InvL_VoltRms_Ref_LCD = P2A_RX.P2AMail_data.Word.Word2 * 0.1;
					g_InvH_Load = (Uint8)(P2A_RX.P2AMail_data.Word.Word3);
					g_InvL_Load = (Uint8)(P2A_RX.P2AMail_data.Word.Word4);

					if(g_InvH_Load == LightLoad)
						SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvLightRevise;
					else if (g_InvH_Load == MiddleLoad)
						SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvMiddleRevise;
					else if (g_InvH_Load == HeavyLoad)
						SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvHeavyRevise;
					else
						SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref;

					if(g_InvL_Load == LightLoad)
						SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvLightRevise;
					else if (g_InvL_Load == MiddleLoad)
						SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvMiddleRevise;
					else if (g_InvL_Load == HeavyLoad)
						SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvHeavyRevise;
					else
						SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref;
				}
				else
				{
					SafetyReg.f32InvL_VoltRms_Ref_LCD = SafetyReg.f32InvL_VoltRms_Ref_LCD;
					SafetyReg.f32InvH_VoltRms_Ref_LCD = SafetyReg.f32InvH_VoltRms_Ref_LCD;
				}
			}
		}
	}
	else
		g_Mod_Status = idle;

	if (u8_hostdrop >= 100)
		u8_hostdrop = 0;
	else
		u8_hostdrop++;
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
		case COEFF_FEEDBACK:
			Coeff_Feedback();
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
	Temp.Mailbox_data.Byte.Frames_num = 0x07;//7帧
	Temp.Mailbox_data.Byte.Code = 0x00;

	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16VGrid_rms;
	Temp.Mailbox_data.Word.Data2 = Ecan_ModuleData.u16IGrid_rms;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16VBusP;
	check = XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x02;//第二帧

	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16VBusN;
	Temp.Mailbox_data.Word.Data2 = Ecan_ModuleData.u16VOutH_rms;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16VOutL_rms;
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x03;//第三帧

	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16VInvH_rms;
	Temp.Mailbox_data.Word.Data2 = Ecan_ModuleData.u16VInvL_rms;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16IInvH_rms;
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x04;//第四帧

	Temp.Mailbox_data.Word.Data1 = Ecan_ModuleData.u16IInvL_rms;
	Temp.Mailbox_data.Word.Data2 = Ecan_ModuleData.u16Temp_PFC;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16Temp_InvH;
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x05;//第五帧

	Temp.Mailbox_data.Word.Data1= Ecan_ModuleData.u16Temp_InvL;
	Temp.Mailbox_data.Word.Data2= Ecan_ModuleData.u16VOutH_Freq;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16VOutL_Freq;
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x06;//the sixth frame

	Temp.Mailbox_data.Word.Data1= Ecan_ModuleData.u16Phase_Diff;
	Temp.Mailbox_data.Word.Data2= Ecan_ModuleData.u16RunTimeHour_L;
	Temp.Mailbox_data.Word.Data3 = Ecan_ModuleData.u16RunTimeHour_H;
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	DataLength = 0x05;  //校验字节也是有效字节
	id = Get_ID(DataLength, ModuleAdd);
	Temp.Mailbox_id.all = id;
	Temp.Mailbox_data.Byte.Current_frame = 0x07;//the seventh frame
	Temp.Mailbox_data.Byte.byte3 = Ecan_ModuleData.Alert.Byte.Alert;
	Temp.Mailbox_data.Byte.byte4 = Ecan_ModuleData.Fault.Byte.Fault1;
	Temp.Mailbox_data.Byte.byte5 = Ecan_ModuleData.Fault.Byte.Fault2;
	Temp.Mailbox_data.Byte.byte6 = Ecan_ModuleData.Fault.Byte.Fault3;

	check ^= XOR(Temp);
	Temp.Mailbox_data.Byte.byte7 = check;
	Temp.Mailbox_data.Byte.byte8 = 0x00;

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
		Ecan_SysParaCalibration.u16VGrid_rms = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16VBusP = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.u16VBusN = mailp.Mailbox_data.Word.Data3;
		break;
	case 2:
		Ecan_SysParaCalibration.u16VInvH_rms = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16VInvL_rms = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.u16IInvH_rms = mailp.Mailbox_data.Word.Data3;
		break;
	case 3:
		Ecan_SysParaCalibration.u16IInvL_rms = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16VOutH_rms = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.u16VOutL_rms = mailp.Mailbox_data.Word.Data3;
		break;
	case 4:
		Ecan_SysParaCalibration.u16Temp_PFC = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16Temp_InvH = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.u16Temp_InvL = mailp.Mailbox_data.Word.Data3;
		break;
	case 5:
		Ecan_SysParaCalibration.u16IGrid_rms = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16InvH_VoltRms_Ref = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.u16InvL_VoltRms_Ref = mailp.Mailbox_data.Word.Data3;
	case 6:
		Ecan_SysParaCalibration.u16VInvH_Comp_Coeff = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16VInvL_Comp_Coeff = mailp.Mailbox_data.Word.Data2;
		Ecan_SysParaCalibration.u16IInvH_para_ave = mailp.Mailbox_data.Word.Data3;
	case 7:
		Ecan_SysParaCalibration.u16IInvL_para_ave = mailp.Mailbox_data.Word.Data1;
		Ecan_SysParaCalibration.u16RestartTimes = mailp.Mailbox_data.Word.Data2;
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
		Ecan_SytemOrder.Defaluts = mailp.Mailbox_data.Word.Data2;
		Ecan_SytemOrder.Output_Enable = mailp.Mailbox_data.Word.Data3;
		break;
	case 2:
		Ecan_SytemOrder.Defaluts = mailp.Mailbox_data.Word.Data2;
		Ecan_SytemOrder.Output_Enable = mailp.Mailbox_data.Word.Data3;
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
	u32_ModuleType = Module_Type;
	u32_ComAdd = Computer_Address;
	u32_ComType = Computer_Type;

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
		length = temp.Mailbox_id.bit.byte_num - 1;  //2017.11.21 GX
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

	Ecan_ModuleData.u16VGrid_rms = (Uint16)(Calc_Result.f32VGrid_rms * 100);
	Ecan_ModuleData.u16IGrid_rms = (Uint16)(Calc_Result.f32IGrid_rms_ave* 100);
	Ecan_ModuleData.u16VBusP = (Uint16)(Calc_Result.f32VBusP * 10);

	Ecan_ModuleData.u16VBusN = (Uint16)(Calc_Result.f32VBusN * 10);
	Ecan_ModuleData.u16VOutH_rms = (Uint16)(Calc_Result.f32VOutH_rms * 100);
	Ecan_ModuleData.u16VOutL_rms = (Uint16)(Calc_Result.f32VOutL_rms * 100);

	Ecan_ModuleData.u16VInvH_rms = (Uint16)(Calc_Result.f32VInvH_rms * 100);
	Ecan_ModuleData.u16VInvL_rms = (Uint16)(Calc_Result.f32VInvL_rms * 100);
	Ecan_ModuleData.u16IInvH_rms = (Uint16)(Calc_Result.f32IOutH_rms * 100);

	Ecan_ModuleData.u16IInvL_rms = (Uint16)(Calc_Result.f32IOutL_rms * 100);
    Ecan_ModuleData.u16Temp_PFC = (Uint16)(Calc_Result.f32TempPFC * 10);
    Ecan_ModuleData.u16Temp_InvH = (Uint16)(Calc_Result.f32TempInvH * 10);

    Ecan_ModuleData.u16Temp_InvL = (Uint16)(Calc_Result.f32TempInvL * 10);
    Ecan_ModuleData.u16VOutH_Freq = (Uint16)(Calc_Result.f32VOutHFreq * 10);
    Ecan_ModuleData.u16VOutL_Freq = (Uint16)(Calc_Result.f32VOutLFreq * 10);

    Ecan_ModuleData.u16Phase_Diff = (Uint16)(Calc_Result.f32Phase_Diff_ave * 10);
    Ecan_ModuleData.u16RunTimeHour_L = RunningTime.Hour_L;
    Ecan_ModuleData.u16RunTimeHour_H = RunningTime.Hour_H;
    //告警
    //输出欠压 A03
    Ecan_ModuleData.Alert.bit.VInvUnderRating = g_SysWarningMessage.bit.VInvHUnderRating | \
    											 g_SysWarningMessage.bit.VInvLUnderRating;
    //温度超限告警A04
    Ecan_ModuleData.Alert.bit.OverTemp = g_SysWarningMessage.bit.OverTemp;
    //风扇1异常A05
    Ecan_ModuleData.Alert.bit.Fan1Block = g_SysWarningMessage.bit.Fan1Block;
    //风扇2异常A06
    Ecan_ModuleData.Alert.bit.Fan2Block = g_SysWarningMessage.bit.Fan2Block;
    //ECAN故障告警A10
    Ecan_ModuleData.Alert.bit.Ecan_Fault = g_SysWarningMessage.bit.ECAN_Fault;

    //输入欠压输出降额保护A12
    Ecan_ModuleData.Alert.bit.InvOverLoad = g_SysWarningMessage.bit.InvH_OverLoad |  \
    		g_SysWarningMessage.bit.InvL_OverLoad;

    //逆变器不同步A13
    Ecan_ModuleData.Alert.bit.InvAsyn = g_SysWarningMessage.bit.InvAsyn;

    //故障
    //输入欠压F01
    Ecan_ModuleData.Fault.bit.VGridUnderRating = g_SysFaultMessage.bit.VGridUnderRating;
    //输入过压F02
    Ecan_ModuleData.Fault.bit.VGridOverRating = g_SysFaultMessage.bit.VGridOverRating;

    //PFC可恢复故障F03
    Ecan_ModuleData.Fault.bit.PFC_Recover_Fault = g_SysFaultMessage.bit.recoverSW_Bus_UVP | \
    										g_SysFaultMessage.bit.OCP_AC_RMS | \
    										g_SysFaultMessage.bit.BusVoltUnbalanceFault;

    //轨道输出过压F04-1
    Ecan_ModuleData.Fault.bit.InvH_OVP = g_SysFaultMessage.bit.unrecoverSW_InvH_OVP |  \
    										g_SysFaultMessage.bit.unrecoverHW_InvH_OVP;
    //局部输出过压F04-2
    Ecan_ModuleData.Fault.bit.InvL_OVP = g_SysFaultMessage.bit.unrecoverSW_InvL_OVP | \
    										g_SysFaultMessage.bit.unrecoverHW_InvL_OVP;

    //过温关机F05
    Ecan_ModuleData.Fault.bit.OverTempFault = g_SysFaultMessage.bit.InvTempOverLimit | \
    									   g_SysFaultMessage.bit.PfcOverTempFault;

    //风扇故障（两风扇同时异常F06
    Ecan_ModuleData.Fault.bit.Fan_Fault = g_SysFaultMessage.bit.DcFanFault;

    //启动异常F07
    Ecan_ModuleData.Fault.bit.Launch_Fault = g_SysFaultMessage.bit.DcFuseFault |\
    										   g_SysFaultMessage.bit.HWADFault;
    //输入频率异常F11
    Ecan_ModuleData.Fault.bit.FreGridFault = g_SysFaultMessage.bit.FreGridOverRating | \
    										   g_SysFaultMessage.bit.FreGridUnderRating;
    //轨道输出频率异常F12
    Ecan_ModuleData.Fault.bit.FreInv_Fault = g_SysFaultMessage.bit.FreInvOverRating | \
    											g_SysFaultMessage.bit.FreInvUnderRating;

    //轨道输出过载F13-1
    Ecan_ModuleData.Fault.bit.InvH_OverLoad = g_SysFaultMessage.bit.InvH_OverLoad | \
    											g_SysFaultMessage.bit.recoverHW_InvH_OCP;
    //局部输出过载F13-2
    Ecan_ModuleData.Fault.bit.InvL_OverLoad = g_SysFaultMessage.bit.InvL_OverLoad | \
    											g_SysFaultMessage.bit.recoverHW_InvL_OCP;

    //短路或并机极端异常F14
    Ecan_ModuleData.Fault.bit.Unrecover_InvOCP = g_SysFaultMessage.bit.unrecoverHW_InvH_OCP | \
		    								   g_SysFaultMessage.bit.unrecoverHW_InvL_OCP;

    //并机均流故障F15
    Ecan_ModuleData.Fault.bit.Unrecover_InvCurrShar_Fault = g_SysFaultMessage.bit.unrecoverInvHCurrSharFault | \
		    									  g_SysFaultMessage.bit.unrecoverInvLCurrSharFault;

    //PFC不可恢复故障F03
    Ecan_ModuleData.Fault.bit.PFC_Unrecover_Fault = g_SysFaultMessage.bit.unrecoverHW_OCP_AC | \
    										g_SysFaultMessage.bit.unrecoverHW_Bus_OVP | \
    										g_SysFaultMessage.bit.unrecoverSW_Bus_OVP;

    //反复启动锁机保护F17
	 Ecan_ModuleData.Fault.bit.Unrecover_RestartNum = g_SysFaultMessage.bit.unrecover_RestartNum;
	 //并机线断F18
	 Ecan_ModuleData.Fault.bit.SynLine_Cut = g_SysFaultMessage.bit.unrecoverHW_SynLine_cut;
}

/*=============================================================================*
 * FUNCTION: 	Data_Format_Conver()
 * PURPOSE : 	参数校正执行程序
 *============================================================================*/
void Para_Revise_Oper(Uint8 temp)
{
	Uint8 Revised_Num;

	Revised_Num = temp;

	switch (Revised_Num)
	{
	case 1:
		if (Ecan_SysParaCalibration.u16VGrid_rms > 800 && Ecan_SysParaCalibration.u16VGrid_rms < 1200)
			ADCalibration.f32VGrid = (float32)(Ecan_SysParaCalibration.u16VGrid_rms * 0.001);

		if (Ecan_SysParaCalibration.u16VBusP > 900 && Ecan_SysParaCalibration.u16VBusP < 1100)
			ADCalibration.f32VBusP  = (float32)(Ecan_SysParaCalibration.u16VBusP * 0.001);

		if (Ecan_SysParaCalibration.u16VBusN > 900 && Ecan_SysParaCalibration.u16VBusN < 1100)
			ADCalibration.f32VBusN = (float32)(Ecan_SysParaCalibration.u16VBusN * 0.001);

		break;
	case 2:
		if (Ecan_SysParaCalibration.u16VInvH_rms > 800 && Ecan_SysParaCalibration.u16VInvH_rms < 1200)
			ADCalibration.f32VInvH = (float32)(Ecan_SysParaCalibration.u16VInvH_rms * 0.001);

		if (Ecan_SysParaCalibration.u16VInvL_rms > 800 && Ecan_SysParaCalibration.u16VInvL_rms < 1200)
			ADCalibration.f32VInvL = (float32)(Ecan_SysParaCalibration.u16VInvL_rms * 0.001);

		if (Ecan_SysParaCalibration.u16IInvH_rms > 800 && Ecan_SysParaCalibration.u16IInvH_rms < 1200)
			ADCalibration.f32IOutH = (float32)(Ecan_SysParaCalibration.u16IInvH_rms * 0.001);

		break;
	case 3:
		if (Ecan_SysParaCalibration.u16IInvL_rms > 800 && Ecan_SysParaCalibration.u16IInvL_rms < 1200)
			ADCalibration.f32IOutL= (float32)(Ecan_SysParaCalibration.u16IInvL_rms * 0.001);

		if (Ecan_SysParaCalibration.u16VOutH_rms > 800 && Ecan_SysParaCalibration.u16VOutH_rms < 1200)
			ADCalibration.f32VOutH = (float32)(Ecan_SysParaCalibration.u16VOutH_rms * 0.001);

		if (Ecan_SysParaCalibration.u16VOutL_rms > 800 && Ecan_SysParaCalibration.u16VOutL_rms < 1200)
			ADCalibration.f32VOutL = (float32)(Ecan_SysParaCalibration.u16VOutL_rms * 0.001);

		break;
	case 4:
		if (Ecan_SysParaCalibration.u16Temp_PFC > 800 && Ecan_SysParaCalibration.u16Temp_PFC < 1200)  //2017.11.21 GX
			ADCalibration.f32TempPFC = (float32)(Ecan_SysParaCalibration.u16Temp_PFC * 0.001);

		if (Ecan_SysParaCalibration.u16Temp_InvH > 800 && Ecan_SysParaCalibration.u16Temp_InvH < 1200)  //2017.12.1 GX
			ADCalibration.f32TempInvH = (float32)(Ecan_SysParaCalibration.u16Temp_InvH * 0.001);

		if (Ecan_SysParaCalibration.u16Temp_InvL > 800 && Ecan_SysParaCalibration.u16Temp_InvL < 1200)  //2017.12.1 GX
			ADCalibration.f32TempInvL = (float32)(Ecan_SysParaCalibration.u16Temp_InvL * 0.001);

		break;
	case 5:
		if (Ecan_SysParaCalibration.u16IGrid_rms > 800 && Ecan_SysParaCalibration.u16IGrid_rms < 1200)
			ADCalibration.f32IGrid = (float32)(Ecan_SysParaCalibration.u16IGrid_rms * 0.001);

		if (Ecan_SysParaCalibration.u16InvH_VoltRms_Ref > 2020 && Ecan_SysParaCalibration.u16InvH_VoltRms_Ref < 2420)  //2017.12.1 GX
			SafetyReg.f32InvH_VoltRms_Ref_LCD = (float32)(Ecan_SysParaCalibration.u16InvH_VoltRms_Ref * 0.1);

		if (Ecan_SysParaCalibration.u16InvL_VoltRms_Ref > 1010 && Ecan_SysParaCalibration.u16InvL_VoltRms_Ref  < 1210)  //2017.12.1 GX
			SafetyReg.f32InvL_VoltRms_Ref_LCD = (float32)(Ecan_SysParaCalibration.u16InvL_VoltRms_Ref * 0.1);

		break;
	case 6:
		if (Ecan_SysParaCalibration.u16VInvH_Comp_Coeff > 0 && Ecan_SysParaCalibration.u16VInvH_Comp_Coeff < 200)  //2017.12.1 GX
			Parallel_Reg.f32VInvH_Comp_Coeff = (float32)(Ecan_SysParaCalibration.u16VInvH_Comp_Coeff * 0.001);

		if (Ecan_SysParaCalibration.u16VInvL_Comp_Coeff  > 0 && Ecan_SysParaCalibration.u16VInvL_Comp_Coeff  < 200)  //2017.12.1 GX
			Parallel_Reg.f32VInvL_Comp_Coeff = (float32)(Ecan_SysParaCalibration.u16VInvL_Comp_Coeff * 0.001);

		if (Ecan_SysParaCalibration.u16IInvH_para_ave >= 0.0f && Ecan_SysParaCalibration.u16IInvH_para_ave < 800)  //2017.12.1 GX
		{
			Parallel_Reg.f32IInvH_para_ave = (float32)(Ecan_SysParaCalibration.u16IInvH_para_ave * 0.01);
			InvHParallelCurCheck();
		}

		break;
	case 7:
		if (Ecan_SysParaCalibration.u16IInvL_para_ave >= 0.0f && Ecan_SysParaCalibration.u16IInvL_para_ave < 1000)  //2017.12.1 GX
		{
			Parallel_Reg.f32IInvL_para_ave = (float32)(Ecan_SysParaCalibration.u16IInvL_para_ave * 0.01);
			InvLParallelCurCheck();
		}

		if (Ecan_SysParaCalibration.u16RestartTimes == 0x88)  //2017.12.1 GX
			g_SysFaultMessage.bit.unrecover_RestartNum = 1;

		break;
	default:
		break;
	}

}

/*=============================================================================*
 * FUNCTION: 	void Sys_Set_Oper()
 * PURPOSE : 	上位机命令执行程序
 *============================================================================*/
void Sys_Set_Oper()
{
	if (0 == Ecan_SytemOrder.Defaluts)
	{
		/*ADCalibration.f32VGrid = 1;   //2017.8.16 GX

		ADCalibration.f32IInvH = 1;
		ADCalibration.f32VInvH = 1;

		ADCalibration.f32IInvL = 1;
		ADCalibration.f32VInvL = 1;

		BusCon_Reg.f32BusVolt_Ref = 850;
		SafetyReg.f32VBus_HiLimit = Bus_Over_Volt_Limit;
		ADCalibration.f32TempPFC = 1.0f;
		ADCalibration.f32TempInvH = 1.0f;
		ADCalibration.f32TempInvL = 1.0f;*/
		;
	}
	if (0x0055 == Ecan_SytemOrder.Output_Enable)
	{
		g_SysFaultMessage.bit.recoverHW_InvL_OCP = 1;
		//g_Sys_Current_State = FaultState;
	}
	else if (0x00aa == Ecan_SytemOrder.Output_Enable)
	{
		PfcPWMOutputsDisable();
		InvPWMOutputsDisable();
		RelaysOFF();
		g_Sys_Current_State = PermanentState;
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

void Coeff_Feedback()
{
	Uint8	bStrLen = 0;
	Uint8   DataLength = 0x06;
	Uint32  id = 0;
	Uint8   check = 0x00;
	MAIL	Temp;

	id = Get_ID(DataLength, ModuleAdd);
	Temp.Mailbox_id.all = id;
	Temp.Mailbox_data.Byte.Current_frame = 0x01;//第一帧
	Temp.Mailbox_data.Byte.Frames_num = 0x06;//6帧
	Temp.Mailbox_data.Byte.Code = 0x00;

	Temp.Mailbox_data.Word.Data1 = (Uint16)(ADCalibration.f32VGrid * 1000);
	Temp.Mailbox_data.Word.Data2 = (Uint16)(ADCalibration.f32VBusP * 1000);
	Temp.Mailbox_data.Word.Data3 = (Uint16)(ADCalibration.f32VBusN * 1000);
	check = XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x02;//第二帧

	Temp.Mailbox_data.Word.Data1 = (Uint16)(ADCalibration.f32VInvH * 1000);
	Temp.Mailbox_data.Word.Data2 = (Uint16)(ADCalibration.f32VInvL * 1000);
	Temp.Mailbox_data.Word.Data3 = (Uint16)(ADCalibration.f32IOutH * 1000);
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x03;//第三帧

	Temp.Mailbox_data.Word.Data1 = (Uint16)(ADCalibration.f32IOutL * 1000);
	Temp.Mailbox_data.Word.Data2 = (Uint16)(ADCalibration.f32VOutH * 1000);
	Temp.Mailbox_data.Word.Data3 = (Uint16)(ADCalibration.f32VOutL * 1000);
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x04;//第四帧

	Temp.Mailbox_data.Word.Data1 = (Uint16)(ADCalibration.f32TempPFC * 1000);
	Temp.Mailbox_data.Word.Data2 = (Uint16)(ADCalibration.f32TempInvH * 1000);
	Temp.Mailbox_data.Word.Data3 = (Uint16)(ADCalibration.f32TempInvL * 1000);
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	Temp.Mailbox_data.Byte.Current_frame = 0x05;//第五帧

	Temp.Mailbox_data.Word.Data1 = (Uint16)(ADCalibration.f32IGrid * 1000);
	Temp.Mailbox_data.Word.Data2 = (Uint16)(SafetyReg.f32InvH_VoltRms_Ref_LCD * 10);
	Temp.Mailbox_data.Word.Data3 = (Uint16)(SafetyReg.f32InvL_VoltRms_Ref_LCD * 10);
	check ^= XOR(Temp);

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	/*--------------------------------------------------------------------------*/
	DataLength = 0x05;  //校验字节也是有效字节
	id = Get_ID(DataLength, ModuleAdd);
	Temp.Mailbox_id.all = id;
	Temp.Mailbox_data.Byte.Current_frame = 0x06;//the seventh frame

	Temp.Mailbox_data.Word.Data1 = (Uint16)(Parallel_Reg.f32VInvH_Comp_Coeff * 1000);
	Temp.Mailbox_data.Word.Data2 = (Uint16)(Parallel_Reg.f32VInvL_Comp_Coeff * 1000);

	check ^= XOR(Temp);
	Temp.Mailbox_data.Byte.byte7 = check;
	Temp.Mailbox_data.Byte.byte8 = 0x00;

	u8ECAN_UserDataBuf0[bStrLen] = Temp;
	bStrLen++;
	ECANWrite(u8ECAN_UserDataBuf0, bStrLen);
}

void Output_Revise (void) //2018.4.3 GX
{
	static  Uint8  lightH_temp= 0;
	static  Uint8  heavyH_temp = 0;
	static  Uint8  MiddleH_temp = 0;
	static  Uint8  lightL_temp= 0;
	static  Uint8  heavyL_temp = 0;
	static  Uint8 MiddleL_temp = 0;

	if ( Calc_Result.f32IOutH_rms <= Rated_InvH_OutputCurrentRms * 0.25f &&  Output_VoltRe_Reg.u8InvH_Light_Flag == 0)
	{
		lightH_temp++;
		if ( lightH_temp >= 1)
		{
			SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvLightRevise;
			g_InvH_Load = LightLoad;
			Output_VoltRe_Reg.u8InvH_Light_Flag = 1;
			Output_VoltRe_Reg.u8InvH_Middle_Flag = 0;
			Output_VoltRe_Reg.u8InvH_Heavy_Flag = 0;
			lightH_temp = 0;
		}
	}
	else if ( Calc_Result.f32IOutH_rms >= Rated_InvH_OutputCurrentRms * 0.65f && Output_VoltRe_Reg.u8InvH_Heavy_Flag == 0 )
	{
			heavyH_temp ++;
			if ( heavyH_temp >= 1)
			{
				SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvHeavyRevise;
				g_InvH_Load = HeavyLoad;
				Output_VoltRe_Reg.u8InvH_Light_Flag = 0;
				Output_VoltRe_Reg.u8InvH_Middle_Flag = 0;
				Output_VoltRe_Reg.u8InvH_Heavy_Flag = 1;
				heavyH_temp = 0;
			}
	}
	else if ( (Calc_Result.f32IOutH_rms >= Rated_InvH_OutputCurrentRms * 0.3f && \
			Calc_Result.f32IOutH_rms <= Rated_InvH_OutputCurrentRms * 0.6f ) && Output_VoltRe_Reg.u8InvH_Middle_Flag == 0 )
	{
			MiddleH_temp ++;
			if ( MiddleH_temp >= 1)
			{
				SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvMiddleRevise;
				g_InvH_Load = MiddleLoad;
				Output_VoltRe_Reg.u8InvH_Light_Flag = 0;
				Output_VoltRe_Reg.u8InvH_Middle_Flag = 1;
				Output_VoltRe_Reg.u8InvH_Heavy_Flag = 0;
				MiddleH_temp = 0;
			}
	}
	else
	{
		if ((Output_VoltRe_Reg.u8InvH_Light_Flag == 1 && Calc_Result.f32IOutH_rms >= Rated_InvH_OutputCurrentRms * 0.5f) || \
				(Output_VoltRe_Reg.u8InvH_Heavy_Flag == 1 && Calc_Result.f32IOutH_rms <= Rated_InvH_OutputCurrentRms * 0.5f))
		{
			SafetyReg.f32InvH_VoltRms_Ref = SafetyReg.f32InvH_VoltRms_Ref_LCD * VInvMiddleRevise;
			g_InvH_Load = MiddleLoad;
			Output_VoltRe_Reg.u8InvH_Light_Flag = 0;
			Output_VoltRe_Reg.u8InvH_Middle_Flag = 1;
			Output_VoltRe_Reg.u8InvH_Heavy_Flag = 0;
		}
		else
			SafetyReg.f32InvH_VoltRms_Ref =SafetyReg.f32InvH_VoltRms_Ref;

		MiddleH_temp = 0;
		heavyH_temp = 0;
		lightH_temp = 0;
	}

	if ( Calc_Result.f32IOutL_rms <= Rated_InvL_OutputCurrentRms * 0.25 &&  Output_VoltRe_Reg.u8InvL_Light_Flag == 0)
	{
		lightL_temp++;
		if ( lightL_temp >= 1)
		{
			SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvLightRevise;
			g_InvL_Load = LightLoad;
			Output_VoltRe_Reg.u8InvL_Light_Flag = 1;
			Output_VoltRe_Reg.u8InvL_Middle_Flag = 0;
			Output_VoltRe_Reg.u8InvL_Heavy_Flag = 0;
			lightL_temp = 0;
		}
	}
	else if ( Calc_Result.f32IOutL_rms >= Rated_InvL_OutputCurrentRms * 0.65 && Output_VoltRe_Reg.u8InvL_Heavy_Flag == 0 )
	{
		heavyL_temp ++;
		if ( heavyL_temp >= 1)
		{
			SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvHeavyRevise;
			g_InvL_Load = HeavyLoad;
			Output_VoltRe_Reg.u8InvL_Light_Flag = 0;
			Output_VoltRe_Reg.u8InvL_Middle_Flag = 0;
			Output_VoltRe_Reg.u8InvL_Heavy_Flag = 1;
			heavyL_temp = 0;
		}
	}
	else if ( (Calc_Result.f32IOutL_rms >= Rated_InvL_OutputCurrentRms * 0.3 && \
			Calc_Result.f32IOutL_rms <= Rated_InvL_OutputCurrentRms * 0.6 ) && Output_VoltRe_Reg.u8InvL_Middle_Flag == 0 )
	{
		MiddleL_temp ++;
		if ( MiddleL_temp >= 1)
		{
			SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvMiddleRevise;
			g_InvL_Load = MiddleLoad;
			Output_VoltRe_Reg.u8InvL_Light_Flag = 0;
			Output_VoltRe_Reg.u8InvL_Middle_Flag = 1;
			Output_VoltRe_Reg.u8InvL_Heavy_Flag = 0;
			MiddleL_temp = 0;
		}
	}
	else
	{
		if ((Output_VoltRe_Reg.u8InvL_Light_Flag == 1 && Calc_Result.f32IOutL_rms >= Rated_InvL_OutputCurrentRms * 0.5f) || \
				(Output_VoltRe_Reg.u8InvL_Heavy_Flag == 1 && Calc_Result.f32IOutL_rms <= Rated_InvL_OutputCurrentRms * 0.5f))
		{
			SafetyReg.f32InvL_VoltRms_Ref = SafetyReg.f32InvL_VoltRms_Ref_LCD * VInvMiddleRevise;
			g_InvL_Load = MiddleLoad;
			Output_VoltRe_Reg.u8InvL_Light_Flag = 0;
			Output_VoltRe_Reg.u8InvL_Middle_Flag = 1;
			Output_VoltRe_Reg.u8InvL_Heavy_Flag = 0;
		}
		else
			SafetyReg.f32InvL_VoltRms_Ref =SafetyReg.f32InvL_VoltRms_Ref;

		MiddleL_temp = 0;
		heavyL_temp = 0;
		lightL_temp = 0;
	}
}
