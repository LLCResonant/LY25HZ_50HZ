/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_DataAcquisition.c
 *
 *  PURPOSE  : Data acquisition and protection file of the module.
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/

#include "DSP2833x_Device.h"	// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file


static	ECANStruct	ECANList;							//contain transmit state, the amount of data and pointers of queue in buffer
//pointers of queue in buffer: In-Pointer, Out-Pointer, Start-Pointer, The length of buffer, The number of frames
static	ECANQUEUE	ECANQList;
ECAN_ERROR Ecan_Error;

ECANStruct	*pECANIndex = {NULL};				//eCAN Index pointer
//#pragma DATA_SECTION(szECANRxBuf,"SLOWDATA");

MAIL	szECANRxBuf[MAX_ECAN_BUF_SIZE]={NULL};		//Buffer
MAIL	*pECANBuf = szECANRxBuf;			//Pointer of buffer
Uint8 u8EACNCommFaultCnt = 0;
Uint8 Broadcast_Entry = 0;
Uint8 Transmit_Entry = 0;

void InitECana(void);
Uint16 ECANRead(MAIL *pBuf);
Uint16 ECANWrite(MAIL *pBuf, Uint16 u16Length);
void SetECAN(MAIL *pStartAddr, Uint16 u16Size);
static void InitECANQueue(ECANQUEUE *pQue, MAIL *pStart, Uint16 u16BufSize);
static Uint16 ECANQueDataIn(ECANQUEUE *pQue, MAIL MAILQueData);
static Uint16 ECANQueDataOut(ECANQUEUE *pQue, MAIL *MAILQueData);
void SWI_ECANRXISR();
static Uint8 ECANErrorCheck();

void eCAN_Transmit(MAIL Mail_Tx);
void eCAN_Broadcast(P2AMAIL);


void InitECana(void)
{
	struct ECAN_REGS ECanaShadow;
	Uint32 u32_Add = 0;


	//********************************************
	//*************PART Initialization*************
	//the following part focus on initializationtion
	EALLOW;
	ECanaShadow.CANMC.all=ECanaRegs.CANMC.all;
	//to work in normal mode
	ECanaShadow.CANMC.bit.STM=0;				//to work in normal mode
	//ECanaShadow.CANMC.bit.STM=1;				//to work in Self_test mode
	//to work in eCAN mode

	   ECanaShadow.CANMC.bit.WUBA = 1;
	   ECanaShadow.CANMC.bit.DBO = 1;				//LSB
	//	ECanaShadow.CANMC.bit.DBO = 0;				//MSB
		ECanaShadow.CANMC.bit.ABO = 1;				//Auto bus on.
	//	ECanaShadow.CANMC.bit.ABO = 0;				//Auto bus on.

	ECanaRegs.CANMC.all=ECanaShadow.CANMC.all;
	EDIS;


	//Step 1, set CANME to 0 to disable the mailboxes
	ECanaRegs.CANME.all=0;

	//Step 2, apply to change data area through CANMC
	EALLOW;
	ECanaShadow.CANMC.all=ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CDR=1;
	ECanaRegs.CANMC.all=ECanaShadow.CANMC.all;
	EDIS;

	//Step 3, config the ID,control, data, direction of the mailboxes
	//mailbox 0 config as  acceptting box
	ECanaShadow.CANMD.all=ECanaRegs.CANMD.all;
	ECanaShadow.CANMD.bit.MD0=0;	//deliver
	ECanaShadow.CANMD.bit.MD1=1;	//receiver, point to point
	ECanaShadow.CANMD.bit.MD2=0;	//deliver, broadcast
	ECanaShadow.CANMD.bit.MD3=1;	//receiver, broadcast
	//ECanaShadow.CANMD.bit.MD4=1;
	//ECanaShadow.CANMD.bit.MD5=0;
	ECanaRegs.CANMD.all=ECanaShadow.CANMD.all;
	//config the box's ID


	ECanaMboxes.MBOX0.MSGID.all=0x80000000;//Send mail box do not need mask bit
	//	ECanaMboxes.MBOX0.MSGID.all=0x//extension identifier mode
	u32_Add = ModuleAdd;
	ECanaMboxes.MBOX1.MSGID.all=0xC1400000 | (u32_Add << 16);//Upload to computer
	ECanaMboxes.MBOX2.MSGID.all=0x80000000; //Send mail box do not need mask bit
	ECanaMboxes.MBOX3.MSGID.all=0xCFFF0007; //Broadcast



	//	ECanaMboxes.MBOX3.MSGID.all=0xC607F813;
	//	ECanaMboxes.MBOX3.MSGID.all=0xC607F81B;
	//	ECanaMboxes.MBOX3.MSGID.all=0xC607F823;
	//	ECanaMboxes.MBOX4.MSGID.all=0xC0000011;
	//	ECanaMboxes.MBOX5.MSGID.all=0x80000011;
		//config acceptance filter for MBOX1~MBOX2
		ECanaLAMRegs.LAM0.all = 0xF000FFFF;    	//SCC mode, LAM0 for MBOX0,MBOX1, received information from computer
	//	ECanaLAMRegs.LAM0.all = 0xFFFFFFFF;     	//SCC mode, LAM0 for MBOX0,MBOX1
		ECanaLAMRegs.LAM3.all = 0xF000FFFC;    	//SCC mode, LAM0 for MBOX3, received broadcast information
	//	ECanaLAMRegs.LAM1.all = 0x0000FF02;    	//eCAN mode, LAM0 for MBOX1
	//	ECanaLAMRegs.LAM2.all = 0x000007FF;    	//eCAN mode, LAM0 for MBOX2
	//	ECanaLAMRegs.LAM4.all = 0xFFFFFFFF;     	//eCAN mode, LAM0 for MBOX1
	//	ECanaLAMRegs.LAM2.all = 0xFFFFFFFF;     	//eCAN mode, LAM0 for MBOX2

	//	config data length as 8-bytes
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC=8;
	ECanaMboxes.MBOX1.MSGCTRL.bit.DLC=8;
	ECanaMboxes.MBOX2.MSGCTRL.bit.DLC=8;
	ECanaMboxes.MBOX3.MSGCTRL.bit.DLC=8;
	//ECanaMboxes.MBOX4.MSGCTRL.bit.DLC=8;
	//ECanaMboxes.MBOX5.MSGCTRL.bit.DLC=8;
	//config transmitting prior
	ECanaMboxes.MBOX0.MSGCTRL.bit.TPL=0;
	ECanaMboxes.MBOX1.MSGCTRL.bit.TPL=0;
	ECanaMboxes.MBOX2.MSGCTRL.bit.TPL=0;
	ECanaMboxes.MBOX3.MSGCTRL.bit.TPL=0;
	//ECanaMboxes.MBOX4.MSGCTRL.bit.TPL=0;
	//ECanaMboxes.MBOX5.MSGCTRL.bit.TPL=0;
	//no long-distance answer frame was applied
	ECanaMboxes.MBOX0.MSGCTRL.bit.RTR=0;
	ECanaMboxes.MBOX1.MSGCTRL.bit.RTR=0;
	ECanaMboxes.MBOX2.MSGCTRL.bit.RTR=0;
	ECanaMboxes.MBOX3.MSGCTRL.bit.RTR=0;
	//ECanaMboxes.MBOX4.MSGCTRL.bit.RTR=0;
	//ECanaMboxes.MBOX5.MSGCTRL.bit.RTR=0;
	//write data to boxes' RAM
	ECanaMboxes.MBOX0.MDL.all=0x00000000;
	ECanaMboxes.MBOX0.MDH.all=0x00000000;
	ECanaMboxes.MBOX1.MDL.all=0x00000000;
	ECanaMboxes.MBOX1.MDH.all=0x00000000;
	ECanaMboxes.MBOX2.MDL.all=0x00000000;
	ECanaMboxes.MBOX2.MDH.all=0x00000000;
	ECanaMboxes.MBOX3.MDL.all=0x00000000;
	ECanaMboxes.MBOX3.MDH.all=0x00000000;
	//ECanaMboxes.MBOX4.MDL.all=0x00000000;
	//ECanaMboxes.MBOX4.MDH.all=0x00000000;
	//ECanaMboxes.MBOX5.MDL.all=0x00000000;
	//ECanaMboxes.MBOX5.MDH.all=0x00000000;

	//Step 4, apply for normal operations
	EALLOW;
	ECanaShadow.CANMC.all=ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CDR=0;
	ECanaRegs.CANMC.all=ECanaShadow.CANMC.all;
	EDIS;
	//Step 5,set CANME to 1 to enable mailboxes
	ECanaShadow.CANME.all=ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0=1;
	ECanaShadow.CANME.bit.ME1=1;
	ECanaShadow.CANME.bit.ME2=1;
	ECanaShadow.CANME.bit.ME3=1;
	//ECanaShadow.CANME.bit.ME4=1;
	//ECanaShadow.CANME.bit.ME5=1;
	ECanaRegs.CANME.all=ECanaShadow.CANME.all;

	//*********************************************
	EALLOW; //enable mailbox interrupts
	ECanaRegs.CANMIM.all=0x000000A;			// MBOX1 Receive interrupt
	ECanaRegs.CANMIL.all=0; 							//interrupt will be generated at ECAN0INT
	ECanaRegs.CANGIF0.all=0xFFFFFFFF;
	ECanaRegs.CANGIM.all= 0;
	ECanaRegs.CANGIM.bit.I0EN= 1; //ECAN0INT interrupt application permitted
	EDIS;

	SetECAN(pECANBuf,ECANA_BUF_SIZE);   //Clear and set start address of the queue
	//*************PART Initialization END*********
}

/**********************************************************************
* Function: SetSci()
*
* Description: Clear and set start address of the queue
**********************************************************************/
void SetECAN(MAIL *pStartAddr, Uint16 u16Size)
{
	ECANStruct	*pECAN;
	ECANQUEUE	*pq;

	if( NULL == pECANIndex )   		// ID is zero
	{
		pECAN = &ECANList;
		pECANIndex= pECAN;

		pECAN->pqRx = &ECANQList;
		pq = pECAN->pqRx;
		InitECANQueue(pq, pStartAddr, u16Size);
		pECANBuf += u16Size;

		pECAN->u8TxStatus = ECAN_TX_RDY;    //Status initialization
		pECAN->u16TxLength = 0;            //data length is 0
	}
}

/**********************************************************************
* Function: InitECANQueue()
*
* Description:   queue initialization
**********************************************************************/
static void InitECANQueue(ECANQUEUE *pQue, MAIL *pStart, Uint16 u16BufSize)
{
	pQue->u16Length = 0;
	pQue->u16Size = u16BufSize;
	pQue->pIn = pStart;
	pQue->pOut = pStart;
	pQue->pStart = pStart;
}
/**********************************************************************
* Function: QueDataIn()
*
* Description:   data is put in the queue
**********************************************************************/
static Uint16 ECANQueDataIn(ECANQUEUE *pQue, MAIL MAILQueData)
{
	if(pQue->u16Length == pQue->u16Size) //deal with full queues
	{
		if(pQue->pIn == pQue->pStart)
		{
			*(pQue->pStart + pQue->u16Size - 1) = MAILQueData;//Rewrite previous data
		}
		else
		{
			*(pQue->pIn - 1) = MAILQueData;//Rewrite previous data
		}
		return ECAN_QUE_BUF_FULL;
	}
	else
	{
		*(pQue->pIn) = MAILQueData;
		pQue->u16Length += 1;
		if(pQue->pIn == pQue->pStart + pQue->u16Size - 1)   //whether it is the last in queue
		{
			pQue->pIn = pQue->pStart;
		}
		else
		{
			pQue->pIn += 1;
		}
		return ECAN_QUE_BUF_NORMAL;
	}
}
/**********************************************************************
* Function: QueDataOut()
*
* Description:   data is get out of the queue
**********************************************************************/
static Uint16 ECANQueDataOut(ECANQUEUE *pQue, MAIL *MAILQueData)
{
	if(0 == pQue->u16Length)
	{
		return ECAN_QUE_BUF_EMPTY;
	}
	else
	{
		*MAILQueData = *(pQue->pOut);
		pQue->u16Length -= 1;
		if(pQue->pOut == (pQue->pStart + pQue->u16Size - 1))   //whether the pointer points the last one?
		{
			pQue->pOut = pQue->pStart;
		}
		else
		{
			pQue->pOut += 1;
		}
		return ECAN_QUE_BUF_NORMAL;
	}
}
/**********************************************************************
* Function: eCANRead
*
* Description:   read data
**********************************************************************/
Uint16 ECANRead(MAIL *pBuf)
{
	Uint16 		u16Tmp;
	ECANQUEUE	*pq;
	ECANStruct	*pECAN;

// __asm ("      ESTOP0");
	pECAN = pECANIndex;
	pq = pECAN->pqRx;

	u16Tmp = ECANQueDataOut(pq, pBuf);	//read data in the queue which pECANIndex points

	if(u16Tmp == ECAN_QUE_BUF_EMPTY)
	{
		return ECAN_RX_EMPTY;
	}
	else
	{
		return ECAN_RX_RDY;
	}
}
/**********************************************************************
* Function: ECANWrite
*
* Description:   write data
**********************************************************************/

Uint16 ECANWrite(MAIL *pBuf, Uint16 u16Length)
{
	Uint16 i;
	ECANStruct		*pECAN;

	pECAN = pECANIndex;
	if(pECAN->u8TxStatus == ECAN_TX_BUSY)
	{
		return ECAN_TX_BUSY;
	}

	pECAN->u16TxLength = u16Length;
	pECAN->u8TxStatus = ECAN_TX_BUSY;
	for(i = 0;i < u16Length;i++)
		eCAN_Transmit(*(pBuf + i));

	pECAN->u8TxStatus = ECAN_TX_RDY;
	return ECAN_TX_RDY;
}
/**********************************************************************
* Function: ECANErrorCheck
*
* Description: check fault bits in DSP ECAN Register, if anyone is set,
* set g_StateCheck.bit.ECAN_Fault.
**********************************************************************/

Uint8 ECANErrorCheck()
{
	struct ECAN_REGS ECanaShadow;

	ECanaShadow.CANES.all=ECanaRegs.CANES.all;
	 if((ECanaShadow.CANES.bit.CRCE == 1)||(ECanaShadow.CANES.bit.EP == 1)||(ECanaShadow.CANES.bit.BO == 1))
	{
		EALLOW;
		ECanaShadow.CANMC.all=ECanaRegs.CANMC.all;
		ECanaShadow.CANMC.bit.SRES = 1;
	    ECanaShadow.CANMC.bit.CCR = 0;            // CLEAR CCR = 0, Restart CCR mode
		ECanaShadow.CANMC.all=ECanaRegs.CANMC.all;
		EDIS;

		u8EACNCommFaultCnt++;
		if(u8EACNCommFaultCnt>30)
		{
			g_StateCheck.bit.ECAN_Fault = 1;
			u8EACNCommFaultCnt=0;
		}
		return 1;
	}
	else
		return 0;
}

/**********************************************************************
* Function: SWI_ECANRXISR
*
* Description: Ecan receiving interruption
**********************************************************************/
void SWI_ECANRXISR()
{
	MAIL	MailTmp;
	Uint8 errortemp = 0;
	errortemp = ECANErrorCheck();
	if (errortemp == 0)
	{
		if (ECanaRegs.CANGIF0.bit.MIV0 == 1) //whether mail box 1 receives interruption
		{
//			__asm ("ESTOP0");
			MailTmp.Mailbox_id.all = ECanaMboxes.MBOX1.MSGID.all;
			MailTmp.Mailbox_data.DWord.CANL_Bytes = ECanaMboxes.MBOX1.MDL.all;
			MailTmp.Mailbox_data.DWord.CANH_Bytes = ECanaMboxes.MBOX1.MDH.all;

			ECANQueDataIn(pECANIndex->pqRx,MailTmp);
			ECanaRegs.CANRMP.all=0x00000002;
		}
		else if(ECanaRegs.CANGIF0.bit.MIV0 == 3) //whether mail box 3 receives interruption
		{
			EcanP2A_Rx.P2AMail_id.all = ECanaMboxes.MBOX3.MSGID.all;
			EcanP2A_Rx.P2AMail_data.DWord.CANL_Bytes = ECanaMboxes.MBOX3.MDL.all;
			EcanP2A_Rx.P2AMail_data.DWord.CANH_Bytes = ECanaMboxes.MBOX3.MDH.all;

			Arbitrate(EcanP2A_Rx);
			ECanaRegs.CANRMP.all=0x00000008;
		}
	}
	else
	{
		ECanaRegs.CANRMP.all=0x00000002;
		ECanaRegs.CANRMP.all=0x00000008;
	}
	PieCtrlRegs.PIEACK.bit.ACK9=1;
}

/********************************************************************************************
* Function: eCAN_Transmit()
*
* Description: eCAN transmit function, which is used in communicating with the computer
********************************************************************************************/
void eCAN_Transmit(MAIL Mail_Tx)
{
	struct ECAN_REGS ECanaShadow;
	Uint16 Transmit_Entry = 0;

	//*************PART I**************************
	//the next part focus on transmitting messages
	//Step 1, clear register CANTRS

	ECanaShadow.CANTRR.all = ECanaRegs.CANTRR.all;
	ECanaShadow.CANTRR.bit.TRR0 = 1;
	ECanaRegs.CANTRR.all = ECanaShadow.CANTRR.all;

	ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
	ECanaShadow.CANTA.bit.TA0 = 1;						//Shadow register is used to prevent assignment failure
	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	//Step 2, initialize mailboxes
	//write data to boxes' RAM
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 0;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	ECanaMboxes.MBOX0.MSGID.all = Mail_Tx.Mailbox_id.all;			//set mail box id
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;					//configure data length as 8-bytes

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;


	ECanaMboxes.MBOX0.MDH.all = Mail_Tx.Mailbox_data.DWord.CANH_Bytes;
	ECanaMboxes.MBOX0.MDL.all = Mail_Tx.Mailbox_data.DWord.CANL_Bytes;


	EALLOW;
//	__asm ("      ESTOP0");
	//Step 3, configure TRS to apply for transmitting
	ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
	ECanaShadow.CANTRS.bit.TRS0=1;
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

//	__asm ("      ESTOP0");

	//Step 4, wait for response to complete transmission
	do
	{
		ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
		Transmit_Entry ++;
	}while(ECanaShadow.CANTA.bit.TA0 != 1 && Transmit_Entry < 65535);

	if (Transmit_Entry >= 65535 && ECanaShadow.CANTA.bit.TA0 != 1)
	{
		g_StateCheck.bit.ECAN_Noline = 1;
		if (Ecan_Error.u8Upload_Trans_Error >= 254)
			Ecan_Error.u8Upload_Trans_Error = 0;
		else
			Ecan_Error.u8Upload_Trans_Error ++;
	}
	else
		g_StateCheck.bit.ECAN_Noline = 0;

	Transmit_Entry = 0;

	//Step 5,reset TA and transmission flag by writing 1 to corresponding register bits
	ECanaShadow.CANTA.all = 0;
	//	__asm ("      ESTOP0");
	ECanaShadow.CANTA.bit.TA0 = 1;
	ECanaRegs.CANTA.all=ECanaShadow.CANTA.all;

	EDIS;
//*************PART I END**********************
}
/********************************************************************************************
* Function: eCAN_Broadcast()
*
* Description: eCAN transmit function, which is used in communicating with other modules
********************************************************************************************/

void eCAN_Broadcast(P2AMAIL P2A_Tx)
{
	struct ECAN_REGS ECanaShadow;
	Uint16 Broadcast_Entry = 0;

	//*************PART I**************************
	//the next part focus on transmitting messages
	//Step 1, clear register CANTRS

	ECanaShadow.CANTRR.all = ECanaRegs.CANTRR.all;
	ECanaShadow.CANTRR.bit.TRR2 = 1;
	ECanaRegs.CANTRR.all = ECanaShadow.CANTRR.all;

	ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
	ECanaShadow.CANTA.bit.TA2 = 1;						//Shadow register is used to prevent assignment failure
	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	//Step 2, initialize mailboxes
	//write data to boxes' RAM
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME2 = 0;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	ECanaMboxes.MBOX2.MSGID.all = P2A_Tx.P2AMail_id.all;			//set mail box id
	ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;					//configure data length as 8-bytes

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME2 = 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;


	ECanaMboxes.MBOX2.MDH.all = P2A_Tx.P2AMail_data.DWord.CANH_Bytes;
	ECanaMboxes.MBOX2.MDL.all = P2A_Tx.P2AMail_data.DWord.CANL_Bytes;


	EALLOW;
//	__asm ("      ESTOP0");
	//Step 3, configure TRS to apply for transmitting
	ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
	ECanaShadow.CANTRS.bit.TRS2=1;
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

//	__asm ("      ESTOP0");

	//Step 4, wait for response to complete transmission
	do
	{
		ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
		Broadcast_Entry ++;
	}while(ECanaShadow.CANTA.bit.TA2 != 1&& Broadcast_Entry < 65535);

	if (Broadcast_Entry >= 65535 && ECanaShadow.CANTA.bit.TA2 != 1)
	{
		g_StateCheck.bit.ECAN_Noline = 1;
		if (Ecan_Error.u8Broadcast_Trans_Error>= 254)
			Ecan_Error.u8Broadcast_Trans_Error = 0;
		else
			Ecan_Error.u8Broadcast_Trans_Error ++;
	}
	else
		g_StateCheck.bit.ECAN_Noline = 0;

	Broadcast_Entry = 0;

	//Step 5,reset TA and transmission flag by writing 1 to corresponding register bits
	ECanaShadow.CANTA.all = 0;
//	__asm ("      ESTOP0");
	ECanaShadow.CANTA.bit.TA2 = 1;
	ECanaRegs.CANTA.all=ECanaShadow.CANTA.all;

	EDIS;
//*************PART I END**********************
}
