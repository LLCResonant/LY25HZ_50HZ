/*
 *
 *  Revised on: 2017.10.23 by Gao Xun
 */

#include "DSP2803x_Device.h"	// Peripheral address definitions
#include "F28035_example.h"			// Main include file


static	ECANStruct	ECANList;							//eCAN发送状态、发送数据量、缓冲区队列指针
static	ECANQUEUE	ECANQList;							//eCAN缓冲区队列指针：入指针、出指针、起始指针、缓冲区总长度、现有帧数目
ECAN_ERROR Ecan_Error;

ECANStruct	*pECANIndex = {NULL};				//eCAN索引指针
//#pragma DATA_SECTION(szECANRxBuf,"SLOWDATA");

MAIL	szECANRxBuf[MAX_ECAN_BUF_SIZE]={NULL};		//  开辟的eCAN通讯缓冲区之和
MAIL	*pECANBuf = szECANRxBuf;			//  指针指向该缓冲区，初始化分配空间的时候使用
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
		//	ECanaShadow.CANMC.bit.DBO = 1;				//接收发送低位优先
			ECanaShadow.CANMC.bit.DBO = 0;				//接收发送高位优先
			ECanaShadow.CANMC.bit.ABO = 1;				//Auto bus on.发送故障后自动重新切入总线
		//	ECanaShadow.CANMC.bit.ABO = 0;				//Auto bus on.发送故障后,需要手动清零CCR位重新切入总线

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
		ECanaShadow.CANMD.bit.MD1=1;	//receiver，point to point
		ECanaShadow.CANMD.bit.MD2=0;	//deliver，broadcast
		ECanaShadow.CANMD.bit.MD3=1;	//receiver，broadcast
		//ECanaShadow.CANMD.bit.MD4=1;
		//ECanaShadow.CANMD.bit.MD5=0;
		ECanaRegs.CANMD.all=ECanaShadow.CANMD.all;
		//config the box's ID


		ECanaMboxes.MBOX0.MSGID.all=0x80000000;//发送邮箱不需要屏蔽位
	//	ECanaMboxes.MBOX0.MSGID.all=0x//extension identifier mode,发送邮箱的地址
		u32_Add = ModuleAdd;
		ECanaMboxes.MBOX1.MSGID.all=0xC1400000 | (u32_Add << 16);
		ECanaMboxes.MBOX2.MSGID.all=0x80000000; //extension identifier mode,接收到的为广播命令
		ECanaMboxes.MBOX3.MSGID.all=0xCFFF0007;//样机1



	//	ECanaMboxes.MBOX3.MSGID.all=0xC607F813; //样机2
	//	ECanaMboxes.MBOX3.MSGID.all=0xC607F81B; //样机3
	//	ECanaMboxes.MBOX3.MSGID.all=0xC607F823; //样机4
	//	ECanaMboxes.MBOX4.MSGID.all=0xC0000011;
	//	ECanaMboxes.MBOX5.MSGID.all=0x80000011;
		//config acceptance filter for MBOX1~MBOX2
		ECanaLAMRegs.LAM0.all = 0xF000FFFF;     //SCC mode, LAM0 for MBOX0,MBOX1,点对点通信
	//	ECanaLAMRegs.LAM0.all = 0xFFFFFFFF;     //SCC mode, LAM0 for MBOX0,MBOX1
		ECanaLAMRegs.LAM3.all = 0xF000FFFC;     //SCC mode, LAM0 for MBOX3,接收到的为广播命令
	//	ECanaLAMRegs.LAM1.all = 0x0000FF02;     //eCAN mode, LAM0 for MBOX1,点对点通信
	//	ECanaLAMRegs.LAM2.all = 0x000007FF;     //eCAN mode, LAM0 for MBOX2,接收到的为广播命令
	//	ECanaLAMRegs.LAM4.all = 0xFFFFFFFF;     //eCAN mode, LAM0 for MBOX1,点对点通信
	//	ECanaLAMRegs.LAM2.all = 0xFFFFFFFF;     //eCAN mode, LAM0 for MBOX2,接收到的为广播命令

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
		ECanaRegs.CANMIL.all=0; 					//interrupt will be generated at ECAN0INT
	    ECanaRegs.CANGIF0.all=0xFFFFFFFF;
	    ECanaRegs.CANGIM.all= 0;
		ECanaRegs.CANGIM.bit.I0EN= 1; //ECAN0INT interrupt application permitted
		EDIS;

		SetECAN(pECANBuf,ECANA_BUF_SIZE);   // 清零 并规定各队列的起始地址
	//    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
		//*********************************************

		//*************PART Initialization END*********
		//*********************************************
}



/**********************************************************************
* Function: SetSci()
*
* Description:

//  入口参数： 1） id号  scia=0  scib=1  scic=2
//             2)  Uint8 *pStartAddr   缓冲起始地址
//             3)  缓冲长度  ECANA_BUF_SIZE					64

typedef struct{
	Uint8	*pIn;
	Uint8	*pOut;
	Uint8	*pStart;
	Uint16	u16Length;
	Uint16 	u16Size;
}ECANQUEUE;

typedef struct{
	Uint8	u8TxStatus;
	Uint16	u16TxLength;
	ECANQUEUE	*pqRx;
}ECANStruct;

* Function: SetECAN()
*
* Description:
*  Uint8 *pStartAddr------缓冲指针
*  Uint16 u16Size--------所申请的缓冲区长度
**********************************************************************/
void SetECAN(MAIL *pStartAddr, Uint16 u16Size)
{
	ECANStruct	*pECAN;   			// 结构体 指针
	ECANQUEUE	*pq;     			// 队列结构体  指针

	if( NULL == pECANIndex )   		//  ID号都为零
	{
		pECAN = &ECANList;   			// 相应eCAN的结构体指针
		pECANIndex= pECAN;     			// 加入到索引中

		pECAN->pqRx = &ECANQList;		//队列链接到索引中
		pq = pECAN->pqRx;
		InitECANQueue(pq, pStartAddr, u16Size);
		pECANBuf += u16Size;

		pECAN->u8TxStatus = ECAN_TX_RDY;    //  状态初始化
		pECAN->u16TxLength = 0;            // 发送长度为0
	}
}

/**********************************************************************
* Function:InitECANQueue   指针指向初始地址
*
* Description:   队列初始
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
* Description:   队列   数据入队列
**********************************************************************/
static Uint16 ECANQueDataIn(ECANQUEUE *pQue, MAIL MAILQueData)
{
	if(pQue->u16Length == pQue->u16Size) // 满队列处理
	{
		if(pQue->pIn == pQue->pStart)
		{
			*(pQue->pStart + pQue->u16Size - 1) = MAILQueData;// 改写上一次最新的数据
		}
		else
		{
			*(pQue->pIn - 1) = MAILQueData; // 改写上一次最新的数据
		}
		return ECAN_QUE_BUF_FULL;
	}
	else
	{
		*(pQue->pIn) = MAILQueData;    //  返回队列已满
		pQue->u16Length += 1;            // 正常入队列
		if(pQue->pIn == pQue->pStart + pQue->u16Size - 1)   //  是否到最后
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
* Function: QueDataOut
*
* Description:   队列   数据出队列
**********************************************************************/
static Uint16 ECANQueDataOut(ECANQUEUE *pQue, MAIL *MAILQueData)
{
	if(0 == pQue->u16Length)
	{
		return ECAN_QUE_BUF_EMPTY;
	}
	else
	{
		*MAILQueData = *(pQue->pOut);    //    出队列
		pQue->u16Length -= 1;
		if(pQue->pOut == (pQue->pStart + pQue->u16Size - 1))   //  指针是否指向最后
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
* Description:   eCAN  读取数据
**********************************************************************/
Uint16 ECANRead(MAIL *pBuf)
{
	Uint16 		u16Tmp;
	ECANQUEUE	*pq;
	ECANStruct	*pECAN;

//	  __asm ("      ESTOP0");
	pECAN = pECANIndex;							//可以发送的数据
	pq = pECAN->pqRx;        				//  队列指针

	u16Tmp = ECANQueDataOut(pq, pBuf);			//读取pECANIndex索引下的eCANA接收到的指令数据

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
* Description:   ECAN 写数据
**********************************************************************/
/*********需要再修改************/

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
* Description: 检查ECAN寄存器的错误位，
* 如果有错误g_SysFaultMessage.bit.Ecan_Error置位
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
	    ECanaShadow.CANMC.bit.CCR = 0;            // CLEAR CCR = 0,重新启动CCR模式
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
* Description: Ecan接收中断函数
**********************************************************************/
void SWI_ECANRXISR()
{
	MAIL	MailTmp;
	Uint8 errortemp = 0;
	errortemp = ECANErrorCheck();
	if (errortemp == 0)
	{
		if (ECanaRegs.CANGIF0.bit.MIV0 == 1) 		  //判断是否是1号邮箱接收中断
		{
//			__asm ("ESTOP0");
			MailTmp.Mailbox_id.all = ECanaMboxes.MBOX1.MSGID.all;
			MailTmp.Mailbox_data.DWord.CANL_Bytes = ECanaMboxes.MBOX1.MDL.all;
			MailTmp.Mailbox_data.DWord.CANH_Bytes = ECanaMboxes.MBOX1.MDH.all;

			ECANQueDataIn(pECANIndex->pqRx,MailTmp);
			ECanaRegs.CANRMP.all=0x00000002;
		}
		else if(ECanaRegs.CANGIF0.bit.MIV0 == 3)
		{
			EcanP2A_Rx.P2AMail_id.all = ECanaMboxes.MBOX3.MSGID.all;
			EcanP2A_Rx.P2AMail_data.DWord.CANL_Bytes = ECanaMboxes.MBOX3.MDL.all;
			EcanP2A_Rx.P2AMail_data.DWord.CANH_Bytes = ECanaMboxes.MBOX3.MDH.all;

			/*
			 * 旁路不需要接受广播信息
			 * Arbitrate(EcanP2A_Rx);
			 */
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

/**********************************************************************
* Function: ECANErrorCheck
*
* Description: eCAN发送通用程序
*
**********************************************************************/
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
	ECanaShadow.CANTA.bit.TA0 = 1;						//为防止赋值失败用到了影子寄存器
	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	//Step 2, initialize mailboxes
	//write data to boxes' RAM
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 0;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	ECanaMboxes.MBOX0.MSGID.all = Mail_Tx.Mailbox_id.all;			//设置邮箱发送的ID
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;					//config data length as 8-bytes

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;


	ECanaMboxes.MBOX0.MDH.all = Mail_Tx.Mailbox_data.DWord.CANH_Bytes;
	ECanaMboxes.MBOX0.MDL.all = Mail_Tx.Mailbox_data.DWord.CANL_Bytes;			//数据结构


	EALLOW;
//	__asm ("      ESTOP0");
	//Step 3, config TRS to apply for transmitting
	ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
	ECanaShadow.CANTRS.bit.TRS0=1;
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

//	__asm ("      ESTOP0");

	//Step 4, wait for response to complete transmition
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

	//Step 5,reset TA and transmition flag by writing 1 to corresponding register bits
	ECanaShadow.CANTA.all = 0;
//	__asm ("      ESTOP0");
	ECanaShadow.CANTA.bit.TA0 = 1;
	ECanaRegs.CANTA.all=ECanaShadow.CANTA.all;

	EDIS;
//*************PART I END**********************
}


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
	ECanaShadow.CANTA.bit.TA2 = 1;						//为防止赋值失败用到了影子寄存器
	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	//Step 2, initialize mailboxes
	//write data to boxes' RAM
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME2 = 0;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	ECanaMboxes.MBOX2.MSGID.all = P2A_Tx.P2AMail_id.all;			//设置邮箱发送的ID
	ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;					//config data length as 8-bytes

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME2 = 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;


	ECanaMboxes.MBOX2.MDH.all = P2A_Tx.P2AMail_data.DWord.CANH_Bytes;
	ECanaMboxes.MBOX2.MDL.all = P2A_Tx.P2AMail_data.DWord.CANL_Bytes;			//数据结构


	EALLOW;
//	__asm ("      ESTOP0");
	//Step 3, config TRS to apply for transmitting
	ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
	ECanaShadow.CANTRS.bit.TRS2=1;
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

//	__asm ("      ESTOP0");

	//Step 4, wait for response to complete transmition
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

	//Step 5,reset TA and transmition flag by writing 1 to corresponding register bits
	ECanaShadow.CANTA.all = 0;
//	__asm ("      ESTOP0");
	ECanaShadow.CANTA.bit.TA2 = 1;
	ECanaRegs.CANTA.all=ECanaShadow.CANTA.all;

	EDIS;
//*************PART I END**********************
}
