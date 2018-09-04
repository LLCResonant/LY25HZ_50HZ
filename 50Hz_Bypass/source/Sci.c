/***********************************************************************
 *	File: Sci.c 

 ************************************************************************/
#include "DSP2803x_Device.h"			// Peripheral address definitions
#include "F28035_example.h"


static SciStruct					SciList[MAX_SCI_NO];
static QUEUE						QList[MAX_SCI_NO];
static SciStruct					*pSciIndex[MAX_SCI_NO] = {NULL};
static Uint8						szSciRxBuf[MAX_SCI_BUF_SIZE];//  三种SCI通讯缓冲区之和  
static Uint8						*pSciBuf = szSciRxBuf;//  指针指向该缓冲区
static Uint8						u8SciNO = 0;
static volatile struct SCI_REGS		*pSciRegsIndex[MAX_SCI_NO] = {&SciaRegs};
//指向SCI寄存器的指针数组

void Init_Scia(Uint32 u32BaudRate, Uint16 u16RxMode);

Uint16 SciRead(Uint16 SciId, Uint8 *pBuf);
Uint16 SciWrite(Uint16 SciId, Uint8 *pBuf, Uint16 u16Length);

static void SetSci(Uint16 SciId, Uint8 *pStartAddr, Uint16 u16Size);
static void InitQueue(QUEUE *pQue, Uint8 *pStart, Uint16 u16BufSize);
static Uint16 QueDataIn(QUEUE *pQue, Uint16 u16QueData);
static Uint16 QueDataOut(QUEUE *pQue, Uint8 *pQueData);
static void SciErrorCheck(Uint16 SciId);

//#pragma CODE_SECTION(SciWrite, "ControlLoopInRAM");
/**********************************************************************
* Function: SetSci()
*
* Description: 

//  入口参数： 1） id号  scia=0  scib=1  scic=2
//             2)  Uint8 *pStartAddr   缓冲起始地址
//             3)  缓冲长度  SCIA_BUF_SIZE					128 
//                          SCIB_BUF_SIZE					64
//                         SCIC_BUF_SIZE					256


typedef struct{
	Uint8	*pIn;
	Uint8	*pOut;
	Uint8	*pStart;
	Uint16	u16Length;
	Uint16 	u16Size;
}QUEUE;
			

typedef struct{
	Uint8	u8TxStatus;
	Uint16	u16TxLength;
	QUEUE	*pqRx;
}SciStruct;

* Function: SetSci()
*
* Description: 
*  SciId   ----id 号
*  Uint8 *pStartAddr------缓冲指针
*  Uint16 u16Size--------所申请的缓冲区长度
**********************************************************************/
static void SetSci(Uint16 SciId, Uint8 *pStartAddr, Uint16 u16Size)
{
	SciStruct	*pSci;   // 结构体 指针
	QUEUE		*pq;     //  队列结构体  指针  
	
	if( NULL == pSciIndex[SciId] )   //  ID号都为零
	{
		pSci = &SciList[u8SciNO];   // 队列的起始地址
		pSciIndex[SciId] = pSci;     // 相应ID号的结构体指针 

		pSci->pqRx = &QList[u8SciNO];
		pq = pSci->pqRx;
		InitQueue(pq, pSciBuf, u16Size);
		pSciBuf += u16Size;
		u8SciNO += 1;                      // 为初始化三个SCI服务

		pSci->u8TxStatus = SCI_TX_RDY;    //  状态初始化
		pSci->u16TxLength = 0;             // 发送长度为0
	}
}


/**********************************************************************
* Function: Init_Scia()
*  与PC机 通讯  scia   但使用函数InitScib()
* Description: 
**********************************************************************/
void Init_Scia(Uint32 u32BaudRate, Uint16 u16RxMode)
{
	Uint16 u16Br;
   
    asm(" EALLOW");
	
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up for GPIO29 (SCITXDA)
    GpioCtrlRegs.GPAPUD.bit.GPIO28= 0;    // Enable pull-up for GPIO28 (SCIRXDA)
  	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
  	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
  	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation

	asm(" EDIS");

   	SciaRegs.SCICCR.all = 0x0007;			// 1 stop bit, No loopback, No parity, async mode, 
   											// idle-line protocol, 8 char bits,  
   	SciaRegs.SCICTL1.all = 0x0003;  			// RX_err_INT disable, sleepMode disable, RX/TX enable
                                  			// Disable RX ERR, SLEEP, TXWAKE
   	SciaRegs.SCICTL2.bit.TXINTENA = 1;		// 0 = disable, 1 = enable TXRDY INT, in FIFO, this INT used as FIFO_Int
   	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;		// 1 = enable RXrdy/BRKINT, 0 = disable  	
   	SciaRegs.SCIFFTX.all = 0xE040;		
   	SciaRegs.SCIFFCT.all = 0x00;
   	SciaRegs.SCICTL1.all = 0x0023;     		// Relinquish SCI from Reset, by software reset
   	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;	// enable TX
   	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;	// enable RX
	SciaRegs.SCIPRI.bit.FREE = 1; 			// free run enabled, 0 = soft emulation suspend, 1 = free run, 

	u16Br = ((LSPCLK_FREQ/u32BaudRate)/8)-1;
	SciaRegs.SCIHBAUD = GET_HBYTE_OF_WORD(u16Br);	// BRR setting, H byte, when BaudRate=9600.	brr=0x01E7
   	SciaRegs.SCILBAUD = GET_LBYTE_OF_WORD(u16Br);			// BRR setting, L byte
//	SciaRegs.SCIHBAUD=0x0000;
// SciaRegs.SCILBAUD=0x00c2;
	if(MODE_INT == u16RxMode)                   //  scia 的接受中断 
   	{
		SciaRegs.SCIFFRX.all = 0x6061;
		PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// Enable SCIB_RXINT in PIE group 9
		IER |= M_INT9;							// Enable INT9 in IER to enable PIE group 9
	}
	else
	{
		SciaRegs.SCIFFRX.all = 0x2041;
	}
	
	SetSci(ID_SCIA, pSciBuf, SCIA_BUF_SIZE);   // 清零 并规定各队列的起始地址
}

/**********************************************************************
* Function:InitQueue   指针指向初始地址
*
* Description:   队列初始
**********************************************************************/
static void InitQueue(QUEUE *pQue, Uint8 *pStart, Uint16 u16BufSize)
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
static Uint16 QueDataIn(QUEUE *pQue, Uint16 u16QueData)
{
	if(pQue->u16Length == pQue->u16Size) // 满队列处理
	{
		if(pQue->pIn == pQue->pStart) 
		{
			*(pQue->pStart + pQue->u16Size - 1) = u16QueData;// 改写上一次最新的数据
		}
		else
		{
			*(pQue->pIn - 1) = u16QueData; // 改写上一次最新的数据
		}
		return QUE_BUF_FULL;
	}
	else
	{
		*(pQue->pIn) = u16QueData;    //  返回队列已满
		pQue->u16Length += 1;            // 正常入队列
		if(pQue->pIn == pQue->pStart + pQue->u16Size - 1)   //  是否到最后 
		{
			pQue->pIn = pQue->pStart;
		}
		else
		{
			pQue->pIn += 1;
		}
		return QUE_BUF_NORMAL;
	}
}
/**********************************************************************
* Function: QueDataOut
*
* Description:   队列   数据出队列
**********************************************************************/
static Uint16 QueDataOut(QUEUE *pQue, Uint8 *pQueData)
{
	if(0 == pQue->u16Length)
	{
		return QUE_BUF_EMPTY;
	}
	else
	{
		*pQueData = *(pQue->pOut);    //    出队列
		pQue->u16Length -= 1;
		if(pQue->pOut == (pQue->pStart + pQue->u16Size - 1))   //  指针是否指向最后
		{
			pQue->pOut = pQue->pStart;
		}
		else
		{
			pQue->pOut += 1;
		}
		return QUE_BUF_NORMAL;
	}
}
/**********************************************************************
* Function: SciRead
*
* Description:   SCI  读取数据
**********************************************************************/
Uint16 SciRead(Uint16 SciId, Uint8 *pBuf)
{
	Uint16 		u16Tmp;
	QUEUE		*pq;
	SciStruct	*pSci;	
	
	SciErrorCheck(SciId);
	pSci = pSciIndex[SciId];
	pq = pSci->pqRx;        //  队列指针
	//OS_ENTER_CRITICAL();
	u16Tmp = QueDataOut(pq, pBuf);
	//OS_EXIT_CRITICAL();
	if(u16Tmp == QUE_BUF_EMPTY)
	{
		return SCI_RX_EMPTY;
	}
	else
	{
		return SCI_RX_RDY;
	}
}
/**********************************************************************
* Function: SciWrite
*
* Description:   SCI 写数据
**********************************************************************/
Uint16 SciWrite(Uint16 SciId, Uint8 *pBuf, Uint16 u16Length)
{
	Uint16 i;
	SciStruct		*pSci;
	 
	pSci = pSciIndex[SciId];	
	if(pSci->u8TxStatus == SCI_TX_BUSY)
	{
		return SCI_TX_BUSY;
	}
	
	//OS_ENTER_CRITICAL();
	pSci->u16TxLength = u16Length;
	pSci->u8TxStatus = SCI_TX_BUSY;	
	for(i = 0;i < u16Length;i++)
	{
		while(pSciRegsIndex[SciId]->SCIFFTX.bit.TXFFST != 0) 
		{ 
		}
    	pSciRegsIndex[SciId]->SCITXBUF = *(pBuf + i);
	}
	pSci->u8TxStatus = SCI_TX_RDY;
	//OS_EXIT_CRITICAL();
	return SCI_TX_RDY;
}


/**********************************************************************
* Function: SWI_SCIARXISR
*
* Description:   SCI A 接受中断服务子程序
**********************************************************************/

void SWI_SCIARXISR()
{
	Uint8 u8Tmp;
	u8Tmp = SciaRegs.SCIRXBUF.all;
	QueDataIn(pSciIndex[ID_SCIA]->pqRx,u8Tmp);  //数据进队列
	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;  // Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
}

void SWI_SCILINRXISR()    //GX LIN
{
	Uint8 u8Tmp, LinL0IntVect;
	LinL0IntVect = LinaRegs.SCIINTVECT0.all & 0xf;
	if (LinL0IntVect == 11)
	{
		u8Tmp = LinaRegs.SCIRD & 0xff;
		QueDataIn(pSciIndex[ID_SCIA]->pqRx,u8Tmp);  //数据进队列
	}
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
}

void SciErrorCheck(Uint16 SciId)
{
	if( pSciRegsIndex[SciId]->SCIRXST.bit.RXERROR == 1 )
	{
		pSciRegsIndex[SciId]->SCICTL1.bit.SWRESET = 0;
	}
}
