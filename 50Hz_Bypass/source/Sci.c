/***********************************************************************
 *	File: Sci.c 

 ************************************************************************/
#include "DSP2803x_Device.h"			// Peripheral address definitions
#include "F28035_example.h"


static SciStruct					SciList[MAX_SCI_NO];
static QUEUE						QList[MAX_SCI_NO];
static SciStruct					*pSciIndex[MAX_SCI_NO] = {NULL};
static Uint8						szSciRxBuf[MAX_SCI_BUF_SIZE];//  ����SCIͨѶ������֮��  
static Uint8						*pSciBuf = szSciRxBuf;//  ָ��ָ��û�����
static Uint8						u8SciNO = 0;
static volatile struct SCI_REGS		*pSciRegsIndex[MAX_SCI_NO] = {&SciaRegs};
//ָ��SCI�Ĵ�����ָ������

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

//  ��ڲ����� 1�� id��  scia=0  scib=1  scic=2
//             2)  Uint8 *pStartAddr   ������ʼ��ַ
//             3)  ���峤��  SCIA_BUF_SIZE					128 
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
*  SciId   ----id ��
*  Uint8 *pStartAddr------����ָ��
*  Uint16 u16Size--------������Ļ���������
**********************************************************************/
static void SetSci(Uint16 SciId, Uint8 *pStartAddr, Uint16 u16Size)
{
	SciStruct	*pSci;   // �ṹ�� ָ��
	QUEUE		*pq;     //  ���нṹ��  ָ��  
	
	if( NULL == pSciIndex[SciId] )   //  ID�Ŷ�Ϊ��
	{
		pSci = &SciList[u8SciNO];   // ���е���ʼ��ַ
		pSciIndex[SciId] = pSci;     // ��ӦID�ŵĽṹ��ָ�� 

		pSci->pqRx = &QList[u8SciNO];
		pq = pSci->pqRx;
		InitQueue(pq, pSciBuf, u16Size);
		pSciBuf += u16Size;
		u8SciNO += 1;                      // Ϊ��ʼ������SCI����

		pSci->u8TxStatus = SCI_TX_RDY;    //  ״̬��ʼ��
		pSci->u16TxLength = 0;             // ���ͳ���Ϊ0
	}
}


/**********************************************************************
* Function: Init_Scia()
*  ��PC�� ͨѶ  scia   ��ʹ�ú���InitScib()
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
	if(MODE_INT == u16RxMode)                   //  scia �Ľ����ж� 
   	{
		SciaRegs.SCIFFRX.all = 0x6061;
		PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// Enable SCIB_RXINT in PIE group 9
		IER |= M_INT9;							// Enable INT9 in IER to enable PIE group 9
	}
	else
	{
		SciaRegs.SCIFFRX.all = 0x2041;
	}
	
	SetSci(ID_SCIA, pSciBuf, SCIA_BUF_SIZE);   // ���� ���涨�����е���ʼ��ַ
}

/**********************************************************************
* Function:InitQueue   ָ��ָ���ʼ��ַ
*
* Description:   ���г�ʼ
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
* Description:   ����   ���������
**********************************************************************/
static Uint16 QueDataIn(QUEUE *pQue, Uint16 u16QueData)
{
	if(pQue->u16Length == pQue->u16Size) // �����д���
	{
		if(pQue->pIn == pQue->pStart) 
		{
			*(pQue->pStart + pQue->u16Size - 1) = u16QueData;// ��д��һ�����µ�����
		}
		else
		{
			*(pQue->pIn - 1) = u16QueData; // ��д��һ�����µ�����
		}
		return QUE_BUF_FULL;
	}
	else
	{
		*(pQue->pIn) = u16QueData;    //  ���ض�������
		pQue->u16Length += 1;            // ���������
		if(pQue->pIn == pQue->pStart + pQue->u16Size - 1)   //  �Ƿ���� 
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
* Description:   ����   ���ݳ�����
**********************************************************************/
static Uint16 QueDataOut(QUEUE *pQue, Uint8 *pQueData)
{
	if(0 == pQue->u16Length)
	{
		return QUE_BUF_EMPTY;
	}
	else
	{
		*pQueData = *(pQue->pOut);    //    ������
		pQue->u16Length -= 1;
		if(pQue->pOut == (pQue->pStart + pQue->u16Size - 1))   //  ָ���Ƿ�ָ�����
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
* Description:   SCI  ��ȡ����
**********************************************************************/
Uint16 SciRead(Uint16 SciId, Uint8 *pBuf)
{
	Uint16 		u16Tmp;
	QUEUE		*pq;
	SciStruct	*pSci;	
	
	SciErrorCheck(SciId);
	pSci = pSciIndex[SciId];
	pq = pSci->pqRx;        //  ����ָ��
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
* Description:   SCI д����
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
* Description:   SCI A �����жϷ����ӳ���
**********************************************************************/

void SWI_SCIARXISR()
{
	Uint8 u8Tmp;
	u8Tmp = SciaRegs.SCIRXBUF.all;
	QueDataIn(pSciIndex[ID_SCIA]->pqRx,u8Tmp);  //���ݽ�����
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
		QueDataIn(pSciIndex[ID_SCIA]->pqRx,u8Tmp);  //���ݽ�����
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
