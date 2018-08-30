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
#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"


static SciStruct					SciList[MAX_SCI_NO];
static QUEUE						QList[MAX_SCI_NO];
static SciStruct					*pSciIndex[MAX_SCI_NO] = {NULL, NULL, NULL};
static Uint8						szSciRxBuf[MAX_SCI_BUF_SIZE];
static Uint8						*pSciBuf = szSciRxBuf;
static Uint8						u8SciNO = 0;
static volatile struct SCI_REGS		*pSciRegsIndex[MAX_SCI_NO] = {&SciaRegs,&ScibRegs,&ScicRegs};

void InitScia(Uint32 u32BaudRate, Uint16 u16RxMode);
void InitScib(Uint32 u32BaudRate, Uint16 u16RxMode);
void InitScic(Uint32 u32BaudRate, Uint16 u16RxMode);
Uint16 SciRead(Uint16 SciId, Uint8 *pBuf);
Uint16 SciWrite(Uint16 SciId, Uint8 *pBuf, Uint16 u16Length);

static void SetSci(Uint16 SciId, Uint8 *pStartAddr, Uint16 u16Size);
static void InitQueue(QUEUE *pQue, Uint8 *pStart, Uint16 u16BufSize);
static Uint16 QueDataIn(QUEUE *pQue, Uint16 u16QueData);
static Uint16 QueDataOut(QUEUE *pQue, Uint8 *pQueData);
static void SciErrorCheck(Uint16 SciId);

#pragma CODE_SECTION(SciWrite, "ControlLoopInRAM");

/**********************************************************************
* Function: SetSci()
*
* Description: 
**********************************************************************/
static void SetSci(Uint16 SciId, Uint8 *pStartAddr, Uint16 u16Size)
{
	SciStruct	*pSci;
	QUEUE		*pq;
	
	if( NULL == pSciIndex[SciId] )
	{
		pSci = &SciList[u8SciNO];
		pSciIndex[SciId] = pSci;

		pSci->pqRx = &QList[u8SciNO];
		pq = pSci->pqRx;
		InitQueue(pq, pSciBuf, u16Size);
		pSciBuf += u16Size;
		u8SciNO += 1;

		pSci->u8TxStatus = SCI_TX_RDY;
		pSci->u16TxLength = 0;
	}
}

/**********************************************************************
* Function: InitScia()
*
* Description: 
**********************************************************************/
void InitScia(Uint32 u32BaudRate, Uint16 u16RxMode)
{
    Uint16 u16Br;
    
    asm(" EALLOW");							// Enable EALLOW protected register access

	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = 3;  // Asynch input GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;   // Configure GPIO63 for SCITXDC operation

    asm(" EDIS");							// Disable EALLOW protected register access

	//----------------------------
    //--- Set parameters for SCI A
    //----------------------------


	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback // No parity,8 char bits, // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK, // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
//	SciaRegs.SCIHBAUD    =0x0001;     //150MHz
//	SciaRegs.SCILBAUD    =0x00e7;
	SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
	//scia_fifo_init=============================================================
	SciaRegs.SCIFFTX.all=0xC000;
	SciaRegs.SCIFFRX.all=0x0021;
	SciaRegs.SCIFFCT.all=0x0;
	SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
	SciaRegs.SCIFFTX.bit.TXFFIENA = 1;
   	
   	u16Br = ((LSPCLK_FREQ/u32BaudRate)/8)-1;
   	SciaRegs.SCIHBAUD = GET_HBYTE_OF_WORD(u16Br);	         
   	SciaRegs.SCILBAUD = GET_LBYTE_OF_WORD(u16Br);


		if(MODE_INT == u16RxMode)
   		{		    
			SciaRegs.SCIFFRX.bit.RXFFOVF = 1;
			SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
			
			PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// Enable SCIA_RXINT in PIE group 9
			IER |= M_INT9;							// Enable INT9 in IER to enable PIE group 9
		}
		else
		{
			SciaRegs.SCIFFRX.bit.RXFFOVF = 0;
			SciaRegs.SCIFFRX.bit.RXFFIENA = 0;
		}

/*
	SciaRegs.SCICCR.all = 0x0007;			// 1 stop bit, No loopback, No parity, async mode, 
   											// idle-line protocol, 8 char bits,  
   	SciaRegs.SCICTL1.all = 0x0003;  			// RX_err_INT disable, sleepMode disable, RX/TX enable
                                  			// Disable RX ERR, SLEEP, TXWAKE
   	SciaRegs.SCICTL2.bit.TXINTENA = 1;		// 0 = disable, 1 = enable TXRDY INT, in FIFO, this INT used as FIFO_Int
   	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;		// 1 = enable RXrdy/BRKINT, 0 = disable
   	SciaRegs.SCIFFCT.all = 0x00;
   	SciaRegs.SCICTL1.all = 0x0023;     		// Relinquish SCI from Reset, by software reset
   	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;	// enable TX
   	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;	// enable RX
	SciaRegs.SCIPRI.bit.FREE = 1; 			// free run enabled, 0 = soft emulation suspend, 1 = free run, 
   	SciaRegs.SCIFFTX.all = 0xE040;
   	
   	u16Br = ((LSPCLK_FREQ/u32BaudRate)/8)-1;
   	SciaRegs.SCIHBAUD = GET_HBYTE_OF_WORD(u16Br);	         
   	SciaRegs.SCILBAUD = GET_LBYTE_OF_WORD(u16Br);
   	
   	if(MODE_INT == u16RxMode)
   	{		    
		SciaRegs.SCIFFRX.all = 0x6061;
			
		PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// Enable SCIA_RXINT in PIE group 9
		IER |= M_INT9;							// Enable INT9 in IER to enable PIE group 9
	}
	else
	{
		SciaRegs.SCIFFRX.all = 0x2041;
	}
*/	
	SetSci(ID_SCIA, pSciBuf, SCIA_BUF_SIZE);



}

/**********************************************************************
* Function: InitScib()
*
* Description: 
**********************************************************************/
void InitScib(Uint32 u32BaudRate, Uint16 u16RxMode)
{
	Uint16 u16Br;
   
    asm(" EALLOW");
	
    GpioCtrlRegs.GPAPUD.bit.GPIO22= 0;    // Enable pull-up for GPIO22 (SCITXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)
  	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)
  	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation
  	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation

	asm(" EDIS");

   	ScibRegs.SCICCR.all = 0x0007;			// 1 stop bit, No loopback, No parity, async mode, 
   											// idle-line protocol, 8 char bits,  
   	ScibRegs.SCICTL1.all = 0x0003;  			// RX_err_INT disable, sleepMode disable, RX/TX enable
                                  			// Disable RX ERR, SLEEP, TXWAKE
//   	ScibRegs.SCICTL2.bit.TXINTENA = 1;		// 0 = disable, 1 = enable TXRDY INT, in FIFO, this INT used as FIFO_Int
   	ScibRegs.SCICTL2.all = 0;
   	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;		// 1 = enable RXrdy/BRKINT, 0 = disable  	

   	ScibRegs.SCIFFTX.all = 0xc000;			//0xE040;
   	ScibRegs.SCIFFRX.all = 0x0021;
   	ScibRegs.SCIFFCT.all = 0x00;
   	ScibRegs.SCICTL1.all = 0x0023;     		// Relinquish SCI from Reset, by software reset
   	ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;	// enable TX
   	ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;	// enable RX
//	ScibRegs.SCIPRI.bit.FREE = 1; 			// free run enabled, 0 = soft emulation suspend, 1 = free run,
	ScibRegs.SCIFFTX.bit.TXFFIENA = 1;

	u16Br = ((LSPCLK_FREQ/u32BaudRate)/8)-1;
	ScibRegs.SCIHBAUD = GET_HBYTE_OF_WORD(u16Br);	// BRR setting, H byte, when BaudRate=9600.	brr=0x01E7
   	ScibRegs.SCILBAUD = GET_LBYTE_OF_WORD(u16Br);			// BRR setting, L byte
	
	if(MODE_INT == u16RxMode)
		{
		ScibRegs.SCIFFRX.bit.RXFFOVF = 1;
		ScibRegs.SCIFFRX.bit.RXFFIENA = 1;

		PieCtrlRegs.PIEIER9.bit.INTx3 = 1;		// Enable SCIA_RXINT in PIE group 9
		IER |= M_INT9;							// Enable INT9 in IER to enable PIE group 9
	}
	else
	{
		ScibRegs.SCIFFRX.bit.RXFFOVF = 0;
		ScibRegs.SCIFFRX.bit.RXFFIENA = 0;
	}

	SetSci(ID_SCIB, pSciBuf, SCIB_BUF_SIZE);
}

/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : Sci.c
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/
void InitScic(Uint32 u32BaudRate, Uint16 u16RxMode)
{
    Uint16 u16Br;
    
    asm(" EALLOW");							// Enable EALLOW protected register access

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    	// Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   	// Enable pull-up for GPIO63 (SCITXDC)
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  	// Asynch input GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   	// Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   	// Configure GPIO63 for SCITXDC operation

	
    asm(" EDIS");							// Disable EALLOW protected register access

   	ScicRegs.SCICCR.all = 0x0007;			// 1 stop bit, No loopback, No parity, async mode, 
   											// idle-line protocol, 8 char bits,  
   	ScicRegs.SCICTL1.all = 0x0003;  			// RX_err_INT disable, sleepMode disable, RX/TX enable
                                  			// Disable RX ERR, SLEEP, TXWAKE
   	ScicRegs.SCICTL2.bit.TXINTENA = 1;		// 0 = disable, 1 = enable TXRDY INT, in FIFO, this INT used as FIFO_Int
   	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;		// 1 = enable RXrdy/BRKINT, 0 = disable
   	ScicRegs.SCIFFTX.all = 0xE040;		
   	ScicRegs.SCIFFCT.all = 0x00;
   	ScicRegs.SCICTL1.all = 0x0023;     		// Relinquish SCI from Reset, by software reset
   	ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;	// enable TX
   	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;	// enable RX
	ScicRegs.SCIPRI.bit.FREE = 1; 			// free run enabled, 0 = soft emulation suspend, 1 = free run, 

	u16Br = ((LSPCLK_FREQ/u32BaudRate)/8)-1;
	ScicRegs.SCIHBAUD = GET_HBYTE_OF_WORD(u16Br);	// BRR setting, H byte, when BaudRate=9600.	brr=0x01E7
   	ScicRegs.SCILBAUD = GET_LBYTE_OF_WORD(u16Br);			// BRR setting, L byte
	
	if(MODE_INT == u16RxMode)
   	{
		ScicRegs.SCIFFRX.all = 0x6061;
		PieCtrlRegs.PIEIER8.bit.INTx5 = 1;		  // Enable SCIC_RXINT in PIE group 8
		IER |= M_INT8;							  // Enable INT8 in IER to enable PIE group 8
	}
	else
	{
		ScibRegs.SCIFFRX.all = 0x2041;
	}
	
	SetSci(ID_SCIC, pSciBuf, SCIC_BUF_SIZE);
}


static void InitQueue(QUEUE *pQue, Uint8 *pStart, Uint16 u16BufSize)
{
	pQue->u16Length = 0;
	pQue->u16Size = u16BufSize;
	pQue->pIn = pStart;
	pQue->pOut = pStart;
	pQue->pStart = pStart;
}

static Uint16 QueDataIn(QUEUE *pQue, Uint16 u16QueData)
{
	if(pQue->u16Length == pQue->u16Size)
	{
		if(pQue->pIn == pQue->pStart)
		{
			*(pQue->pStart + pQue->u16Size - 1) = u16QueData;
		}
		else
		{
			*(pQue->pIn - 1) = u16QueData;
		}
		return QUE_BUF_FULL;
	}
	else
	{
		*(pQue->pIn) = u16QueData;
		pQue->u16Length += 1;
		if(pQue->pIn == pQue->pStart + pQue->u16Size - 1)
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

static Uint16 QueDataOut(QUEUE *pQue, Uint8 *pQueData)
{
	if(0 == pQue->u16Length)
	{
		return QUE_BUF_EMPTY;
	}
	else
	{
		*pQueData = *(pQue->pOut);
		pQue->u16Length -= 1;
		if(pQue->pOut == (pQue->pStart + pQue->u16Size - 1))
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

Uint16 SciRead(Uint16 SciId, Uint8 *pBuf)
{
	Uint16 		u16Tmp;
	QUEUE		*pq;
	SciStruct	*pSci;	
	
	SciErrorCheck(SciId);
	pSci = pSciIndex[SciId];
	pq = pSci->pqRx;
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

void SWI_SCIARXISR()
{
	Uint8 u8Tmp;
	u8Tmp = SciaRegs.SCIRXBUF.all;
	QueDataIn(pSciIndex[ID_SCIA]->pqRx,u8Tmp);
	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;  // Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
}

void SWI_SCIBRXISR()
{
	Uint8 u8Tmp;
	u8Tmp = ScibRegs.SCIRXBUF.all;
	QueDataIn(pSciIndex[ID_SCIB]->pqRx,u8Tmp);
	ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;  // Clear Overflow flag
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
}

void SWI_SCICRXISR()
{
	Uint8 u8Tmp;
	u8Tmp = ScicRegs.SCIRXBUF.all;
	QueDataIn(pSciIndex[ID_SCIC]->pqRx,u8Tmp);
	ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;  // Clear Overflow flag
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;
}

void SciErrorCheck(Uint16 SciId)
{
	if( pSciRegsIndex[SciId]->SCIRXST.bit.RXERROR == 1 )
	{
		pSciRegsIndex[SciId]->SCICTL1.bit.SWRESET = 0;
	}
}
