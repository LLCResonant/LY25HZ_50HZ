#ifndef SCI_H
#define SCI_H

typedef char							int8;
typedef unsigned char					Uint8;

/************************************************************************************
*Queue structure																	*
*************************************************************************************/
typedef struct{
	Uint8	*pIn;
	Uint8	*pOut;
	Uint8	*pStart;
	Uint16	u16Length;
	Uint16 	u16Size;
}QUEUE;
			
/********************************************************************************************
*Sci structure																		*
*Including  transmit and receive queue structure and Tx,Rx threshold control variables
***********************************************************************************************/
typedef struct{
	Uint8	u8TxStatus;
	Uint16	u16TxLength;
	QUEUE	*pqRx;
}SciStruct;

#define	OS_ENTER_CRITICAL()				asm("	SETC	INTM")
#define	OS_EXIT_CRITICAL()				asm("	CLRC	INTM")				

#define GET_HBYTE_OF_WORD(WORD)			((Uint8)((WORD) >> 8))
#define GET_LBYTE_OF_WORD(WORD)			((Uint8)((WORD) & 0x00FF))

#define ID_SCIA							0
#define ID_SCIB							1
#define ID_SCIC							2

#define FLASH_COM						ID_SCIB

#define MODE_INT						0
#define MODE_INQUIRE					1

#define QUE_BUF_NORMAL					0
#define QUE_BUF_FULL					1
#define QUE_BUF_EMPTY					2

#define	SCI_TX_RDY						0
#define	SCI_TX_BUSY						1

#define	SCI_RX_RDY						0
#define	SCI_RX_EMPTY					1

#define MAX_SCI_NO						3
#define SCIA_BUF_SIZE					128
#define SCIB_BUF_SIZE					128	//64
#define SCIC_BUF_SIZE					128	//256
#define MAX_SCI_BUF_SIZE				(SCIA_BUF_SIZE + SCIB_BUF_SIZE + SCIC_BUF_SIZE)

#define NULL							0

#define COMMUNICATION_BAUDRATE			9600
#define FALSH_PROGRAM_BAUDRATE			38400
#define MASTER_COMM_BAUDRATE			38400
#define TEST_COMM_BAUDRATE				9600

extern void InitScia(Uint32 u32BaudRate, Uint16 u16RxMode);
extern void InitScib(Uint32 u32BaudRate, Uint16 u16RxMode);
extern void InitScic(Uint32 u32BaudRate, Uint16 u16RxMode);
extern Uint16 SciRead(Uint16 SciId, Uint8 *pBuf);
extern Uint16 SciWrite(Uint16 SciId, Uint8 *pBuf, Uint16 u16Length);
#endif




