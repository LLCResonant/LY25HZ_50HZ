/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_DataAcquisition.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/
#ifndef 	Ecan_Basic_H
#define	Ecan_Basic_H

#define 	ECAN_QUE_BUF_NORMAL		0
#define 	ECAN_QUE_BUF_FULL				1
#define 	ECAN_QUE_BUF_EMPTY			2

#define	ECAN_TX_RDY						0
#define	ECAN_TX_BUSY					1

#define	ECAN_RX_RDY						0
#define	ECAN_RX_EMPTY					1
#define 	ECANA_BUF_SIZE					15
#define 	MAX_ECAN_BUF_SIZE			( ECANA_BUF_SIZE )

#define 	NULL									0

#define 	ECAN_COMMUNICATION_BAUDRATE			100000

#define 	ECAN_MAX_COMMAND_LENGTH 				50
#define 	ECAN_CHAR_ENTER 									13

/************************************************************************************
*MAIL structure																	*
*************************************************************************************/
/*========================Definition of Enhanced CAN ID (communicate with computer)============================*/
/*
 * __ __ __ |__  __ __ __ __    __ __ |__ __  __ __ __ __    |__ __ __ __  __ __ __ |__    __ __ __ __  __ |__ __ __
 * 31    29 28                        22 21                        15                      9   8                      3   2       0
 */
typedef union
{
	long all:32;
	struct
	{
		Uint32 byte_num:3;						//0~2 bit: the number of bytes
		Uint32 source_address:6;				//3~8 bit: the source address
		Uint32 source_type:7;					//9~15 bit: the source type
		Uint32 target_address:6;				//16~21 bit: the target address
		Uint32 target_type:7;					//22~28 bit: the target type
		Uint32 rsv3:2;								//29~30 bit: reserved bits and default 0
		Uint32 IDE:1;								//31bit: reserved bit and default 0
	}bit;
}MAILBOX_ID;

/*--------------------------------The definition of ECAN 8 bytes data frames------------------------------------*/
typedef union
{
	struct
	{
		Uint32 CANL_Bytes:32;
		Uint32 CANH_Bytes:32;
	}DWord;

	struct
	{
		Uint8 Current_frame:4;		//the sequence number of the current frame
		Uint8 Frames_num:4;   		//the number of frames contained in a data package
		Uint8 Code:8;					//Error Code
		Uint8 byte3:8;					//the first significant byte or data index number
		Uint8 byte4:8;
		Uint8 byte5:8;
		Uint8 byte6:8;
		Uint8 byte7:8;
		Uint8 byte8:8;					//the last significant byte or check code
	}Byte;
	
	struct
	{
		Uint16 Data0:16;
		Uint16 Data1:16;
		Uint16 Data2:16;
		Uint16 Data3:16;
	}Word;
}MAILBOX_DATA;
/*----------------------the definition of frame (4 control bytes+8 data bytes)--------------------------------------*/
typedef struct
{
	MAILBOX_ID 		Mailbox_id;
	MAILBOX_DATA    Mailbox_data;
}MAIL;


/************************************************************************************
*Queue structure																	*
*************************************************************************************/
typedef struct{
	MAIL	*pIn;
	MAIL	*pOut;
	MAIL	*pStart;
	Uint16	u16Length;
	Uint16 	u16Size;
}ECANQUEUE;

/*********************************************************************************************
*ECAN structure																		*
*Including transmiting and receiving queue structure and Tx, Rx threshold control variables
**********************************************************************************************/
typedef struct{
	Uint8	u8TxStatus;
	Uint16	u16TxLength;
	ECANQUEUE	*pqRx;
}ECANStruct;

/***********************************************************************************************
*P2PMAIL structure																		*
*Including  transmiting and receiving queue structure and Tx, Rx threshold control variables*
************************************************************************************************/
/*========================Definition of Enhanced CAN ID (communicate with other modules)============================*/
/*
 * __ __ __ |__  __ __ __ __    __ __ |__ __   __ __ __ __    |__ __ __ __  __ __ __ |__    __ __ __ __  __ |__ |__ __
 * 31    29 28                        22 21                           15                      9   8                       3   2   1   0
 */
typedef union
{
	long all:32;
	struct
	{
		Uint32 rsv2:2;								//0~1 bit£¬all set mean broadcast
		Uint32 bus:1;								//2 bit£¬bus bit£¬set means no host£¬clear means existing host
		Uint32 source_address:6;				//3~8 bit: the source address
		Uint32 source_type:7;					//9~15 bit: the source type
		Uint32 target_address:6;				//16~21 bit: the target address
		Uint32 target_type:7;					//22~28 bit: the target type
		Uint32 rsv3:2;								//29~30 bit: reserved bits and default 0
		Uint32 IDE:1;								//31 bit: reserved bit and default 0
	}bit;
}P2AMAIL_ID;
/*--------------------------------The definition of ECAN 8 bytes data frames------------------------------------*/
typedef union
{
	struct
	{
		Uint32 CANL_Bytes:32;
		Uint32 CANH_Bytes:32;
	}DWord;

	struct
	{
		Uint16 Word1:16;  //Reference of InvH
		Uint16 Word2:16;  //Reference of InvL
		Uint16 Word3:16;
		Uint16 Word4:16;
	}Word;

	struct
	{
		Uint8 byte1:8;
		Uint8 byte2:8;
		Uint8 byte3:8;
		Uint8 byte4:8;
		Uint8 byte5:8;
		Uint8 byte6:8;
		Uint8 byte7:8;
		Uint8 byte8:8;			//the last significant byte or check code
	}Byte;
}P2AMAIL_DATA;
/*--------------------------------the definition of frame (4 control bytes+8 data bytes)--------------------------------------*/
typedef struct
{
	P2AMAIL_ID 		P2AMail_id;
	P2AMAIL_DATA    P2AMail_data;
}P2AMAIL;
extern P2AMAIL EcanP2A_Tx, EcanP2A_Rx;

/*----------------------------------definition of ECAN transmit error register-----------------------------------------*/
typedef struct
{
	Uint8		u8Upload_Trans_Error;
	Uint8		u8Broadcast_Trans_Error;
}ECAN_ERROR;
extern ECAN_ERROR Ecan_Error;
/************************************************************************************
*Global Function																	*
*************************************************************************************/
extern Uint16 ECANRead(MAIL *pBuf);
extern Uint16 ECANWrite(MAIL *pBuf, Uint16 u16Length);
extern void SetECAN(MAIL *pStartAddr, Uint16 u16Size);
extern void InitECana(void);
extern void SWI_ECANRXISR(void);
extern void eCAN_Transmit(MAIL);
extern void eCAN_Broadcast(P2AMAIL);

#endif
