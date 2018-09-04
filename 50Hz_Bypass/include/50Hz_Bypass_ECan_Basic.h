/*
 * Original name: 15KW_LLC_EVCHAGER_eCAN.h
 *
 *  Created on: 2016��1��
 *      Author: SUN WEN JIN
 *  Revised on: 2017.10.23 by Gao Xun
 */


#ifndef Ecan_Basic_H
#define	Ecan_Basic_H

//#define	OS_ENTER_CRITICAL()				asm("	SETC	INTM")
//#define	OS_EXIT_CRITICAL()				asm("	CLRC	INTM")

//#define GET_HBYTE_OF_WORD(WORD)			((Uint8)((WORD) >> 8))
//#define GET_LBYTE_OF_WORD(WORD)			((Uint8)((WORD) & 0x00FF))


#define ECAN_QUE_BUF_NORMAL				0
#define ECAN_QUE_BUF_FULL				1
#define ECAN_QUE_BUF_EMPTY				2

#define	ECAN_TX_RDY						0
#define	ECAN_TX_BUSY					1

#define	ECAN_RX_RDY						0
#define	ECAN_RX_EMPTY					1
#define ECANA_BUF_SIZE					15
#define MAX_ECAN_BUF_SIZE				( ECANA_BUF_SIZE )

#define NULL							0

#define ECAN_COMMUNICATION_BAUDRATE			100000

#define ECAN_MAX_COMMAND_LENGTH 	50
#define ECAN_CHAR_ENTER 			13

//DryCrtl
#define DryCrtl_ON				GpioDataRegs.GPASET.bit.GPIO20 = 1;  //2018.1.22 GX ECAN test
#define DryCrtl_OFF				GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;

/************************************************************************************
*MAIL structure																	*
*************************************************************************************/
/*========================��ǿ��CANͨ��29λ֡��ʶ������============================*/
typedef union
{
	long all:32;
	struct
	{
		Uint32 byte_num:3;						//0~2λ�����ֽ���
		Uint32 source_address:6;				//3~8λԴ��ַ
		Uint32 source_type:7;					//9~15λԴ����
		Uint32 target_address:6;				//16~21λĿ���ַ
		Uint32 target_type:7;					//22~28λĿ������
		Uint32 rsv3:2;						//29~30λΪ����λ ����Ϊ0
		Uint32 IDE:1;						//31λΪ����λ ����Ϊ0
	}bit;
}MAILBOX_ID;

/*----------------------8�ֽ�CAN��������------------------------------------*/
typedef union
{
	struct
	{
		Uint32 CANL_Bytes:32;
		Uint32 CANH_Bytes:32;
	}DWord;

	struct
	{
		Uint8 Current_frame:4;		//��ǰ֡�����
		Uint8 Frames_num:4;   		//���ݰ���֡��
		Uint8 Code:8;				//������
		Uint8 byte3:8;					//��Ч���ݵ�һ���ֽڻ�������������
		Uint8 byte4:8;
		Uint8 byte5:8;
		Uint8 byte6:8;
		Uint8 byte7:8;
		Uint8 byte8:8;			//��Ч�������һ���ֽڻ���У���ֽ�
	}Byte;
	
	struct
	{
		Uint16 Data0:16;
		Uint16 Data1:16;
		Uint16 Data2:16;
		Uint16 Data3:16;
	}Word;
}MAILBOX_DATA;
/*----------------------֡����(4�ֽڿ���+8�ֽ�����)���ܹ�12�ֽ�����--------------------------------------*/
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

/************************************************************************************
*ECAN structure																		*
*Including  tranmit and receive queue structure and Tx,Rx threshold control variabls*
*************************************************************************************/
typedef struct{
	Uint8	u8TxStatus;
	Uint16	u16TxLength;
	ECANQUEUE	*pqRx;
}ECANStruct;

/************************************************************************************
*P2PMAIL structure																		*
*Including  tranmit and receive queue structure and Tx,Rx threshold control variabls*
*************************************************************************************/
/*========================��ǿ��CANͨ�ŵ�Ե�29λ֡��ʶ������============================*/
typedef union
{
	long all:32;
	struct
	{
		Uint32 rsv2:2;						//0~1λ��ȫ1�������ǹ㲥
		Uint32 bus:1;						//2λ��ĸ�ߣ�1����û��������0����������
		Uint32 source_address:6;				//3~8λԴ��ַ
		Uint32 source_type:7;					//9~15λԴ����
		Uint32 target_address:6;				//16~21λĿ���ַ
		Uint32 target_type:7;					//22~28λĿ������
		Uint32 rsv3:2;						//29~30λΪ����λ ����Ϊ0
		Uint32 IDE:1;						//31λΪ����λ ����Ϊ0
	}bit;
}P2AMAIL_ID;
/*----------------------8�ֽ�CAN��������------------------------------------*/
typedef union
{
	struct
	{
		Uint32 CANL_Bytes:32;  //�ֲ��ο�
		Uint32 CANH_Bytes:32;  //����ο�
	}DWord;

	struct
	{
		Uint16 Word1:16;  //�ֲ��ο�
		Uint16 Word2:16;  //����ο�
		Uint16 Word3:16;  //�ֲ��ο�
		Uint16 Word4:16;  //����ο�
	}Word;

	struct
	{
		Uint8 Address:8;		//�����Ƚϵ�ַ
		Uint8 byte2:8;
		Uint8 byte3:8;
		Uint8 byte4:8;
		Uint8 byte5:8;
		Uint8 byte6:8;
		Uint8 byte7:8;
		Uint8 byte8:8;			//��Ч�������һ���ֽڻ���У���ֽ�
	}Byte;
}P2AMAIL_DATA;
/*----------------------֡����(4�ֽڿ���+8�ֽ�����)���ܹ�12�ֽ�����--------------------------------------*/
typedef struct
{
	P2AMAIL_ID 		P2AMail_id;
	P2AMAIL_DATA    P2AMail_data;
}P2AMAIL;
extern P2AMAIL EcanP2A_Tx, EcanP2A_Rx;

/*----------------------------------���ʹ������������-------------------------------------------------------*/
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
