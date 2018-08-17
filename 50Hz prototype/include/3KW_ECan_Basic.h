/*
 * Original name: 15KW_LLC_EVCHAGER_eCAN.h
 *
 *  Created on: 2016年1月
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
/*========================增强型CAN通信29位帧标识符定义============================*/
/*
 * __ __ __ |__  __ __ __ __    __ __ |__ __  __ __ __ __    |__ __ __ __  __ __ __ |__    __ __ __ __  __ |__ __ __
 * 31    29 28                        22 21                        15                      9   8                      3   2       0
 */
typedef union
{
	long all:32;
	struct
	{
		Uint32 byte_num:3;						//0~2位包含字节数
		Uint32 source_address:6;				//3~8位源地址
		Uint32 source_type:7;					//9~15位源类型
		Uint32 target_address:6;				//16~21位目标地址
		Uint32 target_type:7;					//22~28位目标类型
		Uint32 rsv3:2;						//29~30位为保留位 设置为0
		Uint32 IDE:1;						//31位为保留位 设置为0
	}bit;
}MAILBOX_ID;

/*----------------------8字节CAN数据域定义------------------------------------*/
typedef union
{
	struct
	{
		Uint32 CANL_Bytes:32;
		Uint32 CANH_Bytes:32;
	}DWord;

	struct
	{
		Uint8 Current_frame:4;		//当前帧的序号
		Uint8 Frames_num:4;   		//数据包总帧数
		Uint8 Code:8;				//错误码
		Uint8 byte3:8;					//有效数据第一个字节或是数据索引号
		Uint8 byte4:8;
		Uint8 byte5:8;
		Uint8 byte6:8;
		Uint8 byte7:8;
		Uint8 byte8:8;			//有效数据最后一个字节或是校验字节
	}Byte;
	
	struct
	{
		Uint16 Data0:16;
		Uint16 Data1:16;
		Uint16 Data2:16;
		Uint16 Data3:16;
	}Word;
}MAILBOX_DATA;
/*----------------------帧定义(4字节控制+8字节数据)，总共12字节数据--------------------------------------*/
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
/*========================增强型CAN通信点对点29位帧标识符定义============================*/
/*
 * __ __ __ |__  __ __ __ __    __ __ |__ __  __ __ __ __    |__ __ __ __  __ __ __ |__    __ __ __ __  __ |__ |__ __
 * 31    29 28                        22 21                        15                      9   8                      3   2   1   0
 */
typedef union
{
	long all:32;
	struct
	{
		Uint32 rsv2:2;						//0~1位，全1代表这是广播
		Uint32 bus:1;						//2位，母线，1代表没有主机，0代表有主机
		Uint32 source_address:6;				//3~8位源地址
		Uint32 source_type:7;					//9~15位源类型
		Uint32 target_address:6;				//16~21位目标地址
		Uint32 target_type:7;					//22~28位目标类型
		Uint32 rsv3:2;						//29~30位为保留位 设置为0
		Uint32 IDE:1;						//31位为保留位 设置为0
	}bit;
}P2AMAIL_ID;
/*----------------------8字节CAN数据域定义------------------------------------*/
typedef union
{
	struct
	{
		Uint32 CANL_Bytes:32;  //局部参考
		Uint32 CANH_Bytes:32;  //轨道参考
	}DWord;

	struct
	{
		Uint16 Word1:16;  //局部参考
		Uint16 Word2:16;  //轨道参考
		Uint16 Word3:16;  //局部参考
		Uint16 Word4:16;  //轨道参考
	}Word;

	struct
	{
		Uint8 Address:8;		//本机比较地址
		Uint8 byte2:8;
		Uint8 byte3:8;
		Uint8 byte4:8;
		Uint8 byte5:8;
		Uint8 byte6:8;
		Uint8 byte7:8;
		Uint8 byte8:8;			//有效数据最后一个字节或是校验字节
	}Byte;
}P2AMAIL_DATA;
/*----------------------帧定义(4字节控制+8字节数据)，总共12字节数据--------------------------------------*/
typedef struct
{
	P2AMAIL_ID 		P2AMail_id;
	P2AMAIL_DATA    P2AMail_data;
}P2AMAIL;
extern P2AMAIL EcanP2A_Tx, EcanP2A_Rx;

/*----------------------------------发送错误计数器定义-------------------------------------------------------*/
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
