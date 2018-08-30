/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_Scia_LCDs.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/

#ifndef 	SCIA_LCDs_H
#define	SCIA_LCDs_H

#define		COM_RCV_EN				GpioDataRegs.GPBCLEAR.bit.GPIO50=1;
#define		COM_SEND_EN				GpioDataRegs.GPBSET.bit.GPIO50=1;

#define   COM_CODE_WRITE_CONTROL    (0x06)  				//set writing control order
#define   COM_CODE_READ_CONTROL     (0x03)  				//set read control order
#define   COM_CODE_READ_STATE     	(0x04)  						//read running state parameters
#define   COM_CODE_READ_STATE_NEW   (0x74)
#define   COM_DELAY_TIME		    (0x1FF0)

unsigned int CRCB16(unsigned int  CRCini,unsigned int *ADDRini,unsigned int BITnum);
unsigned int CRCW16(unsigned int CRCini,unsigned int *ADDRini,unsigned int BITnum);
void ComSendByte(unsigned int data);
void ComSendWords(unsigned int *addr,unsigned int data);
void ComRcvDeal(void);

//void ComWriteOnce(void);

void ComOverTimeDeal(void);

struct COMFLAG_BITS
{
	Uint16	SciRcvStart:1;
	Uint16	OverTimeFlag:1;
	Uint16	OverTimeDelay:9;
};

struct COMCNT 
{
    Uint16 ComRecByteNum;
    Uint16 SaveFactorySettings;
};

extern struct COMFLAG_BITS ComFlag;
extern struct COMCNT ComCnt;

extern Uint16 ComRcvData[10]; 
extern Uint16 ComSendData[10];

extern void SetValue_StateRefresh(void);
extern void SetVaule_ControlRefresh(void);
extern void GetVaule_ControlRefresh(Uint16 address);

#endif
