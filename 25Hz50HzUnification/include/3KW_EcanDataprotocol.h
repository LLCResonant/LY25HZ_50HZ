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

#ifndef 	ECanAc_H
#define	ECanAc_H

#define 	Computer_Address  		0x02
#define 	Computer_Type 		    0x00
#define 	Module_Type 		    	LY25HZ
#define 	Module_Address 		    0x01
extern Uint8   	ModuleAdd;
extern Uint8  	u8_hostdrop;
extern Uint8 	g_InvH_Load;
extern Uint8 	g_InvL_Load;

#define	DATA_NORMAL      		0x00
#define	DATA_OVERLENGTH  	0x80
#define	CHECK_ERROR      		0x90
#define	DATAB_LOST     			0xa0
#define	OTHERS           				0xb0

#define	DATA_UPLOAD          		0x00
#define	SINGLEDATA_REVISED   	0x01
#define	MULTIDATA_REVISED    	0x02
#define	SINGLEDATA_SET       		0x03
#define	MULTIDATA_SET        		0x04
#define	SINGLEDATA_CONFIG    	0x05
#define	MULTIDATA_CONFIG     	0x06
#define	COEFF_FEEDBACK    		0x07
#define	LightLoad      0x01
#define	MiddleLoad   0x02
#define	HeavyLoad    0x03

/************************************************************************************
*ECAN intermediate data list*
*Include module to computer data and computer to module order
*************************************************************************************/
/*------------------------------the definition of Alert and Fault--------------------------------------*/
typedef union
{
	struct
	{
		Uint8 	Alert:8;
	}Byte;
	struct
	{
		//Alert
		Uint8 VInvUnderRating:1;
		Uint8 OverTemp:1;
		Uint8 Fan1Block:1;
		Uint8 Fan2Block:1;
		Uint8 :1;
		Uint8 InvOverLoad :1;
		Uint8 InvAsyn :1;
		Uint8 rsvr7 :1;
	}bit;
}ECAN_MODULE_Alert;
typedef union
{
	struct
	{
		Uint8	Fault1:8;
		Uint8	Fault2:8;
		Uint8   	Fault3:8;
	}Byte;
	struct
	{
		//Fault1
		Uint8 VGridUnderRating:1;
		Uint8 VGridOverRating:1;
		Uint8 FreGridFault:1;
		Uint8 PFC_Recover_Fault:1;
		Uint8 InvH_OVP:1;
		Uint8 InvL_OVP:1;
		Uint8 FreInv_Fault:1;
		Uint8 OverTempFault:1;
		//Fault2
		Uint8 Fan_Fault:1;
		Uint8 Launch_Fault:1;
		Uint8 InvH_OverLoad:1;
		Uint8 InvL_OverLoad:1;
		Uint8 Unrecover_InvOCP:1;
		Uint8 Unrecover_InvCurrShar_Fault:1;
		Uint8 PFC_Unrecover_Fault:1;
		Uint8 Unrecover_RestartNum:1;
		//Fault3
		Uint8 SynLine_Cut:1;
		Uint8 ECAN_Fault:1;
		Uint8 u8rsvr2:1;
		Uint8 u8rsvr3:1;
		Uint8 u8rsvr4:1;
		Uint8 u8rsvr5:1;
		Uint8 u8rsvr6:1;
		Uint8 u8rsvr7:1;

	}bit;
}ECAN_MODULE_Fault;
/*----------------------------The intermediate variable of the data frame sent by a module to a computer------------------------------------*/
typedef struct
{
	Uint16 			u16VGrid_rms;
	Uint16 			u16IGrid_rms;
	Uint16      	u16VBusP;

	Uint16      	u16VBusN;
	Uint16			u16VOutH_rms;
	Uint16			u16VOutL_rms;

	Uint16			u16VInvH_rms;
	Uint16			u16VInvL_rms;
	Uint16			u16IInvH_rms;

	Uint16			u16IInvL_rms;
	Uint16			u16Temp_PFC;
	Uint16			u16Temp_InvH;

	Uint16			u16Temp_InvL;
	Uint16			u16VOutH_Freq;
	Uint16			u16VOutL_Freq;

	Uint16			u16Phase_Diff;
	Uint16			u16RunTimeHour_L;
	Uint16			u16RunTimeHour_H;

	ECAN_MODULE_Alert	Alert;
	ECAN_MODULE_Fault	Fault;
	Uint8				Check;
}ECAN_MODULE_DATA;
extern ECAN_MODULE_DATA Ecan_ModuleData;

/*------------------------------The intermediate variable of the data frame sent by a computer to a module------------------------------------*/
typedef struct
{
	Uint16 		u16VGrid_rms;
	Uint16		u16VBusP;
	Uint16		u16VBusN;
	Uint16		u16VInvH_rms;
	Uint16		u16VInvL_rms;
	Uint16		u16IInvH_rms;
	Uint16		u16IInvL_rms;
	Uint16		u16VOutH_rms;
	Uint16		u16VOutL_rms;
	Uint16		u16Temp_PFC;
	Uint16 		u16Temp_InvH;
	Uint16 		u16Temp_InvL;
	Uint16 		u16IGrid_rms;
	Uint16 		u16IInvH_para_ave;
	Uint16 		u16IInvL_para_ave;
	Uint16 		u16RestartTimes;
	Uint16 		u16InvH_VoltRms_Ref;
	Uint16 		u16InvL_VoltRms_Ref;
	Uint16 		u16VInvH_Comp_Coeff; //Current sharing compensation coefficient
	Uint16 		u16VInvL_Comp_Coeff;//Current sharing compensation coefficient
}ECAN_REVISED;
extern ECAN_REVISED Ecan_SysParaCalibration;

/*------------------------------The intermediate variable of the control frame sent by a computer to a module------------------------------------*/
typedef struct
{
	Uint16 		rsvr0;
	Uint16		Defaluts;
	Uint16		Output_Enable;
	Uint16 		rsvr1;
}ECAN_ORDER;
extern ECAN_ORDER Ecan_SytemOrder;

enum   MODULE_STATUS
{
 idle, Master, Slave
} ;
extern enum	MODULE_STATUS	g_Mod_Status;

extern void Arbitrate(P2AMAIL);
extern void Broadcast(void);
extern void Output_Revise (void);

#endif
