/*
 *  Revised on: 2017.10.23 by Gao Xun
 */


#ifndef ECanAc_H
#define	ECanAc_H

#define 	Computer_ADDRESS  		0x02					//��λ����ַ
#define 	Computer_TYPE 		    0x00					//��λ������
#define 	MODULE_TYPE 		    0x05					//ģ������
#define MODULEADDRESS	GpioDataRegs.AIODAT.all			//ģ���ַ

#define	DATA_NORMAL      0x00	//��������
#define	DATA_OVERLENGTH  0x80	//���ݳ���
#define	CHECK_ERROR      0x90	//У�����
#define	DATABAG_LOST     0xa0	//���ݰ�������
#define	OTHERS           0xb0	//����


#define	DATA_UPLOAD          0x00		//�����ϴ�
#define	SINGLEDATA_REVISED   0x01		//��������У��
#define	MULTIDATA_REVISED    0x02		//�������У��
#define	SINGLEDATA_SET       0x03		//������������
#define	MULTIDATA_SET        0x04		//�����������
#define	SINGLEDATA_CONFIG    0x05		//������������
#define	MULTIDATA_CONFIG     0x06		//�����������
#define	LightLoad      0x01
#define	MiddleLoad   0x02
#define	HeavyLoad    0x03


/************************************************************************************
*ECAN intermediate data list*
*Include module to computer data and computer to module order
*************************************************************************************/
/*----------------------------����͹��϶���--------------------------------------*/
typedef union
{
	struct
	{
		Uint8 	Alert:8;
	}Byte;
	struct
	{
		//Alert
		Uint8 OverTemp:1;
		Uint8 Fan1Block:1;
		Uint8 Fan2Block:1;
		Uint8 Fan_Fault:1;
		Uint8 :1;
		Uint8 :1;
		Uint8 :1;
		Uint8 :1;
	}bit;
}ECAN_MODULE_Alert;

typedef union
{
	struct
	{
		Uint8	Fault1:8;
		Uint8   	Fault2:8;
	}Byte;
	struct
	{
		//Fault1
		Uint8 VGridUnderRating:1;
		Uint8 VGridOverRating:1;
		Uint8 FreGridFault:1;
		Uint8 OverTempFault:1;
		Uint8 Bypass_SCR_Fault:1;
		Uint8 Inv_SCR_Fault:1;
		Uint8 Launch_Fault:1;
		Uint8 :1;
		//Fault2
		Uint8 :1;
		Uint8 :1;
		Uint8 u8rsvr2:1;
		Uint8 u8rsvr3:1;
		Uint8 u8rsvr4:1;
		Uint8 u8rsvr5:1;
		Uint8 u8rsvr6:1;
		Uint8 u8rsvr7:1;
	}bit;
}ECAN_MODULE_Fault;
/*----------------------------ģ�鷢�������������֡����������м����------------------------------------*/
typedef struct
{
	Uint16 			u16VGrid_rms;   //�����ѹ��Чֵ
	Uint16 			u16VGrid_Freq;
	Uint16 			u16IGrid_rms;  //Input current RMS
	Uint16			u16VOut_rms;	//Trail load voltage RMS
	Uint16			u16Temperature;		//ģ���¶�
	Uint16			u16RunTimeu16Hour_L;
	Uint16			u16RunTimeu16Hour_H;

	ECAN_MODULE_Alert	Alert;
	ECAN_MODULE_Fault	Fault;  //ģ����Ϻ;���
	Uint8				Check;	//У��
}ECAN_MODULE_DATA;
extern ECAN_MODULE_DATA Ecan_ModuleData;

/*------------------------------���������ģ��Ľ���֡����������м����------------------------------------*/
typedef struct
{
	Uint16 			u16VGrid_rms;  //�����ѹ��Чֵ
	Uint16			u16VOut_rms;	//�����ѹ��Чֵ
	Uint16 			u16Temperature;	    //����¶�
	Uint16 			u16IGrid_rms;  //�����ѹ��Чֵ
}ECAN_REVISED;
extern ECAN_REVISED Ecan_SytemREVISED;

/*------------------------------���������ģ�������֡����������м����------------------------------------*/
typedef struct
{
	Uint16 		rsvr0;
	Uint16		Defaluts;		//�ָ�Ĭ��У��ֵ
	Uint16		Output_Enable;	//ģ�������ѹ����/ֹͣ
	Uint16 		rsvr1;
}ECAN_ORDER;
extern ECAN_ORDER Ecan_SysParaCalibration;

enum   MODULE_STATUS
{
 idle, Master, Slave
} ;
extern enum	MODULE_STATUS	g_Mod_Status;

extern Uint8 ModuleAdd;
extern void Arbitrate(P2AMAIL);
extern void Broadcast(void);
extern void Output_Revise (void);

#endif
