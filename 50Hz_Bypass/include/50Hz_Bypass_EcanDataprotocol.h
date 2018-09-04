/*
 *  Revised on: 2017.10.23 by Gao Xun
 */


#ifndef ECanAc_H
#define	ECanAc_H

//#define GET_4LBITS_OF_BTYE(WORD)			((Uint8)((WORD) & 0x0F))
//#define GET_4HBITS_OF_BTYE(WORD)			((Uint8)((WORD) & 0xF0))
//#define GET_LOWBTYE_FROM_DWORD(DWORD)			((Uint8)((DWORD) & 0x00000007))			

#define 	Computer_ADDRESS  		0x02					//��λ����ַ
#define 	Computer_TYPE 		    0x00					//��λ������
#define 	MODULE_TYPE 		    0x05					//ģ������
#define 	MODULEADDRESS 		    0x01					//ģ���ַ
extern Uint8   ModuleAdd;
extern Uint8  u8_hostdrop;
extern Uint8  ECAN_timer1;
extern Uint8 g_InvH_Load;
extern Uint8 g_InvL_Load;

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
		Uint8	Fault1:8;
		Uint8	Fault2:8;
		Uint8   Fault3:8;
	}Byte;
	struct
	{
		//Alert
		Uint8 VINVUderRating_A:1;  //���Ƿѹ
		Uint8 OverTemp_A:1;		//�¶ȳ��޸澯
		Uint8 Fan1Block_A:1;		//����1�쳣
		Uint8 Fan2Block_A:1;		//����2�쳣
		Uint8 Ecan_A:1;		//ģ�鲢��ͨѶ�澯
		Uint8 ACPowerDerating_A :1;	//�ﵽ�޶�澯
		Uint8 UnAsyn_A :1;//�������ͬ��
		Uint8 rsvr7 :1;

		//Fault1
		Uint8 VGridUnderRating_F:1;	//����Ƿѹ
		Uint8 VGridOverRating_F:1;	//�����ѹ
		Uint8 FreGridFault_F:1;			//����Ƶ���쳣
		Uint8 PFC_Recover_Fault_F:1;			//PFC�ɻָ�����
		Uint8 InvH_OVP_F:1;				//��������ѹ
		Uint8 InvL_OVP_F:1;				//�ֲ������ѹ
		Uint8 FreInv_Fault_F:1;			//���Ƶ���쳣
		Uint8 OverTemp_F:1;				//���¹ػ�
		//Fault2
		Uint8 Double_Fan_Fault_F:1;	//���ȹ��ϣ�������ͬʱ�쳣��
		Uint8 Launch_Fault_F:1;			//�����쳣
		Uint8 InvH_OverLoad_F:1;			//����������
		Uint8 InvL_OverLoad_F:1;			//�ֲ��������
		Uint8 Output_Shorted_F:1;			//����ʧ��
		Uint8 Curr_Shar_Fault_F:1;			//������������
		Uint8 PFC_Unrecover_Fault_F:1;	//PFC���ɻָ�����
		Uint8 Repeatedly_Launch_F:1;		//�����������
		//Fault3
		Uint8 SynLine_broken:1;//�����߶�
		Uint8 u8rsvr1:1;
		Uint8 u8rsvr2:1;
		Uint8 u8rsvr3:1;
		Uint8 u8rsvr4:1;
		Uint8 u8rsvr5:1;
		Uint8 u8rsvr6:1;
		Uint8 u8rsvr7:1;

	}bit;
}ECAN_MODULE_ERROR;
/*----------------------------ģ�鷢�������������֡����������м����------------------------------------*/
typedef struct
{
	Uint16 			Input_Volt_Rms;   //�����ѹ��Чֵ
	Uint16 			Input_Curr_Rms;  //Input current RMS
	Uint16      		BusP_Volt;		//BusP voltage

	Uint16      		BusN_Volt;		//BusN voltage
	Uint16				OutH_Volt_Rms;	//Trail load voltage RMS
	Uint16				OutL_Volt_Rms;	//Local load voltage RMS

	Uint16				InvH_Volt_Rms;	//��������ѹ��Чֵ
	Uint16				InvL_Volt_Rms;	//�ֲ������ѹ��Чֵ
	Uint16				InvH_Cur_Rms;	//������������Чֵ
	
	Uint16				InvL_Cur_Rms;	//�ֲ����������Чֵ
	Uint16				PFC_Temp;		//ģ���¶�
	Uint16				InvH_Temp;		//����¶�
	
	Uint16				InvL_Temp;		//�ֲ��¶�
	Uint16				InvH_Freq;		//���Ƶ��
	Uint16				InvL_Freq;			//�ֲ�Ƶ��

	Uint16				Phase_Lead;		//�ֲ���ǰ�������λ
	Uint16				RunTimeHour_L;
	Uint16				RunTimeHour_H;

	ECAN_MODULE_ERROR	Error;  //ģ����Ϻ;���
	Uint8				Check;	//У��
}ECAN_MODULE_DATA;
extern ECAN_MODULE_DATA Ecan_ModuleData;

/*------------------------------���������ģ��Ľ���֡����������м����------------------------------------*/
typedef struct
{
	Uint16 		Input_Volt_Rms;  //�����ѹ��Чֵ
	Uint16			Bus_P;		//PFC������ѹƽ��ֵ�����ƣ�
	Uint16			Bus_N;	//PFC������ѹƽ��ֵ��������
	Uint16			InvH_Volt_Rms;	//��������ѹ��Чֵ
	Uint16			InvL_Volt_Rms;	//�ֲ������ѹ��Чֵ
	Uint16			InvH_Cur_Rms;	//������������Чֵ
	Uint16			InvL_Cur_Rms;	//�ֲ����������Чֵ
	Uint16			InvH_OutV_Rms;	//������������Чֵ
	Uint16			InvL_OutV_Rms;	//�ֲ����������Чֵ
	Uint16			PFC_Temp;		//PFC�¶�
	Uint16 		InvH_Temp;	    //����¶�
	Uint16 		InvL_Temp;      //�ֲ��¶�
	Uint16 		Input_Cur_Rms;  //�����ѹ��Чֵ
	Uint16 		Aver_Curr_InvH; //���ƽ������
	Uint16 		Aver_Curr_InvL; //�ֲ�ƽ������
	Uint16 		RestartOverTimes;	//��������
	Uint16 		INVH_Volt_Ref; 	//Trail(220V) output voltage reference
	Uint16 		INVL_Volt_Ref;//Local(110V) output voltage reference
	Uint16 		INVH_Drop_Coeff; 	//Trail(220V) output voltage reference
	Uint16 		INVL_Drop_Coeff;//Local(110V) output voltage reference
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
