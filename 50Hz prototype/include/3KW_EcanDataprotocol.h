/*
 *  Revised on: 2017.10.23 by Gao Xun
 */


#ifndef ECanAc_H
#define	ECanAc_H

//#define GET_4LBITS_OF_BTYE(WORD)			((Uint8)((WORD) & 0x0F))
//#define GET_4HBITS_OF_BTYE(WORD)			((Uint8)((WORD) & 0xF0))
//#define GET_LOWBTYE_FROM_DWORD(DWORD)			((Uint8)((DWORD) & 0x00000007))			

#define 	Computer_ADDRESS  		0x02					//上位机地址
#define 	Computer_TYPE 		    0x00					//上位机类型
#define 	MODULE_TYPE 		    0x05					//模块类型25hZ是5 50Hz是7，这里因为借用25Hz ecan校正程序所以为5
#define 	MODULEADDRESS 		    0x01					//模块地址
extern Uint8   ModuleAdd;
extern Uint8  u8_hostdrop;
extern Uint8  ECAN_timer1;
extern Uint8 g_Inv_Load;

#define	DATA_NORMAL      0x00	//数据正常
#define	DATA_OVERLENGTH  0x80	//数据超限
#define	CHECK_ERROR      0x90	//校验错误
#define	DATABAG_LOST     0xa0	//数据包不完整
#define	OTHERS           0xb0	//其他


#define	DATA_UPLOAD          0x00		//数据上传
#define	SINGLEDATA_REVISED   0x01		//单个数据校正
#define	MULTIDATA_REVISED    0x02		//多个数据校正
#define	SINGLEDATA_SET       0x03		//单个数据设置
#define	MULTIDATA_SET        0x04		//多个数据设置
#define	SINGLEDATA_CONFIG    0x05		//单个数据配置
#define	MULTIDATA_CONFIG     0x06		//多个数据配置
#define	COEFF_FEEDBACK    0x07		//多个数据配置
#define	LightLoad      0x01
#define	MiddleLoad   0x02
#define	HeavyLoad    0x03

/************************************************************************************
*ECAN intermediate data list*
*Include module to computer data and computer to module order
*************************************************************************************/
/*----------------------------警告和故障定义--------------------------------------*/
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
		Uint8 VINVUderRating_A:1;  //输出欠压
		Uint8 OverTemp_A:1;		//温度超限告警
		Uint8 Fan1Block_A:1;		//风扇1异常
		Uint8 Fan2Block_A:1;		//风扇2异常
		Uint8 Ecan_A:1;					//模块并机通讯告警
		Uint8 ACPowerDerating_A :1;	//达到限额告警
		Uint8 UnAsyn_A :1;//逆变器不同步
		Uint8 BypassWarning:1; //旁路告警，功率模块不使用
		//Fault1
		Uint8 VGridUnderRating_F:1;	//输入欠压
		Uint8 VGridOverRating_F:1;		//输入过压
		Uint8 FreGridFault_F:1;		//输入频率异常
		Uint8 PFC_Recover_Fault_F:1;			//PFC可恢复故障
		Uint8 Inv_OVP_F:1;			//逆变器输出过压
		Uint8 rsvr0:1;			//
		Uint8 FreInv_Fault_F:1;		//输出频率异常
		Uint8 OverTemp_F:1;			//过温关机
		//Fault2
		Uint8 Double_Fan_Fault_F:1;	//风扇故障（两风扇同时异常）
		Uint8 Launch_Fault_F:1;			//启动异常
		Uint8 Inv_OverLoad_F:1;			//输出过载
		Uint8 rsvr1:1;			//
		Uint8 Output_Shorted_F:1;		//短路或者极端过载
		Uint8 Curr_Shar_Fault_F:1;		//并机均流故障
		Uint8 PFC_Unrecover_Fault_F:1;//PFC不可恢复故障
		Uint8 Repeatedly_Launch_F:1;//反复起机故障
		//Fault3
		Uint8 SynLine_broken:1;//并机线断
		Uint8 BypassFault:1;
		Uint8 u8rsvr2:1;
		Uint8 u8rsvr3:1;
		Uint8 u8rsvr4:1;
		Uint8 u8rsvr5:1;
		Uint8 u8rsvr6:1;
		Uint8 u8rsvr7:1;
	}bit;
}ECAN_MODULE_ERROR;
/*----------------------------模块发给计算机的数据帧的数据域的中间变量------------------------------------*/
typedef struct
{
	Uint16 			Input_Volt_Rms;   //输入电压有效值
	Uint16 			Input_Curr_Rms;  //Input current RMS
	Uint16      		BusP_Volt;		//BusP voltage

	Uint16      		BusN_Volt;		//BusN voltage
	Uint16				Out_Volt_Rms;	//Inverter load voltage RMS
	Uint16				Inv_Volt_Rms;	//逆变器输出电压有效值
	
	Uint16				Inv_Cur_Rms;	//逆变器输出电流有效值
	Uint16				PFC_Temp;		//PFC温度
	Uint16				Inv_Temp;		//逆变器温度
	
	Uint16				Inv_Freq;		//逆变器频率
	Uint16				RunTimeHour_L;
	Uint16				RunTimeHour_H;

	ECAN_MODULE_ERROR	Error;  //模块故障和警告
	Uint8				Check;	//校验
}ECAN_MODULE_DATA;
extern ECAN_MODULE_DATA Ecan_ModuleData;

/*------------------------------计算机发给模块的矫正帧的数据域的中间变量------------------------------------*/
typedef struct
{
	Uint16 		Input_Volt_Rms;  //输入电压有效值
	Uint16			Bus_P;		//PFC整流电压平均值（控制）
	Uint16			Bus_N;	//PFC整流电压平均值（保护）
	Uint16			Inv_Volt_Rms;	//逆变器输出电压有效值
	Uint16			Inv_Cur_Rms;	//逆变器输出电流有效值
	Uint16			Inv_OutV_Rms;	//逆变器负载电压有效值
	Uint16			PFC_Temp;		//PFC温度
	Uint16 		Inv_Temp;	    //逆变器温度
	Uint16 		Input_Cur_Rms;  //输入电压有效值
	Uint16 		Aver_Curr_Inv; //逆变平均电流
	Uint16 		RestartOverTimes;	//反复启动
	Uint16 		INV_Volt_Ref; 	//Inverter(220V) output voltage reference
	Uint16 		INV_Drop_Coeff; 	//Inverter(220V) output voltage droop coefficient
}ECAN_REVISED;
extern ECAN_REVISED Ecan_SytemREVISED;

/*------------------------------计算机发给模块的命令帧的数据域的中间变量------------------------------------*/
typedef struct
{
	Uint16 		rsvr0;
	Uint16		Defaluts;		//恢复默认校正值
	Uint16		Output_Enable;	//模块输出电压启动/停止
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

#endif
