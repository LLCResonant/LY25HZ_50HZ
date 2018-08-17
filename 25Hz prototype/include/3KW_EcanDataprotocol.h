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
#define 	MODULE_TYPE 		    0x05					//模块类型
#define 	MODULEADDRESS 		    0x01					//模块地址
extern Uint8   ModuleAdd;
extern Uint8  u8_hostdrop;
extern Uint8  ECAN_timer1;
extern Uint8 g_InvH_Load;
extern Uint8 g_InvL_Load;

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
		Uint8 u8rsvr1:1;
		Uint8 ACPowerDerating_A :1;	//达到限额告警
		Uint8 UnAsyn_A :1;//逆变器不同步
		Uint8 rsvr7 :1;

		//Fault1
		Uint8 VGridUnderRating_F:1;	//输入欠压
		Uint8 VGridOverRating_F:1;	//输入过压
		Uint8 FreGridFault_F:1;			//输入频率异常
		Uint8 PFC_Recover_Fault_F:1;			//PFC可恢复故障
		Uint8 InvH_OVP_F:1;				//轨道输出过压
		Uint8 InvL_OVP_F:1;				//局部输出过压
		Uint8 FreInv_Fault_F:1;			//输出频率异常
		Uint8 OverTemp_F:1;				//过温关机
		//Fault2
		Uint8 Double_Fan_Fault_F:1;	//风扇故障（两风扇同时异常）
		Uint8 Launch_Fault_F:1;			//启动异常
		Uint8 InvH_OverLoad_F:1;			//轨道输出过载
		Uint8 InvL_OverLoad_F:1;			//局部输出过载
		Uint8 Output_Shorted_F:1;			//短路或者极端过流
		Uint8 Curr_Shar_Fault_F:1;			//并机均流故障
		Uint8 PFC_Unrecover_Fault_F:1;	//PFC不可恢复故障
		Uint8 Repeatedly_Launch_F:1;		//反复起机故障
		//Fault3
		Uint8 SynLine_broken:1;//并机线断
		Uint8 Ecan_F:1;		//ECAN通讯故障
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
	Uint16 				Input_Volt_Rms;   //输入电压有效值
	Uint16 				Input_Curr_Rms;  //Input current RMS
	Uint16      		BusP_Volt;		//BusP voltage

	Uint16      		BusN_Volt;		//BusN voltage
	Uint16				OutH_Volt_Rms;	//Trail load voltage RMS
	Uint16				OutL_Volt_Rms;	//Local load voltage RMS

	Uint16				InvH_Volt_Rms;	//轨道输出电压有效值
	Uint16				InvL_Volt_Rms;	//局部输出电压有效值
	Uint16				InvH_Cur_Rms;	//轨道输出电流有效值
	
	Uint16				InvL_Cur_Rms;	//局部输出电流有效值
	Uint16				PFC_Temp;		//模块温度
	Uint16				InvH_Temp;		//轨道温度
	
	Uint16				InvL_Temp;		//局部温度
	Uint16				InvH_Freq;		//轨道频率
	Uint16				InvL_Freq;			//局部频率

	Uint16				Phase_Lead;		//局部超前轨道的相位
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
	Uint16		Bus_P;		//PFC整流电压平均值（控制）
	Uint16		Bus_N;	//PFC整流电压平均值（保护）
	Uint16		InvH_Volt_Rms;	//轨道输出电压有效值
	Uint16		InvL_Volt_Rms;	//局部输出电压有效值
	Uint16		InvH_Cur_Rms;	//轨道输出电流有效值
	Uint16		InvL_Cur_Rms;	//局部输出电流有效值
	Uint16		InvH_OutV_Rms;	//轨道输出负载电压有效值
	Uint16		InvL_OutV_Rms;	//局部输出负载电压有效值
	Uint16		PFC_Temp;		//PFC温度
	Uint16 		InvH_Temp;	    //轨道温度
	Uint16 		InvL_Temp;      //局部温度
	Uint16 		Input_Cur_Rms;  //输入电压有效值
	Uint16 		Aver_Curr_InvH; //轨道平均电流
	Uint16 		Aver_Curr_InvL; //局部平均电流
	Uint16 		RestartOverTimes;	//反复启动
	Uint16 		INVH_Volt_Ref; 	//Trail(220V) output voltage reference
	Uint16 		INVL_Volt_Ref;//Local(110V) output voltage reference
	Uint16 		INVH_Drop_Coeff; 	//Trail(220V) output voltage droop coefficient
	Uint16 		INVL_Drop_Coeff;//Local(110V) output voltage droop coefficient
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
extern void Output_Revise (void);

#endif
