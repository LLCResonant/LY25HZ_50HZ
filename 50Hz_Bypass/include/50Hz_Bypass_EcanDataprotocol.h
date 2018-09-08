/*
 *  Revised on: 2017.10.23 by Gao Xun
 */


#ifndef ECanAc_H
#define	ECanAc_H

#define 	Computer_ADDRESS  		0x02					//上位机地址
#define 	Computer_TYPE 		    0x00					//上位机类型
#define 	MODULE_TYPE 		    0x05					//模块类型
#define MODULEADDRESS	GpioDataRegs.AIODAT.all			//模块地址

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
/*----------------------------模块发给计算机的数据帧的数据域的中间变量------------------------------------*/
typedef struct
{
	Uint16 			u16VGrid_rms;   //输入电压有效值
	Uint16 			u16VGrid_Freq;
	Uint16 			u16IGrid_rms;  //Input current RMS
	Uint16			u16VOut_rms;	//Trail load voltage RMS
	Uint16			u16Temperature;		//模块温度
	Uint16			u16RunTimeu16Hour_L;
	Uint16			u16RunTimeu16Hour_H;

	ECAN_MODULE_Alert	Alert;
	ECAN_MODULE_Fault	Fault;  //模块故障和警告
	Uint8				Check;	//校验
}ECAN_MODULE_DATA;
extern ECAN_MODULE_DATA Ecan_ModuleData;

/*------------------------------计算机发给模块的矫正帧的数据域的中间变量------------------------------------*/
typedef struct
{
	Uint16 			u16VGrid_rms;  //输入电压有效值
	Uint16			u16VOut_rms;	//输出电压有效值
	Uint16 			u16Temperature;	    //轨道温度
	Uint16 			u16IGrid_rms;  //输入电压有效值
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
