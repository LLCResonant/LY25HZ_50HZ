/*=============================================================================*
 *         Copyright(c) 2009-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : I2C.h 
 *
 *  PURPOSE  : provide Head file for I2C.c
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *      
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *    
 *============================================================================*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2cEeprom_H
#define  I2cEeprom_H

#define SDA_R   GpioDataRegs.GPBDAT.bit.GPIO32;     //SDA 读状态
#define SDA_W0  GpioDataRegs.GPBCLEAR.bit.GPIO32=1; //SDA 输出0 写状态
#define SDA_W1  GpioDataRegs.GPBSET.bit.GPIO32=1;   //SDA 输出1 写状态
#define SCL_0   GpioDataRegs.GPBCLEAR.bit.GPIO33=1; //SCL 输出0
#define SCL_1   GpioDataRegs.GPBSET.bit.GPIO33=1;   //SCL 输出1
#define DELAY_UNIT	10								//宏定义延时时间常数
//Uint16 i2c_err;								//Eeprom读写错误指示
struct Time
{
	Uint8 Second;
	Uint8 Minute;
	Uint16 Hour_L;
	Uint16 Hour_H;
	Uint8 OverFlow;
	Uint16 TimeCheck;
};
extern  struct Time  RunningTime;

/******************************函数声明****************************/
void writebyte(Uint16 addr,Uint16 data);
Uint16 readbyte(Uint16 addr);
extern void Eeprom_Gpio_Init(void);
void delay(Uint16 time);
void begintrans();
void stoptrans();
void ack();
void bytein(Uint8 ch);
Uint8 byteout(void);
/*****************IIC模拟总线部分*************************************/

/*****************AT24C512应用函数声明部分****************************/
extern void AT24c512_WriteByte(Uint16 addr,Uint8 data);//向指定地址写入一字节数据
extern Uint8 AT24c512_ReadByte(Uint16 addr);//从指定地址读出一字节数据
extern void AT24c512_WriteWord(Uint16 addr,Uint16 data);
extern Uint16 AT24c512_ReadWord(Uint16 addr);
extern void AT24c512_WriteSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
extern void AT24c512_ReadSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
extern void AT24c512_WriteSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
extern void AT24c512_ReadSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
extern void AT24c512_ClearSeriseWord(Uint16 startaddr,Uint16 wordNumber);

#endif

/*****************AT24C512应用函数声明部分结束***************************/

//#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
//#include "DSP2833x_I2C_defines.h"


/*struct DEF_I2CADDR
{
	Uint16 u16BlockAddr;
	Uint16 u16ByteAddr;
};

typedef struct DEF_I2CADDR TypeI2cAddr;


typedef struct I2CMSG TypeI2cMsg;*/

// Prototype statements for functions found within this file.
/*void   InitI2C(void);
Uint16 I2C_Reset(void);
void   I2C_ConvAddr(Uint16, TypeI2cAddr * );
Uint16 I2C_PollACK(TypeI2cMsg * );
Uint16 I2C_Write_InByte(Uint16, Uint16* , Uint16);
Uint16 I2C_Write_InWord(Uint16, Uint16* , Uint16);
Uint16 I2C_Write_Page(TypeI2cMsg * );
Uint16 I2C_WriteData(TypeI2cMsg * );
Uint16 I2C_Read_InByte(Uint16, Uint16* , Uint16);
Uint16 I2C_Read_InWord(Uint16, Uint16* , Uint16);
Uint16 I2C_Read_MultiByte(TypeI2cMsg * );
Uint16 I2C_ReadData(TypeI2cMsg * );*/

//note: Although the address of ST25C16 is 0xA0, but in 7-bit address mode,
//the address is 0x50. see I2CSAR Field Descriptions.
/*#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          16
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x00

#define	ST25C16_ADDR_LIMIT			0x07FF
#define ST25C16_ADDR_MASK			0x07FF
#define ST25C16_ADDR_ROW_MASK_W		0xFFFC
#define ST25C16_ADDR_ROW_MASK_B		0xFFF8
#define	ST25C16_ADDR_BLOCK_MASK		0xFF00

#define ST25C16_ADDR_ROW_LEN_W		0x04
#define ST25C16_ADDR_ROW_LEN_B		0x08
#define	ST25C16_ADDR_BLOCK_LEN		0x100

#define I2C_W_R_OK						0
#define I2C_W_R_EXCEED_LIMIT			1
#define	I2C_W_R_DATA_AMOUNT_IS_ZERO		2
#define	I2C_R_FAILED					3
#define I2C_W_FAILED					4

#define	I2C_MAX_TIMES_PAGE_WRITE		3
#define	I2C_MAX_TIMES_DYMMY_WRITE		3
#define	I2C_MAX_TIMES_RESET				3
#define	I2C_MAX_MS_SEMPEND	10
#endif*/

//--- end of file -----------------------------------------------------

