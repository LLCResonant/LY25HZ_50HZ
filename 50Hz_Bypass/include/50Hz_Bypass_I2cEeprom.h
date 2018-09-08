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

#define SDA_R   GpioDataRegs.GPADAT.bit.GPIO23;     //SDA 读状态
#define SDA_W0  GpioDataRegs.GPACLEAR.bit.GPIO23=1; //SDA 输出0 写状态
#define SDA_W1  GpioDataRegs.GPASET.bit.GPIO23=1;   //SDA 输出1 写状态
#define SCL_0   GpioDataRegs.GPBCLEAR.bit.GPIO33=1; //SCL 输出0
#define SCL_1   GpioDataRegs.GPBSET.bit.GPIO33=1;   //SCL 输出1
#define DELAY_UNIT	10								//宏定义延时时间常数

/******************************函数声明****************************/
void writebyte(Uint16 addr,Uint16 data);
Uint16 readbyte(Uint16 addr);
void Eeprom_Gpio_Init(void);
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

//--- end of file -----------------------------------------------------
