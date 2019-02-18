/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_I2cEeprom.h
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.8.11		001					NUAA XING
 *============================================================================*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  	I2cEeprom_H
#define  	I2cEeprom_H

/*********************************IIC simulative bus****************************************/
#define SDA_R   			GpioDataRegs.GPBDAT.bit.GPIO32;     				//SDA Read State
#define SDA_W0  		GpioDataRegs.GPBCLEAR.bit.GPIO32=1; 			//SDA Output 0; 	Write State
#define SDA_W1  		GpioDataRegs.GPBSET.bit.GPIO32=1;   			//SDA Output 1;	Write State
#define SCL_0   			GpioDataRegs.GPBCLEAR.bit.GPIO33=1; 			//SCL Output 0
#define SCL_1   			GpioDataRegs.GPBSET.bit.GPIO33=1;   			//SCL Output 1
#define DELAY_UNIT	10																	//Time delay

/********************************The definition of functions********************************/
void writebyte(Uint16 addr,Uint16 data);
Uint16 readbyte(Uint16 addr);
extern void Eeprom_Gpio_Init(void);
void delay(Uint16 time);
void begintrans();
void stoptrans();
void ack();
void bytein(Uint8 ch);
Uint8 byteout(void);

/*****************The definition of AT24C512 application function****************************/
extern void AT24c512_WriteByte(Uint16 addr,Uint8 data);//Write a data byte into a memory space with a specified address
extern Uint8 AT24c512_ReadByte(Uint16 addr);//Read a data byte from a memory space with a specified address
extern void AT24c512_WriteWord(Uint16 addr,Uint16 data);
extern Uint16 AT24c512_ReadWord(Uint16 addr);
extern void AT24c512_WriteSeriesByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
extern void AT24c512_ReadSeriesByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
extern void AT24c512_WriteSeriesWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
extern void AT24c512_ReadSeriesWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
extern void AT24c512_ClearSeriesWord(Uint16 startaddr,Uint16 wordNumber);

#endif

//--- end of file -----------------------------------------------------

