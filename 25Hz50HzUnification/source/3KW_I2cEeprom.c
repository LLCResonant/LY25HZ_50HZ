/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_I2cEeprom.c
 *
 *  PURPOSE  :
 *
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/

#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file

Uint16 u16i2c_err;							//Eeprom error indication

/******************************function declaration****************************/
void writebyte(Uint16 addr,Uint16 data);
Uint16 readbyte(Uint16 addr);
void Eeprom_Gpio_Init(void);
void delay(Uint16 time);
void begintrans();
void stoptrans();
void ack();
void bytein(Uint8 ch);
Uint8 byteout(void);
/*****************IIC Simulation Bus*************************************/

/*****************AT24C512 Application Function Declaration****************************/
void AT24c512_WriteByte(Uint16 addr,Uint8 data);//write a byte data into an address space
Uint8 AT24c512_ReadByte(Uint16 addr);//read a byte data from an address space
void AT24c512_WriteWord(Uint16 addr,Uint16 data);
Uint16 AT24c512_ReadWord(Uint16 addr);
void AT24c512_WriteSeriesByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
void AT24c512_ReadSeriesByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
void AT24c512_WriteSeriesWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
void AT24c512_ReadSeriesWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
void AT24c512_ClearSeriesWord(Uint16 startaddr,Uint16 wordNumber);
/*****************End of AT24C512 Application Function Declaration***************************/


///////////////////Don not change the following content///////////////////////////////////

void Eeprom_Gpio_Init(void)
{
	EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;	  	//pull up
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;   		//output port
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;  	// IO port
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3; 	// out of sync

    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;	  	//pull up
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;   		//output port
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;  	//IO port
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;   //out of sync
    EDIS;
}

__inline void SDA_READ(void)
{
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO32=0;       //Input, SDA
    EDIS;
}

__inline void SDA_WRITE(void)
{
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO32=1;       //Output. SDA
    EDIS;
}
///=========================GPIO SIMULATE I2c communication=====================*/
void delay(Uint16 time) 					//delay function
{
    for(; time>0 ; time--)
    {
        asm(" nop");
        asm(" nop");
        asm(" nop");
        asm(" nop");
		asm(" nop");
        asm(" nop");
        asm(" nop");
        asm(" nop");
		asm(" nop");
		asm(" nop");
    }
}
void begintrans(void)       				//send START signal
{
    SDA_W1;         								//SDA=1
    delay(DELAY_UNIT * 10);         		//delay
    SDA_WRITE();            					//SDA direction is output to EEPROM
    delay(DELAY_UNIT * 10);         		//delay
    SCL_1;          									//SCL=1
    delay(DELAY_UNIT * 10);         		//delay
    SDA_W0;         								//SDA=0
    delay(DELAY_UNIT * 10);        		//delay
}
void stoptrans(void)        				//send STOP signal
{
    SDA_WRITE();            					//SDA direction is output to EEPROM
    delay(DELAY_UNIT * 10);        		//delay
    SDA_W0;         								//SDA=0
    delay(DELAY_UNIT * 10);         		//delay
    SCL_1;          									//SCL=1
    delay(DELAY_UNIT * 10);         		//delay
    SDA_W1;         								//SDA=1
    delay(DELAY_UNIT * 10);
}
void writenoack()
{
    SDA_W1;
    delay(DELAY_UNIT * 10);
    SCL_1;
    delay(DELAY_UNIT * 10);
    SCL_0;
}
/********************************************
inner function，output ACK=0
********************************************/
void WriteACK()
{
    SDA_W0;
    delay(DELAY_UNIT * 10);
    SCL_1;
    delay(DELAY_UNIT * 10);
    SCL_0;
}
void ack(void)              				//wait ACK signal
{
    Uint16 d;
    Uint16  i;

    SDA_READ();             				//SDA direction is input from EEPROM
    delay(DELAY_UNIT * 10);          	//delay
    SCL_1;          								//SCL=1
    delay(DELAY_UNIT * 10);         	//delay
    i = 0;
    do
    {
        d = SDA_R;
        i++;
        delay(DELAY_UNIT);
    }
    while((d == 1) && (i <= 500));      //wait EEPROM output voltage level gets low , 4ms later end the loop automatically

    if (i >= 499)
    {
        u16i2c_err = 0xff;
    }

    i = 0;
    SCL_0;          									//SCL=0
    delay(DELAY_UNIT * 10);          		//delay
}

void bytein(Uint8 ch)  					//write a byte into EEPROM
{
    Uint8 i;
    SCL_0;          								//SCL=0
    delay(DELAY_UNIT * 10);			//delay
    SDA_WRITE();            				//SDA direction is output to EEPROM
    delay(DELAY_UNIT * 10);         	//delay
    for(i=8;i>0;i--)
    {
        if ((ch & 0x80)== 0)
    	{
            SDA_W0;     						//Data serial input into EEPROM through SDA
            delay(DELAY_UNIT * 10);	//delay
    	}
        else
    	{
            SDA_W1;
            delay(DELAY_UNIT * 10);	//delay
    	}
        SCL_1;      								//SCL=1
        delay(DELAY_UNIT * 10);      	//delay
        ch <<= 1;
        SCL_0;      								//SCL=0
        delay(DELAY_UNIT * 10);      	//delay
    }
    ack();
}

Uint8 byteout(void)        				//read a byte out of EEPROM
{
    unsigned char i;
    Uint8 ch;
    ch = 0;

    SDA_READ();             				//SDA direction is out of EEPROM
    delay(DELAY_UNIT * 10);         	//delay
    for(i=8;i>0;i--)
    {
        ch <<= 1;
        SCL_1;      								//SCL=1
        delay(DELAY_UNIT * 10);      	//delay
        ch |= SDA_R;    						//Data serial output from EEPROM through SDA
        delay(DELAY_UNIT * 10);     	//delay
        SCL_0;      								//SCL=0
        delay(DELAY_UNIT * 10);      	//delay
    }
    return(ch);
}
/////////////////////////////////Above content are I2c simulation part（do not change above function）////////////////////////////////////////////

////////////////////////////////////AT24C512 Application function////////////////////////////////////////////////////////////
/*=============================================================================*
 * FUNCTION: void AT24c512_WriteByte(Uint16 addr, Uint8 data)
 * PURPOSE:	write a byte into a specific address space of AT24C512
 * PRARMETER: addr--address；data--a byte data
 * RETURN: none
 *============================================================================*/
void AT24c512_WriteByte(Uint16 addr,Uint8 data)
{
	begintrans();									//begin
    bytein(0xA0);  								//write writing control byte 0xA0, according to different IC
    bytein( (Uint8)(addr & 0xff00)>>8);       //write specific address
	bytein( (Uint8)(addr & 0x00ff));       		//write specific address
    bytein(data);      									//write data into EEPROM
    stoptrans();									//stop
    delay(8000);
	delay(8000);	
}
/*=============================================================================*
 * FUNCTION: Uint8 AT24c512_ReadByte(Uint16 addr)
 * PURPOSE:	read a byte from a specific address space of AT24C512
 * PRARMETER: addr--address
 * RETURN: a byte data
 *============================================================================*/
Uint8 AT24c512_ReadByte(Uint16 addr)
{
	Uint8 c;    
    begintrans();       								//begin
    bytein(0xA0);  									//write writing control byte 0xA0, according to different IC
    bytein( (Uint8)(addr & 0xff00)>>8);   //write specific address
	bytein( (Uint8)(addr & 0x00ff));       	//write specific address
    begintrans();       								//begin
    bytein(0xA1);       								//write reading control byte 0xA1, according to different IC
    c = byteout();      								//read data from EEPROM
    stoptrans();        								//stop
    delay(2000);        								//delay
    return(c);	
}
/*=============================================================================*
 * FUNCTION: void AT24c512_WriteWord(Uint16 addr, Uint16 data)
 * PURPOSE:	write a word into a specific address space of AT24C512
 * PRARMETER: addr--address；data--a word data
 * RETURN: none
 *============================================================================*/
void AT24c512_WriteWord(Uint16 addr,Uint16 data)
{
	Uint8 HighData = 0;
	Uint8 LowData  = 0;
	LowData  = (Uint8)(data & 0x00ff);
  	HighData = (Uint8)( data >> 8 );
	AT24c512_WriteByte(addr,LowData);
	AT24c512_WriteByte(addr+1,HighData);
}
/*=============================================================================*
 * FUNCTION: Uint16 AT24c512_ReadWord(Uint16 addr)
 * PURPOSE:	read a word from a specific address space of AT24C512
 * PRARMETER: addr--address
 * RETURN: a word data
 *============================================================================*/
Uint16 AT24c512_ReadWord(Uint16 addr)
{
	Uint8 HighData = 0;
	Uint8 LowData  = 0;
	Uint16 Data = 0;
	LowData  = AT24c512_ReadByte(addr);
	HighData = AT24c512_ReadByte(addr+1);
	Data     = (Uint16)(HighData << 8) + (Uint16)LowData;
	return(Data);
}
/*=============================================================================*
 * FUNCTION: void AT24c512_WriteSeriseByte(Uint16 startaddr, Uint8* pdata, Uint16 byteNumber)
 * PURPOSE:	write series bytes into specific address space of AT24C512
 * PRARMETER: startaddr-- the start address  *pdata-- data pointer byteNumber-- the number of the bytes
 * RETURN: none
 *============================================================================*/
void AT24c512_WriteSeriesByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber)
{
	Uint16 i;
	for(i = 0; i < byteNumber; i++)
	{
		AT24c512_WriteByte(startaddr + i,* (pdata + i));
	}
}
/*=============================================================================*
 * FUNCTION: void AT24c512_ReadSeriseByte(Uint16 startaddr, Uint8* pdata, Uint16 byteNumber)
 * PURPOSE:	read series bytes out of specific address space of AT24C512
 * PRARMETER: startaddr-- the start address  *pdata-- data pointer byteNumber-- the number of the bytes
 * RETURN: none
 *============================================================================*/
void AT24c512_ReadSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber)
{
	Uint16 i;
	for(i = 0; i < byteNumber; i++)
	{
		* pdata = AT24c512_ReadByte(startaddr);
		startaddr++;
		pdata++;
	}
}
/*=============================================================================*
 * FUNCTION: void AT24c512_WriteSeriseWord(Uint16 startaddr, Uint16* pdata, Uint16 wordNumber)
 * PURPOSE:	write series words into specific address space of AT24C512
 * PRARMETER: startaddr-- the start address  *pdata-- data pointer wordNumber-- the number of the words
 * RETURN: none
 *============================================================================*/
void AT24c512_WriteSeriesWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
{
	Uint16 i;
	//0000=startaddr
	//i = 0  0000 0001  startaddr = 0
	//i = 1  0002 0003  startaddr = 1
	for(i = 0; i < wordNumber; i++)
	{
		AT24c512_WriteWord(startaddr + i,* (pdata + i));
		startaddr += 1;
	}	
}
/*=============================================================================*
 * FUNCTION: void AT24c512_ClearSeriesWord(Uint16 startaddr, Uint16 wordNumber)
 * PURPOSE:	clear series words of specific address space of AT24C512
 * PRARMETER: startaddr-- the start address  wordNumber-- the number of the words
 * RETURN: none
 *============================================================================*/
void AT24c512_ClearSeriesWord(Uint16 startaddr,Uint16 wordNumber)
{
	Uint16 i;
	for(i = 0; i < wordNumber; i++)
	{
		AT24c512_WriteWord(startaddr + i,0x0000);
		startaddr += 1;
	}
}
/*=============================================================================*
 * FUNCTION: void AT24c512_ReadSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
 * PURPOSE:	write series words into specific address space of AT24C512
 * PRARMETER: startaddr-- the start address  *pdata-- data pointer wordNumber-- the number of the words
 * RETURN: none
 *============================================================================*/
void AT24c512_ReadSeriesWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
{
	Uint16 i;
	for(i = 0; i < wordNumber; i++)
	{
		* pdata = AT24c512_ReadWord(startaddr + i);
		startaddr++;
		pdata++;
	}	
}
/*=================================================================*/
#if 0
Uint16 AT24c512_ReadWord(Uint16 addr)
{
	Uint16 c;
	Uint8 HighData = 0;
	Uint8 LowData  = 0;    
    begintrans();       									//begin
    bytein(0xA0);  										//write writing control byte 0XA0
    bytein( (Uint8)(addr & 0xff00)>>8);       //write specific address
	bytein( (Uint8)(addr & 0x00ff));      		 //write specific address
    begintrans();       									//begin
    bytein(0xA1);       									//write read control byte 0xA1
    LowData = byteout();      						//Read data from EEPROM
	HighData = byteout();
	
    stoptrans();        									//stop
    delay(2000);        									//delay
	c = (Uint16)(HighData << 8) + (Uint16)LowData;
    return(c);	
}
#endif
#if 0
void AT24c512_WriteWord(Uint16 addr,Uint16 data)
{
	Uint8 HighData = 0;
	Uint8 LowData  = 0;
	LowData  = (Uint8)(data & 0x00ff);
  	HighData = (Uint8)( data >> 8 );
	bytein(0xA0);  										//write writing control byte 0XA0
    bytein( (Uint8)(addr & 0xff00)>>8);       //write specific address
	bytein( (Uint8)(addr & 0x00ff));       		//write specific address
	bytein(LowData);      								//write data into EEPROM
	bytein(HighData);
    stoptrans();											//stop
    delay(8000);
}
#endif
//===========================================================================
// No more.
//===========================================================================
