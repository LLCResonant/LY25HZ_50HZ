/********************************************************************
程序说明：1.调用Eerom_Gpio_Init函数，初始化与Eeprom相关的IO
		  2.调用 writebyte(Uint16 addr,Uint16 data); //写Eeprom
            	 readbyte(Uint16 addr);				 //读Eeprom
		  3.查看读取的内容与写入内容是否一致
********************************************************************/

#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file

Uint16 i2c_err;							//Eeprom读写错误指示

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
void AT24c512_WriteByte(Uint16 addr,Uint8 data);//向指定地址写入一字节数据
Uint8 AT24c512_ReadByte(Uint16 addr);//从指定地址读出一字节数据
void AT24c512_WriteWord(Uint16 addr,Uint16 data);
Uint16 AT24c512_ReadWord(Uint16 addr);
void AT24c512_WriteSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
void AT24c512_ReadSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber);
void AT24c512_WriteSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
void AT24c512_ReadSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber);
void AT24c512_ClearSeriseWord(Uint16 startaddr,Uint16 wordNumber);
/*****************AT24C512应用函数声明部分结束***************************/


///////////////////以下程序不可改动///////////////////////////////////

void Eeprom_Gpio_Init(void)
{
	EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;	  	//上拉
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;   	// 输出端口
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;  	// IO口
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3; 	// 不同步

    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;	  	//上拉
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;   	// 输出端口
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;  	// IO口
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;   // 不同步
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
void delay(Uint16 time) 					//延时函数
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
void begintrans(void)       				//发送START 信号
{
    SDA_W1;         						//SDA=1
    delay(DELAY_UNIT * 10);         		//延时
    SDA_WRITE();            				//SDA 方向为输出到EEPROM
    delay(DELAY_UNIT * 10);         		//延时
    SCL_1;          						//SCL=1
    delay(DELAY_UNIT * 10);         		//延时
    SDA_W0;         						//SDA=0
    delay(DELAY_UNIT * 10);        			//延时
}
void stoptrans(void)        				//发送STOP 信号
{
    SDA_WRITE();            				//SDA方向为输出到EEPROM
    delay(DELAY_UNIT * 10);        			//延时
    SDA_W0;         						//SDA=0
    delay(DELAY_UNIT * 10);         		//延时
    SCL_1;          						//SCL=1
    delay(DELAY_UNIT * 10);         		//延时
    SDA_W1;         						//SDA=1
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
内部函数，输出ACK=0
********************************************/
void WriteACK()
{
    SDA_W0;
    delay(DELAY_UNIT * 10);
    SCL_1;
    delay(DELAY_UNIT * 10);
    SCL_0;
}
void ack(void)              				//等待ACK 信号
{
    Uint16 d;
    Uint16  i;

    SDA_READ();             				//SDA方向为从EEPROM 输入
    delay(DELAY_UNIT * 10);          		//延时
    SCL_1;          						//SCL=1
    delay(DELAY_UNIT * 10);         		//延时
    i = 0;
    do
    {
        d = SDA_R;
        i++;
        delay(DELAY_UNIT);
    }
    while((d == 1) && (i <= 500));      	//等待EEPROM 输出低电平,4ms后退出循环

    if (i >= 499)
    {
        i2c_err = 0xff;
    }

    i = 0;
    SCL_0;          						//SCL=0
    delay(DELAY_UNIT * 10);          		//延时
}

void bytein(Uint8 ch)  					//向EEPROM 写入一个字节
{
    Uint8 i;
    SCL_0;          						//SCL=0
    delay(DELAY_UNIT * 10);					//延时
    SDA_WRITE();            				//SDA方向为输出到EEPROM
    delay(DELAY_UNIT * 10);         		//延时
    for(i=8;i>0;i--)
    {
        if ((ch & 0x80)== 0)
    	{
            SDA_W0;     					//数据通过SDA 串行移入EEPROM
            delay(DELAY_UNIT * 10);				//延时
    	}
        else
    	{
            SDA_W1;
            delay(DELAY_UNIT * 10);				//延时
    	}
        SCL_1;      						//SCL=1
        delay(DELAY_UNIT * 10);      		//延时
        ch <<= 1;
        SCL_0;      						//SCL=0
        delay(DELAY_UNIT * 10);      		//延时
    }
    ack();
}

Uint8 byteout(void)        				//从EEPROM 输出一个字节
{
    unsigned char i;
    Uint8 ch;
    ch = 0;

    SDA_READ();             				//SDA 的方向为从EEPROM 输出
    delay(DELAY_UNIT * 10);         			//延时
    for(i=8;i>0;i--)
    {
        ch <<= 1;
        SCL_1;      						//SCL=1
        delay(DELAY_UNIT * 10);      	   //延时
        ch |= SDA_R;    					//数据通过SDA 串行移出EEPROM
        delay(DELAY_UNIT * 10);     		//延时
        SCL_0;      						//SCL=0
        delay(DELAY_UNIT * 10);      		//延时
    }
    return(ch);
}
/////////////////////////////////以上为I2c 模拟部分（以上程序不可动）////////////////////////////////////////////

////////////////////////////////////AT24C512应用程序部分////////////////////////////////////////////////////////////
/***************************************************************
*名称: void AT24c512_WriteByte(Uint16 addr,Uint8 data)
*描述: 向AT24C512指定地址写入一字节数据
*参数: addr--地址；data--一字节数据
*返回: 无
****************************************************************/
void AT24c512_WriteByte(Uint16 addr,Uint8 data)
{
	begintrans();							//开始
    bytein(0xA0);  //写入写控制字0xA0  此地方要做改动！
    bytein( (Uint8)(addr & 0xff00)>>8);       //写入指定地址
	bytein( (Uint8)(addr & 0x00ff));       //写入指定地址
    bytein(data);      						//写入待写入EEPROM 的数据
    stoptrans();							//停止
    delay(8000);
	delay(8000);	
}
/***************************************************************
*名称: Uint8 AT24c512_ReadByte(Uint16 addr)
*描述: 从AT24c512指定地址读出一字节数据
*参数: addr--地址
*返回: 读出的数据
****************************************************************/
Uint8 AT24c512_ReadByte(Uint16 addr)
{
	Uint8 c;    
    begintrans();       					//开始
    bytein(0xA0);  							//写入写控制字0xA0
    bytein( (Uint8)(addr & 0xff00)>>8);       //写入指定地址
	bytein( (Uint8)(addr & 0x00ff));       //写入指定地址       					//写入指定地址
    begintrans();       					//开始
    bytein(0xA1);       					//写入读控制字0xA1
    c = byteout();      					//读出EEPROM 涑龅氖�
    stoptrans();        					//停止
    delay(2000);        					//延时
    return(c);	
}
/***************************************************************
*名称: void AT24c512_WriteWord(Uint16 addr,Uint16 data)
*描述: 向AT24c512指定地址写入一字数据
*参数: addr-地址；data-一字数据
*返回: 无
****************************************************************/
void AT24c512_WriteWord(Uint16 addr,Uint16 data)
{
	Uint8 HighData = 0;
	Uint8 LowData  = 0;
	LowData  = (Uint8)(data & 0x00ff);
  	HighData = (Uint8)( data >> 8 );
	AT24c512_WriteByte(addr,LowData);
	AT24c512_WriteByte(addr+1,HighData);

}
/***************************************************************
*名称: Uint16 AT24c512_ReadWord(Uint16 addr)
*描述: 从AT24c512指定地址读出一字数据
*参数: addr-地址；
*返回: 读出的一字数据
****************************************************************/
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
/***************************************************************
*名称: void AT24c512_WriteSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber)
*描述:
*参数: 无
*返回: 无
****************************************************************/
void AT24c512_WriteSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber)
{
	Uint16 i;
	for(i = 0; i < byteNumber; i++)
	{
		AT24c512_WriteByte(startaddr + i,* (pdata + i));
	}
			
}
/***************************************************************
*名称: void AT24c512_ReadSeriseByte(Uint16 startaddr,Uint8* pdata,Uint16 byteNumber)
*描述:
*参数: 无
*返回: 无
****************************************************************/
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
/***************************************************************
*名称:void AT24c512_WriteSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
*描述:
*参数: 无
*返回: 无
****************************************************************/
void AT24c512_WriteSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
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
void AT24c512_ClearSeriseWord(Uint16 startaddr,Uint16 wordNumber)
{
	Uint16 i;
	for(i = 0; i < wordNumber; i++)
	{
		AT24c512_WriteWord(startaddr + i,0x0000);
		startaddr += 1;
	}
}
/***************************************************************
*名称: void AT24c512_ReadSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
*描述:
*参数: 无
*返回: 无
****************************************************************/
void AT24c512_ReadSeriseWord(Uint16 startaddr,Uint16* pdata,Uint16 wordNumber)
{
	Uint16 i;
	for(i = 0; i < wordNumber; i++)
	{
		* pdata = AT24c512_ReadWord(startaddr + i);
		startaddr++;
		pdata++;
	}	
}
/***************************************************************
*名称:
*描述:
*参数: 无
*返回: 无
****************************************************************/
#if 0
Uint16 AT24c512_ReadWord(Uint16 addr)
{
	Uint16 c;
	Uint8 HighData = 0;
	Uint8 LowData  = 0;    
    begintrans();       					//开始
    bytein(0xA0);  							//写入写控制字0xA0
    bytein( (Uint8)(addr & 0xff00)>>8);       //写入指定地址
	bytein( (Uint8)(addr & 0x00ff));       //写入指定地址       					//写入指定地址
    begintrans();       					//开始
    bytein(0xA1);       					//写入读控制字0xA1
    LowData = byteout();      					//读出EEPROM 输出的数据
	HighData = byteout();
	
    stoptrans();        					//停止
    delay(2000);        					//延时
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
	bytein(0xA0);  //写入写控制字0xA0  此地方要做改动！
    bytein( (Uint8)(addr & 0xff00)>>8);       //写入指定地址
	bytein( (Uint8)(addr & 0x00ff));       //写入指定地址
	bytein(LowData);      					//写入待写入EEPROM 的数据
	bytein(HighData);
    stoptrans();							//停止
    delay(8000);
}
#endif
////////////////////////////////////AT24C512应用程序部分////////////////////////////////////////////////////////////
/***************************************************************
*名称:
*描述:
*参数: 无
*返回: 无
****************************************************************/

//===========================================================================
// No more.
//===========================================================================
