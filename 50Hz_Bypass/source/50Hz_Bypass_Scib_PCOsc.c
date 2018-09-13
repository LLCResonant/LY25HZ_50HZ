/*=============================================================================*
 *         Copyright(c) 2009-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 5KW_Scib_Interface.c
 *   
 *  PURPOSE  : SCIb for IPOMS (Virtual PCOsci by Ken)
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-19      V0.1           Ken      	    Created   
 *
 *----------------------------------------------------------------------------
 *  GLOBAL VARIABLES
 *    NAME                                    DESCRIPTION
 *          
 *      
 *----------------------------------------------------------------------------
 *  GLOBAL FUNCTIONS
 *    NAME                                    DESCRIPTION
 *    
 *   Scib_SnatchGraph(void);    
 *============================================================================*/


#include "DSP2803x_Device.h"			// Peripheral address definitions
#include "F28035_example.h"					// Main include file

typedef int (*pFunc1)(void);

/******************************variable definition******************************/
Uint8  u8Scib_UserDataBuf0[200];  //200
Uint8  u8Scib_CommandBuffer0[50];
Uint8  *pScib_CommandIn0;
Uint8  u8Scib_Temp = 0;
Uint16 u16Scib_CommandLength = 0;

// snatch graph part
Uint8  u8Scib_SnatchGraphEnable = 1;
Uint16 u16Scib_Interval = 0;
Uint16 u16Scib_Interval1 = 0;
Uint8  u8Scib_wTrigger = 0;
Uint8  u8Scib_wTriggerSource = 0;

Uint16 u16Scib_SnatchDataCnt = 0;
Uint16 u16Scib_SaveDataCnt = 0;
Uint16 u16Scib_TransmitCnt = 0;

Uint16 u16Scib_CompareVal = 0;
Uint8  u8Scib_Sign = 0;

Uint8  u8Scib_DataKind[4] = {0,0,0,0};

Uint16  u16Scib_GraphDataBuff[2][500];   // [4][500]  500######CHU    410
Uint16  u16Scib_TransmitDataBuff[500];   // [500]     500######CHU     410  407

Uint8 u8Scib_HighByte = 0;
Uint8 u8Scib_LowByte = 0;
Uint8 u8Scib_SendHighHalfByte = 1;
/******************************variable definition******************************/

/******************************fuciton list******************************/
Uint8 Scib_SnatchGraph(void);

void Scib_Parsing(void);
void Scib_Q1Command(void);
void Scib_Q3Command(void);
void Scib_QDCommand(void);
int16 swGetRLineVolt(void);
int16 swGetSLineVolt(void);
int16 swGetTLineVolt(void);
int16 swGetRCurr(void);
int16 swGetFault0(void);
int16 swGetFault1(void);
int16 swGetFault2(void);
int16 swGetFault3(void);
int16 swGetFault4(void);
int16 swGetFault5(void);
int16 swGetFaultUnrecover1(void);
int16 swGetFaultUnrecover2(void);
Uint8 sbNumToAscii(Uint16 u16Number, int8 i8Exponent, Uint8 *pbBuffer);
void Scib_WriteBinary(Uint16 *pstart, Uint16 u16Length);
void Scib_Q1CommandIndex(void);
/******************************fuciton list******************************/
pFunc1 GetDataSubArray[30] =
{
	swGetFault0,swGetFault0,swGetFault1,swGetFault2,//3
	swGetFault3,swGetFault4,swGetFault5,swGetFaultUnrecover1,//7
	swGetFaultUnrecover2,swGetRLineVolt,swGetSLineVolt,swGetTLineVolt,//11
	swGetRCurr
} ;

void COMM_TEST( void )
{
	Uint8 a[5];
	a[0] = '1';
	a[1] = '2';
	a[2] = '3';
	a[3] = '4';
	a[4] = '5';
	SciWrite(ID_SCIA, a, 5);
}
/*=============================================================================*
 * FUNCTION: TSK_Scib(void)
 * PURPOSE : 100ms Task Schedule For Scib
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     //Scib_TaskInitial();
 *     //SEM_pend();
 *     //Scib_Read();
 *     //Scib_Parsing();
 *
 * CALLED BY: 
 *  //   INT32 DSP Bios
 * 
 *============================================================================*/
void TSK_SCI_B(void)
{   
  
 //    Scib_TaskInitial(0, u8Scib_RxBuffer0, 50, con_SciTypeSci);
    pScib_CommandIn0 = u8Scib_CommandBuffer0;	//  buffer 的起始地址  // 只执行一次
  							
    while(1)
    {
        if(SEM_pend(&SEM_TimeBase500ms, SYS_FOREVER) == 1);
        {
//		    COMM_TEST();
         //   Scib_Q1Command();
	       while(1)
	        {
	            u8Scib_Temp = SciRead(ID_SCIA, pScib_CommandIn0); //

                if(SCI_RX_EMPTY == u8Scib_Temp)
				{
					break;    // 如队列空  就会返回  等待下一次调度  
				}
				if(u16Scib_CommandLength >= con_MAX_COMMAND_LENGTH)
				{
					pScib_CommandIn0 = u8Scib_CommandBuffer0;
					u16Scib_CommandLength = 0;
				}
				else if(0x5A == (*pScib_CommandIn0))
				{
				//	FlashProgramming_TSK();
				}
				else if((con_CHAR_ENTER == (*pScib_CommandIn0)) || (0x0A == (*pScib_CommandIn0)))  // 终止符Deal with command with 0x0a
				{
				    Scib_Parsing();    // 处理命令
					pScib_CommandIn0 = u8Scib_CommandBuffer0;
					u16Scib_CommandLength = 0;
				}
				else
				{
					u16Scib_CommandLength++;
					pScib_CommandIn0++;
	          	} 
		    }
        }
    }
}

/*=============================================================================*
 * FUNCTION: Scib_Parsing(void)
 * PURPOSE : Scib Command Parse
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     //Scib_Q1Command();
 *     //Scib_Q3Command();
 *     //Scib_QDCommand();
 *     
 *
 * CALLED BY: 
 *     //INT32 TASK_Scib() 
 * 
 *============================================================================*/                
void Scib_Parsing(void)
{  static Uint16 Command_Index=0;
	switch(u8Scib_CommandBuffer0[0])
	{
    case 'Q':
        {   
            if('1' == u8Scib_CommandBuffer0[1])
            {
            if (Command_Index==0)
			  {Scib_Q1Command();Command_Index=1;}
			  else
			  {Scib_Q1CommandIndex();
			   Command_Index=0;
			   }
                
            }
            if('3' == u8Scib_CommandBuffer0[1])
            {
                Scib_Q3Command();
            }
            if('D' == u8Scib_CommandBuffer0[1])
            {
                Scib_QDCommand();
            }
        }
        break;

    default:

        break;
	}
	
}
/*=============================================================================*
 * FUNCTION: Scib_Q1Command(void)
 * PURPOSE : Send Data to IPOMS For Q1 Command
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *     //sbNumToAscii(); 
 *     //Scib_Write();
 *     
 * CALLED BY: 
 *   //  INT32 Scib_Parsing() 
 * 
 *============================================================================*/
void Scib_Q1Command(void)          //  进行数据准备   存储到 u8Scib_UserDataBuf0【200】 并发送      
{    //  发送数据的长度 是变化的   如 123  就是 ascii 三位   12  就发送ascii 两位
	Uint8	bStrLen;  
	Uint8	bStrLen1;
	Uint8   *pDataBuf;
	
	bStrLen1 = 0;
	bStrLen = 0;

	bStrLen = sbNumToAscii( g_Sys_State, 0, u8Scib_UserDataBuf0); //1   g_Sys_Current_State  当前系统状态
	u8Scib_UserDataBuf0[bStrLen++] = 32;   //  各个数据段之间的间隔

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(_IQint(Calc_Result.iq20VGrid_RMS) * 10, 0, pDataBuf);//4
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(_IQint(Calc_Result.iq20IGrid_RMS) * 10, 0, pDataBuf);//5
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
		
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(_IQint(Calc_Result.iq20GridFreq) , 0, pDataBuf);//7  //GX 830
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32; 

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii( _IQint(_IQ(55)), 0, pDataBuf);  //11
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32; 

//＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte0, 0, pDataBuf);  //13
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_SysFaultMessage.Word.byte1, 0, pDataBuf);   //14
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte0, 0, pDataBuf);//17
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

    pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(g_StateCheck.Word.byte1, 0, pDataBuf);//18
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
	
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(100, 0, pDataBuf);//20 GX
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];
	bStrLen1 = sbNumToAscii(_IQint(SafetyReg.iq20VGrid_LowLimit), 0, pDataBuf);//21
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;

///////////////////////////////////////////////////////////＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
	      	
	SciWrite(ID_SCIA, u8Scib_UserDataBuf0, bStrLen);   //  原来是ID_SCIB   ###   CHU   发送数据
}


//=========================================================================================

//=================================================================================
void Scib_Q1CommandIndex(void)          //  进行数据准备   存储到 u8Scib_UserDataBuf0【200】 并发送      
{    //  发送数据的长度 是变化的   如 123  就是 ascii 三位   12  就发送ascii 两位
	Uint8	bStrLen;  
	Uint8	bStrLen1;
	Uint8   *pDataBuf;
	Uint16   i=0;
	bStrLen1 = 0;
	bStrLen = 0;
    
    for (i=1;i<=47;i++)
	{
	pDataBuf = &u8Scib_UserDataBuf0[bStrLen];     // 新数据段的起始地址
	bStrLen1 = sbNumToAscii(i, 0, pDataBuf);    //2   长度
	bStrLen += bStrLen1;
	u8Scib_UserDataBuf0[bStrLen++] = 32;
    }
	
	SciWrite(ID_SCIA, u8Scib_UserDataBuf0, bStrLen);   //  原来是ID_SCIB   ###   CHU   发送数据
	//Scib_WriteBinary(u8Scib_UserDataBuf0,bStrLen);

} 
              
/*=============================================================================*
 * FUNCTION: Scib_Q3Command(void)
 * PURPOSE : Pick up parameter for Snatch Graph fuction
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *    //Scib_Write
 *  
 *     
 *     
 *     
 *
 * CALLED BY: 
 * //    INT32 Scib_Parsing() 
 * 
 *============================================================================*/
void Scib_Q3Command(void)
{
	Uint16 u16SnatchDataCntTemp, u16IntervalTemp;
	Uint8 u8DataKindTemp1, u8DataKindTemp2, u8DataKindTemp3, u8DataKindTemp4;
	Uint8 u8TriggerSourceTemp, u8TriggerTemp, u8SignTemp;
	Uint16 i, u16LengthTemp, u16CompareValTemp;
	
	u16LengthTemp = u16Scib_CommandLength;
	
	for(i = 0;i < u16LengthTemp;i++)
	{
		u8Scib_UserDataBuf0[i] = u8Scib_CommandBuffer0[i];    // 命令由u8Scib_CommandBuffer0切换u8Scib_UserDataBuf0		
	}

	//Scib_Write(u8Scib_UserDataBuf0, u16LengthTemp);
	SciWrite(ID_SCIA, u8Scib_UserDataBuf0, u16LengthTemp);   // 回送命令
	
	//aaa
	u16SnatchDataCntTemp = (u8Scib_CommandBuffer0[2] - 48) * 100 + (u8Scib_CommandBuffer0[3] - 48) * 10 + u8Scib_CommandBuffer0[4] - 48;
	//bbb
	u16IntervalTemp = (u8Scib_CommandBuffer0[6] - 48) * 100 + (u8Scib_CommandBuffer0[7] - 48) * 10 + u8Scib_CommandBuffer0[8] - 48;
	//cc
	u8DataKindTemp1 = (u8Scib_CommandBuffer0[10] - 48) * 10 + u8Scib_CommandBuffer0[11] - 48;
	//dd
	u8DataKindTemp2 = (u8Scib_CommandBuffer0[13] - 48) * 10 + u8Scib_CommandBuffer0[14] - 48;
	//ee
	u8DataKindTemp3 = (u8Scib_CommandBuffer0[16] - 48) * 10 + u8Scib_CommandBuffer0[17] - 48;
	//ff
	u8DataKindTemp4 = (u8Scib_CommandBuffer0[19] - 48) * 10 + u8Scib_CommandBuffer0[20] - 48;
	//gg
	u8TriggerSourceTemp = (u8Scib_CommandBuffer0[22] - 48) * 10 + u8Scib_CommandBuffer0[23] - 48;
	//h
	u8TriggerTemp = u8Scib_CommandBuffer0[25] - 48;
	//+/-
	u8SignTemp = u8Scib_CommandBuffer0[27] - 48;
	//iiiii
	u16CompareValTemp = (u8Scib_CommandBuffer0[29] - 48) * 10000 + (u8Scib_CommandBuffer0[30] - 48) * 1000 \
	 + (u8Scib_CommandBuffer0[31] - 48) * 100 + (u8Scib_CommandBuffer0[32] - 48) * 10 + u8Scib_CommandBuffer0[32] - 48;
	
	if((u16SnatchDataCntTemp > 500) ||(u16IntervalTemp > 500) ||(u8TriggerTemp > 4)) 
	    
	{
        return;
	}
	
	//OS_ENTER_CRITICAL();
	u16Scib_SaveDataCnt = 0;
	u16Scib_SnatchDataCnt = u16SnatchDataCntTemp;
	u16Scib_Interval = u16IntervalTemp;
	u16Scib_Interval1 = u16Scib_Interval;
	
	u8Scib_DataKind[0] = u8DataKindTemp1;
	u8Scib_DataKind[1] = u8DataKindTemp2;
	u8Scib_DataKind[2] = u8DataKindTemp3;
	u8Scib_DataKind[3] = u8DataKindTemp4;
	u8Scib_wTriggerSource = u8TriggerSourceTemp;
	u8Scib_wTrigger = u8TriggerTemp;
	u8Scib_Sign = u8SignTemp;
	
	u16Scib_CompareVal = u16CompareValTemp;
	u16Scib_TransmitCnt = 0;
	//OS_EXIT_CRITICAL();
}

 /*=============================================================================*
 * FUNCTION: Scib_QDCommand(void)
 * PURPOSE :  Send Data to IPOMS For QD Command
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *    
 *  //Scib_WriteBinary()
 *     
 *     
 *     
 *
 * CALLED BY: 
 *     //INT32 Scib_Parsing() 
 * 
 *============================================================================*/
void Scib_QDCommand(void)
{
	Uint8 u8Temp;
	int16 i;
	int32 i32CheckSum;
	
	u8Temp = u8Scib_CommandBuffer0[2] - 48;   //  行数
	
	if((u8Temp >=4 ) || (0 == u16Scib_TransmitCnt))	
	{
	    return;
    }
	u16Scib_TransmitDataBuff[0] = 0x01;		//SOH
	u16Scib_TransmitDataBuff[1] = u16Scib_TransmitCnt;	//length
	
	i32CheckSum =(int32)(0x01 + u16Scib_TransmitCnt);
	for(i = 0;i < u16Scib_TransmitCnt;i++)
	{
		u16Scib_TransmitDataBuff[2 + i] = u16Scib_GraphDataBuff[u8Temp][i];//把第几行的数据放到发送buffer
		i32CheckSum += u16Scib_GraphDataBuff[u8Temp][i];
	}	
	
	//u16Scib_TransmitDataBuff[2+u16Scib_TransmitCnt]=i32CheckSum>>16;
	u16Scib_TransmitDataBuff[2 + u16Scib_TransmitCnt] = (Uint16)(i32CheckSum & 0x0000FFFF);

	Scib_WriteBinary(u16Scib_TransmitDataBuff, u16Scib_TransmitCnt + 3);
	// 转换为 字节 发送  数据格式  01  长度  数据  +   数据和的低16位
}
 /*=============================================================================*
 * FUNCTION: Scib_SnatchGraph(void)
 * PURPOSE :  SnatchGraph Data in the switching interrupt
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS:
 *    
 *  
 *     
 *     
 *     
 *
 * CALLED BY: 
 *     //INT32 Scib_QDCommand() 
 * 
 *============================================================================*/
Uint8 Scib_SnatchGraph(void)    //  图形数据更新
{
	int16 i,j;
	
	//trigger condition check
	if(0 == u8Scib_SnatchGraphEnable)
	{ 
	  return(false);
	}

	if(0 == u8Scib_wTrigger)      //  上位机使能 命令
	{
	  return(false);
	}
	if(u16Scib_Interval1 > 0)     //  上位机使能 命令  
	{
		if(0 == (--u16Scib_Interval1))
		{
			u16Scib_Interval1 = u16Scib_Interval;   //  上位机使能 命令
		}
		else
		{
			return(false);      //  返回
		}
	}
	
	for(i = 0;i < 2;i++)
	{   
	    j=u8Scib_DataKind[i];     //  每个PWM波周期更新 一列 四个数据
		if(0 == j)
		{		
		    continue;
		}
		u16Scib_GraphDataBuff[i][u16Scib_SaveDataCnt] = (Uint16)GetDataSubArray[j]();  
		// 获取数据存放到  二位数组
	}
        ++u16Scib_SaveDataCnt;
	if(u16Scib_SaveDataCnt == u16Scib_SnatchDataCnt) //u16Scib_SnatchDataCnt  上位机给定数值
	{
		u16Scib_TransmitCnt = u16Scib_SnatchDataCnt;
		u16Scib_SnatchDataCnt = 0;
		u16Scib_SaveDataCnt = 0;
		u8Scib_wTrigger = 0;
		u8Scib_wTriggerSource = 0;
		u8Scib_SnatchGraphEnable = 1;
		return(true);	
	}
	return(false);
}

int16 swGetFault0(void)
{
	return (0);

}	

int16 swGetFault1(void)
{
 return (0);
}

int16 swGetFault2(void)
{
return ( AdcResult.ADCRESULT1);	
}

int16 swGetFault3(void)
{
 //   return(BooostCon_Reg1.i32Boost_Duty);//4  
//  return(CurrConReg.i32Bus_Error_k); //modified by Ken
return(_IQtoF(GetRealValue.iq20IGrid) * 100);
}
int16 swGetFault4(void)
{  return ( AdcResult.ADCRESULT1);
//	return(Calc_Result.i32OutputQ);//5
}
int16 swGetFault5(void)
{
	return(AdcResult.ADCRESULT1);
}

int16 swGetFaultUnrecover1(void)
{
	return(GetRealValue.iq20IGrid * 100);
}
int16 swGetFaultUnrecover2(void)
{
	return(GetRealValue.iq20IGrid * 100);
}
int16 swGetRLineVolt(void)
{
	return(_IQint(GetRealValue.iq20VGrid));
}
int16 swGetSLineVolt(void)
{
	return(GetRealValue.iq20IGrid * 100);
}
int16 swGetTLineVolt(void)
{
	return(GetRealValue.iq20IGrid * 100);
}
int16 swGetRCurr(void)
{
	return(GetRealValue.iq20IGrid * 100);
}

/*=============================================================================*
 * FUNCTION: sbNumToAscii
 * PURPOSE :  Convert input u16Number into max. 8 digital numbers including
 *			  decimal represented byASCII code.
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *     
 *   
 *     
 *
 * CALLED BY: 
 *    
 * Scib_Q1Command   
 *    
 * 
 *============================================================================*/
Uint8 sbNumToAscii(Uint16 u16Number, int8 i8Exponent, Uint8 *pbBuffer)
{
      //   待转化的数据不能超过65535   转化为十进制数  对应的Ascii码
	Uint8 u8No;
	int8 i,j;
    Uint8 bArrayTemp[8] = {0,0,0,0,0,0,0,0};
    Uint32 u32NumberTemp;
	j = 0;

	if((0 == u16Number) || ((u16Number != 0) && (i8Exponent < -6)))
	{
		*pbBuffer = '0';
		u8No = 1;
	}
	else
	{
		u32NumberTemp = (Uint32)u16Number;
		for(i = 0;i < i8Exponent;i++)
		{
			if(u32NumberTemp <= 9999999)
			{
			    u32NumberTemp = u32NumberTemp * 10;
			}
			else 
			{
				u32NumberTemp = 99999999;
			}
		}
		while ((u32NumberTemp > 0) || (i8Exponent < 0))
		{
			i = u32NumberTemp % 10;
			u32NumberTemp = u32NumberTemp / 10;
			bArrayTemp[j] = i + 0x30;
			j++;

			i8Exponent++;
			if(0 == i8Exponent)
			{
				bArrayTemp[j] = '.'; 
				j++;
				if(0 == u32NumberTemp)
				{   
					bArrayTemp[j] = '0'; 
					j++;
				}
			}
		}
		u8No = 0;
		j--;
		while (j >= 0)
		{
			*(pbBuffer + u8No) = bArrayTemp[j];
			u8No++;
			j--;
		}
	}/*	end of number !=0 */
	return(u8No);/* char length*/	   
}

/*=============================================================================*
 * FUNCTION: Scib_WriteBinary
 * PURPOSE :  Write a 16bit data to  Scib Tx Port
 * INPUT: 
 *     void  // TRUE:disable the dog；FALSE:enable the dog
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *    // void sSplit()
 *     
 *   
 *     
 *
 * CALLED BY: 
 *     Scib_QDCommand()
 *    
 *    
 * 
 *============================================================================*/
void Scib_WriteBinary(Uint16 *pstart, Uint16 u16Length)
{
    int8 i;
	int8 j;
	Uint16 *pData;
	Uint8  u8Data;
	
	u8Scib_SendHighHalfByte = 1;	
    pData = pstart;
	for(i = 0;i < u16Length;i++)
	{
	    //split
        //sSplit(*pData);

	    for(j = 0;j < 2;j++)
	    {   
	        if(1 == u8Scib_SendHighHalfByte)
            {   //write high 8bit
				u8Data = GET_HBYTE_OF_WORD(*pData);
				while(SciWrite(ID_SCIA, &u8Data, 1));
                u8Scib_SendHighHalfByte = 0;    
            }
            else
	        {   //write low 8bit
	            u8Data = GET_LBYTE_OF_WORD(*pData);
				while(SciWrite(ID_SCIA, &u8Data, 1));
                u8Scib_SendHighHalfByte = 1; 
     	    }
	    }

        ++pData;
    }
}
