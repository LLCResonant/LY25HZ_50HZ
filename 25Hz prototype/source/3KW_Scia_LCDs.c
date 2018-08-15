/*=============================================================================*
 *         Copyright(c) 
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : 20KW_Scia_Interface.c
 *   
 *  PURPOSE  : SCIa for PC monitoring
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *     
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

#include "DSP2833x_Device.h"			// Peripheral address definitions
#include "3KW_MAINHEADER.h"			 // Main include file

Uint16 ComRcvData[10]={0,0,0,0,0,0,0,0,0,0}; 
Uint16 ComSendData[10]= {0,0,0,0,0,0,0,0,0,0};
Uint16 u16TxBuf[30];
Uint16 u16RxBuf[10];

struct COMFLAG_BITS ComFlag;
struct COMCNT ComCnt;

//extern volatile union ERRORFLAG ErrorFlag;

void TSK_SCI_A(void)
{

	Uint8 u8Tmp;
	Uint8 u8Data;
	static Uint8 u8Index = 0;


	while(1)
	{
		if(SEM_pend(&SEM_SCIAComm, SYS_FOREVER) == 1) 
		{
			while(1)
			{
				u8Tmp = SciRead(ID_SCIB, &u8Data);
				if(!u8Tmp)
				{
					switch(u8Index)
					{
					case 0:
						if(u8Data == 0x03||u8Data == 0x00)
						{
							ComRcvData[u8Index] = u8Data;
							u8Index++;
						}
						break;
					case 1:
						if(u8Data == 0x03||u8Data == 0x06||u8Data == 0x04)    //can cut down
						{
							ComRcvData[u8Index] = u8Data;
							u8Index++;
						}
						else
						{
							u8Index = 0;
						}
						break;
					default:                        //每500ms读一组数据
						ComRcvData[u8Index] = u8Data;
						u8Index++;
						if(u8Index == 8)
						{							
							ComFlag.SciRcvStart=0;	
						}
						break;
					}
				}
				else
				{
//					u8CommLoseFaultCnt++;
					break;
				}
			}
						
			if(ComFlag.SciRcvStart==0)
			{
				ComCnt.ComRecByteNum = u8Index;
				u8Index = 0;

	    		ComRcvDeal();

				ComFlag.SciRcvStart=1; 
			}
		}		
	}
}


void ComOverTimeDeal()
{
	if(ComFlag.OverTimeDelay>=500)
	{
		ComFlag.OverTimeFlag=1;
	}
}

void SetValue_StateRefresh()
{
Uint8 i = 0;	
	
	u16TxBuf[i++] = 100;													  //Version
	u16TxBuf[i++] = Calc_Result.f32VOutH_rms;
	if (Calc_Result.f32IOutH_rms * 10 < 5)
		u16TxBuf[i++] = 0;
	else
		u16TxBuf[i++] = (Uint16)(Calc_Result.f32IOutH_rms * 10 + 0.5);

	if (g_Sys_Current_State == NormalState )
		u16TxBuf[i++] = Calc_Result.f32VoutHFreq * 10;
	else
		u16TxBuf[i++] = 0;

	u16TxBuf[i++] = 0;
	u16TxBuf[i++] = Calc_Result.f32TempInvH * 10;
	u16TxBuf[i++] = Calc_Result.f32VOutL_rms;
	if (Calc_Result.f32IOutL_rms * 10 < 5)
		u16TxBuf[i++] = 0;
	else
		u16TxBuf[i++] = Calc_Result.f32IOutL_rms * 10;

	if (g_Sys_Current_State == NormalState )
		u16TxBuf[i++] = (Uint16)(Calc_Result.f32VoutLFreq * 10 + 0.5);
	else
		u16TxBuf[i++] = 0;

	u16TxBuf[i++] = Calc_Result.f32Phase_Diff * 10;
	u16TxBuf[i++] = Calc_Result.f32TempInvL * 10;
	u16TxBuf[i++] = Calc_Result.f32VGrid_rms;
	u16TxBuf[i++] = Calc_Result.f32IGrid_rms * 10;
	u16TxBuf[i++] = Calc_Result.f32GridFreq * 10;
	u16TxBuf[i++] = 0 ;
	u16TxBuf[i++] = g_SysWarningMessage.Word.byte0;
	u16TxBuf[i++] = g_SysWarningMessage.Word.byte1;
	u16TxBuf[i++] = g_SysFaultMessage.Word.byte0;
	u16TxBuf[i++] = g_SysFaultMessage.Word.byte1;
	u16TxBuf[i++] = g_SysFaultMessage.Word.byte2;
	u16TxBuf[i++] = g_SysFaultMessage.Word.byte3;
	u16TxBuf[i++] = g_SysFaultMessage.Word.byte4;
	u16TxBuf[i++] = g_SysFaultMessage.Word.unrecover1;
	u16TxBuf[i++] = g_SysFaultMessage.Word.unrecover2;
	u16TxBuf[i++] = g_SysFaultMessage.Word.unrecover3;
}

void SetVaule_ControlRefresh()
{
 static Uint8 s_u8_flag_InitRxBuf = 0;
 Uint16	i;
 Uint16 length = 2;
 if (g_Mod_Status == Master)
 {
	 u16TxBuf[0] = SafetyReg.f32InvH_VoltRms_Ref_LCD;
	 u16TxBuf[1]	= SafetyReg.f32InvL_VoltRms_Ref_LCD;
 }
 else
 {
	 u16TxBuf[0] = SafetyReg.f32InvH_VoltRms_Ref;
	 u16TxBuf[1]	= SafetyReg.f32InvL_VoltRms_Ref;
 }
	
	if(0 == s_u8_flag_InitRxBuf)
	{
		for(i=0;i<length;i++)
		{
		u16RxBuf[i] = u16TxBuf[i];
		}
		s_u8_flag_InitRxBuf = 1;
	}
}


void GetVaule_ControlRefresh( Uint16 address)
{
	if (g_Mod_Status == Master)
	{
		if(address == 0)
		{
			if( u16RxBuf[0] > 211 && u16RxBuf[0] < 229 )
				SafetyReg.f32InvH_VoltRms_Ref_LCD = u16RxBuf[0];
		}
		else if(address == 1)
		{
			if( u16RxBuf[1] > 105 && u16RxBuf[1] < 115)
				SafetyReg.f32InvL_VoltRms_Ref_LCD = u16RxBuf[1];
		}
		else
			;

		Output_VoltRe_Reg.InvL_Light_Flag = 0;
		Output_VoltRe_Reg.InvL_Middle_Flag = 0;
		Output_VoltRe_Reg.InvL_Heavy_Flag = 0;
		Output_VoltRe_Reg.InvH_Light_Flag = 0;
		Output_VoltRe_Reg.InvH_Middle_Flag = 0;
		Output_VoltRe_Reg.InvH_Heavy_Flag = 0;
	}
	else
		;
}


/* ========= Send Byte=======*/
void ComSendByte(unsigned int data)
{
    Uint16 SendDelay=0x0;
	
	ScibRegs.SCITXBUF=data;
	while(ScibRegs.SCICTL2.bit.TXEMPTY == 0)               /*发送数据*/
    {
       SendDelay++;
       if(SendDelay>=COM_DELAY_TIME)
       {
       	  SendDelay=0;
       	  break;
       }          
    }

    SendDelay=0;          
}
/*=====Send Word========*/
void ComSendWords(unsigned int *addr,unsigned int datanum)
{   int i;
	unsigned int SendDataH;
	unsigned int SendDataL;
	if((datanum>0)&&(datanum<90))
	{
		for(i=0;i<datanum;i++)
  	   {  
  		SendDataL=((*(addr+i))&0x00ff);
  		SendDataH=(((*(addr+i))&0xff00)>>8);  
  		ComSendByte(SendDataH);                           
        ComSendByte(SendDataL);         
  	   }
    } 
} 

void ComRcvDeal()
{
	Uint16 *ptCom;
	Uint16 i;
	Uint16 CRCinit=0xFFFF;
	Uint16 CRCres=0x0;
	Uint16 rx_crc=0x0;
	Uint16 crcjisuan=0x0;
	Uint16 rx_command=0x0;
	Uint16 rx_runnum=0x0;
	Uint16 rx_start=0x0;
	Uint16 ADDRinit=0x0;
	static Uint8 u8_Num = 0; //2017.11.21 GX

    if((ComCnt.ComRecByteNum>2)&&(ComCnt.ComRecByteNum<10))
	{	
	    COM_SEND_EN;

		if((ComRcvData[0]==0x03)||(ComRcvData[0]==0x00))  
		{  
		    ComSendData[0]=ComRcvData[0];    //地址码和功能码原样返回
			ComSendData[1]=ComRcvData[1];
			rx_command=ComRcvData[1];
			rx_start=ComRcvData[2];
			rx_start=((rx_start<<8)&0xFF00)|ComRcvData[3];
		    ADDRinit=rx_start/2;

		    if (ComRcvData[0]==0x00)
		    {
		    	if (ComRcvData[3] >= 1 && ComRcvData[3] <= 63 && u8_Num == 0) //2017.12.23 GX
		    	{
		    		ModuleAdd = ComRcvData[3];  //子模块地址  //2017.11.21 GX
		    		InitECana();
		    		u8_Num = 1;
		    	}
		    }

			rx_runnum=ComRcvData[4];
			rx_runnum=((rx_runnum<<8)&0xFF00)|ComRcvData[5];  //数据个数
			ComSendData[2]=rx_runnum * 2;  //数据个数乘2为字节数
		    rx_crc=ComRcvData[7];
			rx_crc=((rx_crc<<8)&0xFF00)|ComRcvData[6];

			crcjisuan=CRCB16(CRCinit,&ComRcvData[0],6);
			if(crcjisuan==rx_crc)
			{
			 	if(rx_command==COM_CODE_READ_CONTROL)		// 0x03
				{	
					
					SetVaule_ControlRefresh();

					ptCom = &u16TxBuf[0]+ADDRinit;
					CRCres = CRCB16(0xffff,&ComSendData[0],3);
					CRCres = CRCW16(CRCres,ptCom,rx_runnum);
					ComSendData[3]=(CRCres&0x00ff);
					ComSendData[4]=(CRCres>>8); 
											
				   for(i=0;i<3;i++)
				   {
						ComSendByte(ComSendData[i]);
				   }
				   ComSendWords(ptCom,rx_runnum);
				   ComSendByte(ComSendData[3]);
				   ComSendByte(ComSendData[4]);
				   

				   DelayUs(300);
                
				   ComFlag.OverTimeDelay=0;
				   ComFlag.OverTimeFlag=0;

				}
			    else if(rx_command==COM_CODE_WRITE_CONTROL)			// 0x06
			    {	 
	 				ptCom=&u16RxBuf[0]+ADDRinit; 
	 			   *ptCom=rx_runnum;             					//将接收数据合成后存入相应单元     

	 			   GetVaule_ControlRefresh(ADDRinit);
	 			   ComCnt.SaveFactorySettings = 1;

	        	   for(i=0;i<8;i++)                        //接收正确数据原样返回
				   {
					   ComSendByte(ComRcvData[i]);
				   }
								
				   ComFlag.OverTimeDelay=0;
				   ComFlag.OverTimeFlag=0;
				}

				else if(rx_command==COM_CODE_READ_STATE)		// 0x04, 0x74
   				{
					
					SetValue_StateRefresh();

					ptCom = &u16TxBuf[0]+ADDRinit;
					CRCres=CRCB16(0xffff,&ComSendData[0],3);
					CRCres=CRCW16(CRCres,ptCom,rx_runnum);

					ComSendData[3]=(CRCres&0x00ff);
					ComSendData[4]=(CRCres>>8); 									
				   for(i=0;i<3;i++)
				   {
						ComSendByte(ComSendData[i]);
				   }

				   ComSendWords(ptCom,rx_runnum);

				   ComSendByte(ComSendData[3]);
				   ComSendByte(ComSendData[4]);
				   DelayUs(300);
//				   DelayMN(0xffff,1);

				   ComFlag.OverTimeDelay=0;
				   ComFlag.OverTimeFlag=0;
				}
			 }

			 else if((rx_command==COM_CODE_READ_STATE)||(rx_command==COM_CODE_READ_CONTROL))
			 {				 
			     ComRcvData[0]|=0x80;
				 ComRcvData[2]=0;
				 CRCres=CRCB16(0xffff,&ComRcvData[1],3);
				 ComRcvData[3]=(CRCres&0x00ff);
				 ComRcvData[4]=(CRCres>>8);
				 for(i=0;i<5;i++)
			     {
					ComSendByte(ComRcvData[i]);
				 }	 				
				 ComFlag.OverTimeDelay=0;
				 ComFlag.OverTimeFlag=0;
			 }
			 else
		     {
		          ;
			 }
	   }	        
	}

//	DelayN(0xff);
    DelayUs(15);

    COM_RCV_EN;
	

}

/*======字节型的CRC校验=======*/
unsigned int CRCB16(unsigned int CRCini,unsigned int *ADDRini,unsigned int BITnum)
{  
	unsigned int i,j;
	if((BITnum>0)&&(BITnum<90))
	{	
		for(i=0;i<BITnum;i++)
		{
			CRCini=CRCini^(*(ADDRini+i));
			for(j=0;j<8;j++)
			{			
				if((CRCini&0x1)==1)
				{
					CRCini=CRCini>>1;
					CRCini^=0xA001;
				} 
				else 
				{
				  CRCini=CRCini>>1;
				}	
			}
		}
	}
	else
	{
		CRCini=0;
	}	
	return(CRCini);
}

/*==========字型CRC校验=============*/
unsigned int CRCW16(unsigned int CRCini,unsigned int *ADDRini,unsigned int BITnum)
{   unsigned int i,j,k;
	unsigned int CRCtempH;
	unsigned int CRCtempL;
	if((BITnum>0)&&(BITnum<90))
	{
		for(i=0;i<BITnum;i++)
		{
			CRCtempL=((*(ADDRini+i))&0x00ff);
			CRCtempH=(((*(ADDRini+i))&0xff00)>>8);	
			CRCini^=CRCtempH;
			for(j=0;j<8;j++)
			{			
				if((CRCini&0x1)==1)
				{   
					CRCini=CRCini>>1;
					CRCini^=0xA001;
				}
				else 
				{
					CRCini=CRCini>>1;
				}	
			}
			CRCini^=CRCtempL;
			for(k=0;k<8;k++)
			{
				
				if((CRCini&0x1)==1)
				{
					CRCini=CRCini>>1;
					CRCini^=0xA001;
				}
				else
				{
					CRCini=CRCini>>1;
				}	
			}
		}
	}
	else
	{
		CRCini=0;
	}
	return(CRCini);
} 






//--- end of file -----------------------------------------------------
