//###########################################################################
//
// FILE:  Flash28335_API.c	
//
// TITLE: F28335 Flash API Example
//
// NOTE:  This example runs from Flash.  First program the example
//        into flash.  The code will then copy the API's to RAM and 
//        modify the flash. 
//
//
//###########################################################################
// $TI Release: F28335 API Release V1.0 $
// $Release Date:
//###########################################################################


/*---- Flash API include file -------------------------------------------------*/
#include "Flash28335_API_Library.h"

/*---- example include file ---------------------------------------------------*/
#include "Flash28335_API.h"

/*---- Standard headers -------------------------------------------------------*/

#include "DSP2833x_Device.h"		// Peripheral address definitions
#include "3KW_MAINHEADER.h"				// Main include file

#include <stdio.h>     

/*---------------------------------------------------------------------------
  Data/Program Buffer used for testing the flash API functions
---------------------------------------------------------------------------*/
#define  WORDS_IN_FLASH_BUFFER 0x100               // Programming data buffer, Words
Uint16  Buffer[WORDS_IN_FLASH_BUFFER];

/*---------------------------------------------------------------------------
  Sector address info
---------------------------------------------------------------------------*/
typedef struct {
     Uint16 *StartAddr;
     Uint16 *EndAddr;
} SECTOR;

typedef struct
{
	Uint32 BasicAddr;
	Uint16 u16Offset;
}FLASH_ADDR;

#define OTP_START_ADDR  0x380400
#define OTP_END_ADDR    0x3807FF


#if FLASH_F28335
#define FLASH_START_ADDR  0x300000
#define FLASH_END_ADDR    0x33FFFF

SECTOR Sector[8] = {
         (Uint16 *)0x338000,(Uint16 *)0x33FFFF,
         (Uint16 *)0x330000,(Uint16 *)0x337FFF,
         (Uint16 *)0x328000,(Uint16 *)0x32FFFF,
         (Uint16 *)0x320000,(Uint16 *)0x327FFF,
         (Uint16 *)0x318000,(Uint16 *)0x31FFFF,
         (Uint16 *)0x310000,(Uint16 *)0x317FFF,
         (Uint16 *)0x308000,(Uint16 *)0x30FFFF,
         (Uint16 *)0x300000,(Uint16 *)0x307FFF

};

#endif
                         
                            
/*--- Global variables used to interface to the flash routines */
extern Uint32 Flash_CPUScaleFactor;
static FLASH_ST FlashStatus;
static Uint16 u16Flag;
static FLASH_ADDR FlashAddr;
static Uint8 szFlashDataBuffer[64];
static volatile struct SCI_REGS *pSciReg = &ScibRegs;

void MemMoveForFlashProgramming();
void FlashProgramming_TSK();

static void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
static Uint16 InitPLLForFlashAPI();
static void ParsingSciRxForFlashProgramming();
static Uint8 CalCheckSumForFlashProgramming(Uint8* pBuf, Uint16 len);
static void SciFlashCallbackFunc(void);
static Uint16 GetBasicAddr();
static Uint16 FlashDataProgramming();
static void SoftReset();
static void ServiceDog();
static Uint16 CsmUnlock();
static void SendInfo(Uint16 u16Code);
static Uint16 CheckAddr(Uint32 u32Addr);
static Uint16 FlashErasing(void);

#pragma CODE_SECTION(FlashProgramming_TSK,"ramfuncs")
#pragma CODE_SECTION(InitPLLForFlashAPI,"ramfuncs")
#pragma CODE_SECTION(ParsingSciRxForFlashProgramming, "ramfuncs")
#pragma CODE_SECTION(CalCheckSumForFlashProgramming, "ramfuncs")
#pragma CODE_SECTION(SciFlashCallbackFunc, "ramfuncs")
#pragma CODE_SECTION(GetBasicAddr, "ramfuncs")
#pragma CODE_SECTION(FlashDataProgramming, "ramfuncs")
#pragma CODE_SECTION(SoftReset,"ramfuncs")
#pragma CODE_SECTION(ServiceDog,"ramfuncs")
#pragma CODE_SECTION(CsmUnlock,"ramfuncs")
#pragma CODE_SECTION(SendInfo,"ramfuncs")
#pragma CODE_SECTION(CheckAddr,"ramfuncs")
#pragma CODE_SECTION(FlashErasing,"ramfuncs")

//#pragma DATA_SECTION(szFlashDataBuffer, "ramdata")
//#pragma DATA_SECTION(u16Flag, "ramdata")

void FlashProgramming_TSK()
{
	Uint16 Status;
	Status = InitPLLForFlashAPI();
	if(Status != STATUS_SUCCESS) 
	{
	   //SendInfo(Status);
	   return;
	}
	 u16Flag = 0;
	 
	 /*------------------------------------------------------------------
 	Unlock the CSM.
    If the API functions are going to run in unsecured RAM
    then the CSM must be unlocked in order for the flash 
    API functions to access the flash.
   
    If the flash API functions are executed from secure memory 
    (L0-L3) then this step is not required.
------------------------------------------------------------------*/
  
	Status = CsmUnlock();
	if(Status != STATUS_SUCCESS) 
	{
	   //SendInfo(Status);
	   return;
	}


	/*------------------------------------------------------------------
	Copy API Functions into SARAM

	The flash API functions MUST be run out of internal 
	zero-waitstate SARAM memory.  This is required for 
	the algos to execute at the proper CPU frequency.
	If the algos are already in SARAM then this step
	can be skipped.  
	DO NOT run the algos from Flash
	DO NOT run the algos from external memory
	------------------------------------------------------------------*/


	/*------------------------------------------------------------------
	Initalize Flash_CPUScaleFactor.

	Flash_CPUScaleFactor is a 32-bit global variable that the flash
	API functions use to scale software delays. This scale factor 
	must be initalized to SCALE_FACTOR by the user's code prior
	to calling any of the Flash API functions. This initalization
	is VITAL to the proper operation of the flash API functions.  

	SCALE_FACTOR is defined in Example_Flash2833x_API.h as   
	 #define SCALE_FACTOR  1048576.0L*( (200L/CPU_RATE) )
	 
	This value is calculated during the compile based on the CPU 
	rate, in nanoseconds, at which the algorithums will be run.
	------------------------------------------------------------------*/

	Flash_CPUScaleFactor = SCALE_FACTOR;

	/*------------------------------------------------------------------
	Initalize Flash_CallbackPtr.

	Flash_CallbackPtr is a pointer to a function.  The API uses
	this pointer to invoke a callback function during the API operations.
	If this function is not going to be used, set the pointer to NULL
	NULL is defined in <stdio.h>.  
	------------------------------------------------------------------*/

	// Jump to SARAM and call the Flash API functions
	OS_ENTER_CRITICAL();
	//ResetScib();
	InitScib(FALSH_PROGRAM_BAUDRATE, MODE_INQUIRE);

	Flash_CallbackPtr = &SciFlashCallbackFunc; 
	u16Flag = 1;
	(*Flash_CallbackPtr)();
}

static Uint16 InitPLLForFlashAPI()
{
	if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 1)
   {
       if (SysCtrlRegs.PLLCR.bit.DIV != PLLCR_VALUE)
       {
   
          
          EALLOW;
          // Before setting PLLCR turn off missing clock detect
          SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
          SysCtrlRegs.PLLCR.bit.DIV = PLLCR_VALUE;
          EDIS;
   
          // Wait for PLL to lock.
          // During this time the CPU will switch to OSCCLK/2 until
          // the PLL is stable.  Once the PLL is stable the CPU will 
          // switch to the new PLL value. 
          //
          // This time-to-lock is monitored by a PLL lock counter.   
          //   
          // The watchdog should be disabled before this loop, or fed within 
          // the loop.   
   
          EALLOW;
          SysCtrlRegs.WDCR= 0x0068;
          EDIS;
   
          // Wait for the PLL lock bit to be set.  
          // Note this bit is not available on 281x devices.  For those devices
          // use a software loop to perform the required count. 
   
          while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1) { }
          
          EALLOW;
          SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
          EDIS;
       }
	   return STATUS_SUCCESS;
   }
   
   // If the PLL is in limp mode, shut the system down
   else 
   {
      // Replace this line with a call to an appropriate
      // SystemShutdown(); function. 
      return  WRONG_PLL_MODE;
   }
}


/*------------------------------------------------------------------
   CsmUnlock

   Unlock the code security module (CSM)
 
   Parameters:
  
   Return Value:
 
            STATUS_SUCCESS         CSM is unlocked
            STATUS_FAIL_UNLOCK     CSM did not unlock
        
   Notes:
     
-----------------------------------------------------------------*/
static Uint16 CsmUnlock()
{
    volatile Uint16 temp;
    
    // Load the key registers with the current password
    // These are defined in Example_Flash2833x_CsmKeys.asm
    
    EALLOW;
    CsmRegs.KEY0 = PRG_key0;
    CsmRegs.KEY1 = PRG_key1;
    CsmRegs.KEY2 = PRG_key2;
    CsmRegs.KEY3 = PRG_key3;
    CsmRegs.KEY4 = PRG_key4;
    CsmRegs.KEY5 = PRG_key5;
    CsmRegs.KEY6 = PRG_key6;
    CsmRegs.KEY7 = PRG_key7;   
    EDIS;

    // Perform a dummy read of the password locations
    // if they match the key values, the CSM will unlock 
        
    temp = CsmPwl.PSWD0;
    temp = CsmPwl.PSWD1;
    temp = CsmPwl.PSWD2;
    temp = CsmPwl.PSWD3;
    temp = CsmPwl.PSWD4;
    temp = CsmPwl.PSWD5;
    temp = CsmPwl.PSWD6;
    temp = CsmPwl.PSWD7;

    // If the CSM unlocked, return succes, otherwise return
    // failure.
    if ( (CsmRegs.CSMSCR.all & 0x0001) == 0) 
    {
    	return STATUS_SUCCESS;
	}
    else 
    {
    	return STATUS_FAIL_CSM_LOCKED;
	}
    
}

void MemMoveForFlashProgramming()
{
	// Copy the Flash API functions to SARAM
    MemCopy(&Flash28_API_LoadStart, &Flash28_API_LoadEnd, &Flash28_API_RunStart);

    // We must also copy required user interface functions to RAM. 
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

	//MemCopy(&RamdataLoadStart, &RamdataLoadEnd, &RamdataRunStart);
}

/*------------------------------------------------------------------
  Simple memory copy routine to move code out of flash into SARAM
-----------------------------------------------------------------*/

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    { 
       *DestAddr++ = *SourceAddr++;
    }
    return;
}

/*------------------------------------------------------------------
 
-----------------------------------------------------------------*/

static Uint16 FlashErasing(void)
{

   Uint16  Status;
   float32 Version;        // Version of the API in floating point
   Uint16  VersionHex;     // Version of the API in decimal encoded hex

/*------------------------------------------------------------------
  Check the version of the API
  
  Flash_APIVersion() returns the version in floating point.
  FlashAPIVersionHex() returns the version as a decimal encoded hex.
  
  FlashAPIVersionHex() can be used to avoid processing issues
  associated with floating point values.    
------------------------------------------------------------------*/
   VersionHex = Flash_APIVersionHex();
   if(VersionHex != 0x0210)
   {
       // Unexpected API version
	   return WRONG_FLASHAPI_VERSION;
   }   


   Version = Flash_APIVersion();
   if(Version != (float32)2.10)
   {
       // Unexpected API version
       return WRONG_FLASHAPI_VERSION;
   }

/*------------------------------------------------------------------
  Before programming make sure the sectors are Erased. 

------------------------------------------------------------------*/

   // Example: Erase Sector B,C
   // Sectors A and D have the example code so leave them 
   // programmed.
   
   // SECTORB, SECTORC are defined in Flash2833x_API_Library.h
   //Status = Flash_Erase((SECTORB|SECTORC),&FlashStatus);
	Status = Flash_Erase(0xFF, &FlashStatus);
	if(Status != STATUS_SUCCESS) 
	{
	   return Status;
	} 
	u16Flag = 1;
	return Status;
}

static void SciFlashCallbackFunc(void)
{
	Uint16 u16RxD;
	static Uint16 u16Index = 0;
	while(u16Flag)
	{
		while(pSciReg->SCIFFRX.bit.RXFFST == 0) 
		{
		}
		u16RxD = pSciReg->SCIRXBUF.all;
		switch(u16Index)
		{
		case 0:
			if(':' == u16RxD)
			{
				szFlashDataBuffer[u16Index] = u16RxD;
				u16Index++;
			}
			break;
		case 1:
			szFlashDataBuffer[u16Index] = u16RxD;
			u16Index++;
			break;
		case 2:
			szFlashDataBuffer[u16Index] = u16RxD;
			u16Index++;
			break;
		case 3:
			szFlashDataBuffer[u16Index] = u16RxD;
			u16Index++;
			break;
		case 4:
			if(szFlashDataBuffer[0] == ':' && (u16RxD == 0x00 || u16RxD == 0x01 || u16RxD == 0x04 || u16RxD == 0x10))
			{
				szFlashDataBuffer[u16Index] = u16RxD;
				u16Index++;
			}
			else
			{
				u16Index = 0;
			}
			break;
		default:
			szFlashDataBuffer[u16Index] = u16RxD;
			u16Index++;
			if(szFlashDataBuffer[1] + 6 == u16Index)
			{
				u16Index = 0;	
				ParsingSciRxForFlashProgramming();
			}
			else
			{
				if(u16Index > 64)
				{
					u16Index = 0;
				}
			}
			break;
		}
	}
}

static void ParsingSciRxForFlashProgramming()
{
	Uint16 Status;
	if( szFlashDataBuffer[szFlashDataBuffer[1] + 5] == CalCheckSumForFlashProgramming(&szFlashDataBuffer[1], szFlashDataBuffer[1] + 4) )
	{
		switch(szFlashDataBuffer[4])
		{
		case 0x00:
			u16Flag = 0;
			Status = FlashDataProgramming();
			SendInfo( Status );
			/*if(Status != STATUS_SUCCESS) 
			{
			    SendErrorInfo( Status );
			}
			else
			{
				//ScibWrite(szFlashDataBuffer, szFlashDataBuffer[1] + 6);
				SciWrite(ID_SCIB, szFlashDataBuffer, szFlashDataBuffer[1] + 6);
			}*/
			break;
		case 0x01:
			//ScibWrite(szFlashDataBuffer, szFlashDataBuffer[1] + 6);
			//SciWrite(ID_SCIB, szFlashDataBuffer, szFlashDataBuffer[1] + 6);
			SendInfo( Status );
			SoftReset();
			break;
		case 0x04:
			Status = GetBasicAddr();
			SendInfo( Status );
			/*if(Status != STATUS_SUCCESS) 
			{
			    SendErrorInfo( Status );
			}
			else
			{
				//ScibWrite(szFlashDataBuffer, szFlashDataBuffer[1] + 6);
				SciWrite(ID_SCIB, szFlashDataBuffer, szFlashDataBuffer[1] + 6);
			}*/
			break;
		case 0x10:
			u16Flag = 0;
			Status = FlashErasing();
			SendInfo( Status );
			/*if(Status != STATUS_SUCCESS) 
			{
			    SendErrorInfo( Status );
			}
			else
			{
				//ScibWrite(szFlashDataBuffer, szFlashDataBuffer[1] + 6);
				SciWrite(ID_SCIB, szFlashDataBuffer, szFlashDataBuffer[1] + 6);
			}*/
			break;
		default:
			break;
		}
	}
	else
	{
		SendInfo( CHECKSUM_ERROR );
	}
}

static Uint16 GetBasicAddr()
{
	FlashAddr.u16Offset = 0;
	FlashAddr.BasicAddr = ( szFlashDataBuffer[5] << 8 ) | szFlashDataBuffer[6];
	return CheckAddr( FlashAddr.BasicAddr << 16 );
}

static Uint16 FlashDataProgramming()
{
	Uint16 i;
	Uint16 Status;
	Uint16* addr;
	FlashAddr.u16Offset = ( szFlashDataBuffer[2] << 8 ) | szFlashDataBuffer[3];
	Status = CheckAddr( ( FlashAddr.BasicAddr << 16 ) | FlashAddr.u16Offset );
	if(Status != STATUS_SUCCESS) 
	{
	    u16Flag = 1;
	    return Status;
	}
	addr = (Uint16 *) (( FlashAddr.BasicAddr << 16 ) | FlashAddr.u16Offset);
	for( i=0;i<(szFlashDataBuffer[1]>>1);i++ )
	{
		Buffer[i] = ( szFlashDataBuffer[(i<<1)+5] << 8 ) + szFlashDataBuffer[(i<<1)+6];
	}
	Status = Flash_Program(addr,Buffer,szFlashDataBuffer[1]>>1,&FlashStatus);
    if(Status != STATUS_SUCCESS) 
	{
	    u16Flag = 1;
	    return Status;
	}
	Status = Flash_Verify(addr,Buffer,szFlashDataBuffer[1]>>1,&FlashStatus);
    if(Status != STATUS_SUCCESS)
    {
        u16Flag = 1;
        return Status;
    }
	u16Flag = 1;
	return STATUS_SUCCESS;
}

static Uint8 CalCheckSumForFlashProgramming(Uint8* pBuf, Uint16 len)
{
	Uint16 i;
	Uint16 u16Sum;
	u16Sum = 0;
	for(i = 0;i < len;i++)
	{
		u16Sum = u16Sum + *(pBuf + i);
	}
	u16Sum = ~u16Sum;
	u16Sum += 1;
	return ( u16Sum & 0x00FF );
}
     
static void ServiceDog()
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;
}

static void SoftReset()
{
	ServiceDog();
	// Enable the watchdog
	EALLOW;
	SysCtrlRegs.WDCR = 0x0028;  
	EDIS;
}

static void SendInfo(Uint16 u16Code)
{
	Uint8 szResponseBuf[7];
	szResponseBuf[0] = 0x3A;
	szResponseBuf[1] = 0x01;
	szResponseBuf[2] = 0x00;
	szResponseBuf[3] = 0x00;
	szResponseBuf[4] = 0x80 + szFlashDataBuffer[4];
	szResponseBuf[5] = u16Code;
	szResponseBuf[6] = CalCheckSumForFlashProgramming(&szResponseBuf[1], 5);
	//ScibWrite( szResponseBuf, 7 );
	SciWrite(FLASH_COM, szResponseBuf, 7);
}

static Uint16 CheckAddr(Uint32 u32Addr)
{
	if( u32Addr > FLASH_END_ADDR || u32Addr < FLASH_START_ADDR )
	{
		return INVALID_ADDRESS;
	}
	else
	{
		return STATUS_SUCCESS;
	}
}
