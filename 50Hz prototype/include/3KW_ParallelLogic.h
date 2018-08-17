/*=============================================================================*
 *         Copyright(c) 2010-2011, ALL RIGHTS RESERVED
 *
 *  FILENAME : 3KW_ParallelLogic.h
 *  PURPOSE  : header files 50KW_Regulator.c 
 *			       define constant, struct declaration, extern varibles 
 *
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    
 *
 ******************************************************************************/

#ifndef PARALLEL_LOGIC_H
#define PARALLEL_LOGIC_H



//******************************************************************************
// 220V Output SCR Control
#define INV_SCR_ON					GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;//2017.6.29 GX inverter test
#define INV_SCR_OFF				GpioDataRegs.GPASET.bit.GPIO14 = 1;//2017.6.29 GX inverter test

// 220V Output Relay Control
#define INV_RELY_ON				GpioDataRegs.GPASET.bit.GPIO13 = 1;
#define INV_RELY_OFF				GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;

// Output Sync Signal
#define SYNC_COM1_ON				GpioDataRegs.GPASET.bit.GPIO27 = 1;
#define SYNC_COM1_OFF				GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

// Inverter Parallel Signal
#define SYNC_COM2_ON				GpioDataRegs.GPBSET.bit.GPIO51 = 1;
#define SYNC_COM2_OFF				GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;

#define SYNC_COM2_LEVEL				GpioDataRegs.GPADAT.bit.GPIO26

// Inverter Parallel Control
#define Inv_CurrShare_ON			GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
#define Inv_CurrShare_OFF			GpioDataRegs.GPASET.bit.GPIO25 = 1;

#define SyncPhase_Hi1Limit          0.53f * 6.283185307
#define	SyncPhase_Low1Limit         0.47f * 6.283185307
#define	SyncPhase_Hi2Limit          0.51f * 6.283185307
#define	SyncPhase_Low2Limit         0.49 * 6.283185307
#define SyncPhase_Hi3Limit          0.55f * 6.283185307
#define	SyncPhase_Low3Limit         0.45f * 6.283185307
#define SyncPhase_ReguStep1         0.001f * 6.283185307
#define SyncPhase_ReguStep2         0.01f * 6.283185307
#define SyncPhase_Hi4Limit          0.503f * 6.283185307
#define	SyncPhase_Low4Limit         0.497f * 6.283185307

//******************************************************************************
struct PARALLEL_REG
{
	Uint16 	u16Cnt_COM1_Receive;
	Uint16  u16Cnt_SCR_ON;
	Uint16 	u16Cnt_COM1_Receive_max;  //2017.5.23 GX
};

extern  struct PARALLEL_REG  Parallel_Reg;



union  PARALLEL_STATE
{
    Uint16 all:16;
    struct
    {
        Uint16  byte0:8;
        Uint16  byte1:8;
    }Word;

    struct
    {
        Uint16  SyncPhase_Flag:1;    // B0		1: SyncPhase is Ok
        Uint16  InvSoftStart_EN:1;    // B1
        Uint16  B2:1;    // B2
        Uint16  B3:1;    // B3
        Uint16  B4:1;    // B4
        Uint16  B5:1;    // B5
        Uint16  B6:1;    // B6
        Uint16  B7:1;    // B7     0: hardware  interrupt   1:softwar force  interrupt

        Uint16  SelfPhaseOut_EN:1;    // B0
        Uint16  :1;      //B1
        Uint16  :1; //B2
        Uint16  B11:1;     //B3
        Uint16  B12:1;     //B4
        Uint16  B13:1;  //B5
        Uint16  B14:1; //B6
        Uint16  B15:1;    //B7
    }bit;
};
extern union PARALLEL_STATE  g_ParaLogic_State;



/*=============================================================================*
 * 	Extern functions declaration
 *============================================================================*/

/* Initialize the starting Duty and loop PID parameters */
extern void ECAP1_INT_SyncPhase_Control(void);
extern void SyncLogic_Control(void);
extern void InvParallel_Control(void);
extern void InvRelay_Control(void);

#endif

//--- end of file -----------------------------------------------------
