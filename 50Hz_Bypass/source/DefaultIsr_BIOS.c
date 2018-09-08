/**********************************************************************
* File: DefaultIsr_nonBIOS.c
* Devices: TMS320F2803x
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   07/02/09 - original (D. Alter)
**********************************************************************/

#include "F28035_example.h"				// Main include file

//---------------------------------------------------------------------
void INT13_ISR(void)							// 0x000D1A  INT13 (CPU Timer1)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void INT14_ISR(void)							// 0x000D1C  INT14 (CPU Timer2)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void DATALOG_ISR(void)							// 0x000D1E  DATALOG  (CPU data logging interrupt)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void RTOSINT_ISR(void)							// 0x000D20  RTOSINT (CPU RTOS interrupt)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EMUINT_ISR(void) 							// 0x000D22  EMUINT (CPU emulation interrupt)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void NMI_ISR(void)								// 0x000D24  NMI (XNMI interrupt)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ILLEGAL_ISR(void)							// 0x000D26  ILLEGAL (Illegal operation trap)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER1_ISR(void)							// 0x000D28  USER1 (Software #1)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER2_ISR(void)							// 0x000D2A  USER2 (Software #2)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER3_ISR(void)							// 0x000D2C  USER3 (Software #3)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER4_ISR(void)							// 0x000D2E  USER4 (Software #4)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER5_ISR(void)							// 0x000D30  USER5 (Software #5)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER6_ISR(void)							// 0x000D32  USER6 (Software #6)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER7_ISR(void)							// 0x000D34  USER7 (Software #7)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER8_ISR(void)							// 0x000D36  USER8 (Software #8)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER9_ISR(void)							// 0x000D38  USER9 (Software #9)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER10_ISR(void)							// 0x000D3A  USER10 (Software #10)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER11_ISR(void)							// 0x000D3C  USER11 (Software #11)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void USER12_ISR(void)							// 0x000D3E  USER12 (Software #12)
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT1_ISR(void)							// PIE1.1 @ 0x000D40  ADCINT1
{  	
 	//--- Manage the ADC registers
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//Clear ADCINT1 flag reinitialize for next SOC	
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group
  
    ADC_INT_Control();
}

//---------------------------------------------------------------------
void ADCINT2_ISR(void)							// PIE1.2 @ 0x000D42  ADCINT2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
												// PIE1.3 @ 0x000D44  reserved

//---------------------------------------------------------------------
void XINT1_ISR(void)							// PIE1.4 @ 0x000D46  XINT1
{    
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void XINT2_ISR(void)							// PIE1.5 @ 0x000D48  XINT2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT9_ISR(void)							// PIE1.6 @ 0x000D4A  ADCINT9
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void TINT0_ISR(void)							// PIE1.7 @ 0x000D4C  TINT0 (CPU TIMER 0)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void WAKEINT_ISR(void)							// PIE1.8 @ 0x000D4E  WAKEINT (LPM/WD)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM1_TZINT_ISR(void)						// PIE2.1 @ 0x000D50  EPWM1_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM2_TZINT_ISR(void)						// PIE2.2 @ 0x000D52  EPWM2_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM3_TZINT_ISR(void)						// PIE2.3 @ 0x000D54  EPWM3_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM4_TZINT_ISR(void)						// PIE2.4 @ 0x000D56  EPWM4_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM5_TZINT_ISR(void)						// PIE2.5 @ 0x000D58  EPWM5_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM6_TZINT_ISR(void)						// PIE2.6 @ 0x000D5A  EPWM6_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM7_TZINT_ISR(void)						// PIE2.7 @ 0x000D5C  EPWM7_TZINT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;		// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
												// PIE2.8 @ 0x000D5E  reserved
     
//---------------------------------------------------------------------
void EPWM1_INT_ISR(void)						// PIE3.1 @ 0x000D60  EPWM1_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM2_INT_ISR(void)						// PIE3.2 @ 0x000D62  EPWM2_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM3_INT_ISR(void)						// PIE3.3 @ 0x000D64  EPWM3_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM4_INT_ISR(void)						// PIE3.4 @ 0x000D66  EPWM4_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void EPWM5_INT_ISR(void)						// PIE3.5 @ 0x000D68  EPWM5_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}
//---------------------------------------------------------------------
void EPWM6_INT_ISR(void)						// PIE3.6 @ 0x000D6A  EPWM6_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}
//---------------------------------------------------------------------
void EPWM7_INT_ISR(void)						// PIE3.7 @ 0x000D6C  EPWM7_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}
//---------------------------------------------------------------------
												// PIE3.8 @ 0x000D6E  reserved

//---------------------------------------------------------------------
void ECAP1_INT_ISR(void)						// PIE4.1 @ 0x000D70  ECAP1_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;		// Must acknowledge the PIE group

	ECap1Regs.ECCLR.bit.INT = 1;				// Clear the ECAP1 interrupt flag
	ECap1Regs.ECCLR.bit.CEVT1 = 1;				// Clear the CEVT3 flag
}

//---------------------------------------------------------------------
												// PIE4.2 @ 0x000D72  reserved
												// PIE4.3 @ 0x000D74  reserved
												// PIE4.4 @ 0x000D76  reserved
												// PIE4.5 @ 0x000D78  reserved
												// PIE4.6 @ 0x000D7A  reserved
// INT 4.7---------------------------------------------------------------------
void HRCAP1_INT_ISR(void)    // HRCAP-1
{
	/*EALLOW;
	if (HRCap1Regs.HCIFR.bit.RISEOVF == 1)
		ESTOP0;                    // Another rising edge detected before ISR serviced.

	HRCap1Regs.HCCTL.bit.RISEINTE = 0; // Disable rising edge interrupts

    HRCap1Regs.HCICLR.all = 0x001F;          // Clear all HRCAP interrupts
    HRCap1Regs.HCCTL.bit.RISEINTE = 1;       // Re-enable rising edge interrupts

    HRCap1Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.
	EDIS;*/
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);

}
//---------------------------------------------------------------------
// INT 4.8---------------------------------------------------------------------
void HRCAP2_INT_ISR(void)    // HRCAP-2
{
	/*EALLOW;
	if (HRCap2Regs.HCIFR.bit.RISEOVF == 1)
		ESTOP0;                    // Another rising edge detected before ISR serviced.

	//HRCAP2_By_to_Inv();

	HRCap2Regs.HCCTL.bit.RISEINTE = 0; // Disable rising edge interrupts

    HRCap2Regs.HCICLR.all = 0x001F;          // Clear all HRCAP interrupts
    HRCap2Regs.HCCTL.bit.RISEINTE = 1;       // Re-enable rising edge interrupts

    HRCap2Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.
	EDIS;*/
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void EQEP1_INT_ISR(void)						// PIE5.1 @ 0x000D80  EQEP1_INT
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}
//---------------------------------------------------------------------
												// PIE5.2 @ 0x000D82  reserved
												// PIE5.3 @ 0x000D84  reserved
												// PIE5.4 @ 0x000D86  reserved
												// PIE5.5 @ 0x000D88  reserved
												// PIE5.6 @ 0x000D8A  reserved
												// PIE5.7 @ 0x000D8C  reserved
												// PIE5.8 @ 0x000D8E  reserved
//---------------------------------------------------------------------
void SPIRXINTA_ISR(void)						// PIE6.1 @ 0x000D90  SPIRXINTA (SPI-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void SPITXINTA_ISR(void)						// PIE6.2 @ 0x000D92  SPITXINTA (SPI-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;		// Must acknowledge the PIE group
     
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void SPIRXINTB_ISR(void)						// PIE6.3 @ 0x000D94  SPIRXINTB (SPI-B)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;		// Must acknowledge the PIE group
     
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void SPITXINTB_ISR(void)						// PIE6.4 @ 0x000D96  SPITXINTB (SPI-B)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
												// PIE6.5 @ 0x000D98  reserved
												// PIE6.6 @ 0x000D9A  reserved
												// PIE6.7 @ 0x000D9C  reserved
												// PIE6.8 @ 0x000D9E  reserved

//---------------------------------------------------------------------
												// PIE7.1 @ 0x000DA0  reserved
												// PIE7.2 @ 0x000DA2  reserved
												// PIE7.3 @ 0x000DA4  reserved
												// PIE7.4 @ 0x000DA6  reserved
												// PIE7.5 @ 0x000DA8  reserved
												// PIE7.6 @ 0x000DAA  reserved
												// PIE7.7 @ 0x000DAC  reserved
												// PIE7.8 @ 0x000DAE  reserved

//---------------------------------------------------------------------
void I2CINT1A_ISR(void)							// PIE8.1 @ 0x000DB0  I2CINT1A (I2C-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	//SWI_post( &I2C_swi );                      //GX
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);

}

//---------------------------------------------------------------------
void I2CINT2A_ISR(void)							// PIE8.2 @ 0x000DB2  I2CINT2A (I2C-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
												// PIE8.3 @ 0x000DB4  reserved
												// PIE8.4 @ 0x000DB6  reserved
												// PIE8.5 @ 0x000DB8  reserved
												// PIE8.6 @ 0x000DBA  reserved
												// PIE8.7 @ 0x000DBC  reserved
												// PIE8.8 @ 0x000DBE  reserved

//---------------------------------------------------------------------
void SCIRXINTA_ISR(void)						// PIE9.1 @ 0x000DC0  SCIRXINTA (SCI-A)
{    
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Must acknowledge the PIE group
	PieCtrlRegs.PIEIER9.bit.INTx1 = 0;  // Disable INT for a while

     
	SWI_post(&SWI_SCIARXINT);
// Next two lines for debug only - remove after inserting your ISR
//	asm (" ESTOP0");							// Emulator Halt instruction
//	while(1);
}

//---------------------------------------------------------------------
void SCITXINTA_ISR(void)						// PIE9.2 @ 0x000DC2  SCITXINTA (SCI-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void LIN0INTA_ISR(void)							// PIE9.3 @ 0x000DC4  LIN0INTA (LIN-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Must acknowledge the PIE group
	PieCtrlRegs.PIEIER9.bit.INTx3 = 0;

    SWI_post(&SWI_SCILINRXINT);
}

//---------------------------------------------------------------------
void LIN1INTA_ISR(void)							// PIE9.4 @ 0x000DC6  LIN1INTA (LIN-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ECAN0INTA_ISR(void)						// PIE9.5 @ 0x000DC8  ECAN0INTA (ECAN-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	// Must acknowledge the PIE group
	SWI_ECANRXISR();
}

//---------------------------------------------------------------------
void ECAN1INTA_ISR(void)						// PIE9.6 @ 0x000DCA  ECAN1INTA (ECAN-A)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
												// PIE9.7 @ 0x000DCC  reserved
												// PIE9.8 @ 0x000DCE  reserved

//---------------------------------------------------------------------
												// PIE10.1 @ 0x000DD0  (ADCINT1 - see PIE1.1)
												// PIE10.2 @ 0x000DD2  (ADCINT2 - see PIE1.2)

//---------------------------------------------------------------------
void ADCINT3_ISR(void)							// PIE10.3 @ 0x000DD4  ADCINT3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT4_ISR(void)							// PIE10.4 @ 0x000DD6  ADCINT4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT5_ISR(void)							// PIE10.5 @ 0x000DD8  ADCINT5
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT6_ISR(void)							// PIE10.6 @ 0x000DDA  ADCINT6
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT7_ISR(void)							// PIE10.7 @ 0x000DDC  ADCINT7
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void ADCINT8_ISR(void)							// PIE10.8 @ 0x000DDE  ADCINT8
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
void CLA1_INT1_ISR(void)						// PIE11.1 @ 0x000DE0  CLA1_INT1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
void CLA1_INT2_ISR(void)						// PIE11.2 @ 0x000DE2  CLA1_INT2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void CLA1_INT3_ISR(void)						// PIE11.3 @ 0x000DE4  CLA1_INT3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void CLA1_INT4_ISR(void)						// PIE11.4 @ 0x000DE6  CLA1_INT4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void CLA1_INT5_ISR(void)						// PIE11.5 @ 0x000DE8  CLA1_INT5
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void CLA1_INT6_ISR(void)						// PIE11.6 @ 0x000DEA  CLA1_INT6
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void CLA1_INT7_ISR(void)						// PIE11.7 @ 0x000DEC  CLA1_INT7
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void CLA1_INT8_ISR(void)						// PIE11.8 @ 0x000DEE  CLA1_INT8
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}  

//---------------------------------------------------------------------
void XINT3_ISR(void)							// PIE12.1 @ 0x000DF0  XINT3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
												// PIE12.2 @ 0x000DF2 reserved
												// PIE12.3 @ 0x000DF4 reserved
												// PIE12.4 @ 0x000DF6 reserved
												// PIE12.5 @ 0x000DF8 reserved
												// PIE12.6 @ 0x000DFA reserved

//---------------------------------------------------------------------
void LVF_ISR(void)								// PIE12.7 @ 0x000DFC  LVF (CLA OVERFLOW)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
void LUF_ISR(void)								// PIE12.8 @ 0x000DFE  LUF (CLA UNDERFLOW)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;	// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
} 

//---------------------------------------------------------------------
void rsvd_ISR(void)								// Reserved PIE vectors
{
// This ISR is for reserved PIE vectors.  It should never be reached by
// properly executing code.  If you get here, it means something is wrong.

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");							// Emulator Halt instruction
	while(1);
}


//--- end of file -----------------------------------------------------
















