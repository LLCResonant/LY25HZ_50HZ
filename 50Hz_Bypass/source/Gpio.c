/**********************************************************************
* File: Gpio.c
* Devices: TMS320F2803x
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   07/02/09 - original (D. Alter)
**********************************************************************/
#include "F28035_example.h"				// Main include file


/**********************************************************************
* Function: InitGpio()
*
* Description: Initializes the shared GPIO pins on the F2803x.
**********************************************************************/
void InitGpio(void)
{
	asm(" EALLOW");								// Enable EALLOW protected register access

//--- Group A pins
	GpioCtrlRegs.GPACTRL.all  = 0x00000000;		// QUALPRD = SYSCLKOUT for all group A GPIO
	GpioCtrlRegs.GPAQSEL1.all = 0x00000000;		// No qualification for all group A GPIO 0-15
	GpioCtrlRegs.GPAQSEL2.all = 0x00000000;		// No qualification for all group A GPIO 16-31
	GpioCtrlRegs.GPADIR.all   = 0x00000000;		// All group A GPIO are inputs
	GpioCtrlRegs.GPAPUD.all   = 0xFFFFFFFF;		// All Pullups disabled

    // GPIO 0-3 are used as ePWM port
	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 0;		// 0=GPIO               1=EPWM1A     2=rsvd       3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0;		// 0=GPIO               1=EPWM1B     2=rsvd       3=COMP1OUT
	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1;		// 0=GPIO               1=EPWM2A     2=rsvd       3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 0;		// 0=GPIO               1=EPWM2B     2=SPISOMIA   3=COMP2OUT
    
    // GPIO 4-11 are unused
	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1;		// 0=GPIO               1=EPWM3A     2=rsvd       3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 0;		// 0=GPIO               1=EPWM3B     2=SPISIMOA   3=ECAP1
	GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0;		// 0=GPIO               1=EPWM4A     2=EPWMSYNCI  3=EPWMSYNCO
	GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0;		// 0=GPIO               1=EPWM4B     2=SCIRXDA    3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;		// 0=GPIO               1=EPWM5A     2=rsvd       3=ADCSOCAO
	GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 3;		// 0=GPIO               1=EPWM5B     2=LINTXA     3=HRCAP1
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;		// 0=GPIO               1=EPWM6A     2=rsvd       3=ADCSOCBO
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;		// 0=GPIO               1=EPWM6B     2=LINRXA     3=HRCAP2

	// GPIO12 output pin for Relay Control
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;		// 0=GPIO               1=TZ1        2=SCITXDA    3=SPISIMOB

    // GPIO 12-15 are unused, and disabled if target is 28035PAG
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;		// 0=GPIO               1=TZ2        2=rsvd       3=SPISOMIB
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;		// 0=GPIO               1=TZ3        2=LINTXA     3=SPICLKB
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;		// 0=GPIO               1=TZ1        2=LINRXA     3=SPISTEB

	// GPIO16 input pin for Input OverCurrent Detect
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;		// 0=GPIO               1=SPISIMOA   2=rsvd       3=TZ2

	// GPIO17 input pin for Bus OverVoltage Detect
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;		// 0=GPIO               1=SPISOMIA   2=rsvd       3=TZ3

    // GPIO 18 is unused, and disabled if target is 28035PAG
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;		// 0=GPIO               1=SPICLKA    2=LINTXA     3=XCLKOUT

	// GPIO19 input pin for Grid Frequency Detect
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;		// 0=GPIO/XCLKIN        1=SPISTEA    2=LINRXA     3=ECAP1

    // GPIO 20-24 are unused, and disabled if target is 28035PAG
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;		// 0=GPIO               1=EQEP1A     2=rsvd       3=COMP1OUT
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;		// 0=GPIO               1=EQEP1B     2=rsvd       3=COMP2OUT
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;		// 0=GPIO               1=EQEP1S     2=rsvd       3=LINTXA
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;		// 0=GPIO               1=EQEP1I     2=rsvd       3=LINRXA
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;		// 0=GPIO               1=ECAP1      2=rsvd       3=SPISIMOB

    // GPIO 25-27 are unused, and disabled if target is 28035PAG
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;		// 0=GPIO               1=rsvd       2=rsvd       3=SPISOMIB
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;		// 0=GPIO               1=rsvd       2=rsvd       3=SPICLKB
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;		// 0=GPIO               1=rsvd       2=rsvd       3=SPISTEB

	// GPIO 28-29 are used as SCI-A for debug
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;		// 0=GPIO               1=SCIRXDA    2=SDAA       3=TZ2
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;		// 0=GPIO               1=SCITXDA    2=SCLA       3=TZ3

	// GPIO 30 input pin for LLC State Detect
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;		// 0=GPIO               1=CANRXA     2=rsvd       3=rsvd

	// GPIO 31 output pin for PFC State
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;		// 0=GPIO               1=CANTXA     2=rsvd       3=rsvd

//--- Group B pins
	GpioCtrlRegs.GPBCTRL.all  = 0x00000000;		// QUALPRD = SYSCLKOUT for all group B GPIO
	GpioCtrlRegs.GPBQSEL1.all = 0x00000000;		// No qualification for all group B GPIO 32-38
	GpioCtrlRegs.GPBDIR.all   = 0x00000000;		// All group B GPIO are inputs
	GpioCtrlRegs.GPBPUD.all   = 0x00000000;		// All group B pullups enabled

	// GPIO 32-33 are used as I2C
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;		// 0=GPIO               1=SDAA      2=EPWMSYNCI  3=ADCSOCAO
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;		// 0=GPIO               1=SCLA      2=EPWMSYNCO  3=ADCSOCBO

	// GPIO 34-38 are unused
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;		// 0=GPIO               1=COMP2OUT  2=rsvd       3=COMP3OUT
	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;		// 0=GPIO (TDI)         1=rsvd      2=rsvd       3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;		// 0=GPIO (TMS)         1=rsvd      2=rsvd       3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;		// 0=GPIO (TDO)         1=rsvd      2=rsvd       3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;		// 0=GPIO/XCLKIN (TCK)  1=rsvd      2=rsvd       3=rsvd
	
	// GPIO 39-44 are unused, and disabled if target is 28035PAG
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;		// 0=GPIO               1=rsvd      2=rsvd       3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;		// 0=GPIO               1=EPWM7A    2=rsvd       3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;		// 0=GPIO               1=EPWM7B    2=rsvd       3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;		// 0=GPIO               1=rsvd      2=rsvd       3=COMP1OUT
	GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;		// 0=GPIO               1=rsvd      2=rsvd       3=COMP2OUT
	GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;		// 0=GPIO               1=rsvd      2=rsvd       3=rsvd

// Analog I/O Mux pins
	GpioCtrlRegs.AIOMUX1.bit.AIO2 = 0;			// 0,1=AIO2             2,3=ADCINA2/COMP1A
	GpioCtrlRegs.AIOMUX1.bit.AIO4 = 0;			// 0,1=AIO4             2,3=ADCINA4/COMP2A
	GpioCtrlRegs.AIOMUX1.bit.AIO6 = 0;			// 0,1=AIO6             2,3=ADCINA6
	GpioCtrlRegs.AIOMUX1.bit.AIO10 = 0;			// 0,1=AIO10            2,3=ADCINB2/COMP1B
	GpioCtrlRegs.AIOMUX1.bit.AIO12 = 0;			// 0,1=AIO12            2,3=ADCINB4/COMP2B
	GpioCtrlRegs.AIOMUX1.bit.AIO14 = 0;			// 0,1=AIO14            2,3=ADCINB6

//--- Low-power mode selection
	GpioIntRegs.GPIOLPMSEL.all = 0x00000000;	// No pin selected for HALT and STANBY wakeup (reset default)



//--- Selected pin configurations

	// GPIO0 COM2_OUT.D
	GpioCtrlRegs.GPADIR.bit.GPIO0  = 1;
    GpioDataRegs.GPASET.bit.GPIO0 = 1;		// output 0 by default // disable by default

	// GPIO1 COM1_OUT.D
	GpioCtrlRegs.GPADIR.bit.GPIO1  = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;		// output 0 by default // disable by default

	// GPIO3  Fan1.Cntl.D
	GpioCtrlRegs.GPADIR.bit.GPIO3  = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;		// output 0 by default // disable by default

	// GPIO5  Fan1.Clk.D
	GpioCtrlRegs.GPADIR.bit.GPIO5  = 0;

	// GPIO6	ProtectionSet0
	GpioCtrlRegs.GPADIR.bit.GPIO6  = 0;

	// GPIO7 ProtectionSet1
	GpioCtrlRegs.GPADIR.bit.GPIO7  = 0;

	// GPIO8  LED_Fault/R.D
	GpioCtrlRegs.GPADIR.bit.GPIO8  = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;		// output 0 by default // disable by default

	// GPIO10  DryCtnl.D
	GpioCtrlRegs.GPADIR.bit.GPIO10  = 1;
    GpioDataRegs.GPASET.bit.GPIO10 = 1;		// output 1 by default // disable by default

	// GPIO11 COM2_I/P/F.D
	GpioCtrlRegs.GPADIR.bit.GPIO11  = 0;

	// GPIO16  Switch.D
	GpioCtrlRegs.GPADIR.bit.GPIO16  = 0;

    // GPIO17  LED_Warning/Y.D
	GpioCtrlRegs.GPADIR.bit.GPIO17  = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;		// output 0 by default // disable by default

    // GPIO18  LED_Power/G.D
	GpioCtrlRegs.GPADIR.bit.GPIO18  = 1;
    GpioDataRegs.GPASET.bit.GPIO18 = 1;		// output 1 by default // disable by default

    // GPIO19  Module_Lock.D
	GpioCtrlRegs.GPADIR.bit.GPIO19  = 0;

    // GPIO20  Fan2.Clk.D
	GpioCtrlRegs.GPADIR.bit.GPIO20  = 0;

    // GPIO21  Fan2.Cntl.D
	GpioCtrlRegs.GPADIR.bit.GPIO21  = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;		// output 0 by default // disable by default

    // GPIO22  COM4_OUT.D
	GpioCtrlRegs.GPADIR.bit.GPIO22  = 1;
    GpioDataRegs.GPASET.bit.GPIO22 = 1;		// output 0 by default // disable by default

    // GPIO32  COM4_I/P.F.D
	GpioCtrlRegs.GPADIR.bit.GPIO22  = 0;

	//²¦Âë¿ª¹Ø
	GpioCtrlRegs.AIODIR.bit.AIO2  = 0;
	GpioCtrlRegs.AIODIR.bit.AIO4  = 0;
	GpioCtrlRegs.AIODIR.bit.AIO6  = 0;
	GpioCtrlRegs.AIODIR.bit.AIO10 = 0;
	GpioCtrlRegs.AIODIR.bit.AIO12 = 0;
	GpioCtrlRegs.AIODIR.bit.AIO14 = 0;

//--- Finish up
	asm(" EDIS");								// Disable EALLOW protected register access

} // end InitGpio()


//--- end of file -----------------------------------------------------
