/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : Gpio.c
 *
 *  PURPOSE  :
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/


#include "DSP2833x_Device.h"		// Peripheral address definitions
#include "3KW_MAINHEADER.h"			// Main include file

/*=============================================================================*
 * FUNCTION: InitGpio()
 * PURPOSE : I/O hardware module initialization.
 * INPUT: 
 *     void
 *
 * RETURN: 
 *     void
 *
 * CALLS: 
 *     void
 *
 * CALLED BY: 
 *     Main.c  
 * 
 *============================================================================*/
void InitGpio(void)
{
	asm(" EALLOW");					// Enable EALLOW protected register access

	/*---------- Group A pins ------------*/

	GpioCtrlRegs.GPACTRL.all  = 0x00000000;		// QUALPRD = SYSCLKOUT for all group A GPIO
		
	GpioCtrlRegs.GPAQSEL1.all = 0x00000000;		// No qualification for all group A GPIO 0-15

	GpioCtrlRegs.GPAQSEL2.all = 0x00000000;		// No qualification for all group A GPIO 16-31

	GpioCtrlRegs.GPADIR.all   = 0x00000000;		// All group A GPIO are inputs
	
	GpioCtrlRegs.GPAPUD.all   = 0xFFFFFFFF;		// All Pullups disabled due to external pull down


	// GPIO 0-11 are used as ePWM port
	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;		// 0=GPIO  1=EPWM1A     2=rsvd       
												// 3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0;		// 0=GPIO  1=EPWM1B     2=ECAP6
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;          // 3=rsvd
	GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1;		// 0=GPIO  1=EPWM2A     2=rsvd       
												// 3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 0;//2017.6.9 ZJX		// 0=GPIO  1=EPWM2B     2=ECAP5
												// 3=rsvd
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 0; //2017.6.9 ZJX input

	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1;		// 0=GPIO  1=EPWM3A     2=rsvd       
												// 3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 1;		// 0=GPIO  1=EPWM3B     2=MFSRA      
												// 3=ECAP1
	GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 1;		// 0=GPIO  1=EPWM4A     2=EPWMSYNCI  
												// 3=EPWMSYNCO
	GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 1;		// 0=GPIO  1=EPWM4B     2=MCLKRA     
												// 3=ECAP2
	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 1;		// 0=GPIO  1=EPWM5A     2=CANTXB     
												// 3=ADCSOCAO
	GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 1;		// 0=GPIO  1=EPWM5B     2=SCITXDB    
												// 3=ECAP3
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;		// 0=GPIO  1=EPWM6A     2=CANRXB     
												// 3=ADCSOCBO
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;		// 0=GPIO  1=EPWM6B     2=SCIRXDB    

	// GPIO12 Output pin for FanCntl
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;		// 0=GPIO  1=TZ1        2=CANTXB
												// 3=SPISIMOB
	// GPIO13 Output pin for 110V Output Relay Control, INVL for 25Hz and 50Hz
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;		// 0=GPIO  1=TZ2        2=CANRXB     
												// 3=SPISOMIB
	// GPIO14 Output pin for 110V Output SCR Control, INVL for 25Hz and 50Hz
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;		// 0=GPIO  1=TZ3/XHOLD  2=SCITXDB
												// 3=SPICLKB
	// GPIO15 Output pin for 220V Output SCR Control, INVH for 25Hz
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;		// 0=GPIO  1=TZ4/XHOLDA 2=SCIRXDB
												// 3=SPISTEB
	// GPIO16 Output pin for 220V Output Relay Control, INVH for 25Hz
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;		// 0=GPIO  1=SPISIMOA   2=CANTXB
		                             			// 3=TZ5
	// GPIO17 Input pin for COM4IPF
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;		// 0=GPIO  1=SPISOMIA   2=CANRXB
												// 3=TZ6
	// GPIO18/19 are used as eCAN-A
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;		// 0=GPIO  1=SPICLKA    2=SCITXDB
												// 3=CANRXA
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;		// 0=GPIO  1=SPISTEA    2=SCIRXDB
												// 3=CANTXA
	// GPIO20 Output pin for Dry Contact
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;		// 0=GPIO  1=EQEP1A     2=MDXA
												// 3=CANTXB
	// GPIO21 Output pin for Input Relay
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;		// 0=GPIO  1=EQEP1B     2=MDRA
												// 3=CANRXB
	// GPIO22/23, SCI-B to Communicate with Display Board
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;		// 0=GPIO  1=EQEP1S     2=MCLKXA
												// 3=SCITXDB
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;		// 0=GPIO  1=EQEP1I     2=MFSXA
												// 3=SCIRXDB
	// GPIO24 ecap pin for COM1IPF
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;		// 0=GPIO  1=ECAP1      2=EQEP2A
												// 3=MDXB
	// GPIO25 Output pin for Cs_Cntl1
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;		// 0=GPIO  1=ECAP2      2=EQEP2B
		                                        // 3=MDRB
	// GPIO26 Input pin for COM2IPF
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;		// 0=GPIO  1=ECAP3      2=EQEP2I
												// 3=MCLKXB
	// GPIO27 Output pin for COM1OUT
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;		// 0=GPIO  1=ECAP4      2=EQEP2S
												// 3=MFSXB
	// GPIO28-31 are unused I/O, set input pins
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;		// 0=GPIO  1=SCIRXDA    2=XZCS6
    									        // 3=XZCS6
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;		// 0=GPIO  1=SCITXDA    2=XA19
    											// 3=XA19
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;		// 0=GPIO  1=CANRXA     2=XA18
										        // 3=XA18
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;		// 0=GPIO  1=CANTXA     2=XA17
											    // 3=XA17

	/*---------- Group B pins ------------*/
	
	// QUALPRD = SYSCLKOUT for all group B GPIO
	GpioCtrlRegs.GPBCTRL.all  = 0x00000000;		
	// No qualification for all group B GPIO 32-47
	GpioCtrlRegs.GPBQSEL1.all = 0x00000000;		
	// No qualification for all group B GPIO 48-63
	GpioCtrlRegs.GPBQSEL2.all = 0x00000000;		
	// All group B GPIO are inputs
	GpioCtrlRegs.GPBDIR.all   = 0x00000000;		
	// All group B pullups disabled due to external pull down
	GpioCtrlRegs.GPBPUD.all   = 0xFFFFFFFF;		

	// GPIO32/33 are used as I2C port
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;		// 0=GPIO  1=SDAA      2=EPWMSYNCI  
												// 3=ADCSOCAO
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;		// 0=GPIO  1=SCLA      2=EPWMSYNCO  
												// 3=ADCSOCBO
	// GPIO38-43 are unused I/O, set input pin.
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;		// 0=GPIO  1=ECAP1     2=XREADY     
												// 3=XREADY
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;		// 0=GPIO  1=SCITXDA   2=XR/W
    											// 3=XR/W
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;		// 0=GPIO  1=SCIRXDA   2=XZCS0
    											// 3=XZCS0
	GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;		// 0=GPIO  1=ECAP2     2=XZCS7      
												// 3=XZCS7
	GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;		// 0=GPIO  1=rsvd      2=XWE0       
												// 3=XWE0
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;		// 0=GPIO  1=rsvd      2=XA16       
												// 3=XA16
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;		// 0=GPIO  1=rsvd      
												// 2=XA0/XWE1   3=XA0/XWE1
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;		// 0=GPIO  1=rsvd      2=XA1        
												// 3=XA1
	//GPIO42 Bypass indication pin
	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;		// 0=GPIO  1=rsvd      2=XA2        
												// 3=XA2
	//GPIO43 Single module indication pin
	GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;		// 0=GPIO  1=rsvd      2=XA3        
												// 3=XA3

	// GPIO44 Input pin for Input AC Over Current Protection
	GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;		// 0=GPIO  1=rsvd      2=XA4        
												// 3=XA4
	// GPIO45 Input pin for DC Bus Over Voltage Protection
	GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 0;		// 0=GPIO  1=rsvd      2=XA5        
												// 3=XA5	
	// GPIO46 Input pin for 110V Output AC Over Current Protection
	GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;		// 0=GPIO  1=rsvd      2=XA6        
												// 3=XA6
	// GPIO47 Input pin for 110V Output AC Over Voltage Protection
	GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;		// 0=GPIO  1=rsvd      2=XA7        
												// 3=XA7
	// GPIO48 Output pin for COM4OUT(50Hz) and Cs_Cntl3(25Hz)
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;		// 0=GPIO  1=ECAP5     2=XD31       
												// 3=XD31
	// GPIO49 Input pin for Output Voltages Capture
	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;		// 0=GPIO  1=ECAP6     2=XD30       
												// 3=XD30
	// GPIO50 Output pin for SCIA-Enable/Disable
	GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;		// 0=GPIO  1=EQEP1A    2=XD29       
												// 3=XD29
	// GPIO51 Output pin for COM2OUT
	GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;		// 0=GPIO  1=EQEP1B    2=XD28       
												// 3=XD28
	// GPIO52-61 are unused I/O, set input pin.
	GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;		// 0=GPIO  1=EQEP1S    2=XD27       
												// 3=XD27
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;		// 0=GPIO  1=EQEP1I    2=XD26
												// 3=XD26
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;		// 0=GPIO  1=SPISIMOA  2=XD25
												// 3=XD25     
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;		// 0=GPIO  1=SPISOMIA  2=XD24
												// 3=XD24     
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;		// 0=GPIO  1=SPICLKA   2=XD23
											 	// 3=XD23
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;		// 0=GPIO  1=SPISTEA   2=XD22       
												// 3=XD22
	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;		// 0=GPIO  1=MCLKRA    2=XD21       
												// 3=XD21
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;		// 0=GPIO  1=MFSRA     2=XD20       
												// 3=XD20
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;		// 0=GPIO  1=MCLKRB    2=XD19       
												// 3=XD19
	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;		// 0=GPIO  1=MFSRB     2=XD18       
												// 3=XD18
	// GPIO62/63 SCI-C for PC Monitoring
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;		// 0=GPIO  1=SCIRXDC   2=XD17       
												// 3=XD17
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;		// 0=GPIO  1=SCITXDC   2=XD16       
												// 3=XD16

	/* ------------Group C pins-----------*/
	
	// All group C GPIO are inputs
	GpioCtrlRegs.GPCDIR.all   = 0x00000000;		
	// All group C pullups disabled due to external pull down
	GpioCtrlRegs.GPCPUD.all   = 0xFFFFFFFF;		
	
	// GPIO64-75 are unused I/O, set input pin
	GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;		// 0=GPIO  1=GPIO      2=XD15       
												// 3=XD15
	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;		// 0=GPIO  1=GPIO      2=XD14       
												// 3=XD14
	GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;		// 0=GPIO  1=GPIO      2=XD13       
												// 3=XD13
	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;		// 0=GPIO  1=GPIO      2=XD12       
												// 3=XD12
	GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 0;		// 0=GPIO  1=GPIO      2=XD11       
												// 3=XD11
	GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 0;		// 0=GPIO  1=GPIO      2=XD10       
												// 3=XD10
	GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0;		// 0=GPIO  1=GPIO      2=XD9        
												// 3=XD9
	GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0;		// 0=GPIO  1=GPIO      2=XD8        
												// 3=XD8
	GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 0;		// 0=GPIO  1=GPIO      2=XD7        
												// 3=XD7
	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0;		// 0=GPIO  1=GPIO      2=XD6        
												// 3=XD6
	GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 0;		// 0=GPIO  1=GPIO      2=XD5        
												// 3=XD5
	GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 0;		// 0=GPIO  1=GPIO      2=XD4        
												// 3=XD4

	// GPIO76 Input pin for Fan2 Speed Detection
	GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 0;		// 0=GPIO  1=GPIO      2=XD3        
	GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0;
	// 3=XD3
	// GPIO77 Input pin for Fan1 Speed Detection
	GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0;		// 0=GPIO  1=GPIO      2=XD2        
	GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0;											// 3=XD2

	// GPIO78/79 are unused I/O, set input pin
	GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0;		// 0=GPIO  1=GPIO      2=XD1        
												// 3=XD1
	GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0;		// 0=GPIO  1=GPIO      2=XD0        
												// 3=XD0
	// GPIO80 Input pin for 220V Output AC Over Current Protection
	GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;		// 0=GPIO  1=GPIO      2=XA8        
												// 3=XA8
	// GPIO81 Input pin for 220V Output AC Over Voltage Protection
	GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;		// 0=GPIO  1=GPIO      2=XA9        
												// 3=XA9
	// GPIO82 Output pin for Driver Reset
	GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;		// 0=GPIO  1=GPIO      2=XA10, unused       
												// 3=XA10
	// GPIO83 is unused, set as input pin
	GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;		// 0=GPIO  1=GPIO      2=XA11       
												// 3=XA11
	// Boot mode pins
	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;		// 0=GPIO  1=GPIO      2=XA12       
												// 3=XA12
	GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;		// 0=GPIO  1=GPIO      2=XA13       
												// 3=XA13
	GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;		// 0=GPIO  1=GPIO      2=XA14       
												// 3=XA14
	GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;		// 0=GPIO  1=GPIO      2=XA15       
												// 3=XA15

/*------------------------------------------------------------------------
                 GPIO direction and initial status settings
  -----------------------------------------------------------------------*/

	/* ------------Group A pins-----------*/

	// GPIO12 Output pin for FanCntl
	GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;		// output 0 by default
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;			// Configures the GPIO pin as an output

	// GPIO13 Output pin for 110V Output Relay Control, INVL for 25Hz and 50Hz
	GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;		// output 0 by default
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;			// Configures the GPIO pin as an output

	// GPIO14 Output pin for 110V Output SCR Control, INVL for 25Hz and 50Hz
	GpioDataRegs.GPASET.bit.GPIO14 = 1;			// output 1 by default
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;			// Configures the GPIO pin as an output

	// GPIO15 Output pin for 220V Output SCR Control, INVH for 25Hz
	GpioDataRegs.GPASET.bit.GPIO15 = 1;			// output 1 by default
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;			// Configures the GPIO pin as an output

	// GPIO16 Output pin for 220V Output Relay Control, INVH for 25Hz
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;		// output 0 by default	 
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;			// Configures the GPIO pin as an output

	// GPIO20 Output pin for Dry Contact
	GpioDataRegs.GPASET.bit.GPIO20 = 1;		// output 1 by default
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;			// Configures the GPIO pin as an output

	// GPIO21 Output pin for Input Relay
	GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;		// output 0 by default
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;			// Configures the GPIO pin as an output												// 3=CANRXB

	// GPIO25 Output pin for Cs_Cntl1
	GpioDataRegs.GPASET.bit.GPIO25 = 1;			// output 0 by default
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;			// Configures the GPIO pin as an output

    // GPIO27 Output pin for COM1OUT
	GpioDataRegs.GPASET.bit.GPIO27 = 1;			// output 1 by default
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;			// Configures the GPIO pin as an output

	/* ------------Group B pins-----------*/

	// GPIO42 Input for whether there is a bypass module. Set means no bypass, Clear means existing bypass
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;	 // Configures the GPIO pin as an input
	// GPIO43 Input for whether the system is single module. Set means more than one module, clear mean single module.
	GpioCtrlRegs.GPBDIR.bit.GPIO43 = 0;			// Configures the GPIO pin as an input

	// GPIO48 Output pin for COM4OUT(50Hz) and Cs_Cntl3(25Hz)
	GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;		// output 0 by default
	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;			// Configures the GPIO pin as an output

	//GPIO50 Output pin for SCIA-Enable/Disable
	GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;		// output 0 by default
	GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;			// Configures the GPIO pin as an output

	// GPIO51 Output pin for COM2OUT
	GpioDataRegs.GPBSET.bit.GPIO51 = 1;		// output 1 by default
	GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;			// Configures the GPIO pin as an output

	/* ------------Group C pins-----------*/

	// GPIO82 Output pin for Driver Reset
	GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;		// output 0 by default
	GpioCtrlRegs.GPCDIR.bit.GPIO82  = 1;		// Configures the GPIO pin as an output


	/* Re-assign Input qualification for every digital input according to above 
	settings:
 	(1) 6 samples;
 	(2) 00-Syn with SysClock, 01- 3 samples, 10- 6 samples, 11- Asyn  */

	// GPIO17 input pin for COM4IPF
	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 0x2;

	// GPIO26 Input pin for COM2IPF
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0x2;

	// GPIO44 Input pin for Input AC Over Current Protection
	GpioCtrlRegs.GPBQSEL1.bit.GPIO44 = 0x2;

	// GPIO45 Input pin for DC Bus Over Voltage Protection
	GpioCtrlRegs.GPBQSEL1.bit.GPIO45 = 0x2;

	// GPIO46 Input pin for 110V Output AC Over Current Protection
	GpioCtrlRegs.GPBQSEL1.bit.GPIO46 = 0x2;

	// GPIO47 Input pin for 110V Output AC Over Voltage Protection
	GpioCtrlRegs.GPBQSEL1.bit.GPIO47 = 0x2;

	// GPIO80 Input pin for 220V Output AC Over Current Protection
	GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0;

	// GPIO81 Input pin for 220V Output AC Over Voltage Protection
	GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;

	// GPIO49 Input pin for Output Voltages Capture
	GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 0x2;

/*------------------------------------------------------------------------
                      External interrupt selection
  -----------------------------------------------------------------------*/

	GpioIntRegs.GPIOXINT1SEL.all = 0x0000;		// GPIO0 is XINT1 source
	GpioIntRegs.GPIOXINT2SEL.all = 0x0000;		// GPIO0 is XINT2 source
	GpioIntRegs.GPIOXINT3SEL.all = 0x0000;		// GPIO32 is XINT3 source
	GpioIntRegs.GPIOXINT4SEL.all = 0x0000;		// GPIO32 is XINT4 source
	GpioIntRegs.GPIOXINT5SEL.all = 0x0000;		// GPIO32 is XINT5 source
	GpioIntRegs.GPIOXINT6SEL.all = 0x0000;		// GPIO32 is XINT6 source
	GpioIntRegs.GPIOXINT7SEL.all = 0x0000;		// GPIO32 is XINT7 source
	GpioIntRegs.GPIOXNMISEL.all = 0x0000;		// GPIO0 is XNMI source

	XIntruptRegs.XINT1CR.all = 0x0000;			// XINT1 disabled
	XIntruptRegs.XINT2CR.all = 0x0000;			// XINT2 disabled
	XIntruptRegs.XINT3CR.all = 0x0000;			// XINT3 disabled
	XIntruptRegs.XINT4CR.all = 0x0000;			// XINT4 disabled
	XIntruptRegs.XINT5CR.all = 0x0000;			// XINT5 disabled
	XIntruptRegs.XINT6CR.all = 0x0000;			// XINT6 disabled
	XIntruptRegs.XINT7CR.all = 0x0000;			// XINT7 disabled
	XIntruptRegs.XNMICR.all = 0x0000;			// XNMI disabled

	/* Low-power mode selection */
	// Any one of GPIO 0-31 can be used as LPM wakeup pin.
	// No pin selected for HALT and STANBY wakeup (reset default)
	GpioIntRegs.GPIOLPMSEL.all = 0x00000000;	


	/* Finish up */
	asm(" EDIS");			// Disable EALLOW protected register access								

} // end InitGpio()






//--- end of file -----------------------------------------------------
