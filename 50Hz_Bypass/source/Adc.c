/**********************************************************************
* File: Adc.c
* Devices: TMS320F2803x
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   07/02/09 - original (D. Alter)
**********************************************************************/
#include "F28035_example.h"				// Main include file

void AdcChanSelect(Uint16 ch_no);
Uint16 AdcConversion(void);
/**********************************************************************
* Function: InitAdc()
*
* Description: Initializes the ADC on the F2803x.
**********************************************************************/
void InitAdc(void)
{
	asm(" EALLOW");						// Enable EALLOW protected register access

//--- Reset the ADC module
// Note: The ADC is already reset after a DSP reset, but this example is just showing
// good coding practice to reset the peripheral before configuring it as you never
// know why the DSP has started the code over again from the beginning).  
	//AdcRegs.ADCCTL1.bit.RESET = 1;		// Reset the ADC

// Must wait 2 ADCCLK periods for the reset to take effect.
// Note that ADCCLK = SYSCLKOUT for F2802x/F2803x devices.
	asm(" NOP");
	asm(" NOP");
	
//--- Power-up and configure the ADC
	AdcRegs.ADCCTL1.all = 0x00EC;		// Power-up reference and main ADC
// bit 15        0:      RESET, ADC software reset, 0=no effect, 1=resets the ADC
// bit 14        0:      ADCENABLE, ADC enable, 0=disabled, 1=enabled
// bit 13        0:      ADCBSY, ADC busy, read-only
// bit 12-8      0's:    ADCBSYCHN, ADC busy channel, read-only
// bit 7         1:      ADCPWDN, ADC power down, 0=powered down, 1=powered up
// bit 6         1:      ADCBGPWD, ADC bandgap power down, 0=powered down, 1=powered up 
// bit 5         1:      ADCREFPWD, ADC reference power down, 0=powered down, 1=powered up 
// bit 4         0:      reserved
// bit 3         1:      ADCREFSEL, ADC reference select, 0=internal, 1=external
// bit 2         1:      INTPULSEPOS, INT pulse generation, 0=start of conversion, 1=end of conversion
// bit 1         0:      VREFLOCONV, VREFLO convert, 0=VREFLO not connected, 1=VREFLO connected to B5
// bit 0         0:      Must write as 0.

	DelayUs(1000);						// Wait 1 ms after power-up before using the ADC

	/*
	* SOC0~15 均由EPWM1触发，依次进行SOC0-ADCINA0-ADCRESULT0 SOC1-ADCINB0-ADCRESULT1
	* SOC2-ADCINA1-ADCRESULT2 SOC3-ADCINB1-ADCRESULT3 依此类推
	*/
	AdcRegs.ADCSAMPLEMODE.all = 0xff;		// SOC0  (vs. simultaneous mode)

	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

   	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC2CTL.bit.CHSEL = 1;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC3CTL.bit.CHSEL = 1;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC4CTL.bit.CHSEL =2;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC5CTL.bit.CHSEL = 2;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC6CTL.bit.CHSEL = 3;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC6CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC7CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC7CTL.bit.CHSEL = 3;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC7CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles
  
	AdcRegs.ADCSOC8CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC8CTL.bit.CHSEL = 4;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC8CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC9CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC9CTL.bit.CHSEL = 4;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC9CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC10CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC10CTL.bit.CHSEL = 5;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC10CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC11CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC11CTL.bit.CHSEL = 5;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC11CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC12CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC12CTL.bit.CHSEL = 6;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC12CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC13CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC13CTL.bit.CHSEL = 6;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC13CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC14CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC14CTL.bit.CHSEL = 7;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC14CTL.bit.ACQPS = 6;		 	// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCSOC15CTL.bit.TRIGSEL = 5;			// Trigger using ePWM1-ADCSOCA
	AdcRegs.ADCSOC15CTL.bit.CHSEL = 7;			// Convert channel ADCINA0 (ch0)
    AdcRegs.ADCSOC15CTL.bit.ACQPS = 6;			// Acquisition window set to (6+1)=7 cycles

	AdcRegs.ADCINTSOCSEL1.all = 0;			// No ADCINT triggers SOC0.  TRIGSEL field determines trigger.
    AdcRegs.ADCINTSOCSEL2.all = 0;			// No ADCINT triggers SOC0.  TRIGSEL field determines trigger.

	AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;		// All SOCs handled in round-robin mode

//--- ADCINT1 configuration
	AdcRegs.INTSEL1N2.bit.INT1CONT = 0;			// ADCINT1 pulses regardless of ADCINT1 flag state
	AdcRegs.INTSEL1N2.bit.INT1E = 1;			// Enable ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1SEL = 0x0F;			// EOC15 triggers ADCINT1

	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;			// Enable ADCINT1 in PIE group 1
	IER |= 0x0001;								// Enable INT1 in IER to enable PIE group

//--- Finish up
	AdcRegs.ADCCTL1.bit.ADCENABLE = 1;	// Enable the ADC
	asm(" EDIS");						// Disable EALLOW protected register access

} // end InitAdc()


/* AdcoffsetSelfCal-
   This function re-calibrates the ADC zero offset error by converting the VREFLO reference with
   the ADC and modifying the ADCOFFTRIM register. VREFLO is sampled by the ADC using an internal
   MUX select which connects VREFLO to A5 without sacrificing an external ADC pin. This
   function calls two other functions:
   - AdcChanSelect(channel) selects the ADC channel to convert
   - AdcConversion() initiates several ADC conversions and returns the average
*/
void AdcOffsetSelfCal()
{
    Uint16 AdcConvMean;
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 1;                  //Select internal reference mode
    AdcRegs.ADCCTL1.bit.VREFLOCONV = 1;                 //Select VREFLO internal connection on B5
    AdcChanSelect(13);                                  //Select channel B5 for all SOC
    AdcRegs.ADCOFFTRIM.bit.OFFTRIM = 80;                //Apply artificial offset (+80) to account for a negative offset that may reside in the ADC core
    AdcConvMean = AdcConversion();                      //Capture ADC conversion on VREFLO
    AdcRegs.ADCOFFTRIM.bit.OFFTRIM = 80 - AdcConvMean;  //Set offtrim register with new value (i.e remove artical offset (+80) and create a two's compliment of the offset error)
    AdcRegs.ADCCTL1.bit.VREFLOCONV = 0;                 //Select external ADCIN5 input pin on B5
    EDIS;
}

/*  AdcChanSelect-
    This function selects the ADC channel to convert by setting all SOC channel selects to a single channel.

     * IMPORTANT * This function will overwrite previous SOC channel select settings. Recommend saving
           the previous settings.
 */
void AdcChanSelect(Uint16 ch_no)
{
    AdcRegs.ADCSOC0CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC1CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC2CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC3CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC4CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC5CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC6CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC7CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC8CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC9CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC10CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC11CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC12CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC13CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC14CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC15CTL.bit.CHSEL= ch_no;
} //end AdcChanSelect

/* AdcConversion -
   This function initiates several ADC conversions and returns the average. It uses ADCINT1 and ADCINT2
   to "ping-pong" between SOC0-7 and SOC8-15 and is referred to as "ping-pong" sampling.

     * IMPORTANT * This function will overwrite previous ADC settings. Recommend saving previous settings.
*/
Uint16 AdcConversion(void)
{
    Uint16 index, SampleSize, Mean, ACQPS_Value;
    Uint32 Sum;

    index       = 0;            //initialize index to 0
    SampleSize  = 256;          //set sample size to 256 (**NOTE: Sample size must be multiples of 2^x where is an integer >= 4)
    Sum         = 0;            //set sum to 0
    Mean        = 999;          //initialize mean to known value

    //Set the ADC sample window to the desired value (Sample window = ACQPS + 1)
    ACQPS_Value = 6;
    AdcRegs.ADCSOC0CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC1CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC2CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC3CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC4CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC5CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC6CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC7CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC8CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC9CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC10CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC11CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC12CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC13CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC14CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC15CTL.bit.ACQPS = ACQPS_Value;


    //Enable ping-pong sampling

    // Enabled ADCINT1 and ADCINT2
    AdcRegs.INTSEL1N2.bit.INT1E = 1;
    AdcRegs.INTSEL1N2.bit.INT2E = 1;

    // Disable continuous sampling for ADCINT1 and ADCINT2
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;
    AdcRegs.INTSEL1N2.bit.INT2CONT = 0;

    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    //ADCINTs trigger at end of conversion

    // Setup ADCINT1 and ADCINT2 trigger source
    AdcRegs.INTSEL1N2.bit.INT1SEL = 6;      //EOC6 triggers ADCINT1
    AdcRegs.INTSEL1N2.bit.INT2SEL = 14;     //EOC14 triggers ADCINT2

    // Setup each SOC's ADCINT trigger source
    AdcRegs.ADCINTSOCSEL1.bit.SOC0  = 2;    //ADCINT2 starts SOC0-7
    AdcRegs.ADCINTSOCSEL1.bit.SOC1  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC2  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC3  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC4  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC5  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC6  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC7  = 2;
    AdcRegs.ADCINTSOCSEL2.bit.SOC8  = 1;    //ADCINT1 starts SOC8-15
    AdcRegs.ADCINTSOCSEL2.bit.SOC9  = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC10 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC11 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC12 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC13 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC14 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC15 = 1;

    DelayUs(1000);                  // Delay before converting ADC channels


    //ADC Conversion

    AdcRegs.ADCSOCFRC1.all = 0x00FF;  // Force Start SOC0-7 to begin ping-pong sampling

    while( index < SampleSize ){

        //Wait for ADCINT1 to trigger, then add ADCRESULT0-7 registers to sum
        while (AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){}
        AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   //Must clear ADCINT1 flag since INT1CONT = 0
        Sum += AdcResult.ADCRESULT0;
        Sum += AdcResult.ADCRESULT1;
        Sum += AdcResult.ADCRESULT2;
        Sum += AdcResult.ADCRESULT3;
        Sum += AdcResult.ADCRESULT4;
        Sum += AdcResult.ADCRESULT5;
        Sum += AdcResult.ADCRESULT6;
        Sum += AdcResult.ADCRESULT7;

        //Wait for ADCINT2 to trigger, then add ADCRESULT8-15 registers to sum
        while (AdcRegs.ADCINTFLG.bit.ADCINT2 == 0){}
        AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;   //Must clear ADCINT2 flag since INT2CONT = 0
        Sum += AdcResult.ADCRESULT8;
        Sum += AdcResult.ADCRESULT9;
        Sum += AdcResult.ADCRESULT10;
        Sum += AdcResult.ADCRESULT11;
        Sum += AdcResult.ADCRESULT12;
        Sum += AdcResult.ADCRESULT13;
        Sum += AdcResult.ADCRESULT14;
        Sum += AdcResult.ADCRESULT15;

        index+=16;

    } // end data collection

    //Disable ADCINT1 and ADCINT2 to STOP the ping-pong sampling
    AdcRegs.INTSEL1N2.bit.INT1E = 0;
    AdcRegs.INTSEL1N2.bit.INT2E = 0;

    Mean = Sum / SampleSize;    //Calculate average ADC sample value

    return Mean;                //return the average

}//end AdcConversion











//--- end of file -----------------------------------------------------
