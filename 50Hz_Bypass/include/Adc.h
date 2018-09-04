/*=============================================================================*
 *         Copyright(c) 2010-2011, Convertergy Co., Ltd.
 *                          ALL RIGHTS RESERVED
 *
 *  FILENAME : Adc.h 
 *   
 *  PURPOSE  : Header files for ADC related module.
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2011-02-24      V0.1           Ken      	    Created
  *============================================================================*/

#ifndef ADC_H
#define ADC_H

/*=============================================================================*
 * 	Structures and definition for ADC channels
 *============================================================================*/
// note: the group list is accordance with hardware channel.

/* General purpose AD buffers definition according to hardware */
struct Def_GeneralADgroup
{ 
	int16	i16Vwu; 			// channel A0
	int16	i16Vvw;				// channel A1 
	int16	i16Vuv;				// channel A2		
	int16	i16T_heatsink;		// channel A3
	int16	i16T_controlboard;	// channel A4	
	int16	i16Isulation;		// channel A5
	int16	i16PVDCBus_I;		// channel A6		
	int16	i16PVDCBus_V;		// channel A7
	int16	i16I_Induction_c;	// channel B0 
	int16	i16I_Induction_b;	// channel B1		
	int16	i16I_Induction_a;	// channel B2
	int16	i16GFCI;			// channel B3	
	int16	i16T_transformer;	// channel B4
	int16	i16I_Grid_w;		// channel B5		
	int16	i16I_Grid_v;		// channel B6
	int16	i16I_Grid_u;		// channel B7 
};

typedef struct Def_GeneralADgroup GeneralADgroup; 

#define GeneralADgroup_DEFAULTS	{	\
									0, 0, 0, 0, 0, 0, 0, 0, \
									0, 0, 0, 0, 0, 0, 0, 0 \
								}


/* SVPWM purpose AD buffers definition according to hardware */
struct Def_SVPWMADgroup	
{ 	
	int16	i16Vvw;				// channel A1 
	int16	i16Vuv;				// channel A2		
	int16	i16I_Induction_b;	// channel B1		
	int16	i16I_Induction_a;	// channel B2
	int16	i16I_Induction_u;	// channel B1		
	int16	i16I_Induction_v;	// channel B2
};

typedef struct Def_SVPWMADgroup SVPWMADgroup; 

#define SVPWMgroup_DEFAULTS	{ 	\
								0,0,0,0,0,0 \
							}

struct Def_SVPWMADfloat	
{ 	
	float32	f32Vvw;				// channel A1 
	float32	f32Vuv;				// channel A2		
	float32	f32I_Induction_b;	// channel B1		
	float32	f32I_Induction_a;	// channel B2
	float32	f32I_Induction_u;	// channel B1		
	float32	f32I_Induction_v;	// channel B2
};

typedef struct Def_SVPWMADfloat  SVPWMADfloat; 



struct Def_ADgroupfloat
{ 
	float32	f32Vwu; 			// channel A0
	float32	f32Vvw;				// channel A1 
	float32	f32Vuv;				// channel A2		
	float32	f32T_heatsink;		// channel A3
	float32	f32T_controlboard;	// channel A4	
	float32	f32Isulation;		// channel A5
	float32	f32PVDCBus_I;		// channel A6		
	float32	f32PVDCBus_V;		// channel A7
	float32	f32I_Induction_c;	// channel B0 
	float32	f32I_Induction_b;	// channel B1		
	float32	f32I_Induction_a;	// channel B2
	float32	f32GFCI;			// channel B3	
	float32	f32T_transformer;	// channel B4
	float32	f32I_Grid_w;		// channel B5		
	float32	f32I_Grid_v;		// channel B6
	float32	f32I_Grid_u;		// channel B7 
};

typedef struct Def_ADgroupfloat ADgroupfloat; 

/*=============================================================================*
 * 	Extern Variables declaration
 *============================================================================*/

/*=============================================================================*
 * NAME   : GeneralADbufferTemp
 * PURPOSE: Buffer the ADC results for General AD purpose. 
 * RANGE  : 
 *			structure type, all are int16.
 *
 * CALLED BY: 
 *          void EnergyAccCalc(void) in EnergyCalc module. 
 * 
 *============================================================================*/
extern struct Def_ADgroupfloat GeneralADbufferTemp;


/*=============================================================================*
 * NAME   : SVPWMADbufferTemp
 * PURPOSE: Buffer the ADC results for SVPWM calculation purpose. 
 * RANGE  : 
 *			structure type, all are int16.
 *
 * CALLED BY: 
 *          void dqPLLcontroller(void) and void CurrentPIDcontroller(void) in 
 *			SVPWM module. 
 * 
 *============================================================================*/



/*=============================================================================*
 * NAME   : g_u16GeneralAD_INT_Counter
 * PURPOSE: countering for entering GeneralADCswi in one PWM period. 
 * RANGE  : 
 *			0 - 3
 *
 * CALLED BY: 
 *          void SEQ1INT_ISR(void), and void GeneralADCswi(void)
 * 
 *============================================================================*/
//extern Uint16 g_u16GeneralAD_INT_Counter;


/*=============================================================================*
 * NAME   : g_u16GlobalADCounterInSVPWMPeriod
 * PURPOSE: countering for entering ADC_INT in one PWM period. 
 * RANGE  : 
 *			0 - 4
 *
 * CALLED BY: 
 *          void SEQ1INT_ISR(void), and void GeneralADCswi(void)
 * 
 *============================================================================*/
extern Uint16 g_u16GlobalADCounterInSVPWMPeriod;


/*=============================================================================*
 * 	Extern functions declaration
 *============================================================================*/
/* ADC initialization */

















#endif

//--- end of file -----------------------------------------------------











