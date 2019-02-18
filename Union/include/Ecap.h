/*=============================================================================*
 * Copyright(c)
 * 						ALL RIGHTS RESERVED
 *
 *  FILENAME : Ecap.h
 *
 *  PURPOSE  :
 *  
 *  HISTORY  :
 *    DATE            VERSION         AUTHOR            NOTE
 *    2018.6.2		001					NUAA XING
 *============================================================================*/

#ifndef ECAP_H
#define ECAP_H



/*=============================================================================*
 * 	Extern Variables declaration
 *============================================================================*/

/*=============================================================================*
 * NAME   : i32PhaseSequencyTemp_Vvw, i32PhaseSequencyTemp_Vwu
 * PURPOSE: Detect and check phase sequency. 
 * RANGE  : 
 *			0 - abs(int32).
 *
 * CALLED BY: 
 *          void ECAP2_INT_ISR(void) and void ECAP3_INT_ISR(void). 
 * 
 *============================================================================*/
extern int32 g_i32PhaseSequencyTemp_Vvw;
extern int32 g_i32PhaseSequencyTemp_Vwu; 

/*=============================================================================*
 * NAME   : i32GridFrequencyPriodTemp
 * PURPOSE: Calculate grid frequency according to falling voltage_zero_points. 
 * RANGE  : 
 *			0 - abs(int32).
 *
 * CALLED BY: 
 *          void ECAP1_INT_ISR(void), MainStatusMachine module. 
 * 
 *============================================================================*/
/* The value in i32GridFrequencyPriodTemp is clocked by 150MHz, so the 
   Gridfrequecy should be calculated by:
   Gridfrequecy = (150000000/i32GridFrequencyPriodTemp), unit is Herz */

/*=============================================================================*
 * 	Extern functions declaration
 *============================================================================*/
/* eCAP module initialization */


/* Phase sequency detection by eCAP2/3 */
extern void DetectPhaseSequency(void);

/* Check if the phase_Sequency is normal or reverse */
extern void CheckPhaseSequency(void);



#endif

//--- end of file -----------------------------------------------------

