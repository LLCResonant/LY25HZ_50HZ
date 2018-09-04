***********************************************************************
* File: SetDBGIER.asm
* Devices: TMS320F2803x
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   07/02/09 - original (D. Alter)
***********************************************************************


***********************************************************************
* Function: SetDBGIER()
* Description: Sets the DBGIER register (for realtime emulation)
* DSP: TMS320F2803x
* Include files: none
* Function Prototype: void SetDBGIER(unsigned int)
* Useage: SetDBGIER(value);
* Input Parameters: Uint16 value = value to put in DBGIER register
* Return Value: none
* Notes: none
***********************************************************************
		.def _SetDBGIER
		.text
		
_SetDBGIER:
		MOV 	*SP++,AL
		POP 	DBGIER
		LRETR

; end of function SetDBGIER()
***********************************************************************

       .end
;end of file SetDBGIER.asm
