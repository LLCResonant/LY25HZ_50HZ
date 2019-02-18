***********************************************************************
* File: SetDBGIER.asm
* Devices: TMS320F2833x
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   12/18/07 - original (D. Alter)
***********************************************************************


***********************************************************************
* Function: SetDBGIER()
* Description: Sets the DBGIER register (for realtime emulation)
* DSP: TMS320F28335, TMS320F28334, TMS320F28332
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
