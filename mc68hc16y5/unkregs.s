;	UNKNOWN (UNDOCUMENTED) REGISTERS Y5
;

UNKGPT:			equ $99A	; timer good for tracking WDT toggling

FLASHEN:		equ	$639	; some sort of flashing enable register, or maybe switches Vpp in externally

UNKADCDR:		equ	$2E4	; unknown ADC converter
UNKADCCR:		equ	$20E	;

FEExBASE:		equ	$780	; base addr (+0xFF000) of flash registers
FEExSIZE:		equ	$40     ; size of flash register set per flash block
offFEExMCRlo:	equ	$0001	; 
offFEExCTL:		equ	$0009	;
