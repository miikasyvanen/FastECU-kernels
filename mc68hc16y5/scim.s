;	SINGLE-CHIP INTEGRATION MODULE PERIPHERAL REGISTERS Y1
;	Copyright (c) 1996 by COSMIC Software
;
SCIMCR:		equ	$A00	; Module Configuration Reg
SCIMTR:		equ	$A02	; Factory Test Reg
SYNCR:		equ	$A04	; Clock Synthesizer Ctrl Reg
RSR:		equ	$A07	; Reset Status Reg
SCIMTRE:	equ	$A08	; SCIM Test E Reg
PORTA:		equ	$A0A	; Port A Data Reg
PORTB:		equ	$A0B	; Port B Data Reg
PORTG:		equ	$A0C	; Port G Data Reg
PORTH:		equ	$A0D	; Port H Data Reg
PORTE0:		equ	$A11	; Port E0 Data Reg
PORTE1:		equ	$A13	; Port E1 Data Reg
DDRAB:		equ	$A14	; Port A/B Data Direction Reg
DDRE:		equ	$A15	; Port E Data Direction Reg
PEPAR:		equ	$A17	; Port E Pin Assignment Reg
PORTF0:		equ	$A19	; Port F0 Data Reg
PORTF1:		equ	$A1B	; Port F1 Data Reg
DDRF:		equ	$A1D	; Port F Data Direction Reg
PFPAR:		equ	$A1F	; Port F Pin Assignment Reg
SYPCR:		equ	$A21	; System Protection Ctrl Reg
PICR:		equ	$A22	; Periodic Interrupt Ctrl Reg
PITR:		equ	$A24	; Periodic Interrupt Timer
SWSR:		equ	$A27	; Software Service Reg
PORTFE:		equ	$A29	; Port F Edge Control Reg
PFIVR:		equ	$A2B	; Port F Edge It Vector Reg
PFLVR:		equ	$A2B	; Port F Edge It Level Reg
TSTMSRA:	equ	$A30	; Test Module Master Shift A
TSTMSRA_lo:	equ	$A31
TSTMSRB:	equ	$A32	; Test Module Master Shift B
TSTSC:		equ	$A34	; Test Module Shift Count
TSTRC:		equ	$A36	; Test Module Repeat Counter
CREG:		equ	$A38	; Test Module Control Reg
DREG:		equ	$A3A	; Test Module Distributed Reg
PORTC:		equ	$A41	; Port C Data Reg
CSPAR0:		equ	$A44	; Chip Select Pin Asgn Reg 0
CSPAR1:		equ	$A46	; Chip Select Pin Asgn Reg 1
CSBARBT:	equ	$A48	; Chip Select Base Reg Boot
CSORBT:		equ	$A4A	; Chip Select Option Reg Boot
CSBAR0:		equ	$A4C	; Chip Select Base Reg 0
CSOR0:		equ	$A4E	; Chip Select Option Reg 0
CSBAR3:		equ	$A58	; Chip Select Base Reg 3
CSOR3:		equ	$A5A	; Chip Select Option Reg 3
CSBAR5:		equ	$A60	; Chip Select Base Reg 5
CSOR5:		equ	$A62	; Chip Select Option Reg 5
CSBAR6:		equ	$A64	; Chip Select Base Reg 6
CSOR6:		equ	$A66	; Chip Select Option Reg 6
CSBAR7:		equ	$A68	; Chip Select Base Reg 7
CSOR7:		equ	$A6A	; Chip Select Option Reg 7
CSBAR8:		equ	$A6C	; Chip Select Base Reg 8
CSOR8:		equ	$A6E	; Chip Select Option Reg 8
CSBAR9:		equ	$A70	; Chip Select Base Reg 9
CSOR9:		equ	$A72	; Chip Select Option Reg 9
CSBAR10:	equ	$A74	; Chip Select Base Reg 10
CSOR10:		equ	$A76	; Chip Select Option Reg 10
;
