;	MULTICHANNEL COMMUNICATION  PERIPHERAL REGISTERS Y1
;	Copyright (c) 1996 by COSMIC Software
;
MMCR:		equ	$C00	; Module Configuration Reg
MTEST:		equ	$C02	; MCCI Test Reg
ILSCI:		equ	$C04	; SCI Interrupt Reg
MIVR:		equ	$C05	; SCI Interrupt Vector Reg
ILSPI:		equ	$C06	; SPI Interrupt Reg
PMCPAR:		equ	$C09	; Port Pin Assignment Reg
DDRMC:		equ	$C0B	; Port Data Direction Reg
PORTMC:		equ	$C0D	; Port Data Register
SCCR0A:		equ	$C18	; SCIA Control Reg 0
SCCR1A:		equ	$C1A	; SCIA Control Reg 1
SCSRA:		equ	$C1C	; SCIA Status Register
SCDRA:		equ	$C1E	; SCIA Data Register
SCCR0B:		equ	$C28	; SCIB Control Reg 0
SCCR1B:		equ	$C2A	; SCIB Control Reg 1
SCCR1B_lo:	equ	$C2B
SCSRB:		equ	$C2C	; SCIB Status Register
SCSRB_hi:	equ	$C2C
SCSRB_lo:	equ	$C2D
SCDRB:		equ	$C2E	; SCIB Data Register
SCDRB_lo:	equ	$C2F
SPCR:		equ	$C38	; SPI Control Register
SPSR:		equ	$C3C	; SPI Status Register
SPDR:		equ	$C3E	; SPI Data Register
;
