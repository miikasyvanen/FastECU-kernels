
#include "mcci.s"
#include "unkregs.s"
#include "scim.s"	

;#define	fakedelay 1	; used to make simulations run quickly

;--------------------------------------------------
; XREF / XDEFS 
;--------------------------------------------------

	xdef	serial_set_baud
	xdef	serial_wait_RX
	xdef	serial_write_byte
	xdef	serial_read_byte
	xdef	serial_write_response
	xdef	serial_write_header
	xdef	serial_write_checksum
	xdef	read_command
	xdef	toggle_watchdog
	xdef	delay
	xdef	load_YKY_0xFF000

;--------------------------------------------------
; DATA 
;--------------------------------------------------
	
	switch	.bss

command_length:				ds.w	1
							xdef	command_length

serial_status:				ds.b	1
serial_status_timeout:		equ		0x01
serial_timer:				ds.b	1
serial_checksum:			ds.b	1

max_command_length:			equ		520
serial_buffer:				ds.b	max_command_length
							xdef	serial_buffer

;--------------------------------------------------
; CONSTANTS 
;--------------------------------------------------

; packet header byte sequence
#define	START_SEQ_1	0xBE
#define START_SEQ_2	0xEF

; delay when responding to query (avoids serial collision)
#define	RESPONSE_DELAY	200     ; 1ms

; polling delay
#define	POLLING_DELAY	10      ; 50us

; bail on packets if we dont get chars more quickly than this
#define	SERIAL_TO_DELAY 255 ; ~12ms (POLLING_DELAY * 255)

;--------------------------------------------------
; CODE 
;--------------------------------------------------

	switch	.text
	even

;--------------------------------------------------
; points YK:K to the register pages (0xFF000)
;
; time (including call) = 10+6+2+2+4+8+12 = 44
;--------------------------------------------------
load_YKY_0xFF000:
	pshm	d			; 6
	ldab	#0x0F		; 2
	tbyk				; 2
	ldy		#0xF000		; 4
	pulm	d			; 8
	rts					; 12

;--------------------------------------------------
; delays D time = 2us + 5us * D
; call to: 10
; pre: 8
; loop: 4+2+4+90+6+8+6
; post: 22
; total: 50+120*D
; 
;--------------------------------------------------
delay:
	pshm	y, k						; 8

#ifdef fakedelay
	ldd		#0
#endif

delay_loop:
	cpd		#0							; 4
	beq		delay_done					; 2,6
	subd	#1							; 4
	jsr		toggle_watchdog				; 90
	pshm	d							; 6
	pulm	d							; 8
	bra		delay_loop					; 6
delay_done:
	pulm	k, y						; 10
	rts									; 12


;--------------------------------------------------
; toggle MSRA watchdog timer
; time (no brset) = 8+44+10+8+6+12=90
; time (w  brset) = 8+44+14+2+8+12=90
;--------------------------------------------------
toggle_watchdog:
	bsr		load_YKY_0xFF000			; 44
	brclr	UNKGPT, Y, #0x08, toggle_watchdog_clear ; 10,14

	bset	TSTMSRA_lo, y, #0x40		; 8
	bra		toggle_done					; 6

toggle_watchdog_clear:
	nop
	bclr	TSTMSRA_lo, y, #0x40		; 8

toggle_done:
	rts									; 12

#define baud_9600	0x4E ; 9615 baud
#define baud_38400	0x13 ; 39473 baud
#define baud_62500	0x0C ; 62500 baud

;--------------------------------------------------
; sets the baud rate
;--------------------------------------------------
serial_set_baud:
	bsr		load_YKY_0xFF000
	bsr		serial_change_baudrate_wait_tx_empty
	ldd		#baud_62500
	std		SCCR0B, Y
	rts
	
serial_change_baudrate_wait_tx_empty:
	pshm    D, Y, K
	bra     serial_write_byte_wait_tx_empty
;--------------------------------------------------
; writes the byte in A out the serial port and
; adds it to cur_chksum
; does not complete until there is room in the data register
; and the byte has actually been sent
;--------------------------------------------------
serial_write_byte:
	pshm	d, y, k
	bsr		serial_enable_TX
	bsr		toggle_watchdog
	tab
	addb	serial_checksum, z
	stab	serial_checksum, z
	bsr		load_YKY_0xFF000
	staa	SCDRB_lo, y

serial_write_byte_wait_tx_empty:
	brclr	SCSRB_hi, y, #0x01, serial_write_byte_wait_tx_empty

serial_write_byte_wait_tx_sent:
	brclr	SCSRB_lo, y, #0x80, serial_write_byte_wait_tx_sent
	pulm	k, y, d
	rts

;--------------------------------------------------
; reads a bytes from the serial port and
; adds it to serial_checksum
;--------------------------------------------------
serial_read_byte:
	pshm	y, k
	bclr	serial_status, z, serial_status_timeout
	ldaa	#SERIAL_TO_DELAY
	staa	serial_timer, z
	jsr		load_YKY_0xFF000

serial_read_byte_wait:
	dec		serial_timer, z
	beq		serial_read_byte_timeout

	bsr		serial_enable_RX
	bsr		toggle_watchdog
	ldd		#POLLING_DELAY
	jsr		delay
	

	brclr	SCSRB_lo, y, #0x40, serial_read_byte_wait	; check for RX data
	brclr	SCSRB_lo, y, #0x0E, serial_read_byte_good	; check for errors
	ldd     SCDRB, y
	bra		serial_read_byte_wait

serial_read_byte_good:	
	ldd		SCDRB, y									; get data
	tba													; update checksum
	addb	serial_checksum, z
	stab	serial_checksum, z
	pulm	k, y
	rts

serial_read_byte_timeout:
	clra						; setting A = 0 simplifies error return
								; checking for some code
	bset	serial_status, z, serial_status_timeout
	pulm	k, y
	rts

;--------------------------------------------------
; turn on MCCI transmit mode
; turn off MCCI receive mode
; (half duplex)
;--------------------------------------------------
serial_enable_TX:
	pshm	d, y, k
	jsr		load_YKY_0xFF000
	bset	SCCR1B_lo, y, #0x08
	bclr	SCCR1B_lo, y, #0x04
	pulm	k, y, d
	rts

;--------------------------------------------------
; turn on MCCI receive mode
; turn off MCCI transmit mode
; (half duplex)
;--------------------------------------------------
serial_enable_RX:
	pshm	d, y, k
	jsr		load_YKY_0xFF000
	bset	SCCR1B_lo, y, #0x04
	bclr	SCCR1B_lo, y, #0x08
	pulm	k, y, d
	rts


;--------------------------------------------------		
; send a response from the kernel
; A = response code
; E = # of data bytes in serial_buffer
;--------------------------------------------------
serial_write_response:
	pshm	d, e, x, k
	staa	serial_buffer+0, z		; store response
	bsr		serial_write_header											
	tzx								; point X to serial_buffer
	aix		#serial_buffer
	sube	#0

response_data_loop:
	beq		response_data_done
	ldaa	0, x
	jsr		serial_write_byte
	aix		#1
	sube	#1
	bra		response_data_loop
	
response_data_done:					; write checksum
	bsr		serial_write_checksum
	pulm	k, x, e, d
	rts

serial_write_checksum:
	ldaa	serial_checksum, z
	jsr		serial_write_byte
	rts

serial_write_header:
	ldd		#RESPONSE_DELAY
	jsr		delay

	clr		serial_checksum, z
	ldaa	#START_SEQ_1					; write start sequence
	jsr		serial_write_byte
	ldaa	#START_SEQ_2
	jsr		serial_write_byte
	ted								; write length
	jsr		serial_write_byte
	tba
	jsr		serial_write_byte
	rts

;--------------------------------------------------		
; read a command from PC (blocking)
; results in command_length and serial_buffer[command_length]
;--------------------------------------------------
read_command:
	pshm	d, e, x, y, k

read_command_wait_valid:
	clr		serial_checksum, z
	
	jsr		serial_read_byte							; check for start sequence
	cmpa	#START_SEQ_1
	bne		read_command_wait_valid
	jsr		serial_read_byte
	cmpa	#START_SEQ_2
	bne		read_command_wait_valid
	
	jsr		serial_read_byte							; get command length
	brset	serial_status, z, serial_status_timeout, read_command_wait_valid
	staa	command_length, z
	jsr		serial_read_byte
	brset	serial_status, z, serial_status_timeout, read_command_wait_valid
	staa	command_length+1, z
	
	ldd		command_length, z							; check to make sure command
	subd	#max_command_length							; is no too long
	bgt		read_command_wait_valid
	
	tzx													; point X to serial_buffer
	aix		#serial_buffer

	lde		command_length, z

read_command_data_loop:							; read in (command_length) bytes to X
	beq		read_command_data_done
	jsr		serial_read_byte
	brset	serial_status, z, serial_status_timeout, read_command_wait_valid
	staa	0, x
	aix		#1
	sube	#1
	bra		read_command_data_loop

read_command_data_done:
	jsr		serial_read_byte					; now verify checksum
	brset	serial_status, z, serial_status_timeout, read_command_wait_valid
	tab											; serial_read_byte will have already
	aba											; added the checksum to cur_checksum
	cmpa	serial_checksum, z					; so we need to add checksum to itself 
	bne		read_command_wait_valid				; before making a comparison
	
	pulm	k, y, x, e, d
	rts	

	end

