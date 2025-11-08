#include "scim.s"
#include "mcci.s"
#include "unkregs.s"

;--------------------------------------------------
; XREF / XDEFS 
;--------------------------------------------------

	xref	load_YKY_0xFF000
	xref	delay
	xdef	do_read_programming_voltage
	xdef	do_write_flash_byte
	xdef	do_blank_16k_page

;--------------------------------------------------
; DATA 
;--------------------------------------------------

	switch	.bss

blank_page_addr:		ds.w	1
blank_page_attempts:	ds.b	1

;--------------------------------------------------
; CONSTANTS 
;--------------------------------------------------

max_blank_page_attempts:	equ		6
max_programming_attempts:	equ		50
DELAY_PWPP:					equ		4		; 22us spec: 20-25us
DELAY_TPR:					equ		1		; 7us spec: 1us
DELAY_TEP:					equ		20000	; 100ms spec: 100ms
DELAY_TER:					equ		200		; 1ms spec: 1ms
DELAY_READ_VPP:				equ		200		; 1ms

;--------------------------------------------------
; CODE 
;--------------------------------------------------

	switch	.text
	even

;--------------------------------------------------
; read Vpp
; put result in D
;--------------------------------------------------
do_read_programming_voltage:
	pshm	y, k
	jsr		load_YKY_0xFF000
    bset    UNKADCCR, Y, #20h ; trigger conversion
    ldd     #DELAY_READ_VPP
    jsr     delay
    ldd     UNKADCDR, Y    ; read result
	pulm	k, y
	rts

;--------------------------------------------------
; write a byte to erased flash memory
; address in XK:X
; byte to flash in A
; returns # of programming attempts required in A
; A = 0 means failure
;--------------------------------------------------
do_write_flash_byte:
	pshm 	y, k
	; todo - validate cheksums?
	jsr		set_Y_to_flashCR		; set Y to correct flash CR for addr in X

	bclr	offFEExCTL, y, #0x04	; set to "programming" mode
	bset	offFEExCTL, y, #0x02	; enable programming latches
	staa	0, x					; write the byte to latch
	clrb							; track programming attempts in B

write_flash_byte_attempt:
	incb
	bsr		toggle_vpp_program
	cmpa	0, x
	beq		write_flash_byte_success
	cmpb	#max_programming_attempts
	blt		write_flash_byte_attempt

write_flash_byte_fail:
	clrb							; set attempts to 0 to indicate failure

write_flash_byte_success:
	tba								; copy programming attempts for return value
write_flash_byte_margin:
	; pulse the programming voltage again an 
	; equal number of times as was needed to get
	; the bits to change initially, to insure
	; adequate "burn in"
	tstb
	beq		write_flash_byte_cleanup
	bsr		toggle_vpp_program
	decb	
	bra		write_flash_byte_margin

write_flash_byte_cleanup:
	bclr	offFEExCTL, y, #0x06	; set to "programming" mode, latch off	
	pulm 	k, y
	rts

;--------------------------------------------------
; twiddle Vpp with correct timing for programming
;--------------------------------------------------
toggle_vpp_program:
	pshm	d
	bset	offFEExCTL, y, #0x01	; apply programming voltage
	ldd		#DELAY_PWPP
	jsr		delay
	bclr	offFEExCTL, y, #0x01	; remove programming voltage
	ldd		#DELAY_TPR
	jsr		delay
	pulm	d
	rts

;--------------------------------------------------
; erase a page of flash memory
; address of page in XK:X
; returns # of erase attempts required in A
; A = 0 means failure
;--------------------------------------------------
do_blank_16k_page:

	pshm	e, y, k

	stx		blank_page_addr, z
	ldd		blank_page_addr, z	
	rola						; determine 16k page number from upper 2 bits
	rola						; of X (now in A) and put a bitfield for it 
	rola			 			; in FEExMCRlo	 
	anda	#0x03

	txkb						; in 0x28000-0x2FFFF range?
	cmpb	#2
	bne		no_address_adjust
	suba	#2					; adjust addressess down by 2 pages

no_address_adjust:
	ldab	#1
shift_mask:
	tsta
	beq		done_shifting
	aslb
	deca
	bra		shift_mask
done_shifting:

	bsr		set_Y_to_flashCR	 ; set Y to correct flash CR for addr X
	bset	offFEExCTL, y, #0x06 ; enable erasure and latches
	stab	offFEExMCRlo, y
	clr		blank_page_attempts, z

blank_attempt:
	inc		blank_page_attempts, z
	ldab	blank_page_attempts, z
	cmpb	#max_blank_page_attempts
	ble		blank_attempt2

page_blank_fail:
	clr		blank_page_attempts, z
	bra		blank_page_cleanup

blank_attempt2:
	bsr		toggle_vpp_erase
	ldx		blank_page_addr, z
	lde		#8192

blank_check:
	ldd		0, x
	subd	#0xFFFF
	bne		blank_attempt
	aix		#2
	sube	#1
	bgt		blank_check

page_is_blank:
	; calcuate erase margin, which is actually the sum of all 
	; the erase pulse times done so far. so we need to calculate
	; sum(1..N) to have EM = tep * sum(1..N)
	; since sum(1..N) = N * (N + 1) / 2....
	;
	ldaa	blank_page_attempts, z	; N
	tab
	incb							; N + 1
	mul								; N * (N + 1)
	lsrd							; N * (N + 1) / 2
	bsr		toggle_vpp_erase

blank_page_cleanup:
	bclr	offFEExCTL, y, #0x06 ; disable erasure and latches
	ldaa	blank_page_attempts, z

	pulm	k, y, e
	rts

;--------------------------------------------------
; twiddle Vpp with correct timing for erasing
; - this includes a variable pulse timing which
; is a multiple of B
;--------------------------------------------------
toggle_vpp_erase:
	bset	offFEExCTL, y, #0x01	; apply programming voltage
	clra
	tde
do_delay_tepk:
	ldd		#DELAY_TEP
	jsr		delay
	sube	#1
	bne		do_delay_tepk
	bclr	offFEExCTL, y, #0x01	; remove programming voltage
	ldd		#DELAY_TER
	jsr		delay
	rts

;--------------------------------------------------
; set Y to correct flash CR for addr X
; address in XK:X
; flashCRbase returned in YK:Y
;
; YK:Y = 0xFF780 + 0x40 * XK
;
;--------------------------------------------------
set_Y_to_flashCR:
	pshm	d
	ldab	#0x0F
	tbyk
	ldy		#(0xF000+FEExBASE)
	txkb
	ldaa	#FEExSIZE
	mul
	ady
	pulm	d
	rts

	end


