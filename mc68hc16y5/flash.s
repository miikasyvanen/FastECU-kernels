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
	xdef	do_blank_tpu_block

;--------------------------------------------------
; DATA 
;--------------------------------------------------

	switch	.bss

blank_page_addr:		ds.w	1
blank_page_attempts:	ds.b	1
tpu_latch_addr:			ds.w	1

;--------------------------------------------------
; CONSTANTS 
;--------------------------------------------------

max_blank_page_attempts:	equ		6
TPU_FLASH_BANK:				equ		6		; the TPU flash array is at $60000
TFMCR_TPU:					equ		0x08	; TFMCR low byte: TPU flash is in TPU mode, the CPU cannot erase/program it
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
; erase one 1KB block of the TPU flash (bank 6)
; any address inside the block in XK:X
; returns # of erase attempts required in A
; A = 0 means failure (also if the module is in TPU mode)
;
; Datasheet 13.5.5.2 / table 157: with ERAS and LAT set, the ADDRESS
; written to the array chooses what is erased. A5 = 1 with A4 = 0
; erases the single 1KB block numbered by A3:A2 and leaves the
; shadow registers alone; only a bulk erase (A5 = 0) wipes them.
; The block number is A11:A10 of the address given.
;--------------------------------------------------
do_blank_tpu_block:
	pshm	e, y, k

	stx		blank_page_addr, z
	ldd		blank_page_addr, z
	tab							; B = high byte of the address
	andb	#0x0C				; A3:A2 value = block number * 4 (A11:A10 >> 8)
	orab	#0x20				; A5 = 1, A4 = 0: block erase mode
	anda	#0xF0				; keep the array's 4KB page bits
	std		tpu_latch_addr, z	; where the erase mode gets latched
	ldd		blank_page_addr, z
	andd	#0xFC00				; first byte of the 1KB block
	std		blank_page_addr, z

	bsr		set_Y_to_flashCR	; Y = TFLASH control block
	ldab	offFEExMCRlo, y
	andb	#TFMCR_TPU
	bne		tpu_blank_fail_mode
	clr		blank_page_attempts, z

tpu_blank_attempt:
	inc		blank_page_attempts, z
	ldab	blank_page_attempts, z
	cmpb	#max_blank_page_attempts
	ble		tpu_blank_pulse

tpu_blank_fail:
	clr		blank_page_attempts, z
	bra		tpu_blank_cleanup

tpu_blank_fail_mode:
	clr		blank_page_attempts, z
	bra		tpu_blank_cleanup

tpu_blank_pulse:
	bsr		tpu_erase_pulse		; B = pulse multiplier
	ldx		blank_page_addr, z
	lde		#512				; 1KB in words

tpu_blank_check:
	ldd		0, x
	subd	#0xFFFF
	bne		tpu_blank_attempt
	aix		#2
	sube	#1
	bgt		tpu_blank_check

	; erased: one margin pulse, EM = tep * sum(1..N) = tep * N * (N + 1) / 2
	ldaa	blank_page_attempts, z	; N
	tab
	incb							; N + 1
	mul								; N * (N + 1)
	lsrd							; N * (N + 1) / 2
	bsr		tpu_erase_pulse

tpu_blank_cleanup:
	bclr	offFEExCTL, y, #0x06	; ERAS and LAT off
	ldaa	blank_page_attempts, z

	pulm	k, y, e
	rts

;--------------------------------------------------
; one TPU flash erase pulse (datasheet steps 3-9): erase mode and
; latches on, latch the block address, apply Vpp for B * 100ms,
; remove it, latches off. XK = TPU bank, Y = control block, B = pulse
; multiplier. Preserves nothing but Y/K/Z.
;--------------------------------------------------
tpu_erase_pulse:
	pshm	d
	bset	offFEExCTL, y, #0x06	; ERAS + LAT, verify bit clear
	ldx		tpu_latch_addr, z
	ldd		#0xFFFF
	std		0, x					; any data; the address picks the block
	pulm	d
	bsr		toggle_vpp_erase
	bclr	offFEExCTL, y, #0x06
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
; HD flash: YK:Y = 0xFF780 + 0x40 * XK
; TPU flash (bank 6, array at $60000): YK:Y = 0xFF860
;
;--------------------------------------------------
set_Y_to_flashCR:
	pshm	d
	ldab	#0x0F
	tbyk
	txkb
	cmpb	#TPU_FLASH_BANK
	bne		set_Y_to_hd_flashCR
	ldy		#(0xF000+TFEExBASE)
	pulm	d
	rts

set_Y_to_hd_flashCR:
	ldy		#(0xF000+FEExBASE)
	ldaa	#FEExSIZE
	mul
	ady
	pulm	d
	rts

	end


