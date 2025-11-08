;--------------------------------------------------
; XREF / XDEFS 
;--------------------------------------------------

	xdef	crc32_init
	xdef	crc32_next
	xdef	crc32_get_crc

;--------------------------------------------------
; DATA 
;--------------------------------------------------

	switch	.bss

crc32_reg:	ds.b	4

;--------------------------------------------------
; CONSTANTS 
;--------------------------------------------------

; in order to avoid reversing our input and result, we instead
; reverse the poly and shift backwards
crc32_poly0:	equ		0xEDB8
crc32_poly2:	equ		0x8320

;--------------------------------------------------
; CODE 
;--------------------------------------------------

	switch	.text
	
;--------------------------------------------------
; call to init crc
;--------------------------------------------------
crc32_init:
	bsetw	crc32_reg, z, #0xFFFF
	bsetw	crc32_reg+2, z, #0xFFFF
	rts

;--------------------------------------------------
; pass next byte to crc in A 
;--------------------------------------------------
crc32_next:
	pshm	d, e

	eora	crc32_reg+3, z
	staa	crc32_reg+3, z

	lde		#8

crc32_next_bit:	

	lsrw	crc32_reg, z
	rorw	crc32_reg+2, z
	
	bcc		crc32_no_xor

	ldd		#crc32_poly2
	eord	crc32_reg+2, z
	std		crc32_reg+2, z
	ldd		#crc32_poly0
	eord	crc32_reg, z
	std		crc32_reg, z

crc32_no_xor:

	sube	#1
	bne		crc32_next_bit
	pulm	e, d
	rts
	
;--------------------------------------------------
; call to read crc result
; will return X pointed to CRC array
;--------------------------------------------------
crc32_get_crc:
	; augment message
	pshm	d
	; xor crc with 0xFFFFF
	comw	crc32_reg, z
	comw	crc32_reg+2, z
	tzx
	ldx		#crc32_reg
	pulm	d
	rts

	end


