#include "scim.s"
#include "mcci.s"
#include "unkregs.s"
#include "command.s"


;--------------------------------------------------
; XREF / XDEFS 
;--------------------------------------------------

	xref	serial_set_baud
	xref	serial_wait_RX
	xref	serial_write_byte
	xref	serial_write_response
	xref	serial_write_header
	xref	serial_write_checksum

	xref	crc32_init
	xref	crc32_next
	xref	crc32_get_crc

	xref	read_command

	xdef	do_read_programming_voltage
	xdef	do_write_flash_byte
	xdef	do_blank_16k_page
	xref	do_read_programming_voltage

	xref	toggle_watchdog
	xref	delay

	xref	load_YKY_0xFF000

	xref	command_length
	xref	serial_buffer

;--------------------------------------------------
; DATA 
;--------------------------------------------------

	switch	.bss

peak_write_pulses:	ds.b	1

	; use seperate page at end of ram area for flash buffer
	switch	.data
sz_flash_buffer:	equ		4096
;flash_buffer:		ds.b	sz_flash_buffer
flash_buffer:		ds.b	1	; keep assembler happy


;--------------------------------------------------
; CONSTANTS 
;--------------------------------------------------

min_programming_voltage:	equ	570 ; (11.4V * 50/V)
max_programming_voltage:	equ 630 ; (12.6V * 50/V)

;--------------------------------------------------
; CODE 
;--------------------------------------------------

	switch	.text
	even

					; at this point, SK:SP should be 0x21BFC

	bra		main
	dc.w	0x7C64	; this word is checked by the 02-03 WRX TPU code.
	dc.w	0x0000	; this is nothing
	dc.w	0x0000	; the 0x3941 and this word are checksumed by the
					; TPU code for the 04-05 WRX and must total 0x5AA5.
					; normally the whole odd set of words in the
					; kernel are checksummed, but we can play a trick
					; to make it only checksum these first two odd
					; words
	dc.w	0x0002	; this is nothing
	dc.w	0x0000	; this is reserved as another checksum word in 
					; case we want to tweak the overall checksum to
					; be correct. we could instead alter the previous
					; odd word, but this way old software which plays
					; the checksumming "trick" wont fail.

kernel_version:
	dc.b	"FastECU Subaru HC16 Kernel v0.01",0,0 

main:
	ldab	#2
	tbzk
	ldz		#0
	jsr		serial_set_baud

command_loop:
	jsr		read_command

	tzx
	ldx		#command_table

command_match_loop:
	tst		0, x
	beq		command_not_found
	
	ldaa	serial_buffer+0, z
	cmpa	0, x
	beq		command_match
	
	aix		#4
	bra		command_match_loop

command_match:
	; skip verification if var length command
	tst		1, x
	beq		command_fn_call
	; now let´s verify command length
	ldaa	command_length+1, z
	cmpa	1, x
	bne		command_invalid_length	

command_fn_call:
	ldx		2, x
	jsr		0x20000, x
	bra		command_loop

command_invalid_length:
	ldaa	#kernel_rsp_error_bad_data_length
	lde		#1
	jsr		serial_write_response
	bra     command_loop

command_not_found:
	ldaa	#kernel_rsp_error_bad_command
	lde		#1
	jsr		serial_write_response
	bra		command_loop
		
command_table:
	; format:
	;
	; byte command_id
	; byte data_length (including command, 0 = variable length)
	; word cmd_fn_address
	
	dc.b	kernel_cmd_get_version_info
	dc.b	1
	dc.w	cmd_get_version_info

	dc.b	kernel_cmd_unknown_1
	dc.b	1
	dc.w	kernel_func_unknown_1

	dc.b	kernel_cmd_unknown_2
	dc.b	1
	dc.w	kernel_func_unknown_2
	
	dc.b	kernel_cmd_CRC_area
	dc.b	9
	dc.w	cmd_CRC_area

	dc.b	kernel_cmd_read_area
	dc.b	7
	dc.w	cmd_read_area	

	dc.b	kernel_cmd_read_programming_voltage
	dc.b	1
	dc.w	cmd_read_programming_voltage

	dc.b	kernel_cmd_flash_enable
	dc.b	1
	dc.w	cmd_flash_enable	

	dc.b	kernel_cmd_flash_disable
	dc.b	1
	dc.w	cmd_flash_disable

	dc.b	kernel_cmd_write_flash_buffer
	dc.b	0
	dc.w	cmd_write_flash_buffer

	dc.b	kernel_cmd_validate_flash_buffer
	dc.b	11
	dc.w	cmd_validate_flash_buffer

	dc.b	kernel_cmd_commit_flash_buffer
	dc.b	11
	dc.w	cmd_commit_flash_buffer

	dc.b	kernel_cmd_blank_16k_page
	dc.b	5
	dc.w	cmd_blank_16k_page

	dc.b	0
	dc.b	0

;--------------------------------------------------
; version info 
;--------------------------------------------------

cmd_get_version_info:
	tzx
	ldx		#serial_buffer+1
	tzy
	ldy		#kernel_version	
	lde		#1

next_kernel_version_char:	
	tst		0, y
	beq		end_kernel_version
	ldaa	0, y
	staa	0, x
	aix		#1
	aiy		#1		
	adde	#1
	bra		next_kernel_version_char
	
end_kernel_version:
	ldaa	#kernel_rsp_get_version_info
	lbra	serial_write_response	

;--------------------------------------------------
; CRC32 area 
;--------------------------------------------------
	
kernel_func_unknown_1:
	ldd     #0
	std     serial_buffer+1, z
	ldd     #208h
	std     serial_buffer+3, z
	lde     #5
	ldaa    #kernel_rsp_unknown_1
	lbra    serial_write_response

kernel_func_unknown_2:
	ldd     #0
	std     serial_buffer+1, z
	ldd     #1000h
	std     serial_buffer+3, z
	lde     #5
	ldaa    #kernel_rsp_unknown_2
	lbra    serial_write_response

cmd_CRC_area:
	lde		#5
	jsr		serial_write_header
	ldaa	#kernel_rsp_CRC_area
	jsr		serial_write_byte
	ldab	serial_buffer+2, z
	tbxk
	ldx		serial_buffer+3, z
	lde		serial_buffer+7, z
	jsr		crc32_area				; do all of the work
	lde		#4

cmd_CRC_write_next_byte:
	ldaa	0, x
	jsr		serial_write_byte
	aix		#1
	sube	#1
	bne		cmd_CRC_write_next_byte
	jsr		serial_write_checksum
	rts

; crc32 area starting at X, length e
; crc32 result pointed to by X
crc32_area:
	jsr		crc32_init

crc32_read_loop:
	tste
	beq		crc32_read_done

	; toggle watchdog every 256 iterations to keep things alive
	ted
	tstb
	bne		crc32_skip_toggle
	pshm	y, k
	jsr		toggle_watchdog
	pulm	k, y

crc32_skip_toggle:
	ldaa	0, x
	jsr		crc32_next
	aix		#1
	sube	#1
	bra		crc32_read_loop
	
crc32_read_done:
	jsr		crc32_get_crc
	rts

;--------------------------------------------------
; read area 
;--------------------------------------------------

cmd_read_area:
	lde		serial_buffer+5, z
	adde	#1					; extra byte for response ID
	jsr		serial_write_header

	ldaa	#kernel_rsp_read_area
	jsr		serial_write_byte

	ldab	serial_buffer+2, z
	tbxk
	ldx		serial_buffer+3, z
	sube	#1


cmd_read_loop:
	tste
	beq		cmd_read_done
	ldaa	0, x
	jsr		serial_write_byte
	aix		#1
	sube	#1
	bra		cmd_read_loop
	
cmd_read_done:
	jsr		serial_write_checksum
	rts

;--------------------------------------------------
; read Vpp
;--------------------------------------------------

cmd_read_programming_voltage:
	jsr		do_read_programming_voltage
	std		serial_buffer+1, z
	ldaa	#kernel_rsp_read_programming_voltage
	lde		#3
	lbra	serial_write_response

;--------------------------------------------------
; flash enable
;--------------------------------------------------

cmd_flash_enable:
	jsr		load_YKY_0xFF000
	bset	FLASHEN, y, #1
	ldaa	#kernel_rsp_flash_enable
	lde		#1
	lbra	serial_write_response

;--------------------------------------------------
; flash disable
;--------------------------------------------------

cmd_flash_disable:
	jsr		load_YKY_0xFF000
	bclr	FLASHEN, y, #1
	ldaa	#kernel_rsp_flash_disable
	lde		#1
	lbra	serial_write_response

;--------------------------------------------------
; write flash buffer
;--------------------------------------------------

cmd_write_flash_buffer:
	ldd		serial_buffer+3, z	; get lower 16 bits of address
	andd	#0x0FFF				; mask to 4k
	tzx
	ldx		#flash_buffer		; point x to flash buffer
	adx							; offset by address
	tzy							; point y to serial buffer data
	ldy		#serial_buffer+5
	lde		command_length, z	; determine number of data bytes
	sube	#5

cmd_write_flash_buffer_loop:
	beq		cmd_write_flash_buffer_done
	ldaa	0, y
	staa	0, x
	aix		#1
	aiy		#1
	sube	#1
	bra		cmd_write_flash_buffer_loop

cmd_write_flash_buffer_done:
	ldaa	#kernel_rsp_write_flash_buffer
	lde		#1	
	lbra	serial_write_response

;--------------------------------------------------
; validate flash buffer
;--------------------------------------------------

; todo: combine with commit fn to save code space

cmd_validate_flash_buffer:
	; check crc32

	; get crc32 of flash buffer
	ldd		serial_buffer+3, z	; get lower 16 bits of address ADDED
	andd	#0x0FFF				; mask to 4k ADDED
	tzx
	ldx		#flash_buffer
	adx							; offset by address ADDED
	lde		serial_buffer+5, z	; ORIG #sz_flash_buffer
	jsr		crc32_area
	
	; compare it to expected CRC
	ldd		0, x
	cpd		serial_buffer+7, z
	bne		cmd_commit_flash_buffer_crc_bad
	ldd		2, x
	cpd		serial_buffer+9, z
	bne		cmd_commit_flash_buffer_crc_bad

	ldaa	#kernel_rsp_validate_flash_buffer
	lde		#1
	lbra	serial_write_response


;--------------------------------------------------
; commit flash buffer
;--------------------------------------------------

cmd_commit_flash_buffer:
	; check crc32

	; get crc32 of flash buffer
	ldd		serial_buffer+3, z	; get lower 16 bits of address ADDED
	andd	#0x0FFF				; mask to 4k ADDED
	tzx
	ldx		#flash_buffer
	adx							; offset by address ADDED
	lde		serial_buffer+5, z	; ORIG #sz_flash_buffer
	jsr		crc32_area
	
	; compare it to expected CRC
	ldd		0, x
	cpd		serial_buffer+7, z
	bne		cmd_commit_flash_buffer_crc_bad
	ldd		2, x
	cpd		serial_buffer+9, z
	bne		cmd_commit_flash_buffer_crc_bad

cmd_commit_flash_buffer_crc_ok:
	; check voltage
	jsr		do_read_programming_voltage
	cpd		#min_programming_voltage
	bge		voltage_not_low
	ldaa	#kernel_rsp_error_prog_voltage_low
	bra		cmd_commit_flash_buffer_error	

cmd_commit_flash_buffer_crc_bad:
	ldaa	#kernel_rsp_error_bad_crc
	bra		cmd_commit_flash_buffer_error	

voltage_not_low:
	cpd		#max_programming_voltage
	ble		voltage_ok
	ldaa	#kernel_rsp_error_prog_voltage_high
	bra		cmd_commit_flash_buffer_error	

voltage_ok:
	; write actual flash bytes
	clr		peak_write_pulses, z
	ldab	serial_buffer+2, z	; get dest address in X
	tbxk
	ldx		serial_buffer+3, z
	ldd		serial_buffer+3, z
	andd    #0FFFh
	tzy							; get source address in Y
	ldy		#flash_buffer
	ady
	lde		serial_buffer+5, z	; ORIG #sz_flash_buffer	; bytes to write in E

cmd_commit_flash_buffer_next_byte:
	tste
	beq		cmd_commit_flash_buffer_success	
	ldaa	0, y
	jsr		do_write_flash_byte
	tsta
	bne		cmd_commit_flash_buffer_byte_ok
	; *** write failure ***
	ldaa	#kernel_rsp_error_programming_failure
	bra		cmd_commit_flash_buffer_error	

cmd_commit_flash_buffer_byte_ok:
	; now record worst-case write attempts
	cmpa	peak_write_pulses, z
	ble		cmd_commit_flash_buffer_move_to_next_byte
	staa	peak_write_pulses, z

cmd_commit_flash_buffer_move_to_next_byte:
	; loop to write all bytes in buffer
	sube	#1
	aiy		#1
	aix		#1
	bra		cmd_commit_flash_buffer_next_byte

cmd_commit_flash_buffer_success:
	clr		serial_buffer+1, z
	ldaa	peak_write_pulses, z
	staa	serial_buffer+2, z
	ldaa	#kernel_rsp_commit_flash_buffer
	lde		#3
	lbra	serial_write_response

cmd_commit_flash_buffer_error:
	lde		#1
	lbra	serial_write_response

;--------------------------------------------------
; blank 16k page
;--------------------------------------------------
	
cmd_blank_16k_page:
	; check voltage
	jsr		do_read_programming_voltage
	cpd		#min_programming_voltage
	bge		cmd_blank_voltage_not_low
	ldaa	#kernel_rsp_error_prog_voltage_low
	bra		cmd_commit_flash_buffer_error	

cmd_blank_voltage_not_low:
	cpd		#max_programming_voltage
	ble		cmd_blank_voltage_ok
	ldaa	#kernel_rsp_error_prog_voltage_high
	bra		cmd_commit_flash_buffer_error	

cmd_blank_voltage_ok:
	ldab	serial_buffer+2, z	; get page address in X
	tbxk
	ldx		serial_buffer+3, z
	jsr		do_blank_16k_page
	tsta	
	bne		cmd_blank_16k_page_ok
	; *** erase failure ***
	ldaa	#kernel_rsp_error_programming_failure
	lde		#1
	lbra	serial_write_response

cmd_blank_16k_page_ok:
	clr		serial_buffer+1, z	; return erase pulses
	staa	serial_buffer+2, z
	ldaa	#kernel_rsp_blank_16k_page
	lde		#3
	lbra	serial_write_response

	end
