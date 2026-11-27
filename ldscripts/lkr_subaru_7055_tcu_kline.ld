/*
*****************************************************************************
**
** Linker script for the Subaru Denso SH7055 K-Line TCU kernel (BUILDFOR=TCU BUILDCOMMS=KLINE), running from RAM.
**	- no heap
**	- stack at end of RAM
**
From GNU ld docs :
"
Every loadable or allocatable output section has two addresses. The ?rst is the VMA, or
virtual memory address. This is the address the section will have when the output ?le is
run. The second is the LMA, or load memory address. This is the address at which the
section will be loaded. In most cases the two addresses will be the same. An example of
when they might be diferent is when a data section is loaded into ROM, and then copied
into RAM when the program starts up (this technique is often used to initialize global
variables in a ROM based system). In this case the ROM address would be the LMA, and
the RAM address would be the VMA.
"
ADDR(section) returns the VMA of <section>.
LOADADDR(section) returns the LMA of <section>
*****************************************************************************
*/

/* (c) copyright fenugrec 2022
 * GPLv3
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



/* Entry Point */
ENTRY(RAMjump_entry)

/* Memory areas
 *
 * The Denso SH7055 K-Line TCU bootloader (boot block of 04 BXT 63FE2TA2, 05 FXT 86FE7WA1, 87FE7WA2, 86D12WA1, 86D17WF0; FastECU-RE
 * RE_FINDINGS_97FE606000.md) takes a 34 range with start >= 0xFFFF8000 and end <= 0xFFFFDFFF, checks the 16-bit sum 0x5AA5 on 31 01 01,
 * answers 71 01 01 and then ALWAYS jumps to 0xFFFF8004 (ROM 0x40: SP = 0xFFFFDFA0, jmp @r4 with r4 = 0xFFFF8004), whatever address the
 * image was loaded at. So the image is linked and uploaded at 0xFFFF8004 (like the ECU's 0xFFFF6004), .bss above it and the stack at the
 * top of RAM. Its own RAM (0xFFFF8000-0xFFFF85FF, 0xFFFFDFA0-0xFFFFDFFF) is dead after the jump. */
MEMORY {
  RAM (xw)	: ORIGIN = 0xFFFF8004, LENGTH = 8K - 4
  BUFFER (xw)	: ORIGIN = 0xFFFFA000, LENGTH = 12K
}

REGION_ALIAS("TGT", RAM);

/* Highest address of the user mode stack: below the top-of-RAM dump area of die_trace() */
_stackinit = 0xFFFFDEFC;

/* Define output sections */
SECTIONS
{
	/* program code and other data */
	.text :
	{
		. = ALIGN(4);
		*(.rja)	/* target of the RAMjump. Execution starts here */
		*(.text)           /* .text sections (code) */
		*(.text*)          /* .text* sections (code) */

		. = ALIGN(4);
		_etext = .;        /* define a global symbols at end of code */
	} >TGT

	/* Constant data  */
	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)         /* .rodata sections (constants, strings, etc.) */
		*(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
		. = ALIGN(4);
	} >TGT


	/* Initialized data sections */
	.data :
	{
		. = ALIGN(4);
		_sdata = .;        /* create a global symbol at data start */
		*(.data)           /* .data sections */
		*(.data*)          /* .data* sections */

		. = ALIGN(4);
		_edata = .;        /* define a global symbol at data end */
		_idatalen = . - _sdata;
	} >TGT

ASSERT(_sdata == _edata, "unhandled initialized data !")
/* the whole image is what the bootloader uploads: it has to stay inside the 8KB window (the MEMORY length enforces it) */
ASSERT(. <= 0xFFFFB000, "kernel image runs past the TCU bootloader window 0xFFFF9000-0xFFFFAFFF")

	/* Uninitialized data section */
	. = ALIGN(4);
	.bss :
	{
		_sbss = .;         /* define a global symbol at bss start */
		*(.bss)
		*(.bss*)
		*(COMMON)

		. = ALIGN(4);
		_ebss = .;         /* define a global symbol at bss end */
		_bsslen = . - _sbss;
		_endpayload = .;
  } >BUFFER


	/* Remove information from the standard libraries */
	/DISCARD/ :
	{
	*(.comment)
	libc.a ( * )
	libm.a ( * )
	libgcc.a ( * )
	}

}
