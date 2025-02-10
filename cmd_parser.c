/* stuff for receiving commands etc */

/* (c) copyright fenugrec 2016
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

#include "stypes.h"

#include <string.h>	//memcpy

#include "npk_ver.h"
#include "platf.h"

#include "eep_funcs.h"
#include "iso_cmds.h"
#include "npk_errcodes.h"
#include "crc.h"
#include "cmd_parser.h"

#define MAX_INTERBYTE	1	//ms between bytes that causes a disconnect

//#define KLINE
//#define CAN
//#define CAN_TP

/* concatenate the ReadECUID positive response byte
 * in front of the version string
 */
#if defined(SH7059D_EURO5)
	#if defined(CAN)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7059 CAN EURO5 Diesel CAN Kernel v1.00";
	#elif defined(CAN_TP)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7059 CAN EURO5 Diesel iso15765 Kernel v1.00";
	#endif
#elif defined(SH7058D_EURO4)
    static const u8 kernel_id_string[] = "FastECU Subaru SH7058 CAN EURO4 Diesel Kernel v1.00";
#elif defined(SH7058)
	#if defined(KLINE)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7058 K-Line Kernel v1.00";
	#elif defined(CAN)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7058 CAN Kernel v1.00";
	#elif defined(CAN_TP)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7058 iso15765 Kernel v1.00";
	#endif
#elif defined(SH7055)
	#if defined(KLINE)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7055 K-Line Kernel v1.00";
	#elif defined(CAN)
        static const u8 kernel_id_string[] = "FastECU Subaru SH7055 CAN Kernel v1.00";
	#endif
#endif


/* generic buffer to construct responses. Saves a lot of stack vs
 * each function declaring its buffer as a local var : gcc tends to inline everything
 * but not combine /overlap each buffer.
 * We just need to make sure the comms functions (iso_sendpkt, tx_7F etc) use their own
 * private buffers. */
static u8 txbuf[256];

typedef enum {
	EEPSCI3_PJ2 = 2,
	EEPSCI3_PJ3 = 3,
	EEPSCI4_PF10 = 4,
} EepromPort;

int EEPROM_SCI = 0;
int EEPROM_CS = 0;

//#define EEP_COMMS3
#define EEP_START_BIT     	0x04000000
#define EEP_READ          	0x02000000
#define EEP_WRITE_ENABLE  	0x00C00000
#define EEP_WRITE         	0x01000000
#define EEP_WRITE_DISABLE 	0x00000000

#define SCI3_CS_PJ2			0x0004 // PJ2
#define SCI3_CS_PJ3			0x0008 // PJ3
#define SCI4_CS				0x0400 // PF10

void __attribute__ ((noinline)) delay(int mult) {
	asm("mov #0x0,r5");
	asm("shll2 %0"::"r"(mult):"r5");
	asm("shll r4");
	asm("cmp/hs r4,r5");
	asm("bt 1f");
	asm("0:");
	asm("add #0x1,r5");
	asm("cmp/hs r4,r5");
	asm("bf 0b");
	asm("1:");
}

void set_sci3_cs(void) {
	if (EEPROM_SCI == EEPSCI3_PJ2)
		PJ.DR.WORD |= SCI3_CS_PJ2;      // PJ2 (gen I/O) on - looks like PJ2 is connected to EEPROM CS
	if (EEPROM_SCI == EEPSCI3_PJ3)
		PJ.DR.WORD |= SCI3_CS_PJ3;      // PJ3 (gen I/O) on - looks like PJ3 is connected to EEPROM CS
}

void clear_sci3_cs(void) {
	if (EEPROM_SCI == EEPSCI3_PJ2)
		PJ.DR.WORD &= ~SCI3_CS_PJ2;     // PJ3 (gen I/O) off - CS off
	if (EEPROM_SCI == EEPSCI3_PJ3)
		PJ.DR.WORD &= ~SCI3_CS_PJ3;     // PJ3 (gen I/O) off - CS off
}

void set_sci4_cs(void) {
		PF.DR.WORD |= SCI4_CS;			// PF10 (gen I/O) off  (EEPROM CS off)
}

void clear_sci4_cs(void) {
		PF.DR.WORD &= ~SCI4_CS;         // PF10 (gen I/O) off  (EEPROM CS off)
}

bool __attribute__ ((noinline)) EEPROM_check(void) {

    bool pbpin = false;

	if (EEPROM_SCI == EEPSCI4_PF10) {
		//Toggle PF10 & short delay
		set_sci4_cs();                              // PF10 (gen I/O) on - looks like PF10 is connected to EEPROM CS
		delay(2);
		pbpin = ((PB.DR.WORD & 0x0800) == 0);		// pbpin = 0 if PB11(RxD4) == 1 - maybe the 'dummy bit'
		clear_sci4_cs();                            // PF10 (gen I/O) off - CS off
	}
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3) {
		//Toggle PJ2 & short delay
		set_sci3_cs();
		delay(2);
		pbpin = ((PB.DR.WORD & 0x0200) == 0);       // pbpin = 0 if PB9(RxD3) == 1 - maybe the 'dummy bit'
		clear_sci3_cs();							// PJ2/3 (gen I/O) off - CS off
	}

	return pbpin;



}

void __attribute__ ((noinline)) EEPROM_init(void) {

	if (EEPROM_SCI == EEPSCI4_PF10) {
		// prepare for read
		PFC.PLIR.WORD &= ~0x0200;                          // PL9 (SCK4) not inverted
		PFC.PBCRH.WORD &= ~0x0030;                          // turn off PB10MD0 and PB10MD1 - set PB10 to general I/O
		PB.DR.WORD &= ~0x0400;                             // turn off PB10 (general I/O)
		set_sci4_cs();                              // turn on  PF10 (general I/O) - bring EEPROM CS high
		PFC.PBCRH.WORD = (PFC.PBCRH.WORD & ~0x0030) | 0x0010; // turn off PB10MD0 and PB10MD1 - set PB10 to general I/O THEN set PB10 to TxD4
	}
    else if (EEPROM_SCI == EEPSCI3_PJ2) {
		// prepare for read
		PFC.PLIR.WORD &= ~0x0100;                          // PL8 (SCK3) not inverted
		PFC.PBCRH.WORD &= ~0x0003;                          // turn off PB8MD0 and PB8MD1 - set PB8 to general I/O
		PB.DR.WORD &= ~0x0100;                             // turn off PB8 (general I/O)
		set_sci3_cs();
		PFC.PBCRH.WORD = (PFC.PBCRH.WORD & ~0x0003) | 0x0001; // turn off PB8MD0 and PB8MD1 - set PB8 to general I/O THEN set PB8 to TxD3
	}
    else if (EEPROM_SCI == EEPSCI3_PJ3) { // FOR TESTING
        // prepare for read
        PFC.PLIR.WORD &= ~0x0100;                          // PL8 (SCK3) not inverted
        PFC.PBCRH.WORD &= ~0x0003;                          // turn off PB8MD0 and PB8MD1 - set PB8 to general I/O
        PB.DR.WORD &= ~0x0100;                             // turn off PB8 (general I/O)
        set_sci3_cs();
        PFC.PBCRH.WORD = (PFC.PBCRH.WORD & ~0x0003) | 0x0001; // turn off PB8MD0 and PB8MD1 - set PB8 to general I/O THEN set PB8 to TxD3
    }

	return;

}

void __attribute__ ((noinline)) EEPROM_send_cmd(uint32_t cmd, uint32_t len) {

	uint8_t i, eeprom_cmd[4];

	eeprom_cmd[0] = (uint8_t) ((cmd >> 24) & 0xFF);
	eeprom_cmd[1] = (uint8_t) ((cmd >> 16) & 0xFF);
	eeprom_cmd[2] = (uint8_t) ((cmd >> 8) & 0xFF);
	eeprom_cmd[3] = (uint8_t) (cmd & 0xFF);

	if (EEPROM_SCI == EEPSCI4_PF10) {
		// TX len bytes - send command, 2 bytes for read, 4 bytes for write
		SCI4.SCR.BYTE = (SCI4.SCR.BYTE & 0x0B) | 0x20;     // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output THEN TX enabled
		for (i = 0; i < len; i++) {
			while ((SCI4.SSR.BYTE & 0x80) == 0) {  };      // wait until TDR does not contain valid TX data (ie) it has gone to TSR
			SCI4.TDR = eeprom_cmd[i];                      // set TDR
			SCI4.SSR.BYTE = (SCI4.SSR.BYTE & 0x7F) | 0x78; // clear TDRE and ?? CPU cannot write 1 to the status flags ??
		}
		while ((SCI4.SSR.BYTE & 0x04) == 0) {  };          // wait until transmission ended
		SCI4.SCR.BYTE &= 0x0B;                             // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output
	}
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3) {
		// TX len bytes - send command, 2 bytes for read, 4 bytes for write
		SCI3.SCR.BYTE = (SCI3.SCR.BYTE & 0x0B) | 0x20;     // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output THEN TX enabled
		for (i = 0; i < len; i++) {
			while ((SCI3.SSR.BYTE & 0x80) == 0) {  };      // wait until TDR does not contain valid TX data (ie) it has gone to TSR
			SCI3.TDR = eeprom_cmd[i];                      // set TDR
			SCI3.SSR.BYTE = (SCI3.SSR.BYTE & 0x7F) | 0x78; // clear TDRE and ?? CPU cannot write 1 to the status flags ??
		}
		while ((SCI3.SSR.BYTE & 0x04) == 0) {  };          // wait until transmission ended
		SCI3.SCR.BYTE &= 0x0B;                             // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output
	}

	return;

}

uint16_t __attribute__ ((noinline)) EEPROM_get_rsp(void) {

	uint8_t i, eeprom_rsp[2];

	if (EEPROM_SCI == EEPSCI4_PF10) {
		// RX 2 bytes
		PFC.PLIR.WORD |= 0x0200;                           // PL9 (SCK4) inverted
		SCI4.SCR.BYTE = (SCI4.SCR.BYTE & 0x0B) | 0x30;     // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output THEN TX, RX enabled
		for (i = 0; i < 2; i++) {
			SCI4.TDR = 0x00;                               // set TDR
			SCI4.SSR.BYTE = (SCI4.SSR.BYTE & 0x7F) | 0x78; // clear TDRE and ?? CPU cannot write 1 to the status flags ??
			while ((SCI4.SSR.BYTE & 0x40) == 0) {  };      // wait until RDR contains valid received data
			eeprom_rsp[i] = SCI4.RDR;                      // store RDR data
			SCI4.SSR.BYTE = (SCI4.SSR.BYTE & 0xBF) | 0xB8; // clear RDRF and ?? CPU cannot write 1 to the status flags ??
		}
		SCI4.SCR.BYTE &= 0x0B;                             // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output
		PFC.PLIR.WORD &= ~0x0200;                          // PL9 (SCK4) not inverted
	}
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3) {
		// RX 2 bytes
		PFC.PLIR.WORD |= 0x0100;                           // PL8 (SCK3) inverted
		SCI3.SCR.BYTE = (SCI3.SCR.BYTE & 0x0B) | 0x30;     // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output THEN TX, RX enabled
		for (i = 0; i < 2; i++) {
			SCI3.TDR = 0x00;                               // set TDR
			SCI3.SSR.BYTE = (SCI3.SSR.BYTE & 0x7F) | 0x78; // clear TDRE and ?? CPU cannot write 1 to the status flags ??
			while ((SCI3.SSR.BYTE & 0x40) == 0) {  };      // wait until RDR contains valid received data
			eeprom_rsp[i] = SCI3.RDR;                      // store RDR data
			SCI3.SSR.BYTE = (SCI3.SSR.BYTE & 0xBF) | 0xB8; // clear RDRF and ?? CPU cannot write 1 to the status flags ??
		}
		SCI3.SCR.BYTE &= 0x0B;                             // TXI, RXI, TX, RX, TEI disabled, internal clock/SCI pin used for sync clock output
		PFC.PLIR.WORD &= ~0x0100;                          // PL8 (SCK3) not inverted
	}

	return (uint16_t) ((eeprom_rsp[0] << 8) | eeprom_rsp[1]);

}

uint16_t __attribute__ ((noinline)) EEPROM_TX_2bytes_RX_2bytes(uint32_t cmd) {

	uint16_t rsp;

	EEPROM_init();
	EEPROM_send_cmd(cmd, 2);
	rsp = EEPROM_get_rsp();

	if (EEPROM_SCI == EEPSCI4_PF10)
		clear_sci4_cs();
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3)
		clear_sci3_cs();

	return (uint16_t) rsp;

}

void __attribute__ ((noinline)) EEPROM_TX_bytes(uint32_t cmd, uint8_t len) {

	EEPROM_init();
	EEPROM_send_cmd(cmd, len);

	if (EEPROM_SCI == EEPSCI4_PF10)
		clear_sci4_cs();
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3)
		clear_sci3_cs();

	return;

}

static void __attribute__ ((noinline)) eep_read16_sub(uint8_t addr, uint16_t *dest)
{
    uint8_t not_ready_counter;
    uint32_t cmd;

    not_ready_counter = 0;
    *dest = 0xFFFF;

    while (not_ready_counter < 0xFE) {

        if (EEPROM_check()) {
            not_ready_counter++;
        }
        else {
            cmd = (uint32_t) (EEP_START_BIT | EEP_READ | (addr << 16));
            *dest = EEPROM_TX_2bytes_RX_2bytes(cmd);
            return;
        }
    }

    txbuf[0] = 0x7f;
    txbuf[1] = SID_DUMP;
    txbuf[2] = ISO_NRC_GR;
    send_message(txbuf, 3);
    return;
}

static uint32_t __attribute__ ((noinline)) eep_write16_sub(uint8_t addr, uint8_t *data, uint8_t len) {

    uint8_t ecur, not_ready_counter;
    uint32_t cmd;

    not_ready_counter = 0;
    addr /= 2;	// modify address to fit with eeprom 256*16bit org
    len &= ~1;	// align to 16bits

    while (not_ready_counter < 0xFE) {

        if(EEPROM_check()) {
            not_ready_counter++;
        }
        else {
            for (ecur = 0; ecur < (len / 2); ecur++) {
                cmd = (uint32_t) (EEP_START_BIT | EEP_WRITE_ENABLE);   // perhaps this could be outside the loop
                EEPROM_TX_bytes(cmd, 2);

                cmd = (uint32_t) (EEP_START_BIT | EEP_WRITE);
                cmd	|= (uint32_t) (addr << 16);
                cmd |= (uint32_t) (data[ecur * 2] << 8);
                cmd |= (uint32_t) (data[(ecur * 2) + 1]);
                EEPROM_TX_bytes(cmd, 4);

                delay(7000);  // allow time for write (5ms)
                addr++;
            }

            cmd = (uint32_t) (EEP_START_BIT | EEP_WRITE_DISABLE);
            EEPROM_TX_bytes(cmd, 2);

            return 0;

        }

    }

    return 1;

}

#define datablocksize   0x410
static u8 databuffer[datablocksize];
//#ifdef KLINE
//static u8 sendbuffer[datablocksize];
//#endif
static u32 flashaddr = 0;
static u32 flashbufferindex = 0;

void send_message(u8 *buf, u32 msglen)
{
#ifdef KLINE
    kline_send_message(buf, msglen);
#endif
#ifndef KLINE
    can_send_message(buf, msglen);
#endif
}

static void cmd_max_msg_size(u8 *msg)
{
    u16 datalen = (msg[2] << 8) | msg[3];

    if (datalen != 1)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    databuffer[0] = msg[4] | 0x40;
    databuffer[1] = (flashmessagesize >> 24) & 0xff;
    databuffer[2] = (flashmessagesize >> 16) & 0xff;
    databuffer[3] = (flashmessagesize >> 8) & 0xff;
    databuffer[4] = flashmessagesize & 0xff;
    send_message(databuffer, 5);

    return;
}

static void cmd_max_blk_size(u8 *msg)
{
    u16 datalen = (msg[2] << 8) | msg[3];

    if (datalen != 1)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    databuffer[0] = msg[4] | 0x40;
    databuffer[1] = (flashbuffersize >> 24) & 0xff;
    databuffer[2] = (flashbuffersize >> 16) & 0xff;
    databuffer[3] = (flashbuffersize >> 8) & 0xff;
    databuffer[4] = flashbuffersize & 0xff;
    send_message(databuffer, 5);

    return;
}

static void cmd_get_programming_voltage(u8 *msg)
{
    u16 datalen = (msg[2] << 8) | msg[3];
    u16 prog_voltage = 560;

    if (datalen != 1)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    databuffer[0] = msg[4] | 0x40;
    databuffer[1] = (prog_voltage >> 8) & 0xff;
    databuffer[2] = prog_voltage & 0xff;
    databuffer[3] = (prog_voltage >> 8) & 0xff;
    databuffer[4] = prog_voltage & 0xff;
    send_message(databuffer, 5);

    return;
}

/* flash initialisation, 0xE0 command, called from cmd_loop
 *
 *
 *
 *
 */
static void cmd_flash_init(u8 *msg)
{
    u8 errval;
    u16 datalen = (msg[2] << 8) | msg[3];

    if (datalen != 1)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    //if (msg[4] == SUB_KERNEL_FLASH_DISABLE)
    //    PFC.PDIOR.WORD &= ~0x0100;
    //else if (msg[4] == SUB_KERNEL_FLASH_ENABLE)
    PFC.PDIOR.WORD |= 0x0100;

    if (!platf_flash_init(&errval))
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = errval;
        send_message(databuffer, 3);
        return;
    }

    if (msg[4] == SUB_KERNEL_FLASH_DISABLE)
        platf_flash_protect();
    else if (msg[4] == SUB_KERNEL_FLASH_ENABLE)
        platf_flash_unprotect();

    databuffer[0] = msg[4] | 0x40;
    send_message(databuffer, 1);

    return;
}



/*
 * Erase flash block, Page blank command, called from cmd_loop
 */
static void cmd_erase_block(u8 *msg)
{
    u32 rv = 0;
    u16 datalen = (msg[2] << 8) | msg[3];

    flashbufferindex = 0;

    if (datalen != 5)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    flashaddr = ((msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | msg[8]);

    rv = platf_flash_eb(flashaddr);

    if(rv)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = rv;
        send_message(databuffer, 3);
        return;
    }

    databuffer[0] = (msg[4] | 0x40);
    send_message(databuffer, 1);

    return;
}

/*
 * Load 0x1000 bytes for flashing, write flash buffer command, called from cmd_loop
 */

static void cmd_write_flash_buffer(u8 *msg)
{
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != (5+flashmessagesize))
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    memcpy(&flashbuffer[flashbufferindex], &databuffer[9], flashmessagesize);
    flashbufferindex += flashmessagesize;
    if (flashbufferindex >= flashbuffersize)
        flashbufferindex = 0;

    databuffer[0] = (msg[4] | 0x40);
    send_message(databuffer, 1);

    return;
}

/*
 * Validate flash buffer, validate flash buffer command, called from cmd_loop
 */
static void cmd_validate_flash_buffer(u8 *msg)
{
    //u32 addr;
    u32 len;
    u32 img_crc32;
    u32 rom_crc32;
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != 11)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x31;  // crc32 error
        send_message(databuffer, 3);
        return;
    }

    //addr = (msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | msg[8];
    len = (msg[9] << 8) | msg[10];
    img_crc32 = (msg[11] << 24) | (msg[12] << 16) | (msg[13] << 8) | msg[14];

    rom_crc32 = buffer_crc32(flashbuffer, len);

    if (rom_crc32 == img_crc32)
    {
        databuffer[0] = (msg[4] | 0x40);
        send_message(databuffer, 1);
        return;
    }

    databuffer[0] = 0x7F;
    databuffer[1] = msg[4];
    databuffer[2] = (rom_crc32 >> 24) & 0xff;
    databuffer[3] = (rom_crc32 >> 16) & 0xff;
    databuffer[4] = (rom_crc32 >> 8) & 0xff;
    databuffer[5] = rom_crc32 & 0xff;
    send_message(databuffer, 6);

    return;
}
/* Write flash block, commit flash buffer command, called from cmd_loop
 *
 *
 *
 *
 */
static void cmd_commit_flash_buffer(u8 *msg)
{
    //u32 i;
    u32 rv;
    u32 addr;
    u32 len;
    u32 img_crc32;
    u32 rom_crc32;
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != 11)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x31;  // crc32 error
        send_message(databuffer, 3);
        return;
    }

    addr = (msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | msg[8];
    len = (msg[9] << 8) | msg[10];
    img_crc32 = (msg[11] << 24) | (msg[12] << 16) | (msg[13] << 8) | msg[14];

    rv = platf_flash_wb(addr, (u32) flashbuffer, flashbuffersize);

    if(rv) {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = rv;
        send_message(databuffer, 3);
        return;
    }

    rom_crc32 = crc32((const u8 *)addr, len);

    if (rom_crc32 == img_crc32)
    {
        databuffer[0] = (msg[4] | 0x40);
        send_message(databuffer, 1);
        return;
    }

    databuffer[0] = 0x7F;
    databuffer[1] = msg[4];
    databuffer[2] = (rom_crc32 >> 24) & 0xff;
    databuffer[3] = (rom_crc32 >> 16) & 0xff;
    databuffer[4] = (rom_crc32 >> 8) & 0xff;
    databuffer[5] = rom_crc32 & 0xff;
    send_message(databuffer, 6);

    return;
}

/* checksum command processor, 0xD0 command, called from cmd_loop.
 * args[0,1] : 0x7A and SID
 * args[2,3,4] : # start address
 * args[5,6,7] : # block size
 *
 *
 *
 */
static void cmd_crc(u8 *msg)
{
    u32 addr;
    u32 len;
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != 9)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    addr = (msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | msg[8];
    len = (msg[9] << 24) | (msg[10] << 16) | (msg[11] << 8) | msg[12];

    uint32_t crc;
    crc = crc32((const u8 *)addr, len);

    databuffer[0] = (msg[4] | 0x40);
    databuffer[1] = ((crc >> 24) & 0xff);
    databuffer[2] = ((crc >> 16) & 0xff);
    databuffer[3] = ((crc >> 8) & 0xff);
    databuffer[4] = (crc & 0xff);
    send_message(databuffer, 5);
    return;
}


/* dump eeprom command processor, 0xD8 command, called from cmd_loop.
 * args[0,1] : 0x7A and SID
 * ex.: "7A D6 00 10 00 00 00 00" dumps 1MB of ROM@ 0x0
 *
 */
static void cmd_read_eeprom(u8 *msg) {
    u32 addr;
    u32 len;
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != 7)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    addr = ((msg[6] << 16) | (msg[7] << 8) | msg[8]);
    len = ((msg[9] << 8) | msg[10]);
    EEPROM_SCI = msg[5];

    len &= ~1;	/* align to 16bits */

    databuffer[0] = (msg[4] | 0x40);
    send_message(databuffer, 1);
    //can_idle(750);

    addr /= 2;	/* modify address to fit with eeprom 256*16bit org */
    len &= ~1;	/* align to 16bits */

    uint16_t pbuf[len];
    uint8_t *pstart;	//start of ISO packet
    uint16_t *ebuf = &pbuf[0];	//cheat : form an ISO packet with the pos resp code in pbuf[0]

    int pktlen;
    int ecur;

    pstart = (uint8_t *)(pbuf);

    pktlen = len;

    for (ecur = 0; ecur < (pktlen / 2); ecur += 1) {
        eep_read16_sub((uint8_t) addr + ecur, (uint16_t *)&ebuf[ecur]);
    }

    memcpy(databuffer, pstart, pktlen);
    send_message(databuffer, pktlen);

    return;
}

/*
 * Read ROM area command processor, 0xD8 command, called from cmd_loop.
 * args[0]          : 0x03
 * args[1,2,3,4]    : address
 * args[5,6]        : read area size
 */
static void cmd_read_area(u8 *msg)
{
    u32 addr;
    u32 len;
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != 7) {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    addr = (msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | (msg[8]);
    len = (msg[9] << 8) | (msg[10]);

    databuffer[0] = (msg[4] | 0x40);
    memcpy(&databuffer[1], (void *) addr, len);
    send_message(databuffer, len+1);

    return;
}

/*
 * Kernel ID command processor, 0x01 command, called from cmd_loop.
 * args[0] : 0x01
 */
static void cmd_request_ecuid(u8 *msg)
{
    u16 datalen = (msg[2] << 8) | msg[3];

    if(datalen != 1)
    {
        databuffer[0] = 0x7F;
        databuffer[1] = msg[4];
        databuffer[2] = 0x30;  // general format error
        send_message(databuffer, 3);
        return;
    }

    u8 len = sizeof(kernel_id_string);

    databuffer[0] = (msg[4] | 0x40);
    memcpy(&databuffer[1], kernel_id_string, len);
    send_message(databuffer, len+1);

    return;
}


#if defined(KLINE)

/***********************************************
*
*	K-Line kernel functions
*
*
*
*
*
*
*
***********************************************/

static u16 bufferindex = 0;
static u16 msgindex = 0;
static u16 msglength = 0;
//static u32 flashaddr = 0;
//static u32 flashbufferindex = 0;

/** simple 8-bit sum */
static uint8_t cks_u8(const uint8_t * data, unsigned int len)
{
	uint8_t rv=0;

	while (len > 0) {
		len--;
		rv += data[len];
	}
	return rv;
}

/** send a whole buffer, blocking. */
static void sci_txblock(const uint8_t *buf, uint32_t len)
{
    for (; len > 0; len--) {
        while (!NPK_SCI.SSR.BIT.TDRE) {}	//wait for empty
        NPK_SCI.TDR = *buf;
        buf++;
        NPK_SCI.SSR.BIT.TDRE = 0;		//start tx
    }
}

/** Send a headerless iso9141 packet
 * @param len is clipped to 0xff
 *
 * disables RX during sending to remove halfdup echo. Should be reliable since
 * we re-enable after the stop bit, so K should definitely be back up to '1' again
 *
 * this is blocking
 */
void kline_send_message(u8 *buf, u32 msglen)
{
    if (msglen <= 0)
        return;

    NPK_SCI.SCR.BIT.RE = 0;

    uint32_t len = msglen + 5;

    txbuf[0] = (SUB_KERNEL_START_COMM >> 8) & 0xff;
    txbuf[1] = SUB_KERNEL_START_COMM & 0xff;
    txbuf[2] = (msglen >> 8) & 0xff;
    txbuf[3] = msglen & 0xff;
    //for (u32 i = 0; i < msglen; i++)
    //    txbuf[4+i] = buf[i];
    memcpy(&txbuf[4], buf, msglen);
    txbuf[4+msglen] = cks_u8(txbuf, len-1);

    sci_txblock(txbuf, len);

    //ugly : wait for transmission end; this means re-enabling RX won't pick up a partial byte
    while (!NPK_SCI.SSR.BIT.TEND) {}

    NPK_SCI.SCR.BIT.RE = 1;
    return;
}

enum iso_prc { ISO_PRC_ERROR, ISO_PRC_NEEDMORE, ISO_PRC_DONE };
/** Add newly-received byte to msg;
 *
 * @return ISO_PRC_ERROR if bad header, bad checksum, or overrun (caller's fault)
 *	ISO_PRC_NEEDMORE if ok but msg not complete
 *	ISO_PRC_DONE when msg complete + good checksum
 *
 * Note : the *msg->hi, ->di, ->hdrlen, ->datalen memberes must be set to 0 before parsing a new message
 */

enum iso_prc kline_get_message(u8 *msg, u8 newbyte)
{
    if (bufferindex == 0)
    {
        if (newbyte != 0xbe)
            return ISO_PRC_ERROR;
        msg[0] = newbyte;
    }
    else if (bufferindex == 1)
    {
        if (newbyte != 0xef)
            return ISO_PRC_ERROR;
        msg[1] = newbyte;
    }
    else if (bufferindex == 2)
        msg[2] = newbyte;
    else if (bufferindex == 3)
    {
        msg[3] = newbyte;
        msglength = ((msg[2] << 8) & 0xff) + (msg[3] & 0xff);
        msgindex = 0;
    }
    else if (bufferindex == 4)
        msg[4] = newbyte;

    if (bufferindex > 4 && msgindex <= msglength)
    {
        msgindex++;
        msg[bufferindex] = newbyte;

        if (msgindex >= msglength)
        {
            //return ISO_PRC_DONE;
            u8 cks = cks_u8(msg, bufferindex);
            if (cks == msg[bufferindex])
            {
                msgindex = 0;
                bufferindex = 0;
                msglength = 0;
                return ISO_PRC_DONE;
            }
            return ISO_PRC_ERROR;
        }
    }
    bufferindex++;
    return ISO_PRC_NEEDMORE;
}

/* initialize command parser state machine;
 * updates SCI settings : 62500 bps
 * beware the FER error flag, it disables further RX. So when changing BRR, if the host sends a byte
 * FER will be set, etc.
 */
void cmd_init(u8 brrdiv)
{
    NPK_SCI.SCR.BYTE &= 0xCF;	// disable TX + RX
    NPK_SCI.BRR = brrdiv;		// speed = (div + 1) * 625k
    NPK_SCI.SSR.BYTE &= 0x87;	// clear RDRF + error flags
    NPK_SCI.SCR.BYTE |= 0x30;	// enable TX+RX , no RX interrupts for now
    return;
}

/* command parser; infinite loop waiting for commands.
 * not sure if it's worth the trouble to make this async,
 * what other tasks could run in background ? reflash shit ?
 *
 * This receives valid iso9141 packets; message splitting is by pkt length
 */
void cmd_loop(void)
{
    u16 startcode = 0;
    u8 cmd = 0;
    u8 rxbyte = 0;

    bufferindex = 0;
    msgindex = 0;
    msglength = 0;

    while (1)
    {
		enum iso_prc prv;

		/* in case of errors (ORER | FER | PER), reset state mach. */
        if (NPK_SCI.SSR.BYTE & 0x38)
        {
            memset(databuffer, 0, datablocksize);
            memset(txbuf, 0, 256);
            //sci_rxidle(MAX_INTERBYTE);
			continue;
		}

        if (!NPK_SCI.SSR.BIT.RDRF)
            continue;

		rxbyte = NPK_SCI.RDR;
		NPK_SCI.SSR.BIT.RDRF = 0;

		/* got a byte; parse according to state */
        prv = kline_get_message(databuffer, rxbyte);

        if (prv == ISO_PRC_NEEDMORE)
            continue;

        if (prv != ISO_PRC_DONE) {
            memset(databuffer, 0, datablocksize);
            memset(txbuf, 0, 256);
            //sci_rxidle(MAX_INTERBYTE);
			continue;
		}

        delay(500);

        /* here, we have a complete iso frame */
        startcode = (databuffer[0] << 8) | (databuffer[1] & 0xff);
        if(startcode == SUB_KERNEL_START_COMM)
        {
            cmd = databuffer[4] & 0xFF;
            switch(cmd)
            {
                case SUB_KERNEL_ID: // 0x01 Request kernel ID
                    cmd_request_ecuid(databuffer);
                    break;

                case SUB_KERNEL_CRC: // 0x02 Request CRC32 checksum
                    cmd_crc(databuffer);
                    break;

                case SUB_KERNEL_READ_AREA: // 0x03 Read ROM data
                    cmd_read_area(databuffer);
                    break;

                case SUB_KERNEL_PROG_VOLT: // 0x04 Read programming voltage
                    cmd_get_programming_voltage(databuffer);
                    break;

                case SUB_KERNEL_GET_MAX_MSG_SIZE: // 0x05 Get max message size
                    cmd_max_msg_size(databuffer);
                    break;

                case SUB_KERNEL_GET_MAX_BLK_SIZE: // 0x06 Get max flashblock size
                    cmd_max_blk_size(databuffer);
                    break;

                case SUB_KERNEL_READ_EEPROM: // 0x07 Read EEPROM data
                    cmd_read_eeprom(databuffer);
                    break;

                case SUB_KERNEL_FLASH_ENABLE:
                    cmd_flash_init(databuffer); // 0x20
                    break;

                case SUB_KERNEL_FLASH_DISABLE:
                    cmd_flash_init(databuffer); // 0x21
                    break;

                case SUB_KERNEL_WRITE_FLASH_BUFFER: // 0x22 Write bytes to flashbuffer
                    cmd_write_flash_buffer(databuffer);
                    break;

                case SUB_KERNEL_VALIDATE_FLASH_BUFFER: // 0x23 Validate flashbuffer
                    cmd_validate_flash_buffer(databuffer);
                    break;

                case SUB_KERNEL_COMMIT_FLASH_BUFFER: // 0x24 Commit flashbuffer write
                    cmd_commit_flash_buffer(databuffer);
                    break;

                case SUB_KERNEL_BLANK_PAGE: // 0x25 Erase flash block
                    cmd_erase_block(databuffer);
                    //loadingblocks = true;
                    break;

                default:
                    databuffer[0] = 0x7F;
                    databuffer[1] = databuffer[4];
                    databuffer[2] = 0x34;   // Unrecognised argument
                    send_message(databuffer, 3);
                    break;
            }
        }
        else
        {
            if ((databuffer[4] == 0xFF) && (databuffer[5] == 0xC8))
            {
                databuffer[0] = 0xFF;
                databuffer[1] = 0xC8;
                send_message(databuffer, 2);
                die();
            }
            txbuf[0] = 0x7F;
            txbuf[1] = 0x35;    // Unrecognised command
            memcpy(&txbuf[2], databuffer, 16);
            send_message(txbuf, 18);
        }
        memset(databuffer, 0, datablocksize);
        memset(txbuf, 0, 256);
    }

	die();
}
#endif

/***********************************************
*
*	CAN / ISO15765 kernel functions
*
*
*
*
*
*
*
***********************************************/

#ifndef KLINE

static void can_idle(unsigned us) {
	u32 t0, tc, intv;

	intv = MCLK_GETTS(us) / 1000;	//# of ticks for delay

	t0 = get_mclk_ts();
	while (1) {
		tc = get_mclk_ts();
		if ((tc - t0) >= intv) return;

	}
}

/* receives 8 bytes from CAN Channel 0 mailbox 0 into msg
*
* returns 0 if no data to receive
* returns 1 if success
* returns -1 if no unread message available
*/
static int can_rx8bytes(u8 *msg)
{
    #if defined SH7058
        #if defined KLINE // For SH7058 K-Line models (pre MY07)
            // Will be added later
        #elif defined CAN
            if(!NPK_CAN.RXPR0.BIT.MB0) return 0;

            if(!NPK_CAN.UMSR0.BIT.MB0)
            {
                memcpy(msg, (void *) &NPK_CAN.MB[0].MSG_DATA[0], 8);
                NPK_CAN.RXPR0.WORD = 1;
                return 1;
            }
            NPK_CAN.UMSR0.WORD = 1;
            NPK_CAN.RXPR0.WORD = 1;
            return -1;
        #elif defined CAN_TP // For SH7058 iso15765 models (MY07+)
            if(!NPK_CAN.RXPR0.BIT.MB14) return 0;

            if(!NPK_CAN.UMSR0.BIT.MB14)
            {
                memcpy(msg, (void *) &NPK_CAN.MB[14].MSG_DATA[0], 8);
                NPK_CAN.RXPR0.WORD = 0x4000;
                return 1;
            }
            NPK_CAN.UMSR0.WORD = 0x4000;
            NPK_CAN.RXPR0.WORD = 0x4000;
            return -1;
        #endif
    #elif defined SH7058D_EURO4
        #if defined CAN
            if(!NPK_CAN.RXPR0.BIT.MB0) return 0;

            if(!NPK_CAN.UMSR0.BIT.MB0)
            {
                memcpy(msg, (void *) &NPK_CAN.MB[0].MSG_DATA[0], 8);
                NPK_CAN.RXPR0.WORD = 1;
                return 1;
            }
            NPK_CAN.UMSR0.WORD = 1;
            NPK_CAN.RXPR0.WORD = 1;
            return -1;
        #elif defined CAN_TP
            if(!NPK_CAN.RXPR1.BIT.MB31) return 0;

            if(!NPK_CAN.UMSR1.BIT.MB31)
            {
                memcpy(msg, (void *) &NPK_CAN.MB[31].MSG_DATA[0], 8);
                NPK_CAN.RXPR1.WORD = 0x8000;
                return 1;
            }
            NPK_CAN.UMSR1.WORD = 0x8000;
            NPK_CAN.RXPR1.WORD = 0x8000;
            return -1;
        #endif
    #elif defined SH7059D_EURO5
        #if defined CAN
            if(!NPK_CAN.RXPR0.BIT.MB0) return 0;

            if(!NPK_CAN.UMSR0.BIT.MB0)
            {
                memcpy(msg, (void *) &NPK_CAN.MB[0].MSG_DATA[0], 8);
                NPK_CAN.RXPR0.WORD = 1;
                return 1;
            }
            NPK_CAN.UMSR0.WORD = 1;
            NPK_CAN.RXPR0.WORD = 1;
            return -1;
        #elif defined CAN_TP
            if(!NPK_CAN.RXPR1.BIT.MB31) return 0;

            if(!NPK_CAN.UMSR1.BIT.MB31)
            {
                memcpy(msg, (void *) &NPK_CAN.MB[31].MSG_DATA[0], 8);
                NPK_CAN.RXPR1.WORD = 0x8000;
                return 1;
            }
            NPK_CAN.UMSR1.WORD = 0x8000;
            NPK_CAN.RXPR1.WORD = 0x8000;
            return -1;
        #endif
    #elif defined SH7055
        #if defined KLINE
            // Will be added later
        #elif defined CAN
            if(!NPK_CAN.RXPR.BIT.MB0) return 0;

            if(!NPK_CAN.UMSR.BIT.MB0)
            {
                memcpy(msg, (void *) &NPK_CAN.MD[0][0], 8);
                NPK_CAN.RXPR.WORD = 0x0100;
                return 1;
            }
            NPK_CAN.UMSR.WORD = 0x0100;
            NPK_CAN.RXPR.WORD = 0x0100;
            return -1;
        #endif
    #endif

    return -1;
}


/* transmits 8 bytes from buf via CAN Channel 0 mailbox 1
*
*
*
*
*/
static void can_tx8bytes(const u8 *buf)
{
    #ifdef SH7058
        // For SH7058 K-Line models (pre MY07)
        #if defined CAN
            while (NPK_CAN.TXPR0.BIT.MB1) { };

            NPK_CAN.TXACK0.WORD = 2;
            memcpy((void *) &NPK_CAN.MB[1].MSG_DATA[0], buf, 8);
            NPK_CAN.TXPR0.WORD = 2;

            return;
        #elif defined CAN_TP // For SH7058 iso15765 models (MY07+)
            while (NPK_CAN.TXPR0.BIT.MB15) { };

            NPK_CAN.TXACK0.WORD = 0x8000;
            memcpy((void *) &NPK_CAN.MB[15].MSG_DATA[0], buf, 8);
            NPK_CAN.TXPR0.WORD = 0x8000;

            return;
        #endif
    #elif defined SH7058D_EURO4
        #if defined CAN
            while (NPK_CAN.TXPR0.BIT.MB1) { };

            NPK_CAN.TXACK0.WORD = 2;
            memcpy((void *) &NPK_CAN.MB[1].MSG_DATA[0], buf, 8);
            NPK_CAN.TXPR0.WORD = 2;

            return;
        #elif defined CAN_TP
            while (NPK_CAN.TXPR1.BIT.MB30) { };

            NPK_CAN.TXACK1.WORD = 0x4000;
            memcpy((void *) &NPK_CAN.MB[30].MSG_DATA[0], buf, 8);
            NPK_CAN.TXPR1.WORD = 0x4000;

            return;
        #endif
    #elif defined SH7059D_EURO5
        #if defined CAN
            while (NPK_CAN.TXPR0.BIT.MB1) { };

            NPK_CAN.TXACK0.WORD = 2;
            memcpy((void *) &NPK_CAN.MB[1].MSG_DATA[0], buf, 8);
            NPK_CAN.TXPR0.WORD = 2;

            return;
        #elif defined CAN_TP
            while (NPK_CAN.TXPR1.BIT.MB30) { };

            NPK_CAN.TXACK1.WORD = 0x4000;
            memcpy((void *) &NPK_CAN.MB[30].MSG_DATA[0], buf, 8);
            NPK_CAN.TXPR1.WORD = 0x4000;

            return;
        #endif
    #elif defined SH7055
        #if defined CAN
            while (NPK_CAN.TXPR.BIT.MB1) { };

            NPK_CAN.TXACK.WORD = 0x0200;
            memcpy((void *) &NPK_CAN.MD[1][0], buf, 8);
            NPK_CAN.TXPR.WORD = 0x0200;

            return;
        #endif
    #endif
}

int can_send_message(u8 *msg, u32 msglen)
{
    u32 len = msglen+4;

    if (len < 8)
    {
        txbuf[0] = 0x00 | len;
        txbuf[1] = (SUB_KERNEL_START_COMM >> 8) & 0xFF;
        txbuf[2] = SUB_KERNEL_START_COMM & 0xFF;
        txbuf[3] = (msglen >> 8) & 0xFF;
        txbuf[4] = msglen & 0xFF;
        memcpy(&txbuf[5], msg, msglen);
        can_tx8bytes(txbuf);
        return 0;
    }
    else
    {
        bool first_frame = true;
        static u8 rx8bytes[8];
        u8 bytes_to_send = 0;
        u8 txindex = 2;
        u8 framesize = 8;
        u8 frame_index = 0;
        u8 frame_delay = 1;

        while (len)
        {
            if (!first_frame)
            {
                delay(1);
                int rv = can_rx8bytes(rx8bytes);
                if(rv == 1) // 0 = no data, 1 = data, -1 no unread message available
                {
                    if ((rx8bytes[0] & 0xf0) == 0x30)
                    {
                        memcpy(txbuf, rx8bytes, 8);
                        can_tx8bytes(txbuf);
                        frame_delay = rx8bytes[2];
                    }
                }
            }
            if (first_frame) {
                txbuf[0] = 0x10 | ((len >> 8) & 0x0f);
                txbuf[1] = (len & 0xff);
                txbuf[2] = (SUB_KERNEL_START_COMM >> 8) & 0xFF;
                txbuf[3] = SUB_KERNEL_START_COMM & 0xFF;
                txbuf[4] = (msglen >> 8) & 0xFF;
                txbuf[5] = msglen & 0xFF;
                txindex = 6;
            }
            else {
                txbuf[0] = 0x20 | frame_index;
                txindex = 1;
            }

            bytes_to_send = framesize - txindex;
            if (len < bytes_to_send)
                bytes_to_send = len;

            memcpy(&txbuf[txindex], msg, bytes_to_send);
            len -= bytes_to_send;
            msg += bytes_to_send;
            txindex = 1;

            first_frame = false;
            frame_index++;

            if (frame_index > 15)
                frame_index = 0;
            can_tx8bytes(txbuf);
            delay(frame_delay);
        }
        return 0;
    }
}

int can_get_message(u8 *msg)
{
    static u8 rx8bytes[8];

    int rv = can_rx8bytes(rx8bytes);
    if(rv == 0) // 0 = no data, 1 = data, -1 no unread message available
        return rv;

    if(rv == -1)
        return rv;

    int data_length = 0;
    u8 frame_index = 0;

    if ((rx8bytes[0] & 0xf0) == 0x00)
    {
        memcpy(msg, &rx8bytes[1], 7);
        //u16 msglength = rx8bytes[0] & 0x0f;
        data_length = rx8bytes[0] & 0x0f;
    }
    else if ((rx8bytes[0] & 0xf0) == 0x10)
    {
        memcpy(msg, &rx8bytes[2], 6);
        u32 msgindex = 6;
        u8 msglength = 0;
        //u16 msglength = ((rx8bytes[0] & 0x0f) << 8) + rx8bytes[1];
        data_length = ((rx8bytes[0] & 0x0f) << 8) + rx8bytes[1] - 6;
        txbuf[0] = 0x30;
        txbuf[1] = 0x00;
        txbuf[2] = 0x00;
        can_tx8bytes(txbuf);
        while (data_length > 0)
        {
            delay(1);
            rv = can_rx8bytes(rx8bytes);
            if(rv == 0) // 0 = no data, 1 = data, -1 no unread message available
                continue;

            if ((rx8bytes[0] & 0xf0) == 0x20)
            {
                msglength = 7;
                if (data_length < 7)
                    msglength = data_length;
                memcpy(&msg[msgindex], &rx8bytes[1], msglength);
                msgindex += msglength;
                data_length -= msglength;
            }
            if ((rx8bytes[0] & 0xf0) == 0x30)
            {
                memcpy(txbuf, rx8bytes, 8);
                can_tx8bytes(txbuf);
            }
        }
    }
    else if ((rx8bytes[0] & 0xf0) == 0x20)
    {
        //memcpy(candatabuffer, rx8bytes, 8);
        //can_tx8bytes(candatabuffer);
        return 0;
    }
    else if ((rx8bytes[0] & 0xf0) == 0x30)
    {
        memcpy(databuffer, rx8bytes, 8);
        can_tx8bytes(databuffer);
        return 0;
    }
    frame_index++;

    return rv;
}

#ifdef CAN
void set_mailbox(void)
{
	#ifdef SH7055
		// TX MB1, RX MB0
        u16 can_id = (((NPK_CAN.MC[0][5] << 8) & 0xff00) | (NPK_CAN.MC[0][4] & 0xff));
		can_id &= ~(0x7ff << 5);
        can_id &= 0xfff0;
        can_id |= (0x7e0 << 5);
		NPK_CAN.MC[0][7] = 0x00;
		NPK_CAN.MC[0][6] = 0x00;
		NPK_CAN.MC[0][5] = ((can_id >> 8) & 0xff);
		NPK_CAN.MC[0][4] = (can_id & 0xff);

	    can_id = (((NPK_CAN.MC[1][5] << 8) & 0xff00) | (NPK_CAN.MC[1][4] & 0xff));
		can_id &= ~(0x7ff << 5);
        can_id &= 0xfff0;
        can_id |= (0x7e8 << 5);
		NPK_CAN.MC[1][7] = 0x00;
		NPK_CAN.MC[1][6] = 0x00;
		NPK_CAN.MC[1][5] = ((can_id >> 8) & 0xff);
		NPK_CAN.MC[1][4] = (can_id & 0xff);
	#else
        u16 can_id = NPK_CAN.MB[0].CTRLH.WORD;
        can_id &= ~(0x7ff << 4);
        can_id &= 0xfff8;
        can_id |= (0x7e0 << 4);
        NPK_CAN.MB[0].CTRLH.WORD = can_id;
        NPK_CAN.MB[0].CTRLM.WORD = 0;

        can_id = NPK_CAN.MB[1].CTRLH.WORD;
        can_id &= ~(0x7ff << 4);
        can_id &= 0xfff8;
        can_id |= (0x7e8 << 4);
        NPK_CAN.MB[1].CTRLH.WORD = can_id;
        NPK_CAN.MB[1].CTRLM.WORD = 0;
    #endif
}
#endif

void can_cmd_loop(void)
{
    u16 startcode;
	u8 cmd;
    //static u8 candatabuffer[datablocklength];
    //bool loadingblocks = false;
    //counter8byteblock = 0;

    #if defined(CAN)
        set_mailbox();
    #endif

    flashbufferindex = 0;
    memset(databuffer, 0, datablocksize);
    memset(txbuf, 0, 256);

    while (1)
    {
        int rv = can_get_message(databuffer);
        if(rv == 0) // 0 = no data, 1 = data, -1 no unread message available
            continue;

        if(rv == -1)
        {
            databuffer[0] = 0x7F;
            databuffer[1] = 0x36;   // no unread message available error
            can_send_message(databuffer, 2);
            continue;
        }

        startcode = (databuffer[0] << 8) | (databuffer[1] & 0xff);
        if(startcode == SUB_KERNEL_START_COMM)
        {
            cmd = databuffer[4] & 0xFF;
            switch(cmd)
			{
                case SUB_KERNEL_ID: // 0x01 Request kernel ID
                    cmd_request_ecuid(databuffer);
					break;

                case SUB_KERNEL_CRC: // 0x02 Request CRC32 checksum
                    cmd_crc(databuffer);
					break;

                case SUB_KERNEL_READ_AREA: // 0x03 Read ROM data
                    cmd_read_area(databuffer);
					break;

                case SUB_KERNEL_PROG_VOLT: // 0x04 Read programming voltage
                    cmd_get_programming_voltage(databuffer);
                    break;

                case SUB_KERNEL_GET_MAX_MSG_SIZE: // 0x05 Get max message size
                    cmd_max_msg_size(databuffer);
                    break;

                case SUB_KERNEL_GET_MAX_BLK_SIZE: // 0x06 Get max flashblock size
                    cmd_max_blk_size(databuffer);
                    break;

                case SUB_KERNEL_READ_EEPROM: // 0x07 Read EEPROM data
                    cmd_read_eeprom(databuffer);
                    break;

                case SUB_KERNEL_FLASH_ENABLE:
                    cmd_flash_init(databuffer); // 0x20
                    break;

                case SUB_KERNEL_FLASH_DISABLE:
                    cmd_flash_init(databuffer); // 0x21
                    break;

                case SUB_KERNEL_WRITE_FLASH_BUFFER: // 0x22 Write bytes to flashbuffer
                    cmd_write_flash_buffer(databuffer);
					break;

                case SUB_KERNEL_VALIDATE_FLASH_BUFFER: // 0x23 Validate flashbuffer
                    cmd_validate_flash_buffer(databuffer);
                    break;

                case SUB_KERNEL_COMMIT_FLASH_BUFFER: // 0x24 Commit flashbuffer write
                    cmd_commit_flash_buffer(databuffer);
                    break;

                case SUB_KERNEL_BLANK_PAGE: //  // 0x25 Erase flash block
                    cmd_erase_block(databuffer);
                    //loadingblocks = true;
                    break;

                default:
                    databuffer[0] = 0x7F;
                    databuffer[1] = databuffer[4];
                    databuffer[2] = 0x34;   // Unrecognised argument
                    can_send_message(databuffer, 3);
                    break;
			}
		}
		else
		{
            if ((databuffer[4] == 0xFF) && (databuffer[5] == 0xC8))
			{
                databuffer[0] = 0xFF;
                databuffer[1] = 0xC8;
                can_send_message(databuffer, 2);
                die();
			}
            txbuf[0] = 0x7F;
            txbuf[1] = 0x35;    // Unrecognised command
            memcpy(&txbuf[2], databuffer, 16);
            can_send_message(txbuf, 18);
        }
        memset(databuffer, 0, datablocksize);
        memset(txbuf, 0, 256);
	}
	die();
	return;
}

#endif
