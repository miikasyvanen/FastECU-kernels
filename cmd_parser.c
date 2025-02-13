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

#define MAX_INTERBYTE	10	//ms between bytes that causes a disconnect

/* concatenate the ReadECUID positive response byte
 * in front of the version string
 */
#if defined(SH7058D_EURO4)
static const u8 npk_ver_string[] = "FastECU Subaru SH7058 CAN EURO4 Diesel Kernel v1.00";
#elif defined(SH7058)
static const u8 npk_ver_string[] = "FastECU Subaru SH7058 CAN Kernel v1.00";
#elif defined(SH7055)
static const u8 npk_ver_string[] = "FastECU Subaru SH7055 CAN Kernel v1.00";
#endif

typedef enum {
	EEPSCI3_PJ2 = 2,
	EEPSCI3_PJ3 = 3,
	EEPSCI4_PF10 = 4,
} EepromPort;

int EEPROM_SCI = 0;
int EEPROM_CS = 0;

/* low-level error code, to give more detail about errors than the SID 7F NRC can provide,
 * without requiring the error string list of nisprog to be updated.
 */
static u8 lasterr = 0;

/* make receiving slightly easier maybe */
struct iso14230_msg {
	int	hdrlen;		//expected header length : 1 (len-in-fmt), 2(fmt + len), 3(fmt+addr), 4(fmt+addr+len)
	int	datalen;	//expected data length
	int	hi;		//index in hdr[]
	int	di;		//index in data[]
	u8	hdr[4];
	u8	data[256];	//255 data bytes + checksum
};

/* generic buffer to construct responses. Saves a lot of stack vs
 * each function declaring its buffer as a local var : gcc tends to inline everything
 * but not combine /overlap each buffer.
 * We just need to make sure the comms functions (iso_sendpkt, tx_7F etc) use their own
 * private buffers. */
static u8 txbuf[256];


void set_lasterr(u8 err) {
	lasterr = err;
}

/** simple 8-bit sum */
static uint8_t cks_u8(const uint8_t * data, unsigned int len) {
	uint8_t rv=0;

	while (len > 0) {
		len--;
		rv += data[len];
	}
	return rv;
}

/* sign-extend 24bit number to 32bits,
 * i.e. FF8000 => FFFF8000 etc
 * data stored as big (sh) endian
 */
static u32 reconst_24(const u8 *data) {
	u32 tmp;
	tmp = (data[0] << 16) | (data[1] << 8) | data[2];
	if (data[0] & 0x80) {
		//sign-extend to cover RAM
		tmp |= 0xFF << 24;
	}
	return tmp;
}

/** discard RX data until idle for a given time
 * @param idle : purge until interbyte > idle ms
 *
 * blocking, of course. Do not call from ISR
 */
static void sci_rxidle(unsigned ms) {
	u32 t0, tc, intv;

	if (ms > MCLK_MAXSPAN) ms = MCLK_MAXSPAN;
	intv = MCLK_GETTS(ms);	//# of ticks for delay

	t0 = get_mclk_ts();
	while (1) {
		tc = get_mclk_ts();
		if ((tc - t0) >= intv) return;

		if (NPK_SCI.SSR.BYTE & 0x78) {
			/* RDRF | ORER | FER | PER :reset timer */
			t0 = get_mclk_ts();
			NPK_SCI.SSR.BYTE &= 0x87;	//clear RDRF + error flags
		}
	}
}

/** send a whole buffer, blocking. For use by iso_sendpkt() only */
static void sci_txblock(const uint8_t *buf, uint32_t len) {
	for (; len > 0; len--) {
		while (!NPK_SCI.SSR.BIT.TDRE) {}	//wait for empty
		NPK_SCI.TDR = *buf;
		buf++;
		NPK_SCI.SSR.BIT.TDRE = 0;		//start tx
	}
}

/** Send a headerless iso14230 packet
 * @param len is clipped to 0xff
 *
 * disables RX during sending to remove halfdup echo. Should be reliable since
 * we re-enable after the stop bit, so K should definitely be back up to '1' again
 *
 * this is blocking
 */
static void iso_sendpkt(const uint8_t *buf, int len) {
	u8 hdr[2];
	uint8_t cks;
	if (len <= 0) return;

	if (len > 0xff) len = 0xff;

	NPK_SCI.SCR.BIT.RE = 0;

	if (len <= 0x3F) {
		hdr[0] = (uint8_t) len;
		sci_txblock(hdr, 1);	//FMT/Len
	} else {
		hdr[0] = 0;
		hdr[1] = (uint8_t) len;
		sci_txblock(hdr, 2);	//Len
	}

	sci_txblock(buf, len);	//Payload

	cks = len;
	cks += cks_u8(buf, len);
	sci_txblock(&cks, 1);	//cks

	//ugly : wait for transmission end; this means re-enabling RX won't pick up a partial byte
	while (!NPK_SCI.SSR.BIT.TEND) {}

	NPK_SCI.SCR.BIT.RE = 1;
	return;
}



/* transmit negative response, 0x7F <SID> <NRC>
 * Blocking
 */
static void tx_7F(u8 sid, u8 nrc) {
	u8 buf[3];
	buf[0]=0x7F;
	buf[1]=sid;
	buf[2]=nrc;
	iso_sendpkt(buf, 3);
}


static void iso_clearmsg(struct iso14230_msg *msg) {
	msg->hdrlen = 0;
	msg->datalen = 0;
	msg->hi = 0;
	msg->di = 0;
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

static enum iso_prc iso_parserx(struct iso14230_msg *msg, u8 newbyte) {
	u8 dl;

	// 1) new msg ?
	if (msg->hi == 0) {
		msg->hdrlen = 1;	//at least 1 byte (FMT)

		//parse FMT byte
		if ((newbyte & 0xC0) == 0x40) {
			//CARB mode, not supported
			return ISO_PRC_ERROR;
		}
		if (newbyte & 0x80) {
			//addresses supplied
			msg->hdrlen += 2;
		}

		dl = newbyte & 0x3f;
		if (dl == 0) {
			/* Additional length byte present */
			msg->hdrlen += 1;
		} else {
			/* len-in-fmt : we can set length already */
			msg->datalen = dl;
		}
	}

	// 2) add to header if required
	if (msg->hi != msg->hdrlen) {
		msg->hdr[msg->hi] = newbyte;
		msg->hi += 1;
		// fetch LEN byte if applicable
		if ((msg->datalen == 0) && (msg->hi == msg->hdrlen)) {
			msg->datalen = newbyte;
		}
		return ISO_PRC_NEEDMORE;
	}

	// ) here, header is complete. Add to data
	msg->data[msg->di] = newbyte;
	msg->di += 1;

	// +1 because we need checksum byte too
	if (msg->di != (msg->datalen + 1)) {
		return ISO_PRC_NEEDMORE;
	}

	// ) data now complete. valide cks
	u8 cks = cks_u8(msg->hdr, msg->hdrlen);
	cks += cks_u8(msg->data, msg->datalen);
	if (cks == msg->data[msg->datalen]) {
		return ISO_PRC_DONE;
	}
	return ISO_PRC_ERROR;
}


/* Command state machine */
static enum t_cmdsm {
	CM_IDLE,		//not initted, only accepts the "startComm" request
	CM_READY,		//initted, accepts all commands

} cmstate;

/* flash state machine */
static enum t_flashsm {
	FL_IDLE,
	FL_READY,	//after doing init.
} flashstate;

/* initialize command parser state machine;
 * updates SCI settings : 62500 bps
 * beware the FER error flag, it disables further RX. So when changing BRR, if the host sends a byte
 * FER will be set, etc.
 */

void cmd_init(u8 brrdiv) {
	cmstate = CM_IDLE;
	flashstate = FL_IDLE;
	NPK_SCI.SCR.BYTE &= 0xCF;	//disable TX + RX
	NPK_SCI.BRR = brrdiv;		// speed = (div + 1) * 625k
	NPK_SCI.SSR.BYTE &= 0x87;	//clear RDRF + error flags
	NPK_SCI.SCR.BYTE |= 0x30;	//enable TX+RX , no RX interrupts for now
	return;
}

static void cmd_startcomm(void) {
	// KW : noaddr;  len-in-fmt or lenbyte
	static const u8 startcomm_resp[3] = {0xC1, 0x67, 0x8F};
	iso_sendpkt(startcomm_resp, 3);
	flashstate = FL_IDLE;
}

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
		PJ.DR.WORD |= SCI3_CS_PJ2;                                  // PJ2 (gen I/O) on - looks like PJ2 is connected to EEPROM CS
	if (EEPROM_SCI == EEPSCI3_PJ3)
		PJ.DR.WORD |= SCI3_CS_PJ3;                                  // PJ3 (gen I/O) on - looks like PJ3 is connected to EEPROM CS
}

void clear_sci3_cs(void) {
	if (EEPROM_SCI == EEPSCI3_PJ2)
		PJ.DR.WORD &= ~SCI3_CS_PJ2;                                 // PJ3 (gen I/O) off - CS off
	if (EEPROM_SCI == EEPSCI3_PJ3)
		PJ.DR.WORD &= ~SCI3_CS_PJ3;                                 // PJ3 (gen I/O) off - CS off
}

bool __attribute__ ((noinline)) EEPROM_check(void) {

	bool pbpin;

	if (EEPROM_SCI == EEPSCI4_PF10) {
		//Toggle PF10 & short delay
		PF.DR.WORD |= SCI4_CS;                                  // PF10 (gen I/O) on - looks like PF10 is connected to EEPROM CS
		delay(2);
		pbpin = ((PB.DR.WORD & 0x0800) == 0);                  // pbpin = 0 if PB11(RxD4) == 1 - maybe the 'dummy bit'
		PF.DR.WORD &= ~SCI4_CS;                                 // PF10 (gen I/O) off - CS off
	}
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3) {
		//Toggle PJ2 & short delay
		set_sci3_cs();
		delay(2);
		pbpin = ((PB.DR.WORD & 0x0200) == 0);                  // pbpin = 0 if PB9(RxD3) == 1 - maybe the 'dummy bit'
		clear_sci3_cs();
	}

	return pbpin;



}

void __attribute__ ((noinline)) EEPROM_init(void) {

	if (EEPROM_SCI == EEPSCI4_PF10) {
		// prepare for read
		PFC.PLIR.WORD &= ~0x0200;                          // PL9 (SCK4) not inverted
		PFC.PBCRH.WORD &= ~0x0030;                          // turn off PB10MD0 and PB10MD1 - set PB10 to general I/O
		PB.DR.WORD &= ~0x0400;                             // turn off PB10 (general I/O)
		PF.DR.WORD |= SCI4_CS;                              // turn on  PF10 (general I/O) - bring EEPROM CS high
		PFC.PBCRH.WORD = (PFC.PBCRH.WORD & ~0x0030) | 0x0010; // turn off PB10MD0 and PB10MD1 - set PB10 to general I/O THEN set PB10 to TxD4
	}
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3) {
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
		PF.DR.WORD &= ~SCI4_CS;                             // PF10 (gen I/O) off  (EEPROM CS off)
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3)
		clear_sci3_cs();

	return (uint16_t) rsp;

}

void __attribute__ ((noinline)) EEPROM_TX_bytes(uint32_t cmd, uint8_t len) {

	EEPROM_init();
	EEPROM_send_cmd(cmd, len);

	if (EEPROM_SCI == EEPSCI4_PF10)
		PF.DR.WORD &= ~SCI4_CS;                             // PF10 (gen I/O) off  (EEPROM CS off)
	else if (EEPROM_SCI == EEPSCI3_PJ2 || EEPROM_SCI == EEPSCI3_PJ3)
		clear_sci3_cs();

	return;

}

static void __attribute__ ((noinline)) eep_read16_sub(uint8_t addr, uint16_t *dest) {

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

	tx_7F(SID_DUMP, ISO_NRC_GR);
	return;

}

static void __attribute__ ((noinline)) eep_can_read16_sub(uint8_t addr, uint16_t *dest) {

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
	/*
	txbuf[0] = 0x7A;
	txbuf[1] = 0x7F;
	txbuf[2] = 0x21;
	can_tx8bytes(txbuf);
	can_idle(750);
*/
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

/* dump command processor, called from cmd_loop.
 * args[0] : address space (0: EEPROM, 1: ROM)
 * args[1,2] : # of 32-byte blocks
 * args[3,4] : (address / 32)
 *
 * EEPROM addresses are interpreted as the flattened memory, i.e. 93C66 set as 256 * 16bit will
 * actually be read as a 512 * 8bit array, so block #0 is bytes 0 to 31 == words 0 to 15.
 *
 * ex.: "00 00 02 00 01" dumps 64 bytes @ EEPROM 0x20 (== address 0x10 in 93C66)
 * ex.: "01 80 00 00 00" dumps 1MB of ROM@ 0x0
 *
 */
static void cmd_dump(struct iso14230_msg *msg) {
	u32 addr;
	u32 len;
	u8 space;
	u8 *args = &msg->data[1];	//skip SID byte

	if (msg->datalen != 6) {
		tx_7F(SID_DUMP, ISO_NRC_SFNS_IF);
		return;
	}

	space = args[0];
	len = 32 * ((args[1] << 8) | args[2]);
	addr = 32 * ((args[3] << 8) | args[4]);
	switch (space) {
		case EEPSCI3_PJ2: //SID_DUMP_SUB_EEPROM:
		case EEPSCI3_PJ3:
		case EEPSCI4_PF10:
			EEPROM_SCI = space;
			/* dump eeprom stuff */
			addr /= 2;	/* modify address to fit with eeprom 256*16bit org */
			len &= ~1;	/* align to 16bits */
			while (len) {
				uint16_t pbuf[17];
				uint8_t *pstart;	//start of ISO packet
				uint16_t *ebuf = &pbuf[1];	//cheat : form an ISO packet with the pos resp code in pbuf[0]

				int pktlen;
				int ecur;

				pstart = (uint8_t *)(pbuf) + 1;
				*pstart = SID_DUMP + 0x40;

				pktlen = len;
				if (pktlen > 32) pktlen = 32;

				for (ecur = 0; ecur < (pktlen / 2); ecur += 1) {
					eep_read16_sub((uint8_t) addr + ecur, (uint16_t *)&ebuf[ecur]);
				}
				iso_sendpkt(pstart, pktlen + 1);

				len -= pktlen;
				addr += (pktlen / 2);	//work in eeprom addresses
			}
			break;
		case SID_DUMP_EEPROM:
			/* dump eeprom stuff */
			addr /= 2;	/* modify address to fit with eeprom 256*16bit org */
			len &= ~1;	/* align to 16bits */
			while (len) {
				u16 pbuf[17];
				u8 *pstart;	//start of ISO packet
				u16 *ebuf=&pbuf[1];	//cheat : form an ISO packet with the pos resp code in pbuf[0]

				int pktlen;
				int ecur;

				pstart = (u8 *)(pbuf) + 1;
				*pstart = SID_DUMP + 0x40;

				pktlen = len;
				if (pktlen > 32) pktlen = 32;

				for (ecur = 0; ecur < (pktlen / 2); ecur += 1) {
                    eep_read16((uint8_t) addr + ecur, (uint16_t *)&ebuf[ecur]);
				}
				iso_sendpkt(pstart, pktlen + 1);

				len -= pktlen;
				addr += (pktlen / 2);	//work in eeprom addresses
			}
			break;
		case SID_DUMP_ROM:
			/* dump from ROM */
			txbuf[0] = SID_DUMP + 0x40;
			while (len) {
				int pktlen;
				pktlen = len;
				if (pktlen > 32) pktlen = 32;
				memcpy(&txbuf[1], (void *) addr, pktlen);
				iso_sendpkt(txbuf, pktlen + 1);
				len -= pktlen;
				addr += pktlen;
			}
			break;
		default:
			tx_7F(SID_DUMP, ISO_NRC_SFNS_IF);
			break;
	}	//switch (space)

	return;
}


/* SID 34 : prepare for reflashing */
static void cmd_flash_init(void) {
	u8 errval;

	if (!platf_flash_init(&errval)) {
		tx_7F(SID_FLREQ, errval);
		return;
	}

	txbuf[0] = (SID_FLREQ + 0x40);
	iso_sendpkt(txbuf, 1);
	flashstate = FL_READY;
	return;
}

/* "one's complement" checksum; if adding causes a carry, add 1 to sum. Slightly better than simple 8bit sum
 */
static u8 cks_add8(u8 *data, unsigned len) {
	u16 sum = 0;
	for (; len; len--, data++) {
		sum += *data;
		if (sum & 0x100) sum += 1;
		sum = (u8) sum;
	}
	return sum;
}

/* compare given CRC with calculated value.
 * data is the first byte after SID_CONF_CKS1
 */
static int cmd_romcrc(const u8 *data) {
	unsigned idx;
	// <CNH> <CNL> <CRC0H> <CRC0L> ...<CRC3H> <CRC3L>
/*
    u16 chunkno = (*(data+0) << 8) | *(data+1);
	for (idx = 0; idx < ROMCRC_NUMCHUNKS; idx++) {
		u16 crc;
		data += 2;
		u16 test_crc = (*(data+0) << 8) | *(data+1);
		u32 start = chunkno * ROMCRC_CHUNKSIZE;
		crc = crc16((const u8 *)start, ROMCRC_CHUNKSIZE);
		if (crc != test_crc) {
			return -1;
		}
		chunkno += 1;
	}
*/
    uint32_t start = (*(data+0) << 24) | (*(data+1) << 16) | (*(data+2) << 8) | *(data+3);
    uint32_t size = (*(data+4) << 24) | (*(data+5) << 16) | (*(data+6) << 8) | *(data+7);

    uint32_t crc;
    crc = crc32((const u8 *)start, size);

    txbuf[0] = SID_CONF + 0x40;
    txbuf[1] = SID_CONF_CKS1;
    txbuf[2] = ((crc >> 24) & 0xff);
    txbuf[3] = ((crc >> 16) & 0xff);
    txbuf[4] = ((crc >> 8) & 0xff);
    txbuf[5] = (crc & 0xff);
    iso_sendpkt(txbuf, 6);

	return 0;
}

/* handle low-level reflash commands */
static void cmd_flash_utils(struct iso14230_msg *msg) {
	u8 subcommand;
	u32 tmp;

	u32 rv = ISO_NRC_GR;

	if (flashstate != FL_READY) {
		rv = ISO_NRC_CNCORSE;
		goto exit_bad;
	}

	if (msg->datalen <= 1) {
		rv = ISO_NRC_SFNS_IF;
		goto exit_bad;
	}

	subcommand = msg->data[1];

	switch(subcommand) {
	case SIDFL_EB:
		//format : <SID_FLASH> <SIDFL_EB> <BLOCKNO>
		if (msg->datalen != 3) {
			rv = ISO_NRC_SFNS_IF;
			goto exit_bad;
		}
		rv = platf_flash_eb(msg->data[2]);
		if (rv) {
			rv = (rv & 0xFF) | 0x80;	//make sure it's a valid extented NRC
			goto exit_bad;
		}
		break;
	case SIDFL_WB:
		//format : <SID_FLASH> <SIDFL_WB> <A2> <A1> <A0> <D0>...<D127> <CRC>
		if (msg->datalen != (SIDFL_WB_DLEN + 6)) {
			rv = ISO_NRC_SFNS_IF;
			goto exit_bad;
		}

		if (cks_add8(&msg->data[2], (SIDFL_WB_DLEN + 3)) != msg->data[SIDFL_WB_DLEN + 5]) {
			rv = SID_CONF_CKS1_BADCKS;	//crcerror
			goto exit_bad;
		}

		tmp = (msg->data[2] << 16) | (msg->data[3] << 8) | msg->data[4];
		rv = platf_flash_wb(tmp, (u32) &msg->data[5], SIDFL_WB_DLEN);
		if (rv) {
			rv = (rv & 0xFF) | 0x80;	//make sure it's a valid extented NRC
			goto exit_bad;
		}
		break;
	case SIDFL_UNPROTECT:
		//format : <SID_FLASH> <SIDFL_UNPROTECT> <~SIDFL_UNPROTECT>
		if (msg->datalen != 3) {
			rv = ISO_NRC_SFNS_IF;
			goto exit_bad;
		}
		if (msg->data[2] != (u8) ~SIDFL_UNPROTECT) {
			rv = ISO_NRC_IK;	//InvalidKey
			goto exit_bad;
		}

		platf_flash_unprotect();
		break;
	default:
		rv = ISO_NRC_SFNS_IF;
		goto exit_bad;
		break;
	}

	txbuf[0] = SID_FLASH + 0x40;
	iso_sendpkt(txbuf, 1);	//positive resp
	return;

exit_bad:
	tx_7F(SID_FLASH, rv);
	return;
}


/* ReadMemByAddress */
static void cmd_rmba(struct iso14230_msg *msg) {
	//format : <SID_RMBA> <AH> <AM> <AL> <SIZ>
	/* response : <SID + 0x40> <D0>....<Dn> <AH> <AM> <AL> */

	u32 addr;
	int siz;

	if (msg->datalen != 5) goto bad12;
	siz = msg->data[4];

	if ((siz == 0) || (siz > 251)) goto bad12;

	addr = reconst_24(&msg->data[1]);

	txbuf[0] = SID_RMBA + 0x40;
	memcpy(txbuf + 1, (void *) addr, siz);

	siz += 1;
	txbuf[siz++] = msg->data[1];
	txbuf[siz++] = msg->data[2];
	txbuf[siz++] = msg->data[3];

	iso_sendpkt(txbuf, siz);
	return;

bad12:
	tx_7F(SID_RMBA, ISO_NRC_SFNS_IF);
	return;
}


/* WriteMemByAddr - RAM only */
static void cmd_wmba(struct iso14230_msg *msg) {
	/* WriteMemByAddress (RAM only !) . format : <SID_WMBA> <AH> <AM> <AL> <SIZ> <DATA> , siz <= 250. */
	/* response : <SID + 0x40> <AH> <AM> <AL> */
	u8 rv = ISO_NRC_SFNS_IF;
	u32 addr;
	u8 siz;
	u8 *src;

	if (msg->datalen < 6) goto badexit;
	siz = msg->data[4];

	if (	(siz == 0) ||
		(siz > 250) ||
		(msg->datalen != (siz + 5))) goto badexit;

	addr = reconst_24(&msg->data[1]);

	// bounds check, restrict to RAM
	if (	(addr < RAM_MIN) ||
		(addr > RAM_MAX)) {
		rv = ISO_NRC_CNDTSA; /* canNotDownloadToSpecifiedAddress */
		goto badexit;
	}

	/* write */
	src = &msg->data[5];
	memcpy((void *) addr, src, siz);

	msg->data[0] = SID_WMBA + 0x40;	//cheat !
	iso_sendpkt(msg->data, 4);
	return;

badexit:
	tx_7F(SID_WMBA, rv);
	return;
}

/* set & configure kernel */
static void cmd_conf(struct iso14230_msg *msg) {
	u8 resp[4];
	u32 tmp;

	resp[0] = SID_CONF + 0x40;
	if (msg->datalen < 2) goto bad12;

	switch (msg->data[1]) {
	case SID_CONF_SETSPEED:
		/* set comm speed (BRR divisor reg) : <SID_CONF> <SID_CONF_SETSPEED> <new divisor> */
		iso_sendpkt(resp, 1);
		cmd_init(msg->data[2]);
		sci_rxidle(25);
		return;
		break;
	case SID_CONF_SETEEPR:
		/* set eeprom_read() function address <SID_CONF> <SID_CONF_SETEEPR> <AH> <AM> <AL> */
		if (msg->datalen != 5) goto bad12;
		tmp = (msg->data[2] << 16) | (msg->data[3] << 8) | msg->data[4];
		eep_setptr(tmp);
		iso_sendpkt(resp, 1);
		return;
		break;
	case SID_CONF_CKS1:
		//<SID_CONF> <SID_CONF_CKS1> <CNH> <CNL> <CRC0H> <CRC0L> ...<CRC3H> <CRC3L>
        if (msg->datalen != 10) {
			goto bad12;
		}
		if (cmd_romcrc(&msg->data[2])) {
            //tx_7F(SID_CONF, SID_CONF_CKS1_BADCKS);
			return;
		}
        //iso_sendpkt(resp, 1);
		return;
		break;
	case SID_CONF_LASTERR:
		resp[1] = lasterr;
		lasterr = 0;
		iso_sendpkt(resp, 2);
		return;
		break;
#ifdef DIAG_U16READ
	case SID_CONF_R16:
		{
		u16 val;
		//<SID_CONF> <SID_CONF_R16> <A2> <A1> <A0>
		tmp = reconst_24(&msg->data[2]);
		tmp &= ~1;	//clr lower bit of course
		val = *(const u16 *) tmp;
		resp[1] = val >> 8;
		resp[2] = val & 0xFF;
		iso_sendpkt(resp,3);
		return;
		break;
		}
#endif
	default:
		goto bad12;
		break;
	}

bad12:
	tx_7F(SID_CONF, ISO_NRC_SFNS_IF);
	return;
}


/* command parser; infinite loop waiting for commands.
 * not sure if it's worth the trouble to make this async,
 * what other tasks could run in background ? reflash shit ?
 *
 * This receives valid iso14230 packets; message splitting is by pkt length
 */
void cmd_loop(void) {
	u8 rxbyte;

	static struct iso14230_msg msg;

	//u32 t_last, t_cur;	//timestamps

	iso_clearmsg(&msg);

	while (1) {
		enum iso_prc prv;

		/* in case of errors (ORER | FER | PER), reset state mach. */
		if (NPK_SCI.SSR.BYTE & 0x38) {

			cmstate = CM_IDLE;
			flashstate = FL_IDLE;
			iso_clearmsg(&msg);
			sci_rxidle(MAX_INTERBYTE);
			continue;
		}

		if (!NPK_SCI.SSR.BIT.RDRF) continue;

		rxbyte = NPK_SCI.RDR;
		NPK_SCI.SSR.BIT.RDRF = 0;

		//t_cur = get_mclk_ts();	/* XXX TODO : filter out interrupted messages with t>5ms interbyte ? */

		/* got a byte; parse according to state */
		prv = iso_parserx(&msg, rxbyte);

		if (prv == ISO_PRC_NEEDMORE) {
			continue;
		}
		if (prv != ISO_PRC_DONE) {
			iso_clearmsg(&msg);
			sci_rxidle(MAX_INTERBYTE);
			continue;
		}
		/* here, we have a complete iso frame */

		switch (cmstate) {
		case CM_IDLE:
			/* accept only startcomm requests */
			if (msg.data[0] == SID_STARTCOMM) {
				cmd_startcomm();
				cmstate = CM_READY;
			}
			iso_clearmsg(&msg);
			break;

		case CM_READY:
			switch (msg.data[0]) {
			case SID_STARTCOMM:
				cmd_startcomm();
				iso_clearmsg(&msg);
				break;
			case SID_RECUID:
				iso_sendpkt(npk_ver_string, sizeof(npk_ver_string));
				iso_clearmsg(&msg);
				break;
			case SID_CONF:
				cmd_conf(&msg);
				iso_clearmsg(&msg);
				break;
			case SID_RESET:
				/* ECUReset */
				txbuf[0] = msg.data[0] + 0x40;
				iso_sendpkt(txbuf, 1);
				die();
				break;
			case SID_RMBA:
				cmd_rmba(&msg);
				iso_clearmsg(&msg);
				break;
			case SID_WMBA:
				cmd_wmba(&msg);
				iso_clearmsg(&msg);
				break;
			case SID_DUMP:
				cmd_dump(&msg);
				iso_clearmsg(&msg);
				break;
			case SID_FLASH:
				cmd_flash_utils(&msg);
				iso_clearmsg(&msg);
				break;
			case SID_TP:
				txbuf[0] = msg.data[0] + 0x40;
				iso_sendpkt(txbuf, 1);
				iso_clearmsg(&msg);
				break;
			case SID_FLREQ:
				cmd_flash_init();
				iso_clearmsg(&msg);
				break;
			default:
				tx_7F(msg.data[0], ISO_NRC_SNS);
				iso_clearmsg(&msg);
				break;
			}	//switch (SID)
			break;
		default :
			//invalid state, or nothing special to do
			break;
		}	//switch (cmstate)
	}	//while 1

	die();
}


static u8 flashbuffer[128], counter8byteblock;
static u32 counter128byteblock, flashaddr, num128byteblocks;

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
 static int can_rx8bytes(u8 *msg) {

    #ifdef ssmk
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
    #endif

    return -1;
 }


 /* transmits 8 bytes from buf via CAN Channel 0 mailbox 1
  *
  *
  *
  *
  */
 static void can_tx8bytes(const u8 *buf) {

    #ifdef ssmk
        #ifdef SH7058
        	#if defined KLINE // For SH7058 K-Line models (pre MY07)
        		// Will be added later
        	#elif defined CAN
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
			    while (NPK_CAN.TXPR0.BIT.MB15) { };

			    NPK_CAN.TXACK0.WORD = 0x8000;
			    memcpy((void *) &NPK_CAN.MB[15].MSG_DATA[0], buf, 8);
			    NPK_CAN.TXPR0.WORD = 0x8000;

			    return;
	        #endif
        #elif defined SH7055
	        /* 7055 350nm version */
	        while (NPK_CAN.TXPR.BIT.MB1) { };

	        NPK_CAN.TXACK.WORD = 0x0200;
	        memcpy((void *) &NPK_CAN.MD[1][0], buf, 8);
	        NPK_CAN.TXPR.WORD = 0x0200;

	        return;
        #endif
    #endif

 }


/* flash initialisation, 0xE0 command, called from cmd_loop
 *
 *
 *
 *
 */
static void can_cmd_flash_init(u8 *msg) {

	u8 errval;

	if ((msg[1] & 0x07) != 1) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = 0x30;  // general format error
		can_tx8bytes(txbuf);
		return;
	}

	if (!platf_flash_init(&errval)) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = errval;
		can_tx8bytes(txbuf);
		return;
	}

	if (msg[2] == 0xA5) platf_flash_unprotect();

	txbuf[0] = 0x7A;
	txbuf[1] = (msg[1] & 0xF8) | 0x00;
	can_tx8bytes(txbuf);

	//flashstate = FL_READY;

	return;

}


/* erase flash block, 0xF0 command, called from cmd_loop
 *
 *
 *
 *
 */
 static void can_cmd_erase_block(u8 *msg) {

	u32 rv;

	if ((msg[1] & 0x07) != 6) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = 0x30;  // general format error
		can_tx8bytes(txbuf);
		return;
	}

	memcpy(txbuf, msg, 8);
	flashaddr = ((msg[3] << 24) | (msg[4] << 16) | (msg[5] << 8) | 0x00);
	num128byteblocks = ((msg[6] << 8) | msg[7]);
	counter8byteblock = 0;
	counter128byteblock = 0;

	rv = platf_flash_eb(msg[2]);

	if(rv) {
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = rv;
		can_tx8bytes(txbuf);
		return;
	}

	txbuf[0] = msg[0];
	txbuf[1] = (msg[1] & 0xF8) | 0x00;
	can_tx8bytes(txbuf);

	return;

}

/* load 8 bytes for flashing, no command code (all bytes are data), called from cmd_loop
 *
 *
 *
 *
 */
 static void can_cmd_load8bytes(u8 *msg) {

 	u8 i;

	// this is probably faster than memcpy
	for (i = 0; i < 8; i++) flashbuffer[(8 * counter8byteblock) + i] = msg[i];

 }

/* write flash block, 0xF8 command, called from cmd_loop
 *
 *
 *
 *
 */
 static void can_cmd_flash_128bytes(u8 *msg) {

	u32 i, rv, addr, flashCheckSum;

	if((msg[1] & 0x07) != 3) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = 0x30;  // general format error
		can_tx8bytes(txbuf);
		return;
	}

	memcpy(txbuf, msg, 8);

	flashCheckSum = 0;

	for (i = 0; i < 128; i++) {
		flashCheckSum += flashbuffer[i];
		flashCheckSum = ((flashCheckSum >> 8) & 0xFF) + (flashCheckSum & 0xFF);
	}

	if (flashCheckSum != msg[4]) {
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x02;
		txbuf[2] = 0x31;  // checksum error
		txbuf[3] = flashCheckSum;
		can_tx8bytes(txbuf);
		return;
	}

	if (counter128byteblock != (u32) ((msg[2] << 8) | msg[3])) {
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x05;
		txbuf[2] = 0x32;  // block number error
		txbuf[3] = (counter128byteblock >> 8) & 0xFF;
		txbuf[4] = counter128byteblock & 0xFF;
		txbuf[5] = (num128byteblocks >> 8) & 0xFF;
		txbuf[6] = num128byteblocks & 0xFF;
		can_tx8bytes(txbuf);
		return;
	}

	addr = flashaddr + (128 * counter128byteblock);

	rv = platf_flash_wb(addr, (u32) flashbuffer, 128);

	if(rv) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = rv;
		can_tx8bytes(txbuf);
		return;
	}

	txbuf[0] = 0x7A;
	txbuf[1] = (msg[1] & 0xF8) | 0x00;
	can_tx8bytes(txbuf);

	return;

}

/* kernelid command processor, 0x1A command, called from cmd_loop.
 * args[0,1] : 0x7A and SID
 *
 *
 *
 */
static void can_cmd_ecuid(u8 *msg) {

    u8 len = sizeof(npk_ver_string) - 1;
    //u8 msglen = 0;
    u8 msgindex = 0;
    u8 txindex = 2;

    while (len)
    {
	    txbuf[0] = 0x7A;
	    txbuf[1] = (msg[1] & 0xF8) | 0x06;
	    txbuf[2] = 0x20;
	    txbuf[3] = 0x20;
	    txbuf[4] = 0x20;
	    txbuf[5] = 0x20;
	    txbuf[6] = 0x20;
	    txbuf[7] = 0x20;

        while (len)
        {
        	txbuf[txindex] = npk_ver_string[msgindex];

            txindex++;
            if (txindex > 7)
                txindex = 2;
            msgindex++;
            len--;

            if (msgindex % 6 == 0 || (txindex < 7 && len == 0))
            	break;
        }
      	can_tx8bytes(txbuf);
    }

	return;
}

/* checksum command processor, 0xD0 command, called from cmd_loop.
 * args[0,1] : 0x7A and SID
 * args[2,3,4] : # of 256-byte blocks (modelled on Denso CAN method that has a max of 6 data bytes per 8 byte packet)
 * args[5,6,7] : (starting address / 256)
 *
 *
 *
 */
 static void can_cmd_cks(u8 *msg) {

	u32 len, cks;
	u8 *addr;

	if((msg[1] & 0x07) != 6) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = 0x30;  // general format error
		can_tx8bytes(txbuf);
		return;
	}

	len = ((msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | 0x00);
	addr = (u8 *) ((msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | 0x00);

	cks = 0;
	while (len) {
		cks += (*addr & 0xFF);
		cks = ((cks >> 8) & 0xFF) + (cks & 0xFF);
		len--;
		addr++;
	}

	txbuf[0] = 0x7A;
	txbuf[1] = (msg[1] & 0xF8) | 0x01;
	txbuf[2] = cks & 0xFF;
	can_tx8bytes(txbuf);

	return;

 }


/* dump eeprom command processor, 0xD8 command, called from cmd_loop.
 * args[0,1] : 0x7A and SID
 * ex.: "7A D6 00 10 00 00 00 00" dumps 1MB of ROM@ 0x0
 *
 */
static void can_cmd_dump_eeprom(u8 *msg) {
	u32 addr;
	u32 len;

	if((msg[1] & 0x07) != 6) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xB8) | 0x01;
		txbuf[2] = 0x30;  // general format error
		can_tx8bytes(txbuf);
		return;
	}

	//len = ((msg[2] << 16) | (msg[3] << 8) | msg[4]);
	len = ((msg[3] << 8) | msg[4]);
	addr = ((msg[5] << 16) | (msg[6] << 8) | msg[7]);
	EEPROM_SCI = msg[2];

	len &= ~1;	/* align to 16bits */

	txbuf[0] = 0x7A;
	txbuf[1] = (msg[1] & 0xB8);
	txbuf[2] = msg[2];
	txbuf[3] = msg[3];
	txbuf[4] = msg[4];
	txbuf[5] = msg[5];
	txbuf[6] = msg[6];
	txbuf[7] = msg[7];
	can_tx8bytes(txbuf);
	can_idle(750);

	addr /= 2;	/* modify address to fit with eeprom 256*16bit org */
	len &= ~1;	/* align to 16bits */
	while (len) {
		uint16_t pbuf[8];
		uint8_t *pstart;	//start of ISO packet
		uint16_t *ebuf = &pbuf[0];	//cheat : form an ISO packet with the pos resp code in pbuf[0]

		int pktlen;
		int ecur;

		pstart = (uint8_t *)(pbuf);

		pktlen = len;
		if (pktlen > 8) pktlen = 8;

		for (ecur = 0; ecur < (pktlen / 2); ecur += 1) {
			eep_can_read16_sub((uint8_t) addr + ecur, (uint16_t *)&ebuf[ecur]);
			//txbuf[ecur * 2] = ebuf[ecur] & 0xFF;
			//txbuf[ecur * 2 + 1] = (ebuf[ecur] >> 8) & 0xFF;
		}
		//iso_sendpkt(pstart, pktlen + 1);

		memcpy(txbuf, pstart, pktlen);
		can_tx8bytes(txbuf);
		//can_tx8bytes(pstart);

		len -= pktlen;
		addr += (pktlen / 2);	//work in eeprom addresses
	}

}

/* dump rom command processor, 0xD8 command, called from cmd_loop.
 * args[0,1] : 0x7A and SID
 * args[2,3,4] : # of 256-byte blocks
 * args[5,6,7] : (starting address / 256)
 * based on limitation of 6 data bytes per 8 byte packet
 * ex.: "7A DE 00 10 00 00 00 00" dumps 1MB of ROM@ 0x0
 *
 */
static void can_cmd_dump(u8 *msg) {

	u32 addr;
	u32 len;

	if((msg[1] & 0x07) != 6) {
		memcpy(txbuf, msg, 8);
		txbuf[0] = 0x7F;
		txbuf[1] = (msg[1] & 0xF8) | 0x01;
		txbuf[2] = 0x30;  // general format error
		can_tx8bytes(txbuf);
		return;
	}

	len = ((msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | 0x00);
	addr = ((msg[5] << 24) | (msg[6] << 16) | (msg[7] << 8) | 0x00);

	txbuf[0] = 0x7A;
	txbuf[1] = (msg[1] & 0xF8);
	can_tx8bytes(txbuf);
	can_idle(750);

	while (len) {
		u32 pktlen;
		pktlen = len;
		if (pktlen > 8) pktlen = 8;
		//txbuf[1] = (msg[1] & 0xF8) | pktlen;
		//memcpy(&txbuf[2], (void *) addr, pktlen);
		memcpy(txbuf, (void *) addr, pktlen);
		can_tx8bytes(txbuf);
		len -= pktlen;
		addr += pktlen;
		//can_idle(750);
	}

	return;

}


void can_cmd_loop(void) {

	u8 cmd;
	static u8 currentmsg[8];
	bool loadingblocks = false;
	counter8byteblock = 0;

	while (1) {

		int rv = can_rx8bytes(currentmsg);
		if(rv == 0) continue;

		if(rv == -1) {
			txbuf[0] = 0x7F;
			txbuf[1] = (currentmsg[1] & 0xF8) | 0x01;
			txbuf[2] = 0x36;   // unread message error
			can_tx8bytes(txbuf);
			continue;
		}

		if (loadingblocks) {

			can_cmd_load8bytes(currentmsg);
			counter8byteblock++;
			if (counter8byteblock > 15) {

				counter8byteblock = 0;
				loadingblocks = false;
			}

		}
		else if(currentmsg[0] == 0x7A) {

			cmd = currentmsg[1] & 0xF8;

			switch(cmd) {
			    case SID_CAN_RECUID: // 0xA0
				    can_cmd_ecuid(currentmsg);
				    break;

			    case SID_CAN_DUMP_EEPROM: // 0xB8
				    can_cmd_dump_eeprom(currentmsg);
				    break;

				case 0xD0: // 0xD0
					can_cmd_cks(currentmsg);
					break;

				case SID_CAN_DUMP: // 0xD8
					can_cmd_dump(currentmsg);
					break;

				case 0xE0: // 0xE0
					PFC.PDIOR.WORD |= 0x0100;
					can_cmd_flash_init(currentmsg);
					break;

				case 0xF0: // 0xF0
					can_cmd_erase_block(currentmsg);
					loadingblocks = true;
					break;

				case 0xF8: // 0XF8
					can_cmd_flash_128bytes(currentmsg);
					counter128byteblock++;
					if (counter128byteblock < num128byteblocks)
                        loadingblocks = true;
					break;

				default:
					txbuf[0] = 0x7F;
					txbuf[1] = (currentmsg[1] & 0xF8) | 0x01;
					txbuf[2] = 0x34;   // unrecognised 0x7A command
					can_tx8bytes(txbuf);
					break;

			}

		}
		else {

			if ((currentmsg[0] == 0xFF) && (currentmsg[1] == 0xC8)) {

				txbuf[0] = 0xFF;
				txbuf[1] = 0xC8;
				can_tx8bytes(txbuf);
				die();

			}

			txbuf[0] = 0x7F;
			txbuf[1] = (currentmsg[1] & 0xF8) | 0x01;
			txbuf[2] = 0x35;    // unrecognised command (non 0x7A)
			can_tx8bytes(txbuf);

		}

	}

	die();

	return;

}
