/* functions specific to Subaru ECUs
 *
 * (c) fenugrec 2022
 * (c) rimwall 2022
 * GPLv3
 *
 */

#include <stdint.h>

#include "mfg.h"
#include "platf.h"
#include "wdt.h"



void init_mfg(void) {
	PFC.PDIOR.BIT.B8 = 1;  //required for 7055 02_fxt 350nm, has no effect on 7058

//#if defined(EEP_COMMS4)
	// required to send clock signal to EEPROM
	PFC.PLCRH.WORD &= ~0x0008;  //turn off PL9 MD1   |
	PFC.PLCRH.WORD |= 0x0004;   //turn on PL9 MD0    |  sets PL9 to be SCK4
	PFC.PLIOR.WORD |= 0x0200;   //set PL9 to be an output

	// required to set up CS signal to EEPROM
	PFC.PFCRH.WORD &= ~0x0010;  //PF10 is general I/O
	PFC.PFIOR.WORD |= 0x0400;   //PF10 is an output  |  sets PF10 to be EEPROM CS

	// required to set up Rx on SCI4 for comms with EEPROM
	PFC.PBCRH.WORD &= ~0x0080;  //turn off PB11 MD1  |
	PFC.PBCRH.WORD |= 0x0040;   //turn on PB11 MD0   |  sets PB11 to RxD4
	PFC.PBIOR.WORD &= ~0x0800;   //PB11 is an input

	// required to set up Tx on SCI4 for comms with EEPROM
	PFC.PBCRH.WORD &= ~0x0020;  //turn off PB10 MD1  |
	PFC.PBCRH.WORD |= 0x0010;   //turn on PB10 MD0   |  sets PB10 to TxD4
	PFC.PBIOR.WORD |= 0x0400;   //PB10 is an output

//#elif defined(EEP_COMMS3)
	// required to send clock signal to EEPROM
	PFC.PLCRH.WORD |= 0x0001;   //turn on PL8        |  sets PL8 to be SCK3
	PFC.PLIOR.WORD |= 0x0100;   //set PL8 to be an output

	// required to set up CS signal to EEPROM
	//#if defined(EEP_COMMS3_CS_PJ2)
		PFC.PJCRL.WORD &= ~0x0010;  //PJ2 is general I/O
		PFC.PJIOR.WORD |= 0x0004;   //PJ2 is an output   |  sets PJ3 to be EEPROM CS
	//#elif defined(EEP_COMMS3_CS_PJ3)
		PFC.PJCRL.WORD &= ~0x0040;  //PJ3 is general I/O
		PFC.PJIOR.WORD |= 0x0008;   //PJ3 is an output   |  sets PJ3 to be EEPROM CS
	//#endif

	// required to set up Rx on SCI3 for comms with EEPROM
	PFC.PBCRH.WORD &= ~0x0008;  //turn off PB9 MD1   |
	PFC.PBCRH.WORD |= 0x0004;   //turn on PB9 MD0    |  sets PB9 to RxD3
	PFC.PBIOR.WORD &= ~0x0200;   //PB9 is an input

	// required to set up Tx on SCI3 for comms with EEPROM
	PFC.PBCRH.WORD &= ~0x0002;  //turn off PB8 MD1   |
	PFC.PBCRH.WORD |= 0x0001;   //turn on PB8 MD0    |  sets PB8 to TxD3
	PFC.PBIOR.WORD |= 0x0100;   //PB8 is an output

//#else
	//#error No target specified!
//#endif

	wdt_dr = &PB.DR.WORD;  //manually assign these values, not elegant but will do for now
	wdt_pin = 0x8000;

	// set PD8 to bring FWE high to prepare for erasing or flashing
	PD.DR.WORD |= 0x0100;
}
