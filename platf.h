#ifndef _PLATF_H
#define _PLATF_H
/****** Platform-specific code and defines
 */

/* (c) copyright fenugrec 2016-2020
 * GPLv3
 *
 */


/****** kernel customization ******/
#define SCI_DEFAULTDIV 9	//default value for BRR reg. Speed (kbps) = (20 * 1000) / (32 * (BRR + 1))

/* Uncomment to enable verification of succesful block erase . Adds 128B for the block descriptors + ~ 44B of code */
//#define POSTERASE_VERIFY

/* Uncomment to add diag function for atomic u16 reads */
//#define DIAG_U16READ

/* Uncomment to taint WDT pulse for debug use */
//#define DIAG_TAINTWDT



#include <stdbool.h>
#include "stypes.h"

#define flashbuffersize 0x1000
#define flashchunksize 0x80
#ifndef KLINE
#define flashmessagesize 0x200
#endif
#ifdef KLINE
#define flashmessagesize 0x80
#endif

u8 flashbuffer[flashbuffersize];

/****** mfg- and mcu-specific defines ******
*
* RAM_MIN, RAM_MAX : whole RAM area
* " #include "reg_defines/????" : i/o peripheral registers
*/

/*** WDT and master clock stuff
 * want to toggle the WDT every X ms (2ms on Nissan)
 */

//#define WDT_PER_MS	2
	/* somehow shc sucks at reducing the following :
	 * u16 WDT_MAXCNT = WDT_PER_MS / 1000 * (40*1000*1000ULL / 64))
	 */
//#define WDT_MAXCNT WDT_PER_MS * 40*1000*1000UL / 64 / 1000



#if defined(SH7058)
    #define WDT_MAXCNT 2060 //aim for 6.6ms , although it probably works at 2ms anyway
    //#define WDT_MAXCNT 4125 //aim for 6.6ms , although it probably works at 2ms anyway
    #include "reg_defines/7055_7058_180nm.h"
    #define RAM_MIN	0xFFFF0000
    #define RAM_MAX 	0xFFFFBFFF
    #define NPK_SCI SCI2

#elif defined(SH7059D_EURO5)
    #define WDT_MAXCNT 2060 //aim for 6.6ms , although it probably works at 2ms anyway
    //#define WDT_MAXCNT 4125 //aim for 6.6ms , although it probably works at 2ms anyway
    #include "reg_defines/7055_7058_180nm.h"
    #define RAM_MIN	0xFFFE8000
    #define RAM_MAX 	0xFFFFBFFF
    #define NPK_SCI SCI2

#elif defined(SH7058D_EURO4)
    #define WDT_MAXCNT 2060 //aim for 6.6ms , although it probably works at 2ms anyway
    //#define WDT_MAXCNT 4125 //aim for 6.6ms , although it probably works at 2ms anyway
    #include "reg_defines/7055_7058_180nm.h"
    #define RAM_MIN	0xFFFF0000
    #define RAM_MAX 	0xFFFFBFFF
    #define NPK_SCI SCI2

#elif defined(SH7055)
    #define WDT_MAXCNT 2060 //aim for 3.3ms
    #include "reg_defines/7055_350nm.h"
    #define RAM_MIN	0xFFFF6000
    #define RAM_MAX	0xFFFFDFFF
    #define NPK_SCI SCI2

#else
    #error invalid target for ssmk
#endif

#define NPK_CAN HCAN0

#define MCLK_GETMS(x) ((x) * 16 / 10000)	/* convert ticks to milliseconds */
#define MCLK_GETTS(x) ((x) * 10000 / 16) /* convert millisec to ticks */
#define MCLK_MAXSPAN	10000	/* arbitrary limit (in ms) for time spans measured by MCLK */
/** Get current timestamp from free-running counter */
//uint32_t get_mclk_ts(void);
#define get_mclk_ts(x) (ATU0.TCNT)


/** Ret 1 if ok
 *
 * sets *err to a negative response code if failed
 */
bool platf_flash_init(uint8_t *err);


/** Disable modification (erase/write) to flash.
 *
 * Called this to set the actual calls to erase / write flash to be skipped
 */
void platf_flash_protect(void);

/** Enable modification (erase/write) to flash.
 *
 * If this is not called after platf_flash_init(), the actual calls to erase / write flash are skipped
 */
void platf_flash_unprotect(void);

/** Erase block, see definition of blocks in DS.
 *
 * ret 0 if ok
 */
uint32_t platf_flash_eb(u32 addr);

/** Write block of data. len must be multiple of SIDFL_WB_DLEN
 *
 *
 * @return 0 if ok , response code ( > 0x80) if failed.
 *
 * Note : the implementation must not assume that the src address will be 4-byte aligned !
 */
uint32_t platf_flash_wb(uint32_t dest, uint32_t src, uint32_t len);

/***** Init funcs ****/


/** init platform-specific stuff : SCI, clocks, interrupts, WDT etc */
void init_platf(void);



/** force reset by external supervisor and/or internal WDT.
 */
void die(void);


#endif
