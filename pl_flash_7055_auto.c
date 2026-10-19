/* SH7055 flash back-end that works on both silicon generations.
 *
 * Some SH7055 ECUs carry the 0.35um part (flash controlled through FLMCR1/FLMCR2, pl_flash_7055_350nm.c) and some the
 * 0.18um one, which is the SH7058's flash with a smaller array (FKEY/FCCS/FPCS, erase and write microcode downloaded to
 * RAM, pl_flash_705x_180nm.c). The block layout is the same (EB0..EB15 = 8 x 4KB, 32KB, 7 x 64KB), only the way to erase
 * and program differs, so the right driver has to be chosen at run time.
 *
 * The test is the one both drivers already use to refuse the wrong silicon: FKEY, an 8-bit register at 0xFFFFE804, exists
 * (reads back what was written) only on the 0.18um part. It is repeated at every platf_flash_init() (the host sends 0x20 /
 * 0x21 before touching flash), which then runs the selected driver's own init, including its other silicon checks.
 *
 * Built only with FLASH_AUTO (see the Makefile); the two drivers' API is renamed there.
 */

#include "stypes.h"
#include "platf.h"
#include "npk_errcodes.h"

bool flash350_init(u8 *err);
void flash350_protect(void);
void flash350_unprotect(void);
uint32_t flash350_eb(u32 addr);
uint32_t flash350_wb(uint32_t dest, uint32_t src, uint32_t len);

bool flash180_init(u8 *err);
void flash180_protect(void);
void flash180_unprotect(void);
uint32_t flash180_eb(u32 addr);
uint32_t flash180_wb(uint32_t dest, uint32_t src, uint32_t len);

#ifndef FKEY_RD	/* (the host-side test, tests/test_flash_auto.c, replaces these with a fake register) */
#define FKEY_RD()	(*(volatile u8 *) 0xFFFFE804)	/* only on the 0.18um parts */
#define FKEY_WR(v)	(*(volatile u8 *) 0xFFFFE804 = (v))
#endif

static enum { SILICON_UNKNOWN = 0, SILICON_350NM, SILICON_180NM } silicon = SILICON_UNKNOWN;

/** 0.18um if FKEY is a read/write register, 0.35um if it is not there. Both a 0x33 and a 0xCC pattern have to read back, so a
 * floating bus that happens to return the last value does not pass. */
static bool fkey_present(void)
{
	bool present;

	FKEY_WR(0x33);
	present = (FKEY_RD() == 0x33);
	FKEY_WR(0xCC);
	present = present && (FKEY_RD() == 0xCC);
	FKEY_WR(0);	/* the locked state; the 0.18um driver sets its keys itself */
	return present;
}

bool platf_flash_init(u8 *err)
{
	/* a driver only ever runs after its own init succeeded: until then both back-ends see "protected" */
	silicon = SILICON_UNKNOWN;
	flash350_protect();
	flash180_protect();

	if (fkey_present()) {
		silicon = SILICON_180NM;
		return flash180_init(err);
	}
	silicon = SILICON_350NM;
	return flash350_init(err);
}

void platf_flash_protect(void)
{
	if (silicon == SILICON_180NM)
		flash180_protect();
	else if (silicon == SILICON_350NM)
		flash350_protect();
}

void platf_flash_unprotect(void)
{
	if (silicon == SILICON_180NM)
		flash180_unprotect();
	else if (silicon == SILICON_350NM)
		flash350_unprotect();
}

uint32_t platf_flash_eb(u32 addr)
{
	if (silicon == SILICON_180NM)
		return flash180_eb(addr);
	if (silicon == SILICON_350NM)
		return flash350_eb(addr);
	return PF_ERROR;	/* no platf_flash_init() yet */
}

uint32_t platf_flash_wb(uint32_t dest, uint32_t src, uint32_t len)
{
	if (silicon == SILICON_180NM)
		return flash180_wb(dest, src, len);
	if (silicon == SILICON_350NM)
		return flash350_wb(dest, src, len);
	return PF_ERROR;
}
