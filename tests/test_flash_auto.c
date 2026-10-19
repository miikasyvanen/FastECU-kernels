/* Host-side test of the SH7055 run-time flash driver choice (pl_flash_7055_auto.c): the FKEY register is faked, the two real
 * drivers are replaced by recorders.
 *
 *   tests/run_flash_auto.sh
 */

#include <stdio.h>
#include <string.h>
#include "stypes.h"

/* ---- the fake FKEY register: four kinds of hardware */
enum fake_kind { FK_REGISTER, FK_FLOATING_FF, FK_STUCK_33, FK_ZERO };
static enum fake_kind kind;
static u8 reg;
static u8 fkey_rd(void)
{
	switch (kind) {
	case FK_REGISTER: return reg;          /* 0.18um: reads back what was written */
	case FK_FLOATING_FF: return 0xFF;      /* 0.35um: nothing there */
	case FK_ZERO: return 0x00;
	default: return 0x33;                  /* answers the first pattern only */
	}
}
static void fkey_wr(u8 v) { reg = v; }
#define FKEY_RD() fkey_rd()
#define FKEY_WR(v) fkey_wr(v)

/* ---- recorders standing in for the two drivers */
static char calls[64][16];
static int ncalls;
static void rec(const char *s) { strncpy(calls[ncalls++], s, 15); }
static const char *last(void) { return ncalls ? calls[ncalls - 1] : ""; }
static int count(const char *s) { int n = 0; for (int i = 0; i < ncalls; i++) n += strcmp(calls[i], s) == 0; return n; }
static bool init_result = true;

bool flash350_init(u8 *err) { (void) err; rec("350init"); return init_result; }
void flash350_protect(void) { rec("350prot"); }
void flash350_unprotect(void) { rec("350unprot"); }
uint32_t flash350_eb(u32 a) { (void) a; rec("350eb"); return 0; }
uint32_t flash350_wb(uint32_t d, uint32_t s, uint32_t l) { (void) d; (void) s; (void) l; rec("350wb"); return 0; }
bool flash180_init(u8 *err) { (void) err; rec("180init"); return init_result; }
void flash180_protect(void) { rec("180prot"); }
void flash180_unprotect(void) { rec("180unprot"); }
uint32_t flash180_eb(u32 a) { (void) a; rec("180eb"); return 0; }
uint32_t flash180_wb(uint32_t d, uint32_t s, uint32_t l) { (void) d; (void) s; (void) l; rec("180wb"); return 0; }

#include "pl_flash_7055_auto.c"

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

int main(void)
{
	u8 err = 0;

	/* before any init, nothing may erase or write */
	CHECK(platf_flash_eb(0) == PF_ERROR && platf_flash_wb(0, 0, 0) == PF_ERROR);
	CHECK(ncalls == 0);
	platf_flash_protect();
	platf_flash_unprotect();
	CHECK(ncalls == 0);

	/* 0.18um: FKEY is a real register */
	kind = FK_REGISTER;
	CHECK(platf_flash_init(&err));
	CHECK(strcmp(last(), "180init") == 0 && count("350init") == 0);
	CHECK(reg == 0);                                  /* left in the locked state */
	platf_flash_unprotect();
	CHECK(strcmp(last(), "180unprot") == 0);
	CHECK(platf_flash_eb(0x1000) == 0 && strcmp(last(), "180eb") == 0);
	CHECK(platf_flash_wb(0, 0, 0) == 0 && strcmp(last(), "180wb") == 0);
	platf_flash_protect();
	CHECK(strcmp(last(), "180prot") == 0);

	/* 0.35um: nothing at that address (reads 0xFF, or 0x00) */
	for (int k = FK_FLOATING_FF; k <= FK_STUCK_33; k++) {
		kind = (enum fake_kind) k;
		ncalls = 0;
		CHECK(platf_flash_init(&err));
		CHECK(strcmp(last(), "350init") == 0 && count("180init") == 0);
		platf_flash_unprotect();
		CHECK(strcmp(last(), "350unprot") == 0);
		CHECK(platf_flash_eb(0) == 0 && strcmp(last(), "350eb") == 0);
		CHECK(platf_flash_wb(0, 0, 0) == 0 && strcmp(last(), "350wb") == 0);
	}

	/* every init first protects both drivers, so a failed init leaves nothing enabled; the driver's failure is passed on */
	kind = FK_REGISTER;
	ncalls = 0;
	init_result = false;
	CHECK(!platf_flash_init(&err));
	CHECK(strcmp(calls[0], "350prot") == 0 && strcmp(calls[1], "180prot") == 0 && strcmp(last(), "180init") == 0);
	init_result = true;

	/* re-detection on the next init (a different board is not a concern, but a stale choice must never stick) */
	kind = FK_FLOATING_FF;
	ncalls = 0;
	CHECK(platf_flash_init(&err) && strcmp(last(), "350init") == 0);
	kind = FK_REGISTER;
	ncalls = 0;
	CHECK(platf_flash_init(&err) && strcmp(last(), "180init") == 0);

	printf(failures ? "FAILED (%d)\n" : "ok: 0.18um / 0.35um choice, protection and pass-through\n", failures);
	return failures != 0;
}
