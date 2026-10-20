#Unmaintained, use cmake instead

#This can be changed at compile time as required by
#the gcc toolchain binaries, i.e. if gcc is installed as
#"sh4-none-elf-gcc", then run "make PREFIX=sh4-none-elf ..."
#PREFIX ?= /usr/bin/sh-elf
PREFIX ?= /usr/share/gnush_v13.01_elf-1/bin/sh-elf

lc = $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$1))))))))))))))))))))))))))

#DBGFLAGS=-gdwarf-2

#possible choices : SH7051 SH7055 SH7055_18 SH7055_35 SH7058
#try "make BUILDWHAT=SH7055_18" to override this default.

# For Nissan ECUs (npk protocol)
#BUILDWHAT ?= SH7051
#BUILDWHAT ?= SH7055
#BUILDWHAT ?= SH7055_18

#BUILDWHAT ?= SH7055
BUILDWHAT ?= SH7058

#BUILDCOMMS ?= KLINE
#BUILDCOMMS ?= CAN
BUILDCOMMS ?= CAN_TP

BUILDMEM ?= FLASH
#BUILDMEM ?= EEPROM

# SH7055 flash driver: VER_AUTO (default) builds both the 0.35um (FLMCR) and the 0.18um (SH7058-style, microcode in RAM) drivers
# and picks one at run time (pl_flash_7055_auto.c); VER_350 / VER_180 build a single driver, for debugging.
MCU_REV ?= VER_AUTO

EEPSCOM ?= EEP_COMMS3
#EEPSCOM ?= EEP_COMMS4

#EEP_COMMS3_CS ?= EEP_COMMS3_CS_PJ2
EEP_COMMS3_CS ?= EEP_COMMS3_CS_PJ3

# BUILDFOR=TCU: the Denso SH7055 TCU kernel (use with BUILDWHAT=SH7055 BUILDCOMMS=CAN_TP). The TCU's UDS bootloader loads the image only at 0xFFFF9000 and at most 8KB of it, so the
# code is linked there and the .bss / stack elsewhere (ldscripts/lkr_subaru_7055_tcu.ld); the TCU is 0.35um, so only that flash driver; CAN_TP takes the TCU application's own mailbox
# identifiers (0x7E1 / 0x7E9). Output: ssmk_tcu_can_tp_sh7055.bin
BUILDFOR ?=
ifeq ($(BUILDFOR), TCU)
    override MCU_REV = VER_350
    KFLAGS += -DTCU
endif

# For Subaru ECUs (ssmk protocol)
# BUILDMEM=FLASH: the lean flash kernel. BUILDMEM=EEPROM: the same plus the serial-EEPROM commands (the port, PJ2/PJ3/PF10, is
# selected by the command at run time), built as <name>_eeprom.
ifeq ($(BUILDMEM), EEPROM)
	BUILDTARGET = $(BUILDWHAT)_EEPROM
	KFLAGS += -DEEPROM_SUPPORT
else
	BUILDTARGET = $(BUILDWHAT)
endif

#possible choices : npk ssmk
#try "make BUILDPROTOCOL=ssmk" to override this default.
#BUILDPROTOCOL ?= npk
BUILDPROTOCOL ?= ssmk

# Specify compiler to be used
CC = $(PREFIX)-gcc

# Specify Assembler to be used
#AS = $(PREFIX)-as
AS   = $(PREFIX)-gcc -x assembler-with-cpp

CP   = $(PREFIX)-objcopy
SIZE = $(PREFIX)-size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary -S

# Specify linker to be used
LD = $(PREFIX)-ld

# Specify CPU flag
CPU = -m2 -mb

# Common compiler flags
#OPT = -Os
OPT = -Os -ffunction-sections

DEFS = $(BUILDPROTOCOL)

BUILD_DIR = build
PROJECT_DIR = build/kernels
PROJBASE_TMP = $(BUILDPROTOCOL)_$(BUILDCOMMS)_$(BUILDTARGET)
ifeq ($(MCU_REV), VER_180)
    PROJBASE_TMP = $(BUILDPROTOCOL)_$(BUILDCOMMS)_$(BUILDTARGET)_180
endif
ifeq ($(MCU_REV), VER_350)
    PROJBASE_TMP = $(BUILDPROTOCOL)_$(BUILDCOMMS)_$(BUILDTARGET)_350
endif
ifeq ($(BUILDFOR), TCU)
    PROJBASE_TMP = $(BUILDPROTOCOL)_tcu_$(BUILDCOMMS)_$(BUILDTARGET)
endif
PROJBASE = $(call lc,$(PROJBASE_TMP))
#PROJBASE = $(PROJBASE_TMP) | tr '[:upper:]' '[:lower:]'

BUILD = $(BUILD_DIR)/$(PROJBASE)
PROJECT = $(PROJECT_DIR)/$(PROJBASE)

# -mrelax (with --relax at link): jsr/mov.l -> bsr/shorter forms where the target is in range, ~140 bytes.
# -fno-builtin: memcpy/memset/memcmp are the tiny ones in minimem.c, never inlined into calls to themselves.
CFLAGS = $(CPU) $(DBGFLAGS) $(OPT) -mrelax -fno-builtin -fomit-frame-pointer -std=gnu99 -Wall -Wextra -Wstrict-prototypes \
	-fstack-usage -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(KFLAGS) $(E_CFLAGS)
CFLAGS += -Wno-maybe-uninitialized -Wno-unused-function

LDFLAGS = $(CPU) -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(BUILD).map,--cref,--gc-sections,--relax

ASFLAGS = $(CPU) $(DBGFLAGS) -nostartfiles -Wa,-amhls=$(<:.s=.lst) $(E_ASFLAGS)

ASRC = start_ssm.s

SRC = cmd_parser.c eep_funcs.c main.c crc.c mfg_ssm.c wdt.c minimem.c

ifeq ($(BUILDPROTOCOL), npk)
    ifeq ($(BUILDWHAT), SH7051)
	    SRC += platf_7050.c pl_flash_7051.c
	    LDSCRIPT = ldscripts/lkr_7051.ld
    else ifeq ($(BUILDWHAT), SH7055_35)
	    SRC += platf_7055.c pl_flash_7055_350nm.c
	    LDSCRIPT = ldscripts/lkr_7055_7058.ld
    else
	    SRC += platf_7055.c pl_flash_705x_180nm.c
	    LDSCRIPT = ldscripts/lkr_7055_7058.ld
    endif
endif


ifeq ($(BUILDPROTOCOL), ssmk)
    ifeq ($(BUILDWHAT), SH7055)
        ifeq ($(MCU_REV), VER_180)
	    SRC += platf_7055.c pl_flash_705x_180nm.c
	else ifeq ($(MCU_REV), VER_350)
	    SRC += platf_7055.c pl_flash_7055_350nm.c
	else
	    SRC += platf_7055.c pl_flash_7055_350nm.c pl_flash_705x_180nm.c pl_flash_7055_auto.c
	    KFLAGS += -DFLASH_AUTO
	endif
	    LDSCRIPT = ldscripts/lkr_subaru_7055.ld
        ifeq ($(BUILDFOR), TCU)
	    LDSCRIPT = ldscripts/lkr_subaru_7055_tcu.ld
        endif
    else ifeq ($(BUILDWHAT), SH7059D_EURO5)
            SRC += platf_7055.c pl_flash_705x_180nm.c
 	    LDSCRIPT = ldscripts/lkr_subaru_7059d.ld
    else ifeq ($(BUILDWHAT), SH7058D_EURO4)
	    SRC += platf_7055.c pl_flash_705x_180nm.c
	    LDSCRIPT = ldscripts/lkr_subaru_7058d.ld
    else ifeq ($(BUILDWHAT), SH7058)
 	    SRC += platf_7055.c pl_flash_705x_180nm.c
	    LDSCRIPT = ldscripts/lkr_subaru_7058.ld
    endif
endif

OBJS  = $(ASRC:.s=.o) $(SRC:.c=.o)
PREFIX_OBJS = $(addprefix $(BUILD_DIR)/, $(OBJS))

all: npk_commit.h $(OBJS) $(BUILD).elf $(PROJECT).bin
	@$(SIZE) $(BUILD).elf

%.o: %.c
	@$(CC) -c $(CFLAGS) -D $(BUILDWHAT) -D $(BUILDCOMMS) -D $(MCU_REV) -D $(EEP_COMMS3_CS) -D $(DEFS) -D $(EEPSCOM) -D PLATF=\"$(BUILDWHAT)\" -I . $< -o $(BUILD_DIR)/$@

%.o: %.s
	@$(AS) -c $(ASFLAGS) $< -o $(BUILD_DIR)/$@

%elf: $(OBJS)
	@$(CC) $(PREFIX_OBJS) $(LDFLAGS) -o $@

%hex: %elf
	@$(HEX) $< $@

%bin: %elf
	@$(BIN)  $< $@

#npk_commit.h:
#	git log -n 1 --format=format:"#define NPK_COMMIT \"%h\"%n" HEAD > $@

.PHONY : clean
clean:
	-@rm -f $(OBJS)
	-@rm -f $(SRC:.c=.su)
	-@rm -f  $(BUILD_DIR)/$(PROJBASE)_$(BUILDTARGET).elf
	-@rm -f  $(BUILD_DIR)/$(PROJBASE)_$(BUILDTARGET).map
	-@rm -f  $(PROJECT_DIR)/$(PROJBASE)_$(BUILDTARGET).bin
	-@rm -f  $(SRC:.c=.c.bak)
	-@rm -f  $(SRC:.c=.lst)
	-@rm -f  $(ASRC:.s=.s.bak)
	-@rm -f  $(ASRC:.s=.lst)
