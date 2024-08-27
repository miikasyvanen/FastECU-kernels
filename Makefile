#Unmaintained, use cmake instead

#This can be changed at compile time as required by
#the gcc toolchain binaries, i.e. if gcc is installed as
#"sh4-none-elf-gcc", then run "make PREFIX=sh4-none-elf ..."
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

EEPSCOM ?= EEP_COMMS3
#EEPSCOM ?= EEP_COMMS4

#EEP_COMMS3_CS ?= EEP_COMMS3_CS_PJ2
EEP_COMMS3_CS ?= EEP_COMMS3_CS_PJ3

# For Subaru ECUs (ssmk protocol)
ifeq ($(BUILDMEM), EEPROM)
	ifeq ($(EEPSCOM), EEPSCI3)
		ifeq ($(EEP_COMMS3_CS), EEP_COMMS3_CS_PJ2)
			BUILDTARGET = $(BUILDWHAT)_EEPSCI3_PJ2
		else ifeq ($(EEP_COMMS3_CS), EEP_COMMS3_CS_PJ3)
			BUILDTARGET = $(BUILDWHAT)_EEPSCI3_PJ3
		endif
	else ifeq ($(EEPSCOM), EEPSCI4)
		BUILDTARGET = $(BUILDWHAT)_EEPSCI4
	endif
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
PROJBASE = $(call lc,$(PROJBASE_TMP))
#PROJBASE = $(PROJBASE_TMP) | tr '[:upper:]' '[:lower:]'

BUILD = $(BUILD_DIR)/$(PROJBASE)
PROJECT = $(PROJECT_DIR)/$(PROJBASE)

#-specs=nano.specs

CFLAGS = $(CPU) $(DBGFLAGS) $(OPT) -fomit-frame-pointer -std=gnu99 -Wall -Wextra -Wstrict-prototypes \
	-fstack-usage -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(E_CFLAGS)

LDFLAGS = $(CPU) -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(BUILD).map,--cref,--gc-sections

ASFLAGS = $(CPU) $(DBGFLAGS) -nostartfiles -Wa,-amhls=$(<:.s=.lst) $(E_ASFLAGS)

ASRC = start_ssm.s

SRC = cmd_parser.c eep_funcs.c main.c crc.c mfg_ssm.c wdt.c

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
	    SRC += platf_7055.c pl_flash_7055_350nm.c
	    LDSCRIPT = ldscripts/lkr_subaru_7055.ld
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
	@$(CC) -c $(CFLAGS) -D $(BUILDWHAT) -D $(BUILDCOMMS) -D $(EEP_COMMS3_CS) -D $(DEFS) -D $(EEPSCOM) -D PLATF=\"$(BUILDWHAT)\" -I . $< -o $(BUILD_DIR)/$@

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
