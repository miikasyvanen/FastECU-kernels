# Building the Subaru (ssmk) kernels

All FastECU kernels: `./compile.sh` builds every one into `build/kernels/*.bin` (commit the ones you want in `kernels/`).
One kernel by hand:

    make clean BUILDWHAT=SH7058 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
    make       BUILDWHAT=SH7058 BUILDCOMMS=CAN_TP BUILDMEM=FLASH

(`mkdir -p build/kernels` first; `compile.sh` does it.) Always `make clean` when changing a variable, objects are not rebuilt for it.

## Variables

| Variable | Values | Meaning |
|---|---|---|
| `BUILDWHAT` | `SH7055`, `SH7058`, `SH7058D_EURO4`, `SH7059D_EURO5` | MCU / ECU family |
| `BUILDCOMMS` | `KLINE`, `CAN`, `CAN_TP` | `CAN` = raw CAN frames; `CAN_TP` = ISO 15765 (the kernel uses the host application's mailboxes and IDs) |
| `BUILDMEM` | `FLASH` (default), `EEPROM` | `EEPROM` adds the serial-EEPROM commands (`..._eeprom.bin`) |
| `MCU_REV` | `VER_AUTO` (default), `VER_350`, `VER_180` | SH7055 flash driver, see below |
| `BUILDFOR` | empty, `TCU` | `TCU`: Denso SH7055 TCU kernel, see below |
| `PREFIX` | path prefix | toolchain, default `/usr/share/gnush_v13.01_elf-1/bin/sh-elf` (`$(PREFIX)-gcc`, `-ld`, `-objcopy`, ...) |

Output name: `ssmk_<comms>_<what>[_eeprom|_350|_180].bin`, lower case; TCU builds are `ssmk_tcu_<comms>_<what>.bin` (like the older `ssmk_tcu_can_sh7058.bin`).

## SH7055 flash driver (`MCU_REV`)

The SH7055 exists as 0.35um and 0.18um silicon with different flash controllers.

- **`VER_AUTO` (default)**: builds both drivers (`pl_flash_7055_350nm.c`, `pl_flash_705x_180nm.c`) plus `pl_flash_7055_auto.c`, with `-DFLASH_AUTO`.
  At `platf_flash_init` the kernel writes `0x33` then `0xCC` to `FKEY` (`0xFFFFE804`), which only 0.18um parts have: it reads back, so the 180nm
  driver is used, otherwise the 350nm one. Both drivers stay write-protected until their own init has succeeded. Output has no suffix
  (`ssmk_can_sh7055.bin`).
- **`VER_350`** / **`VER_180`**: one driver only (smaller; for debugging). Output gets `_350` / `_180`.

`MCU_REV` is also passed to the compiler as `-D`, and `platf.h` picks the 180nm register header for `VER_180`, the 350nm one otherwise.
SH7058 / SH7059 have a single flash driver; `MCU_REV` does nothing there.

## Denso SH7055 TCU kernel (`BUILDFOR=TCU`)

    make clean BUILDWHAT=SH7055 BUILDCOMMS=CAN_TP BUILDMEM=FLASH BUILDFOR=TCU
    make       BUILDWHAT=SH7055 BUILDCOMMS=CAN_TP BUILDMEM=FLASH BUILDFOR=TCU     # -> ssmk_tcu_can_tp_sh7055.bin

- Forces `MCU_REV = VER_350` (the TCU is 0.35um): 350nm driver only, no auto-detect, `-DTCU`.
- Links with `ldscripts/lkr_subaru_7055_tcu.ld`: code at `0xFFFF9000`, at most 8KB (the TCU bootloader's fixed upload window), `.bss` and stack
  elsewhere in RAM (not uploaded).
- ISO 15765 on the TCU application's own CAN mailboxes (RX 14 / TX 15), IDs 0x7E1 / 0x7E9.

## Toolchain

GNUSH 13.01 `sh-elf` (gcc 4.7). The binaries are **32-bit x86**. On a 64-bit host without 32-bit glibc (e.g. Fedora) install the i386 libc, or
run each tool through a sandbox with an i386 libc mounted, for example:

1. Extract the toolchain: `rpm2cpio gnush_v13.01_elf-1-1.i386.rpm | cpio -idm`.
2. Extract the Ubuntu `libc6_*_i386.deb` (and `zlib1g`, `libtinfo5`, `libncurses5` i386 debs) with `ar x` + `tar` into `libs/`.
3. Wrapper script `shwrap`:

       bwrap --ro-bind /usr /usr --symlink usr/lib64 /lib64 --symlink usr/bin /bin --symlink usr/sbin /sbin \
         --tmpfs /lib --ro-bind libs/lib/i386-linux-gnu /lib/i386-linux-gnu --ro-bind libs/lib/ld-linux.so.2 /lib/ld-linux.so.2 \
         --ro-bind libs/usr/lib/i386-linux-gnu /lib/i386-usr --ro-bind /etc /etc --proc /proc --dev /dev \
         --bind /tmp /tmp --bind /home /home \
         --setenv LD_LIBRARY_PATH /lib/i386-linux-gnu:/lib/i386-usr "$@"

   (Do not bind into `/usr/lib`, it is read-only; do not `--tmpfs /tmp`.)
4. One tiny script per tool (`sh-elf-gcc`, `-ld`, `-objcopy`, `-size`, `-ar`, `-as`, `-nm`, `-objdump`) in a directory of its own:
   `exec shwrap <real tool path> "$@"`.
5. `make PREFIX=<that directory>/sh-elf BUILDWHAT=... BUILDCOMMS=... BUILDMEM=...` (or put the wrappers where the Makefile's default `PREFIX` points;
   `compile.sh` passes no `PREFIX`, so it needs the default location).

Builds are reproducible: with this setup the committed `kernels/*.bin` are rebuilt byte-identical.
