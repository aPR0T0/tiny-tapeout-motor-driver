# Firmware for tt_um_apr0t0_foc_mcu (RV32E, executes in place from QSPI flash)
CROSS   ?= riscv-none-elf-
CC       = $(CROSS)gcc
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size
CFLAGS   = -march=rv32e -mabi=ilp32e -Os -ffreestanding -fno-builtin -Wall -Wextra \
           -ffunction-sections -fdata-sections -msmall-data-limit=0
LDFLAGS  = -nostdlib -nostartfiles -T link.ld -Wl,--gc-sections -Wl,-Map,build/firmware.map
B        = build

all: $(B)/firmware.hex $(B)/firmware.bin $(B)/selftest_expected.txt

$(B):
	mkdir -p $(B)

$(B)/firmware.elf: crt0.S main.c selftest.c selftest.h foc_regs.h link.ld | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ crt0.S main.c selftest.c -lgcc
	$(SIZE) $@

$(B)/firmware.bin: $(B)/firmware.elf
	$(OBJCOPY) -O binary $< $@

# one byte per line for $readmemh in the flash model
$(B)/firmware.hex: $(B)/firmware.bin
	python3 -c "import sys; d=open('$<','rb').read(); open('$@','w').write(''.join('%02x\n'%b for b in d))"

$(B)/selftest_expected.txt: selftest.c selftest_host.c selftest.h | $(B)
	cc -O2 -o $(B)/selftest_host selftest.c selftest_host.c
	$(B)/selftest_host > $@

foc_regs.h: ../host/foc_regs.py
	python3 ../host/foc_regs.py

clean:
	rm -rf $(B)

.PHONY: all clean
