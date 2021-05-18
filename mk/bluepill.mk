# Generic Makefile fragment for Blue Pill devices and LibOpenCM3
#
# Parameters:
#
#	ROOT			path to root of the repository
#	BINARY			binary to build (without extension)
#	OBJS			list of object files
#	LIB_OBJS		list of library object files
#	WITH_BOOT_LOADER	define if code origin should be shifted by 8K for boot loader
#	WITH_DFU_FLASH		flashing uses dfu-util
#	WITH_SERIAL_FLASH	flashing uses the built-in serial boot-loader
#	MAX_SIZE		complain if the built firmware exceeds this size
#

vpath %.c $(ROOT)/lib
OBJS += $(LIB_OBJS)

OPENCM3_DIR=/home/mj/stm/libopencm3
DEVICE=stm32f103x8

.PHONY: all
all:: $(BINARY).elf

.PHONY: flash
flash: $(BINARY).flash

ifneq ($(V),1)
Q		:= @
NULL		:= 2>/dev/null
endif

include $(OPENCM3_DIR)/mk/genlink-config.mk

ifdef WITH_BOOT_LOADER

# We want to generate a linked script for a different ROM start address
UCW_LDSCRIPT=bootloader-$(DEVICE).ld

bootloader-$(DEVICE).ld: generated.$(DEVICE).ld
	@printf "  GENLNK2 $@\n"
	$(Q)sed '/^ rom /s/ORIGIN = 0x08000000/ORIGIN = 0x08002000/' <$< >$@

else

UCW_LDSCRIPT=$(LDSCRIPT)

endif

PREFIX		?= arm-none-eabi

CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb
OPT		:= -Os
DEBUG		:= -ggdb3
CSTD		?= -std=gnu99

TGT_CFLAGS	+= $(OPT) $(CSTD) $(DEBUG)
TGT_CFLAGS	+= $(ARCH_FLAGS)
TGT_CFLAGS	+= -Wall -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -Wno-parentheses
TGT_CFLAGS	+= -fno-common -ffunction-sections -fdata-sections
TGT_CFLAGS	+= -I. -I$(ROOT)/lib

TGT_CPPFLAGS	+= -MD

TGT_LDFLAGS	+= --static -nostartfiles
TGT_LDFLAGS	+= -T$(UCW_LDSCRIPT)
TGT_LDFLAGS	+= $(ARCH_FLAGS) $(DEBUG)
TGT_LDFLAGS	+= -Wl,-Map=$(*).map -Wl,--cref
TGT_LDFLAGS	+= -Wl,--gc-sections
ifeq ($(V),99)
TGT_LDFLAGS	+= -Wl,--print-gc-sections
endif

LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-rules.mk

%.bin: %.elf
	@printf "  OBJCOPY $< -> $@\n"
	$(Q)$(OBJCOPY) -Obinary $< $@
ifdef MAX_SIZE
	$(Q)if [ $$(stat -c '%s' $@) -gt $(MAX_SIZE) ] ; then echo >&2 "Output too exceeds $(MAX_SIZE) bytes!" ; false ; fi
endif

%.elf: $(OBJS) $(UCW_LDSCRIPT)
	@printf "  LD      $(*).elf\n"
	$(Q)$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $*.elf

%.o: %.c
	@printf "  CC      $(*).c\n"
	$(Q)$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@printf "  CLEAN\n"
	$(Q)rm -f *.elf *.bin *.dfu *.o *.d *.map $(LDSCRIPT) $(UCW_LDSCRIPT)

ifdef WITH_DFU_FLASH

all:: $(BINARY).dfu

%.flash: %.dfu
	@printf "  FLASH  $<\n"
	$(Q)dfu-util $(DFU_ARGS) -D $<

%.dfu: %.bin $(ROOT)/tools/dfu-sign
	@printf "  SIGN    $< -> $@\n"
	$(Q)$(ROOT)/tools/dfu-sign $< $@

$(ROOT)/tools/dfu-sign:
	make -C $(ROOT)/tools

# For the STM32duino-bootloader, we used:
#%.flash: %.bin
#	@printf "  FLASH  $<\n"
#	$(Q)dfu-util -a2 -D $(*).bin

else

ifdef WITH_SERIAL_FLASH

all:: $(BINARY).bin

BOOT_SERIAL ?= /dev/ttyUSB0

%.flash: %.bin
	@printf "  FLASH  $<\n"
	$(Q)$(ROOT)/bin/stm32flash $(BOOT_SERIAL) -i 'dtr,-dtr' -w $< -g 0

.PHONY: reset
reset: all
	$(Q)$(ROOT)/bin/stm32flash $(BOOT_SERIAL) -i 'dtr,-dtr' -g 0

else

all:: $(BINARY).bin

%.flash: %.bin
	@printf "  FLASH  $<\n"
	$(Q)$(ROOT)/bin/st-flash write $(*).bin 0x8000000

.PHONY: reset
reset:
	$(ROOT)/bin/st-flash reset

endif
endif

.SECONDEXPANSION:
.SECONDARY:

-include $(OBJS:.o=.d)
