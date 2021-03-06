PRJ_NAME   = dro
CC         = arm-none-eabi-gcc
SRCDIR     = src
SRC        = $(wildcard $(SRCDIR)/*.c)
ASRC       = $(wildcard $(SRCDIR)/*.s)
OBJ        = $(SRC:.c=.o) $(ASRC:.s=.o)
OBJCOPY    = arm-none-eabi-objcopy
OBJDUMP    = arm-none-eabi-objdump
PROGRAMMER = $(shell which st-flash)
DEVICE     = STM32F1
OPT       ?= -Og
LIBPATHS   = libopencm3
CFLAGS     = -fdata-sections -ffunction-sections -g3 -Wall -mcpu=cortex-m3 -mlittle-endian -mthumb -I $(LIBPATHS)/include/ -I $(SRCDIR) -D$(DEVICE) $(OPT)
ASFLAGS    =  $(CFLAGS)
LDSCRIPT   = $(LIBPATHS)/lib/stm32/f1/stm32f103x8.ld
LDFLAGS    = -T $(LDSCRIPT)  -L$(LIBPATHS)/lib -lopencm3_stm32f1 -lm -lc -u _printf_float --static -nostartfiles -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs
LIBOPENCM3 = $(LIBPATHS)/lib/libopencm3_stm32f1.a

.PHONY: all clean flash burn hex bin

all: $(PRJ_NAME).elf

$(LIBOPENCM3):
	make -C libopencm3 TARGETS=stm32/f1 $(MAKEFLAGS)

$(PRJ_NAME).elf: $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS)
	arm-none-eabi-size $(PRJ_NAME).elf

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.o: %.c $(LIBOPENCM3)
	$(CC) -MMD -c $(CFLAGS) $< -o $@

%.o: %.s $(LIBOPENCM3)
	$(CC) -MMD -c $(ASFLAGS) $< -o $@

-include $(SRCDIR)/*.d

clean:
	rm -f $(OBJ) $(PRJ_NAME).elf $(PRJ_NAME).hex $(PRJ_NAME).bin $(SRCDIR)/*.d

distclean: clean
	make -C libopencm3 clean

# Flash 64k Device
flash:	$(PRJ_NAME).bin
	$(PROGRAMMER) $(FLASHSIZE) write $(PRJ_NAME).bin 0x8000000

# Flash 128k Device
bigflash: $(PRJ_NAME).bin
	$(PROGRAMMER) --flash=128k write $(PRJ_NAME).bin 0x8000000

hex: $(PRJ_NAME).elf
	$(OBJCOPY) -O ihex $(PRJ_NAME).elf $(PRJ_NAME).hex

bin: $(PRJ_NAME).elf
	$(OBJCOPY) -O binary $(PRJ_NAME).elf $(PRJ_NAME).bin
