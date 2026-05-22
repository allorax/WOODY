CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -mcpu=cortex-m3 -mthumb -O2 -Wall -Tstm32f103.ld -nostdlib

all: blink.bin

blink.elf: main.c
	$(CC) $(CFLAGS) -o $@ $^

blink.bin: blink.elf
	$(OBJCOPY) -O binary $< $@

flash: blink.bin
	st-flash write blink.bin 0x08000000

clean:
	rm -f blink.elf blink.bin
