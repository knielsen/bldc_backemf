TARGET=bldc_backemf

GCCDIR=/home/knielsen/devel/study/stellaris-arm/install
BINDIR=/usr/bin
CC=$(BINDIR)/arm-none-eabi-gcc
LD=$(BINDIR)/arm-none-eabi-ld
OBJCOPY=$(BINDIR)/arm-none-eabi-objcopy
LM4FLASH=/home/knielsen/devel/study/stellaris-arm/lm4tools/lm4flash/lm4flash

STARTUP=startup_gcc
LINKSCRIPT=$(TARGET).ld

FP_LDFLAGS= -L$(GCCDIR)/arm-none-eabi/lib/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16 -lm -L$(GCCDIR)/lib/gcc/arm-none-eabi/4.6.2/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16 -lgcc -lc

ARCH_CFLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -DTARGET_IS_BLIZZARD_RA1
INC=-I/home/knielsen/devel/study/stellaris-arm/SW-EK-LM4F120XL-9453 -DPART_LM4F131H5QR
CFLAGS=-g -O3  -std=c99 -Wall -pedantic $(ARCH_CFLAGS) $(INC)
LDFLAGS=--entry ResetISR --gc-sections

OBJS = $(TARGET).o dbg.o led.o nrf.o #usb.o

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STARTUP).o $(LINKSCRIPT)
	$(LD) $(LDFLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP).o $(OBJS) $(FP_LDFLAGS)

$(TARGET).o: $(TARGET).c dbg.h

$(STARTUP).o: $(STARTUP).c

dbg.o: dbg.c dbg.h

led.o: led.c led.h

#usb.c: bldc_backemf.h

nrf.c: bldc_backemf.h

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	$(LM4FLASH) $(TARGET).bin

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP).o

tty:
	stty -F/dev/stellaris raw -echo -hup cs8 -parenb -cstopb 500000

cat:
	cat /dev/stellaris

.PHONY: all clean flash tty cat
