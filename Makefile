TARGET=pov_sender

SWDIR=/home/knielsen/devel/study/stellaris-arm/SW-EK-LM4F120XL-9453

BINDIR=$(GCCDIR)/bin
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
LM4FLASH=lm4flash

STARTUP=startup_gcc
LINKSCRIPT=$(TARGET).ld

FP_LDFLAGS= -lm -lgcc -lc

ARCH_CFLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -ffast-math -DTARGET_IS_BLIZZARD_RA1
# LM4F120H5QR corresponds to TM4C1233H6PM. LM4F131H5QR corresponds to TM4C1236H6PM.
# LM4F121H5QR corresponds to TM4C1232H6PM
#INC=-I$(SWDIR) -DPART_LM4F120H5QR
INC=-I$(SWDIR) -DPART_LM4F131H5QR
CFLAGS=-Dgcc -g -O3  -std=c99 -Wall -pedantic $(ARCH_CFLAGS) $(INC)
LDFLAGS=--entry ResetISR -Wl,--gc-sections

VPATH=$(SWDIR)/boards/ek-lm4f120xl/drivers
VPATH+=$(SWDIR)/utils

OBJS = $(TARGET).o usb_serial_structs.o
LIBS = $(SWDIR)/usblib/gcc-cm4f/libusb-cm4f.a $(SWDIR)/driverlib/gcc-cm4f/libdriver-cm4f.a

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STARTUP).o $(LINKSCRIPT)
	$(LD) $(CFLAGS) $(LDFLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP).o $(OBJS) $(LIBS) $(FP_LDFLAGS)

$(TARGET).o: $(TARGET).c usb_serial_structs.h
usb_serial_structs.o: usb_serial_structs.c usb_serial_structs.h

$(STARTUP).o: $(STARTUP).c

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	$(LM4FLASH) $(TARGET).bin

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP).o

tty:
	stty -F/dev/pov_sender raw -echo -hup cs8 -parenb -cstopb 500000

cat:
	cat /dev/pov_sender

.PHONY: all clean flash tty cat
