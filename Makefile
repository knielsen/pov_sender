TARGET=pov_sender

GCCDIR=/home/knielsen/devel/study/stellaris-arm/install
SWDIR=/home/knielsen/devel/study/stellaris-arm/SW-EK-LM4F120XL-9453

BINDIR=$(GCCDIR)/bin
CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld
OBJCOPY=arm-none-eabi-objcopy
LM4FLASH=lm4flash

STARTUP=startup_gcc
LINKSCRIPT=$(TARGET).ld

FP_LDFLAGS= -L$(GCCDIR)/arm-none-eabi/lib/thumb/cortex-m4 -lm -L$(GCCDIR)/lib/gcc/arm-none-eabi/4.6.2/thumb/cortex-m4 -lgcc -lc

ARCH_CFLAGS=-mthumb -mcpu=cortex-m4 -ffunction-sections -fdata-sections -DTARGET_IS_BLIZZARD_RA1
# LM4F120H5QR corresponds to TM4C1233H6PM. LM4F131H5QR corresponds to TM4C1236H6PM.
# LM4F121H5QR corresponds to TM4C1232H6PM
#INC=-I$(SWDIR) -DPART_LM4F120H5QR
INC=-I$(SWDIR) -DPART_LM4F131H5QR
CFLAGS=-Dgcc -g -O3  -std=c99 -Wall -pedantic $(ARCH_CFLAGS) $(INC)
LDFLAGS=--entry ResetISR --gc-sections

VPATH=$(SWDIR)/boards/ek-lm4f120xl/drivers
VPATH+=$(SWDIR)/utils

OBJS = $(TARGET).o usb_serial_structs.o
LIBS = $(SWDIR)/usblib/gcc-cm4f/libusb-cm4f.a $(SWDIR)/driverlib/gcc-cm4f/libdriver-cm4f.a

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STARTUP).o $(LINKSCRIPT)
	$(LD) $(LDFLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP).o $(OBJS) $(LIBS) $(FP_LDFLAGS)

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
	stty -F/dev/ttyACM0 raw -echo -hup cs8 -parenb -cstopb 500000

cat:
	cat /dev/ttyACM0

.PHONY: all clean flash tty cat
