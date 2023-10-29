PJ =usbio_list usbio_module_info usbio_do usbio_do_channel usbio_do-setting usbio_di usbio_di-setting usbio_ai usbio_ai-setting usbio_ao usbio_ao-setting

CFLAGS =  -I. -I../include
LDFLAGS =

Bit := $(shell getconf LONG_BIT)

ifeq ($(Tool), ARM)
        CROSS_COMPILE   = arm-linux-gnueabihf-
        DLIBS = ../lib/libUSBIO_arm.so.1.0
        SLIBS = ../lib/libUSBIO_arm.a
        #LDFLAGS = -lgcc_s -lc
else
ifeq ($(Tool), ARM64)
        CROSS_COMPILE   = aarch64-linux-gnu-
        DLIBS = ../lib/libUSBIO_arm64.so.1.0
        SLIBS = ../lib/libUSBIO_arm64.a
        LDFLAGS = -Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed
else
        CROSS_COMPILE   =

ifeq ($(Bit), 64)
        DLIBS = ../lib/libUSBIO_64.so.1.0
        SLIBS = ../lib/libUSBIO_64.a
else
        DLIBS = ../lib/libUSBIO_32.so.1.0
        SLIBS = ../lib/libUSBIO_32.a
endif
endif
endif

AS              = $(CROSS_COMPILE)as
LD              = $(CROSS_COMPILE)ld
CC              = $(CROSS_COMPILE)gcc
CPP             = $(CC) -E
AR              = $(CROSS_COMPILE)ar
NM              = $(CROSS_COMPILE)nm
STRIP           = $(CROSS_COMPILE)strip
OBJCOPY         = $(CROSS_COMPILE)objcopy
OBJDUMP         = $(CROSS_COMPILE)objdump

AM_LDFLAGS = -lpthread

all: $(PJ)

%: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<  $(SLIBS) $(AM_LDFLAGS) -lm
	#$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<  $(DLIBS) $(AM_LDFLAGS) -lm

clean:
	rm -f *.o *dlib $(PJ) *~ *.tmp *.bak \#* .\#*
