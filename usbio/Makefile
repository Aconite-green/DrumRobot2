PJ = 'USBIO'
VERSION  = '0.0.12'
HTML_TITLE  = 'ICPDAS USBIO Library for Linux Source Code Tour v$(VERSION)'

all:
	@if [ -e usbio/Makefile ]; then	\
	cd usbio && make;		\
	fi
	cd examples && make

clean:
	rm -f *.o *~ *.tmp \#* .\#*
	@if [ -e usbio/Makefile ]; then \
	cd usbio && make clean;		\
	fi
	cd examples && make clean

