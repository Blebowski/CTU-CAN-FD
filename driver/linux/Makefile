CROSS_COMPILE= arm-linux-gnueabihf-
KDIR ?= /home/mjerabek/projects/kernel/linux-4.15-build/zynq
ARCH=arm

#KDIR ?= /lib/modules/$(shell uname -r)/build
#ARCH ?= x86_64

default:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$$PWD
ifeq ($(shell hostname),hathi)
	-rm -f /srv/nfs4/debian-armhf-devel/ctucanfd.ko
	cp ctucanfd.ko /srv/nfs4/debian-armhf-devel/
endif
clean:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm clean
