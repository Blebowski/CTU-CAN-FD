KDIR ?= /home/mjerabek/projects/kernel/linux-4.15-build/zynq
default:
	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- M=$$PWD
ifeq ($(shell hostname),hathi)
	-rm -f /srv/nfs4/debian-armhf-devel/ctucanfd.ko
	cp ctucanfd.ko /srv/nfs4/debian-armhf-devel/
endif
clean:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm clean
