KDIR ?= /home/mjerabek/projects/kernel/linux-4.9-build/zynq
default:
	$(MAKE) -C $(KDIR) M=$$PWD
	-rm -f /srv/nfs4/debian-armhf-devel/ctucanfd.ko
	cp ctucanfd.ko /srv/nfs4/debian-armhf-devel/
clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
