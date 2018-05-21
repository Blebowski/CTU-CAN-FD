KDIR ?= /home/mjerabek/projects/kernel/linux-4.9-build/zynq
default:
	$(MAKE) -C $(KDIR) M=$$PWD
	cp ctucanfd.ko /srv/nfs4/debian-armhf-devel/
clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
