ifeq ($(KERNEL_VERSION),)

CROSS_COMPILE?= arm-linux-gnueabihf-
KDIR ?= /home/mjerabek/projects/kernel/linux-4.15-build/zynq
ARCH=arm

ifeq ($(shell hostname),hathi)
INSTALL_DIR=/srv/nfs4/debian-armhf-devel
endif

#KDIR ?= /lib/modules/$(shell uname -r)/build
#ARCH ?= x86_64

else
KDIR = /lib/modules/$(KERNEL_VERSION)/build
endif

MAKEARGS := -C $(KDIR)
MAKEARGS += $(if $(ARCH),ARCH=$(ARCH))
MAKEARGS += $(if $(CROSS_COMPILE),CROSS_COMPILE=$(CROSS_COMPILE))

$(warning "$(MAKEARGS)")

default:
	$(MAKE) $(MAKEARGS) M=$$PWD
ifneq ($(INSTALL_DIR),)
	-rm -f $(INSTALL_DIR)/ctucanfd.ko
	cp ctucanfd.ko $(INSTALL_DIR)/
endif

clean:
	$(MAKE) $(MAKEARGS) M=$$PWD clean
