ifeq ($(KDIR),)
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
endif

MAKEARGS := -C $(KDIR)
MAKEARGS += $(if $(ARCH),ARCH=$(ARCH))
MAKEARGS += $(if $(CROSS_COMPILE),CROSS_COMPILE=$(CROSS_COMPILE))

$(warning "$(MAKEARGS)")

.PHONY: default checkpatch clean

default:
	$(MAKE) $(MAKEARGS) M=$$PWD
ifneq ($(INSTALL_DIR),)
	-rm -f $(INSTALL_DIR)/ctucanfd.ko
	-rm -f $(INSTALL_DIR)/ctucanfd_platform.ko
	cp ctucanfd.ko $(INSTALL_DIR)/
	cp ctucanfd_platform.ko $(INSTALL_DIR)/
endif

CTUCANF_SOURCES = ctu_can_fd.c ctu_can_fd_hw.c ctu_can_fd_frame.h ctu_can_fd_hw.h ctu_can_fd_regs.h ctu_can_fd_platform.c ctu_can_fd_pci.c

checkpatch:
	cd $(KDIR) && (! $(KDIR)/source/scripts/checkpatch.pl -f --no-tree $(CTUCANF_SOURCES:%=$(PWD)/%) | grep ERROR:)

clean:
	$(MAKE) $(MAKEARGS) M=$$PWD clean
