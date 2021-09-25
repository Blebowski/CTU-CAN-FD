# CTU CAN FD ip core Linux driver Makefile

# Invoked make without parameters for native build for current running kernel
# Set KERNEL_VERSION variable to compile for specific installed kernel version
# Exact location of Linux kernel build tree can be chosen by KDIR variable
# If you build for different CPU architecture than native and you do not
# use GNUmakefile trick to set CROSS_COMPILE and ARCH then set these to
# matching values else you do not screw only CTU CAN FD build but even
# base kernel build

#CROSS_COMPILE?= arm-linux-gnueabihf-
#KDIR ?= /home/mjerabek/projects/kernel/linux-4.15-build/zynq
#ARCH=arm

ifeq ($(KDIR),)
ifeq ($(KERNEL_VERSION),)
KERNEL_VERSION=$(shell uname -r)
endif

KDIR = /lib/modules/$(KERNEL_VERSION)/build
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

CTUCANFD_SOURCES = ctucanfd_base.c ctucanfd_kframe.h ctucanfd_kregs.h ctucanfd_platform.c ctucanfd_pci.c

checkpatch:
	cd $(KDIR) && (! $(KDIR)/source/scripts/checkpatch.pl -f --no-tree $(CTUCANFD_SOURCES:%=$(PWD)/%) | grep ERROR:)

clean:
	$(MAKE) $(MAKEARGS) M=$$PWD clean
