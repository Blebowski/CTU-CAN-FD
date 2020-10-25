SRCS := ctucanfd_hw.c  ctucanfd_linux_defs.c  userspace_utils.cpp
OBJS := $(addsuffix .o,$(SRCS))
DEPS := $(wildcard *.d)

P := arm-linux-gnueabihf-

CC := $(P)gcc
CXX := $(P)g++
#CC := clang -target armv7a-pc-linux-gnueabi -march=armv7 -mthumb
XFLAGS := -Wall -Wextra -O2 -D__LITTLE_ENDIAN_BITFIELD -mthumb
CFLAGS := $(XFLAGS) -Werror=implicit-function-declaration
CXXFLAGS := $(XFLAGS)
#LDFLAGS := -fuse-ld=gold

all: test regtest
ifeq ($(shell hostname),hathi)
	cp ./test ./regtest /srv/nfs4/debian-armhf-devel/
endif

CTUCANFD_HEADERS = ctu_can_fd.h ctu_can_fd_frame.h ctu_can_fd_hw.h ctu_can_fd_regs.h ctu_can_fd_linux_defs.h

ctu_can_%.h : ctucan%.h
	ln -s $< $@

test: $(OBJS) ctucanfd_userspace.cpp.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
regtest: $(OBJS) regtest.cpp.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
%.c.o: %.c  $(CTUCANFD_HEADERS)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
%.cpp.o: %.cpp  $(CTUCANFD_HEADERS)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

.PHONY: all clean
clean:
	-rm -f test *.o $(DEPS)

-include $(DEPS)
