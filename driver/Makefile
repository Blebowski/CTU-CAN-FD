SRCS := ctu_can_fd_hw.c  ctu_can_fd_linux_defs.c  ctu_can_fd_userspace.cpp
OBJS := $(addsuffix .o,$(SRCS))
DEPS := $(addsuffix .d,$(SRCS))

ifeq ($(shell hostname),majernb)
	P := /home/martin/programs/Xilinx/SDK/2017.4/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-
else
	P := /home/mjerabek/Xilinx/vivado-2017.3/SDK/2017.3/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-
endif
#P := arm-linux-gnueabihf-
CC := $(P)gcc
CXX := $(P)g++
#CC := clang -target armv7a-pc-linux-gnueabi -march=armv7 -mthumb
XFLAGS := -Wall -Wextra -O2 -D__LITTLE_ENDIAN_BITFIELD -mthumb
CFLAGS := $(XFLAGS) -Werror=implicit-function-declaration
CXXFLAGS := $(XFLAGS)
#LDFLAGS := -fuse-ld=gold

all: test
	cp ./test /srv/nfs4/debian-armhf-devel/

test: $(OBJS)
	$(CXX) $(XFLAGS) -o $@ $(OBJS) $(LDFLAGS)
%.c.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
%.cpp.o: %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

.PHONY: all clean
clean:
	-rm test $(OBJS) $(DEPS)

-include $(DEPS)
