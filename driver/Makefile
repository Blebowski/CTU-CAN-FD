SRCS := ctu_can_fd_hw.c  ctu_can_fd_linux_defs.c  userspace_utils.cpp
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

test: $(OBJS) ctu_can_fd_userspace.cpp.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
regtest: $(OBJS) regtest.cpp.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
%.c.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
%.cpp.o: %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

.PHONY: all clean
clean:
	-rm test *.o $(DEPS)

-include $(DEPS)
