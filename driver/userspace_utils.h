#pragma once

extern "C" {
#include "ctu_can_fd_linux_defs.h"
#include "ctu_can_fd_hw.h"
}

#undef abs
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <inttypes.h>
#include <err.h>

struct ctucanfd_priv* ctucanfd_init(uint32_t addr);
unsigned ctu_can_fd_read8(struct ctucanfd_priv *priv, enum ctu_can_fd_regs reg);
unsigned ctu_can_fd_read16(struct ctucanfd_priv *priv, enum ctu_can_fd_regs reg);
