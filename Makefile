# =================================================================
# Makefile - QNX SDP 7.1 Traffic Controller System
# Phase 1: Intersections I1 and I2, fixed_time mode only.
#
# Builds two executables:
#   bin/lc  - Local Controller (run once per intersection: I1, I2)
#   bin/cc  - Central Controller (status display)
#
# Command-line build with QNX Momentics' qcc compiler driver.
# Override CPU for a different QNX target architecture, e.g.:
#   make CPU=aarch64le
# See README.md for the equivalent Momentics IDE build steps.
# =================================================================
CPU ?= x86_64
CC   = qcc
ARCH = -Vgcc_nto$(CPU)

CFLAGS  = $(ARCH) -Wall -Wextra -std=gnu99 -Icommon
LDFLAGS = $(ARCH)

COMMON_SRC = common/protocol.c
LC_SRC     = lc/lc_main.c lc/phase_controller.c lc/signal_output.c lc/status_reporting.c $(COMMON_SRC)
CC_SRC     = cc/cc_main.c $(COMMON_SRC)

.PHONY: all clean lc cc
all: lc cc

lc: bin/lc
cc: bin/cc

bin:
	mkdir -p bin

bin/lc: $(LC_SRC) | bin
	$(CC) $(CFLAGS) -o $@ $(LC_SRC) $(LDFLAGS)

bin/cc: $(CC_SRC) | bin
	$(CC) $(CFLAGS) -o $@ $(CC_SRC) $(LDFLAGS)

clean:
	rm -rf bin
