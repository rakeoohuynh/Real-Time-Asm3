#!/bin/sh
# Builds local_controller and central_controller with qcc. Same result as
# `make -f Makefile.poc`. Run from this directory; the binaries land here.
#
#   ./build.sh              # demo timing (TIME_SCALE_FACTOR=5)
#   ./build.sh 1            # real-world timing
#   TARGET=-Vgcc_ntoaarch64le ./build.sh
#
# If the linker can't find -lpthread, remove it; on QNX 7.1 pthreads are
# part of libc.
set -e

SCALE="${1:-5}"
TARGET="${TARGET:--Vgcc_ntox86_64}"
CFLAGS="$TARGET -Wall -Wextra -Wno-unused-parameter -DTIME_SCALE_FACTOR=$SCALE -Ishared"

LC_SRCS="lc/local_controller.c lc/lc_context.c lc/phase_table.c lc/signal_output.c \
         lc/boom_gate.c lc/railway_protection.c lc/train_schedule.c lc/pedestrian.c \
         lc/fixed_timing.c lc/sensor_driven.c lc/mode_schedule.c lc/right_turn.c lc/status_report.c \
         lc/central_command_server.c lc/console_input.c"

echo "building local_controller (TIME_SCALE_FACTOR=$SCALE) ..."
qcc $CFLAGS -Ilc -o local_controller $LC_SRCS -lpthread -lsocket

echo "building central_controller ..."
qcc $CFLAGS -Icc -o central_controller cc/central_controller.c -lpthread -lsocket

echo "done."
