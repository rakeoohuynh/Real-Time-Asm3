#!/bin/sh
# =====================================================================
# build.sh -- one-shot qcc build, matching the qcc-terminal workflow in
# the README. Equivalent to `make -f Makefile.poc`.
#
#   ./build.sh              # demo timing (TIME_SCALE_FACTOR=5)
#   ./build.sh 1            # real-world timing
#   TARGET=-Vgcc_ntoaarch64le ./build.sh
# =====================================================================
set -e

SCALE="${1:-5}"
TARGET="${TARGET:--Vgcc_ntox86_64}"
CFLAGS="$TARGET -Wall -Wextra -Wno-unused-parameter -DTIME_SCALE_FACTOR=$SCALE"

LC_SRCS="local_controller.c lc_context.c phase_table.c signal_output.c \
         boom_gate.c railway_protection.c train_schedule.c pedestrian.c \
         fixed_timing.c sensor_driven.c right_turn.c status_report.c \
         central_command_server.c console_input.c"

echo "building local_controller (TIME_SCALE_FACTOR=$SCALE) ..."
qcc $CFLAGS -o local_controller $LC_SRCS -lpthread -lsocket

echo "building central_controller ..."
qcc $CFLAGS -o central_controller central_controller.c -lpthread -lsocket

echo "done."
