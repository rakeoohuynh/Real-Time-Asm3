#!/bin/sh
# =====================================================================
# build.sh -- one-shot qcc build, matching the qcc-terminal workflow in
# the README. Equivalent to `make -f Makefile.poc`.
#
#   ./build.sh              # demo timing (TIME_SCALE_FACTOR=5)
#   ./build.sh 1            # real-world timing
#   TARGET=-Vgcc_ntoaarch64le ./build.sh
#
# Layout: shared/ (used by both programs), lc/ (Local Controller),
# cc/ (Central Controller). Run from this directory; the binaries are
# written here.
# =====================================================================
set -e

SCALE="${1:-5}"
TARGET="${TARGET:--Vgcc_ntox86_64}"
CFLAGS="$TARGET -Wall -Wextra -Wno-unused-parameter -DTIME_SCALE_FACTOR=$SCALE -Ishared"

LC_SRCS="lc/local_controller.c lc/lc_context.c lc/phase_table.c lc/signal_output.c \
         lc/boom_gate.c lc/railway_protection.c lc/train_schedule.c lc/pedestrian.c \
         lc/fixed_timing.c lc/sensor_driven.c lc/right_turn.c lc/status_report.c \
         lc/central_command_server.c lc/console_input.c"

echo "building local_controller (TIME_SCALE_FACTOR=$SCALE) ..."
qcc $CFLAGS -Ilc -o local_controller $LC_SRCS -lpthread -lsocket

echo "building central_controller ..."
qcc $CFLAGS -Icc -o central_controller cc/central_controller.c -lpthread -lsocket

echo "done."
