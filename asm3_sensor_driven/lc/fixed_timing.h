/*
 * fixed_timing.h -- the NS/EW green -> yellow -> all-red vehicle cycle.
 *
 * This cycle runs in both modes; sensor-driven mode only shortens greens
 * (see sensor_driven.c). The begin_* functions are also how the
 * pedestrian, railway and override code hand control back to normal
 * traffic.
 */
#ifndef FIXED_TIMING_H
#define FIXED_TIMING_H

#include "lc_context.h"

void fixed_timing_begin_ns_green(lc_context_t *ctx);
void fixed_timing_begin_ew_green(lc_context_t *ctx);

/* Returns 1 if the expired step was handled here, 0 if it belongs to
 * another module. Also handles OVERRIDE_HOLD and FAILSAFE. */
int  fixed_timing_advance(lc_context_t *ctx);

#endif /* FIXED_TIMING_H */
