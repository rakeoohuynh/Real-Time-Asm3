/* =====================================================================
 * fixed_timing.h -- fixed-timing phase sequencing (peak-hour mode).
 *
 * The NS/EW GREEN -> YELLOW -> ALL-RED cycle, and the entry points the
 * other modules use to hand control back to ordinary vehicle operation
 * after a pedestrian or railway interruption.
 * ===================================================================== */
#ifndef FIXED_TIMING_H
#define FIXED_TIMING_H

#include "lc_context.h"

void fixed_timing_begin_ns_green(lc_context_t *ctx);
void fixed_timing_begin_ew_green(lc_context_t *ctx);

/* Handle the expiry of one of the vehicle steps. Returns 1 if it
 * recognised and handled the current step, 0 if the step belongs to
 * another module. */
int  fixed_timing_advance(lc_context_t *ctx);

#endif /* FIXED_TIMING_H */
