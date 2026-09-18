/*
 * pedestrian.h -- Pedestrian_Input_Task and the WALK -> CLEARANCE ->
 * DONT_WALK sequence.
 */
#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

#include "lc_context.h"

/* Button press from the (simulated) push button. Debounced, then latched
 * until the crossing is served; extra presses meanwhile are ignored. */
void pedestrian_input_handle_press(lc_context_t *ctx);

int  pedestrian_request_pending(lc_context_t *ctx);

/* Starts WALK. Only call this at an all-red boundary. */
void pedestrian_begin_walk(lc_context_t *ctx);

/* Returns 1 if the expired step was a pedestrian step. */
int  pedestrian_advance(lc_context_t *ctx);

/* Scaled ms until WALK + CLEARANCE ends, 0 when no crossing is running.
 * Phase_Controller_Task only. */
int  pedestrian_remaining_ms(lc_context_t *ctx);

#endif /* PEDESTRIAN_H */
