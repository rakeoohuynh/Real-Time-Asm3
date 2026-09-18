/*
 * signal_output.h -- Signal_Output_Task: vehicle, pedestrian and
 * right-turn arrow heads.
 *
 * Arrows are separate heads with their own message, not a fourth color
 * of the main signal, so an approach can show a red main signal and a
 * green arrow at the same time.
 */
#ifndef SIGNAL_OUTPUT_H
#define SIGNAL_OUTPUT_H

#include "lc_context.h"

void *signal_output_task(void *arg);

/* Blocking calls from Phase_Controller_Task. dur_s is the real-world
 * duration and is only logged; the caller owns the countdown. Each
 * returns 0 or a FAULT_* code. */
int signal_set_vehicle(lc_context_t *ctx, int head_id, vehicle_state_t state, int dur_s);
int signal_set_pedestrian(lc_context_t *ctx, int crossing_id, ped_state_t state, int dur_s);
int signal_set_right_turn_arrow(lc_context_t *ctx, int head_id, arrow_state_t state, int dur_s);

#endif /* SIGNAL_OUTPUT_H */
