/*
 * railway_protection.h -- Train_Sensor_Task, Railway_Signal_Output_Task
 * and the railway protection sequence (UC-05).
 *
 * Railway protection preempts whatever is running (A44). The phase table
 * allows no movement during it, which also withdraws the right-turn
 * arrows and keeps CC overrides from being applied.
 *
 * Nominal 80s closure:
 *      3s  RAIL_YELLOW        clear the intersection
 *      2s  RAIL_ALLRED
 *     20s  RAIL_PREARRIVAL    quiet hold
 *     15s  RAIL_WARN          crossing lights flashing
 *     10s  RAIL_GATE_LOWER
 *          (50s so far: the lead before the train arrives)
 *     25s  RAIL_OCCUPIED      train on the crossing
 *      5s  RAIL_POST_HOLD     gate stays down
 *     then RAIL_GATE_RAISE until the gate reports open
 */
#ifndef RAILWAY_PROTECTION_H
#define RAILWAY_PROTECTION_H

#include "lc_context.h"

void *railway_signal_task(void *arg);

/* Train sensor events, from the timetable or the 't'/'c' keys. */
void train_sensor_handle_approach(lc_context_t *ctx);
void train_sensor_handle_cleared(lc_context_t *ctx);

void railway_begin_protection(lc_context_t *ctx);
void railway_handle_cleared(lc_context_t *ctx);
int  railway_advance(lc_context_t *ctx);                 /* 1 if the step was ours */
void railway_on_gate_status(lc_context_t *ctx, int pulse_value);

int  railway_is_active(lc_context_t *ctx);

/* Scaled ms until the whole protection sequence should be over: the
 * current countdown plus the nominal length of every remaining step.
 * 0 when protection isn't running. Phase_Controller_Task only. */
int  railway_remaining_ms(lc_context_t *ctx);

#endif /* RAILWAY_PROTECTION_H */
