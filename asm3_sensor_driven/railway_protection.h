/* =====================================================================
 * railway_protection.h -- Train_Sensor_Task, Railway_Signal_Output_Task
 * and the UC-05 railway protection sequence.
 *
 * Railway protection outranks ordinary vehicle, pedestrian and
 * right-turn operation (A44): it preempts whatever step is running, and
 * for as long as it is active the phase table permits no movement at
 * all, which is what withdraws the right-turn arrows and blocks a CC
 * override from being applied.
 *
 * The 80s nominal closure:
 *      3s  RAIL_YELLOW        clearing the intersection
 *      2s  RAIL_ALLRED        clearance interval
 *     20s  RAIL_PREARRIVAL    quiet protection hold
 *     15s  RAIL_WARN          flashing crossing warning       (A15)
 *     10s  RAIL_GATE_LOWER    boom gate in motion             (A16)
 *     ---- 50s pre-arrival protection window                  (A14)
 *     25s  RAIL_OCCUPIED      train on the crossing           (A18)
 *      5s  RAIL_POST_HOLD     gate stays down after the train (A17)
 *     ---- 80s total
 * ===================================================================== */
#ifndef RAILWAY_PROTECTION_H
#define RAILWAY_PROTECTION_H

#include "lc_context.h"

/* Railway_Signal_Output_Task (its own thread). */
void *railway_signal_task(void *arg);

/* Train_Sensor_Task events. Called by the timetable scheduler, and by
 * the console for a manual demo. */
void train_sensor_handle_approach(lc_context_t *ctx);
void train_sensor_handle_cleared(lc_context_t *ctx);

/* Dispatch-loop entry points. */
void railway_begin_protection(lc_context_t *ctx);
void railway_handle_cleared(lc_context_t *ctx);
int  railway_advance(lc_context_t *ctx);                 /* 1 if handled */
void railway_on_gate_status(lc_context_t *ctx, int pulse_value);

/* Is any step of the protection sequence currently running. */
int  railway_is_active(lc_context_t *ctx);

/* Scaled ms until the whole protection sequence should end: the current
 * countdown plus the nominal length of every step still to come. 0 when
 * protection is not active. Phase_Controller_Task only. */
int  railway_remaining_ms(lc_context_t *ctx);

#endif /* RAILWAY_PROTECTION_H */
