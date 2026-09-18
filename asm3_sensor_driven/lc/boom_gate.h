/*
 * boom_gate.h -- Boom_Gate_Controller_Task and gate retry handling.
 *
 * Gate commands are asynchronous: the gate task accepts the command and
 * replies at once, simulates the travel time on its own thread, then
 * reports the result as a PULSE_GATE_STATUS. That keeps the 10s lowering
 * out of the control loop; the railway sequence just counts down while
 * it waits.
 */
#ifndef BOOM_GATE_H
#define BOOM_GATE_H

#include "lc_context.h"

typedef enum {
    GATE_EVT_NONE = 0,
    GATE_EVT_LOCKED,        /* close confirmed */
    GATE_EVT_OPENED,        /* open finished (gate_state says whether it worked) */
    GATE_EVT_CLOSE_RETRY,   /* close failed, retries left */
    GATE_EVT_CLOSE_FAILED   /* close failed, no retries left */
} gate_event_t;

void *boom_gate_task(void *arg);

/* Starts a new close attempt sequence (resets the attempt count). */
void boom_gate_begin_close(lc_context_t *ctx);

/* Tries the close again, using up one attempt. */
void boom_gate_retry_close(lc_context_t *ctx);

void boom_gate_begin_open(lc_context_t *ctx);

/* Decodes a PULSE_GATE_STATUS value, updates gate_state and the fault
 * flag, and tells the railway sequence what happened. */
gate_event_t boom_gate_on_status(lc_context_t *ctx, int pulse_value);

int  boom_gate_attempts_used(void);

#endif /* BOOM_GATE_H */
