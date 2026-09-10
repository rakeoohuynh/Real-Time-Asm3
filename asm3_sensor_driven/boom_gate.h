/* =====================================================================
 * boom_gate.h -- Boom_Gate_Controller_Task and the gate's fault/retry
 * handling.
 *
 * The gate is commanded asynchronously. Phase_Controller_Task issues
 * COMMAND_BOOM_GATE and is replied to immediately with an acceptance;
 * the mechanical travel time then elapses on the gate task's own
 * thread, and the outcome comes back as a GATE_STATUS pulse. That
 * matters for the railway sequence: the 10s lowering interval is a
 * real countdown state ticking at 100ms, not a 10s block inside the
 * safety-critical control loop.
 * ===================================================================== */
#ifndef BOOM_GATE_H
#define BOOM_GATE_H

#include "lc_context.h"

typedef enum {
    GATE_EVT_NONE = 0,
    GATE_EVT_LOCKED,        /* close confirmed                        */
    GATE_EVT_OPENED,        /* open confirmed                         */
    GATE_EVT_CLOSE_RETRY,   /* close failed, attempts still available  */
    GATE_EVT_CLOSE_FAILED   /* close failed, attempts exhausted        */
} gate_event_t;

void *boom_gate_task(void *arg);

/* Start a fresh close sequence (resets the attempt counter). */
void boom_gate_begin_close(lc_context_t *ctx);

/* Re-issue the close after a timeout, consuming one attempt. */
void boom_gate_retry_close(lc_context_t *ctx);

/* Start raising the gate. */
void boom_gate_begin_open(lc_context_t *ctx);

/* Decode a PULSE_GATE_STATUS payload, update ctx->gate_state, and say
 * what the railway sequence should do about it. */
gate_event_t boom_gate_on_status(lc_context_t *ctx, int pulse_value);

int  boom_gate_attempts_used(void);

#endif /* BOOM_GATE_H */
