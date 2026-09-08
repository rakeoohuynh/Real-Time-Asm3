/* =============================================================
 * status_reporting.h - Status_Reporting_Task
 *
 * EVENT_DRIVEN + BACKGROUND task (Assessment 2 Fig. 18) that forwards
 * phase-transition events to the Central Controller. It must never
 * block Phase_Controller_Task (the safety-critical FSM); it receives
 * STATUS notifications from the FSM via non-blocking MsgSendPulse and
 * forwards them to the CC the same way.
 *
 * If the Central Controller is unreachable, this task keeps the LC
 * running autonomously (Assessment 2 assumption A34) and retries the
 * connection quietly in the background - a simplified version of the
 * retry/backoff procedure in UC-08 (Central Controller Communication
 * Loss), scoped down for this fixed_time-only phase.
 * ============================================================= */
#ifndef STATUS_REPORTING_H
#define STATUS_REPORTING_H

#include "protocol.h"

typedef struct {
    intersection_id_t intersection_id;
    int                chid;     /* this task's own channel */
    const char        *cc_name;  /* Central Controller attach-point name (local or QNET path) */
} status_reporting_args_t;

/* pthread entry point. Runs until it receives PULSE_SHUTDOWN on its
 * own channel. */
void *status_reporting_task(void *arg);

#endif /* STATUS_REPORTING_H */
