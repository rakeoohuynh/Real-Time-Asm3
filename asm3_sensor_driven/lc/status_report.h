/*
 * status_report.h -- Status_Reporting_Task, the LC's side of the link to
 * the CC.
 *
 * Every send to the CC happens on this task's thread, so the control
 * loop never waits on the network and the lights keep running if the CC
 * or the link goes away (UC-08).
 */
#ifndef STATUS_REPORT_H
#define STATUS_REPORT_H

#include "lc_context.h"

void *status_reporting_task(void *arg);

/* Queues a status update (or a fault alarm) for the CC. Just sends a
 * pulse; never waits on the CC. */
void  notify_status(lc_context_t *ctx, int is_fault, int fault_code, int head_id);

/* Startup check that the CC's channel can be opened. Diagnostic only:
 * the LC starts either way. */
void  probe_cc_connectivity(lc_context_t *ctx);

#endif /* STATUS_REPORT_H */
