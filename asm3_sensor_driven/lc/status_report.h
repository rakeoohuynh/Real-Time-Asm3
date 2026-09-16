/* =====================================================================
 * status_report.h -- Status_Reporting_Task and the LC's side of the
 * link to the Central Controller.
 *
 * Every CC-facing send lives on this task's own thread, which is what
 * keeps Phase_Controller_Task free of CC I/O and lets the LC keep
 * running lights when the CC or the link is gone (UC-08).
 * ===================================================================== */
#ifndef STATUS_REPORT_H
#define STATUS_REPORT_H

#include "lc_context.h"

void *status_reporting_task(void *arg);

/* Phase_Controller_Task -> Status_Reporting_Task, non-blocking pulse.
 * Never waits for a reply; the CC is not in this call path at all. */
void  notify_status(lc_context_t *ctx, int is_fault, int fault_code, int head_id);

/* One-shot startup connectivity self-check (diagnostic only). */
void  probe_cc_connectivity(lc_context_t *ctx);

#endif /* STATUS_REPORT_H */
