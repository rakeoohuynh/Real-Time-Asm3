/*
 * status_report.c -- Status_Reporting_Task.
 *
 * Status events are coalesced: however many arrive, the CC gets one
 * snapshot of the current state, so an outage doesn't leave a backlog of
 * stale reports to replay. Fault alarms are never dropped this way; each
 * distinct fault is kept until delivered, and faults go out first.
 *
 * Nothing here sleeps. Retry and back-off (UC-08) are deadlines the
 * receive loop waits on, so pulses are still drained during an outage.
 * Every send has a kernel timeout, so a CC that's up but not replying
 * can't stall reporting.
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include "status_report.h"

#define STATUS_SEND_TIMEOUT_MS   500    /* max time for one send to the CC */
#define STATUS_RESEND_GAP_MS    1000    /* gap between retries before the back-off */
#define FAULT_QUEUE_LEN            8

static int      g_cc_coid         = -1;
static int      g_central_link_up = 0;
static int      g_retry_counter   = 0;
static uint64_t g_next_attempt_ms = 0;    /* no send or reconnect before this */

static int      g_status_dirty    = 0;    /* state changed since the last report */
static struct { int code, head; } g_faults[FAULT_QUEUE_LEN];
static int      g_fault_count     = 0;

static int connect_to_cc(lc_context_t *ctx)
{
    char path[128];
    if (ctx->cc_node[0] != '\0')
        snprintf(path, sizeof(path), "/net/%s/dev/name/local/%s", ctx->cc_node, CC_CHANNEL_NAME);
    else
        snprintf(path, sizeof(path), "/dev/name/local/%s", CC_CHANNEL_NAME);

    return name_open(path, 0);
}

static int report_pending(void)
{
    return g_status_dirty || g_fault_count > 0;
}

/* Waits up to timeout_ms (-1 = forever) for a pulse. Returns 1 if one
 * arrived, 0 on timeout or anything else. */
static int receive_pulse(lc_context_t *ctx, long timeout_ms, struct _pulse *pulse)
{
    if (timeout_ms >= 0) {
        /* Arm a 0 ms poll as 1 ns so it doesn't rely on how the kernel
         * treats a zero timeout. */
        uint64_t ns = (timeout_ms > 0) ? (uint64_t)timeout_ms * 1000000ULL : 1;
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &ns, NULL);
    }
    int rcvid = MsgReceive(ctx->status_chid, pulse, sizeof(*pulse), NULL);
    if (rcvid == 0) return 1;
    if (rcvid > 0) MsgError(rcvid, ENOSYS);   /* this channel only takes pulses */
    return 0;
}

static void absorb_pulse(lc_context_t *ctx, const struct _pulse *pulse)
{
    if (pulse->code != PULSE_FAULT_ALARM) {
        g_status_dirty = 1;
        return;
    }

    int code = pulse->value.sival_int & 0xFF;
    int head = (pulse->value.sival_int >> 8) & 0xFF;
    printf("[I%d][Status_Reporting_Task] FAULT_ALARM code=%d head=%d -> CC\n", ctx->id, code, head);
    fflush(stdout);

    for (int i = 0; i < g_fault_count; i++)
        if (g_faults[i].code == code && g_faults[i].head == head) return;   /* already queued */

    if (g_fault_count == FAULT_QUEUE_LEN) {
        printf("[I%d][Status_Reporting_Task] fault queue full, FAULT_ALARM code=%d head=%d dropped "
               "(the fault still shows in fault_flags)\n", ctx->id, code, head);
        fflush(stdout);
        return;
    }
    g_faults[g_fault_count].code = code;
    g_faults[g_fault_count].head = head;
    g_fault_count++;
}

static void fill_snapshot(lc_context_t *ctx, net_report_t *rep)
{
    memset(rep, 0, sizeof(*rep));
    rep->intersection_id = ctx->id;

    pthread_mutex_lock(&ctx->lock);
    rep->phase       = ctx->phase;
    rep->ns_state    = ctx->ns_state;
    rep->ew_state    = ctx->ew_state;
    rep->ped_state   = ctx->ped_state;
    rep->ns_arrow    = ctx->ns_arrow;
    rep->ew_arrow    = ctx->ew_arrow;
    rep->gate_state  = ctx->gate_state;
    rep->mode        = ctx->mode;
    rep->fault_flags = ctx->fault_flags;
    pthread_mutex_unlock(&ctx->lock);
    rep->timestamp   = time(NULL);
}

/* Sends one report with a timeout. Returns 1 on success. On failure
 * (UC-08): retry after a short gap up to STATUS_MAX_RETRY times, then
 * mark the link lost, drop the connection and back off for
 * STATUS_RETRY_DELAY_S. */
static int send_report(lc_context_t *ctx, net_report_t *rep)
{
    net_reply_t reply;
    uint64_t ns = (uint64_t)STATUS_SEND_TIMEOUT_MS * 1000000ULL;
    TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &ns, NULL);

    if (MsgSend(g_cc_coid, rep, sizeof(*rep), &reply, sizeof(reply)) != -1) {
        if (!g_central_link_up) {
            printf("[I%d][Status_Reporting_Task] link to CC up\n", ctx->id);
            fflush(stdout);
        }
        g_central_link_up = 1;
        g_retry_counter = 0;
        return 1;
    }

    int err = errno;
    g_retry_counter++;
    printf("[I%d][Status_Reporting_Task] send to CC failed (attempt %d/%d, errno=%d%s)\n",
           ctx->id, g_retry_counter, STATUS_MAX_RETRY, err,
           (err == ETIMEDOUT) ? ": no reply in time" : "");

    if (g_retry_counter < STATUS_MAX_RETRY) {
        g_next_attempt_ms = lc_now_ms() + STATUS_RESEND_GAP_MS;
    } else {
        if (g_central_link_up)
            printf("[I%d][Status_Reporting_Task] link to CC LOST -- running on our own, will keep retrying\n", ctx->id);
        g_central_link_up = 0;
        g_retry_counter = 0;
        name_close(g_cc_coid);
        g_cc_coid = -1;
        g_next_attempt_ms = lc_now_ms() + (uint64_t)STATUS_RETRY_DELAY_S * 1000u;
    }
    fflush(stdout);
    return 0;
}

/* Sends faults first, then one current-state report. Stops at the first
 * failure and leaves the rest for the next attempt. */
static void flush_reports(lc_context_t *ctx)
{
    if (g_cc_coid < 0) {
        g_cc_coid = connect_to_cc(ctx);
        if (g_cc_coid < 0) {
            g_next_attempt_ms = lc_now_ms() + (uint64_t)STATUS_RETRY_DELAY_S * 1000u;
            return;
        }
        g_status_dirty = 1;   /* after reconnecting, send the current state (UC-08) */
    }

    net_report_t rep;
    while (g_fault_count > 0) {
        fill_snapshot(ctx, &rep);
        rep.type       = NET_FAULT_ALARM;
        rep.fault_code = g_faults[0].code;
        rep.head_id    = g_faults[0].head;
        if (!send_report(ctx, &rep)) return;

        memmove(&g_faults[0], &g_faults[1], (size_t)(g_fault_count - 1) * sizeof(g_faults[0]));
        g_fault_count--;
    }

    if (g_status_dirty) {
        fill_snapshot(ctx, &rep);
        rep.type = NET_STATUS_UPDATE;
        if (!send_report(ctx, &rep)) return;
        g_status_dirty = 0;
    }
}

void *status_reporting_task(void *arg)
{
    lc_context_t *ctx = arg;
    g_cc_coid = connect_to_cc(ctx);
    if (g_cc_coid < 0) {
        printf("[I%d][Status_Reporting_Task] CC not reachable yet, will keep retrying\n", ctx->id);
        fflush(stdout);
        g_next_attempt_ms = lc_now_ms() + (uint64_t)STATUS_RETRY_DELAY_S * 1000u;
    }

    for (;;) {
        struct _pulse pulse;

        /* With something pending, only wait until the next send attempt
         * is due. */
        long timeout_ms = -1;
        if (report_pending()) {
            uint64_t now = lc_now_ms();
            timeout_ms = (g_next_attempt_ms > now) ? (long)(g_next_attempt_ms - now) : 0;
        }
        if (timeout_ms != 0 && receive_pulse(ctx, timeout_ms, &pulse))
            absorb_pulse(ctx, &pulse);

        /* Drain the rest so it all goes out as one report. */
        while (receive_pulse(ctx, 0, &pulse))
            absorb_pulse(ctx, &pulse);

        if (report_pending() && lc_now_ms() >= g_next_attempt_ms)
            flush_reports(ctx);
    }
    return NULL;
}

void notify_status(lc_context_t *ctx, int is_fault, int fault_code, int head_id)
{
    int code = is_fault ? PULSE_FAULT_ALARM : PULSE_STATUS_EVENT;
    int val  = is_fault ? ((fault_code & 0xFF) | ((head_id & 0xFF) << 8)) : 0;

    /* If Status_Reporting_Task is gone this fails with ESRCH. Log it
     * rather than let the report vanish silently. */
    if (MsgSendPulse(ctx->coid_status, SIGEV_PULSE_PRIO_INHERIT, code, val) == -1) {
        printf("[I%d][Phase_Controller_Task] could not queue report (errno=%d%s) -- report dropped\n",
               ctx->id, errno, (errno == ESRCH) ? ", Status_Reporting_Task is gone" : "");
        fflush(stdout);
    }
}

void probe_cc_connectivity(lc_context_t *ctx)
{
    const char *target = ctx->cc_node[0] ? ctx->cc_node : "<same node>";
    printf("[I%d] checking connection to CC (node: %s) ...\n", ctx->id, target);
    fflush(stdout);

    const int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        int probe = connect_to_cc(ctx);
        if (probe >= 0) {
            printf("[I%d] CC REACHABLE (attempt %d/%d) -- opened channel '%s' over %s\n",
                   ctx->id, attempt, max_attempts, CC_CHANNEL_NAME,
                   ctx->cc_node[0] ? "QNET" : "same node");
            fflush(stdout);
            name_close(probe);
            return;
        }
        printf("[I%d] CC not reachable yet (attempt %d/%d, errno=%d: %s)\n",
               ctx->id, attempt, max_attempts, errno, strerror(errno));
        fflush(stdout);
        if (attempt < max_attempts) sleep(1);
    }

    printf("[I%d] *** CC NOT REACHABLE after %d attempts ***\n"
           "     Check that the CC is running, both machines are on the same network,\n"
           "     QNET (lsm-qnet.so) is running on both, and the node name '%s' is right:\n"
           "     `ls /net/%s/dev/name/local/` on this machine should list the CC's channel.\n"
           "     The LC starts anyway and keeps trying to reach the CC in the background.\n",
           ctx->id, max_attempts, target, target);
    fflush(stdout);
}
