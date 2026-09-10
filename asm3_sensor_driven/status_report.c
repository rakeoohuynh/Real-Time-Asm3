/* =====================================================================
 * status_report.c -- Status_Reporting_Task.
 * ===================================================================== */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include "status_report.h"

static int  g_cc_coid       = -1;   /* connection to CC, owned by this task */
static int  g_central_link_up = 0;
static int  g_retry_counter   = 0;

static int connect_to_cc(lc_context_t *ctx)
{
    char path[128];
    if (ctx->cc_node[0] != '\0')
        snprintf(path, sizeof(path), "/net/%s/dev/name/local/%s", ctx->cc_node, CC_CHANNEL_NAME);
    else
        snprintf(path, sizeof(path), "/dev/name/local/%s", CC_CHANNEL_NAME);

    return name_open(path, 0);   /* -1 on failure, errno set */
}

static void send_report(lc_context_t *ctx, net_report_t *rep)
{
    net_reply_t reply;
    int sent_ok = 0;

    if (g_cc_coid >= 0) {
        int rc = MsgSend(g_cc_coid, rep, sizeof(*rep), &reply, sizeof(reply));
        sent_ok = (rc != -1);
    }

    if (sent_ok) {
        if (!g_central_link_up) {
            printf("[I%d][Status_Reporting_Task] link to CC RESTORED\n", ctx->id);
            fflush(stdout);
        }
        g_central_link_up = 1;
        g_retry_counter = 0;
        return;
    }

    /* --- UC-08: retry / back-off / autonomous continuation --- */
    g_retry_counter++;
    printf("[I%d][Status_Reporting_Task] send to CC failed (attempt %d)\n", ctx->id, g_retry_counter);
    if (g_retry_counter >= STATUS_MAX_RETRY) {
        if (g_central_link_up)
            printf("[I%d][Status_Reporting_Task] central_link = LOST -- continuing autonomously\n", ctx->id);
        g_central_link_up = 0;
        sleep(STATUS_RETRY_DELAY_S);
        g_retry_counter = 0;
        if (g_cc_coid >= 0) { ConnectDetach(g_cc_coid); g_cc_coid = -1; }
        g_cc_coid = connect_to_cc(ctx);
        if (g_cc_coid >= 0) {
            /* resync with current state, not the missed report, per UC-08 spec */
            MsgSend(g_cc_coid, rep, sizeof(*rep), &reply, sizeof(reply));
        }
    }
}

void *status_reporting_task(void *arg)
{
    lc_context_t *ctx = arg;
    g_cc_coid = connect_to_cc(ctx);
    if (g_cc_coid < 0)
        printf("[I%d][Status_Reporting_Task] CC not reachable yet, will retry in background\n", ctx->id);

    for (;;) {
        struct _pulse pulse;
        int rcvid = MsgReceive(ctx->status_chid, &pulse, sizeof(pulse), NULL);
        if (rcvid != 0) continue;  /* only pulses expected on this channel */

        net_report_t rep;
        memset(&rep, 0, sizeof(rep));
        rep.intersection_id = ctx->id;

        pthread_mutex_lock(&ctx->lock);
        rep.phase       = ctx->phase;
        rep.ns_state    = ctx->ns_state;
        rep.ew_state    = ctx->ew_state;
        rep.ped_state   = ctx->ped_state;
        rep.ns_arrow    = ctx->ns_arrow;
        rep.ew_arrow    = ctx->ew_arrow;
        rep.gate_state  = ctx->gate_state;
        rep.mode        = ctx->mode;
        rep.fault_flags = ctx->fault_flags;
        pthread_mutex_unlock(&ctx->lock);
        rep.timestamp   = time(NULL);

        if (pulse.code == PULSE_FAULT_ALARM) {
            rep.type       = NET_FAULT_ALARM;
            rep.fault_code = pulse.value.sival_int & 0xFF;
            rep.head_id    = (pulse.value.sival_int >> 8) & 0xFF;
            printf("[I%d][Status_Reporting_Task] FAULT_ALARM code=%d head=%d -> CC\n",
                   ctx->id, rep.fault_code, rep.head_id);
        } else { /* PULSE_STATUS_EVENT */
            rep.type = NET_STATUS_UPDATE;
        }
        fflush(stdout);
        send_report(ctx, &rep);
    }
    return NULL;
}

void notify_status(lc_context_t *ctx, int is_fault, int fault_code, int head_id)
{
    int code = is_fault ? PULSE_FAULT_ALARM : PULSE_STATUS_EVENT;
    int val  = is_fault ? ((fault_code & 0xFF) | ((head_id & 0xFF) << 8)) : 0;

    /* Check the kernel-level return here, at the call site. A dead
     * Status_Reporting_Task channel returns -1/ESRCH, and silently
     * dropping the report would leave the controller believing the CC
     * had been told. */
    if (MsgSendPulse(ctx->coid_status, SIGEV_PULSE_PRIO_INHERIT, code, val) == -1) {
        printf("[I%d][Phase_Controller_Task] STATUS pulse failed (errno=%d%s) -- report dropped\n",
               ctx->id, errno, (errno == ESRCH) ? ": ESRCH, Status_Reporting_Task gone" : "");
        fflush(stdout);
    }
}

void probe_cc_connectivity(lc_context_t *ctx)
{
    const char *target = ctx->cc_node[0] ? ctx->cc_node : "<same node>";
    printf("[I%d] checking connectivity to CC (target node: %s) ...\n", ctx->id, target);
    fflush(stdout);

    const int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        int probe = connect_to_cc(ctx);
        if (probe >= 0) {
            printf("[I%d] CC REACHABLE (attempt %d/%d) -- channel '%s' resolved OK over %s\n",
                   ctx->id, attempt, max_attempts, CC_CHANNEL_NAME,
                   ctx->cc_node[0] ? "QNET" : "same node");
            fflush(stdout);
            ConnectDetach(probe);
            return;
        }
        printf("[I%d] CC not reachable yet (attempt %d/%d, errno=%d: %s)\n",
               ctx->id, attempt, max_attempts, errno, strerror(errno));
        fflush(stdout);
        if (attempt < max_attempts) sleep(1);
    }

    printf("[I%d] *** CC NOT REACHABLE after %d attempts ***\n"
           "     Check: CC process running? Same WiFi/subnet? QNET (io-pkt + npm-qnet.so)\n"
           "     mounted on both machines? Node name '%s' correct (try `ls /net/%s/dev/name/local/`\n"
           "     from a shell on this machine)? -- LC will still start and operate autonomously;\n"
           "     Status_Reporting_Task keeps retrying in the background (UC-08).\n",
           ctx->id, max_attempts, target, target);
    fflush(stdout);
}
