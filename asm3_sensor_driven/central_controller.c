/* =====================================================================
 * central_controller.c
 *
 * QNX Central Controller (CC) proof-of-concept.
 *
 * Design principle (Section 7): the CC monitors, logs, schedules and
 * issues high-level commands; it NEVER touches a signal head directly.
 * Every local controller (LC) retains sole execution authority over its
 * own intersection and keeps running autonomously if the CC or the
 * comms link disappears (see local_controller.c / UC-08).
 *
 * Task map:
 *   Central_Communication_Task -> comm_worker() (a small pool of
 *       threads on ONE shared channel, per UC-06's "CC runs a small
 *       pool of server threads on that channel so it can receive
 *       multiple parallel status instead of serialized status")
 *   Display/Logging            -> display_task()     (thread)
 *   Operator console           -> operator_console_task() (main thread)
 *
 * LC discovery: the CC does not need to be told where each LC lives.
 * The first STATUS_UPDATE/FAULT_ALARM received from an intersection
 * carries the sender's QNET node descriptor (via MsgInfo()); the CC
 * uses that to open an OVERRIDE_COMMAND/MODE_SWITCH connection back to
 * that same LC on demand, which is how a real multi-node QNET deployment
 * finds its peers without static configuration.
 *
 * Build (QNX qcc):
 *   qcc -Vgcc_ntox86_64 -o central_controller central_controller.c -lpthread -lsocket
 * Run:
 *   ./central_controller
 * Operator console commands (typed at the CC's terminal):
 *   status                       - print the last known status table
 *   override <id> <cmd> [seq]    - cmd: allred|nsgreen|ewgreen|dignitary
 *   mode <id> <fixed|sensor>     - request a mode switch
 *   quit
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/dispatch.h>
#include "common.h"

#define MAX_TRACKED_LC     6
#define COMM_POOL_THREADS  3

/* ---------------------------------------------------------------------
 * Per-intersection record, built up as STATUS_UPDATE/FAULT_ALARM arrive.
 * ------------------------------------------------------------------- */
typedef struct {
    int        in_use;
    int        id;
    int32_t    node_desc;         /* QNET node descriptor of the LC, from MsgInfo().nd */
    int        override_coid;     /* cached connection for sending OVERRIDE/MODE_SWITCH, -1 if none */

    lc_phase_t       phase;
    vehicle_state_t  ns_state, ew_state;
    ped_state_t      ped_state;
    control_mode_t   mode;
    uint32_t         fault_flags;
    int              last_fault_code;
    time_t           last_update;
    int              link_seen;   /* have we ever heard from this LC */
} lc_record_t;

static lc_record_t   g_lc[MAX_TRACKED_LC];
static pthread_mutex_t g_lc_lock = PTHREAD_MUTEX_INITIALIZER;
static int            g_cc_chid;
static uint32_t       g_seq_no = 1;

static const char *vname(vehicle_state_t s)
{
    switch (s) { case V_RED: return "RED"; case V_YELLOW: return "YELLOW"; default: return "GREEN"; }
}
static const char *mname(control_mode_t m)
{
    switch (m) { case MODE_FIXED_TIMING: return "FIXED_TIMING"; case MODE_SENSOR_DRIVEN: return "SENSOR_DRIVEN"; default: return "ADVANCED"; }
}
static const char *phname(lc_phase_t p)
{
    switch (p) {
        case LC_PHASE_NS: return "NS";  case LC_PHASE_EW: return "EW";
        case LC_PHASE_PED: return "PED"; case LC_PHASE_RAIL_PROTECT: return "RAIL_PROTECT";
        default: return "FAILSAFE";
    }
}
static const char *pname_local(ped_state_t s)
{
    switch (s) { case P_WALK: return "WALK"; case P_CLEARANCE: return "CLEARANCE"; default: return "DONT_WALK"; }
}

static lc_record_t *find_or_create(int id)
{
    for (int i = 0; i < MAX_TRACKED_LC; i++)
        if (g_lc[i].in_use && g_lc[i].id == id) return &g_lc[i];
    for (int i = 0; i < MAX_TRACKED_LC; i++) {
        if (!g_lc[i].in_use) {
            memset(&g_lc[i], 0, sizeof(g_lc[i]));
            g_lc[i].in_use = 1;
            g_lc[i].id = id;
            g_lc[i].override_coid = -1;
            return &g_lc[i];
        }
    }
    return NULL; /* registry full */
}

/* =========================================================================
 * Central_Communication_Task (thread pool)
 *   All threads MsgReceive() on the SAME channel; QNX round-robins
 *   pending messages across whichever thread calls MsgReceive() next,
 *   which is exactly the "small pool of server threads ... to receive
 *   multiple parallel status instead of serialized status" from UC-06.
 * ========================================================================= */
static void *comm_worker(void *arg)
{
    int worker_no = (int)(intptr_t)arg;
    for (;;) {
        net_report_t   msg;
        struct _msg_info info;
        int rcvid = MsgReceive(g_cc_chid, &msg, sizeof(msg), &info);
        if (rcvid <= 0) continue;  /* pulse -- ignore, nothing else uses this channel as a pulse source */

        net_reply_t ack; memset(&ack, 0, sizeof(ack));
        ack.result = NET_RESULT_ACCEPTED;
        MsgReply(rcvid, EOK, &ack, sizeof(ack));  /* UC-06: reply immediately, no processing delay */

        lc_record_t *rec;
        pthread_mutex_lock(&g_lc_lock);
        rec = find_or_create(msg.intersection_id);
        if (rec) {
            rec->node_desc   = info.nd;   /* remember where this LC lives, for future overrides */
            rec->phase       = msg.phase;
            rec->ns_state    = msg.ns_state;
            rec->ew_state    = msg.ew_state;
            rec->ped_state   = msg.ped_state;
            rec->mode        = msg.mode;
            rec->fault_flags = msg.fault_flags;
            rec->last_update = msg.timestamp;
            rec->link_seen   = 1;
            if (msg.type == NET_FAULT_ALARM) rec->last_fault_code = msg.fault_code;
        }
        pthread_mutex_unlock(&g_lc_lock);

        if (msg.type == NET_FAULT_ALARM) {
            printf("[CC][Central_Communication_Task#%d] *** FAULT_ALARM *** I%d fault=%d head=%d\n",
                   worker_no, msg.intersection_id, msg.fault_code, msg.head_id);
        } else {
            printf("[CC][Central_Communication_Task#%d] STATUS_UPDATE I%d phase=%s ns=%s ew=%s ped=%s mode=%s\n",
                   worker_no, msg.intersection_id, phname(msg.phase),
                   vname(msg.ns_state), vname(msg.ew_state), pname_local(msg.ped_state), mname(msg.mode));
        }
        fflush(stdout);
    }
    return NULL;
}

/* =========================================================================
 * display_task -- periodic console dashboard (CC "displays the status and
 * light settings received from the individual intersections").
 * ========================================================================= */
static void *display_task(void *arg)
{
    (void)arg;
    for (;;) {
        sleep(10);
        printf("\n===================== CC DASHBOARD =====================\n");
        pthread_mutex_lock(&g_lc_lock);
        for (int i = 0; i < MAX_TRACKED_LC; i++) {
            if (!g_lc[i].in_use) continue;
            lc_record_t *r = &g_lc[i];
            time_t age = time(NULL) - r->last_update;
            printf(" I%d  phase=%-11s ns=%-6s ew=%-6s ped=%-9s mode=%-13s faults=0x%02x  (%lds ago)\n",
                   r->id, phname(r->phase), vname(r->ns_state), vname(r->ew_state),
                   pname_local(r->ped_state), mname(r->mode), r->fault_flags, (long)age);
        }
        pthread_mutex_unlock(&g_lc_lock);
        printf("==========================================================\n\n");
        fflush(stdout);
    }
    return NULL;
}

/* =========================================================================
 * Operator-driven commands (UC-03 MODE_SWITCH, UC-07 OVERRIDE_COMMAND)
 * ========================================================================= */
static int connection_for(lc_record_t *rec)
{
    if (rec->override_coid >= 0) return rec->override_coid;

    char chan_name[32], path[160];
    lc_channel_name(chan_name, sizeof(chan_name), rec->id);

    /* NOTE: ND2S_LOCAL_STR / netmgr_ndtostr() signature can vary slightly
     * between QNX SDP header revisions -- check <sys/netmgr.h> on the
     * target toolchain (SDP 7.1 per project notes) and adjust the flag
     * name below if it does not match. */
    if (rec->link_seen && rec->node_desc != 0 && rec->node_desc != ND_LOCAL_NODE) {
        char nodestr[64];
        if (netmgr_ndtostr(ND2S_LOCAL_STR, rec->node_desc, nodestr, sizeof(nodestr)) > 0)
            snprintf(path, sizeof(path), "/net/%s/dev/name/local/%s", nodestr, chan_name);
        else
            snprintf(path, sizeof(path), "/dev/name/local/%s", chan_name);
    } else {
        snprintf(path, sizeof(path), "/dev/name/local/%s", chan_name);
    }

    rec->override_coid = name_open(path, 0);
    return rec->override_coid;
}

/* Sends the command, and follows the UC-07 "reject with wait=N -> CC
 * delays -> resend" flow exactly once (a real operator console would
 * loop/retry further; one retry is enough to demonstrate the protocol). */
static void send_command_with_retry(lc_record_t *rec, net_command_t *cmd)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        int coid = connection_for(rec);
        if (coid < 0) {
            printf("[CC][Operator_Console] cannot reach I%d (link not established)\n", rec->id);
            return;
        }
        net_reply_t reply;
        int rc = MsgSend(coid, cmd, sizeof(*cmd), &reply, sizeof(reply));
        if (rc == -1) {
            printf("[CC][Operator_Console] send to I%d failed (errno=%d) -- link may be down\n", rec->id, errno);
            rec->override_coid = -1; /* force reconnect next time */
            return;
        }
        if (reply.result == NET_RESULT_ACCEPTED) {
            printf("[CC][Operator_Console] I%d ACCEPTED command\n", rec->id);
            return;
        } else if (reply.result == NET_RESULT_WAIT) {
            printf("[CC][Operator_Console] I%d rejected: %s -- waiting %ds then resending (UC-07)\n",
                   rec->id, reply.reason, reply.wait_seconds);
            sleep(reply.wait_seconds);
            continue; /* resend once */
        } else {
            printf("[CC][Operator_Console] I%d rejected (invalid): %s\n", rec->id, reply.reason);
            return;
        }
    }
    printf("[CC][Operator_Console] I%d still not accepted after retry, giving up for now\n", rec->id);
}

static void do_override(int id, const char *cmd_name)
{
    pthread_mutex_lock(&g_lc_lock);
    lc_record_t *rec = find_or_create(id);
    pthread_mutex_unlock(&g_lc_lock);
    if (!rec) { printf("[CC] intersection registry full\n"); return; }

    int cmd_type;
    if      (!strcmp(cmd_name, "allred"))    cmd_type = OVR_FORCE_ALL_RED;
    else if (!strcmp(cmd_name, "nsgreen"))   cmd_type = OVR_FORCE_NS_GREEN;
    else if (!strcmp(cmd_name, "ewgreen"))   cmd_type = OVR_FORCE_EW_GREEN;
    else if (!strcmp(cmd_name, "dignitary")) cmd_type = OVR_DIGNITARY_PATH;
    else { printf("[CC] unknown override type '%s'\n", cmd_name); return; }

    net_command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = NET_OVERRIDE_COMMAND;
    cmd.intersection_id = id;
    cmd.command_type = cmd_type;
    cmd.sequence_no = g_seq_no++;

    printf("[CC][Operator_Console] sending OVERRIDE_COMMAND(%s) to I%d (seq=%u)\n", cmd_name, id, cmd.sequence_no);
    send_command_with_retry(rec, &cmd);
}

static void do_mode_switch(int id, const char *mode_name)
{
    pthread_mutex_lock(&g_lc_lock);
    lc_record_t *rec = find_or_create(id);
    pthread_mutex_unlock(&g_lc_lock);
    if (!rec) { printf("[CC] intersection registry full\n"); return; }

    control_mode_t mode;
    if      (!strcmp(mode_name, "fixed"))  mode = MODE_FIXED_TIMING;
    else if (!strcmp(mode_name, "sensor")) mode = MODE_SENSOR_DRIVEN;
    else { printf("[CC] unknown mode '%s'\n", mode_name); return; }

    net_command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = NET_MODE_SWITCH;
    cmd.intersection_id = id;
    cmd.requested_mode = mode;
    cmd.sequence_no = g_seq_no++;

    printf("[CC][Operator_Console] sending MODE_SWITCH(%s) to I%d (seq=%u)\n", mode_name, id, cmd.sequence_no);
    send_command_with_retry(rec, &cmd);
}

static void print_status_table(void)
{
    pthread_mutex_lock(&g_lc_lock);
    for (int i = 0; i < MAX_TRACKED_LC; i++) {
        if (!g_lc[i].in_use) continue;
        lc_record_t *r = &g_lc[i];
        printf(" I%d  phase=%-11s ns=%-6s ew=%-6s ped=%-9s mode=%-13s faults=0x%02x\n",
               r->id, phname(r->phase), vname(r->ns_state), vname(r->ew_state),
               pname_local(r->ped_state), mname(r->mode), r->fault_flags);
    }
    pthread_mutex_unlock(&g_lc_lock);
}

static void operator_console_task(void)
{
    printf("CC operator console ready. Commands:\n"
           "  status\n"
           "  override <id> <allred|nsgreen|ewgreen|dignitary>\n"
           "  mode <id> <fixed|sensor>\n"
           "  quit\n");
    char line[128];
    while (fgets(line, sizeof(line), stdin)) {
        char verb[32], a1[32], a2[32];
        int n = sscanf(line, "%31s %31s %31s", verb, a1, a2);
        if (n < 1) continue;
        if (!strcmp(verb, "quit")) break;
        else if (!strcmp(verb, "status")) print_status_table();
        else if (!strcmp(verb, "override") && n == 3) do_override(atoi(a1), a2);
        else if (!strcmp(verb, "mode") && n == 3) do_mode_switch(atoi(a1), a2);
        else printf("[CC] unrecognised command\n");
    }
}

int main(void)
{
    memset(g_lc, 0, sizeof(g_lc));
    for (int i = 0; i < MAX_TRACKED_LC; i++) g_lc[i].override_coid = -1;

    name_attach_t *na = name_attach(NULL, CC_CHANNEL_NAME, 0);
    if (!na) { perror("name_attach"); return 1; }
    g_cc_chid = na->chid;

    printf("=== Central Controller starting (channel '%s') ===\n", CC_CHANNEL_NAME);
    fflush(stdout);

    pthread_t th;
    for (int i = 0; i < COMM_POOL_THREADS; i++) {
        pthread_create(&th, NULL, comm_worker, (void *)(intptr_t)i);
        pthread_detach(th);
    }
    pthread_create(&th, NULL, display_task, NULL);
    pthread_detach(th);

    operator_console_task();  /* runs on main thread until "quit" */
    return 0;
}