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
 *   Command delivery           -> command_worker()   (thread) -- sends
 *       queued OVERRIDE_COMMAND/MODE_SWITCH and waits out NET_RESULT_WAIT
 *       retries, so the operator console never blocks on an LC
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
 *   status                       - print the dashboard table now
 *   override <id> <cmd> [seq]    - cmd: allred|nsgreen|ewgreen|dignitary
 *   mode <id> <fixed|sensor>     - request a mode switch
 *   verbose <on|off>             - log every STATUS_UPDATE, or (default)
 *                                  only phase/mode/fault changes
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
#define DASHBOARD_PERIOD_S 10

/* An LC reports only on a state change, so silence is normal for up to
 * one step. The longest ordinary step is a vehicle GREEN; a link is
 * shown STALE after twice that (scaled) plus a margin. */
#define CC_LINK_STALE_S    ((2 * VEHICLE_GREEN_S) / TIME_SCALE_FACTOR + 5)

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
    arrow_state_t    ns_arrow, ew_arrow;
    gate_state_t     gate_state;
    control_mode_t   mode;
    uint32_t         fault_flags;
    int              last_fault_code;
    time_t           last_update; /* CC's clock at receipt, not the LC's timestamp */
    int              link_seen;   /* have we ever heard from this LC */
} lc_record_t;

static lc_record_t   g_lc[MAX_TRACKED_LC];
static pthread_mutex_t g_lc_lock = PTHREAD_MUTEX_INITIALIZER;
static int            g_cc_chid;
static uint32_t       g_seq_no = 1;
static volatile int   g_verbose = 0;   /* "verbose on": log every STATUS_UPDATE */

static const char *vname(vehicle_state_t s)
{
    switch (s) { case V_RED: return "RED"; case V_YELLOW: return "YELLOW"; default: return "GREEN"; }
}
static const char *mname(control_mode_t m)
{
    switch (m) { case MODE_FIXED_TIMING: return "FIXED"; case MODE_SENSOR_DRIVEN: return "SENSOR"; default: return "ADVANCED"; }
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
/* Right-turn arrows are reported separately from the main 3-colour
 * state, so the dashboard can show a green arrow beside a red main:
 * "RED+ARROW". */
static const char *vehicle_label(vehicle_state_t s, arrow_state_t a, char *buf, size_t len)
{
    snprintf(buf, len, "%s%s", vname(s), (a == ARROW_GREEN) ? "+ARROW" : "");
    return buf;
}
static const char *gname(gate_state_t g)
{
    switch (g) {
        case GATE_OPEN:         return "OPEN";
        case GATE_LOWERING:     return "LOWERING";
        case GATE_LOCKED:       return "LOCKED";
        case GATE_RAISING:      return "RAISING";
        default:                return "FAULT";
    }
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

/* Wall-clock HH:MM:SS for log and dashboard stamps. */
static const char *clock_str(char *buf, size_t len)
{
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    strftime(buf, len, "%H:%M:%S", &lt);
    return buf;
}

/* Log one received report. A FAULT_ALARM is always shown. A
 * STATUS_UPDATE is shown in full only in verbose mode; otherwise only
 * first contact and changes of phase, mode or faults are logged. */
static void log_report(int worker_no, const net_report_t *m, int first_contact,
                       lc_phase_t old_phase, control_mode_t old_mode, uint32_t old_faults)
{
    char ts[16], ns[16], ew[16], f_old[64], f_new[64];
    clock_str(ts, sizeof(ts));
    vehicle_label(m->ns_state, m->ns_arrow, ns, sizeof(ns));
    vehicle_label(m->ew_state, m->ew_arrow, ew, sizeof(ew));
    fault_flags_str(m->fault_flags, f_new, sizeof(f_new));

    if (m->type == NET_FAULT_ALARM) {
        printf("[CC %s] *** FAULT_ALARM I%d: %s (head %d) ***\n",
               ts, m->intersection_id, fault_name(m->fault_code), m->head_id);
    } else if (g_verbose) {
        printf("[CC %s][Central_Communication_Task#%d] STATUS_UPDATE I%d phase=%s ns=%s ew=%s ped=%s gate=%s mode=%s faults=%s\n",
               ts, worker_no, m->intersection_id, phname(m->phase), ns, ew,
               pname_local(m->ped_state), gname(m->gate_state), mname(m->mode), f_new);
    } else if (first_contact) {
        printf("[CC %s] I%d connected: phase=%s NS:%s EW:%s mode=%s\n",
               ts, m->intersection_id, phname(m->phase), ns, ew, mname(m->mode));
    } else {
        if (m->phase != old_phase)
            printf("[CC %s] I%d phase %s -> %s  (NS:%s EW:%s gate=%s)\n",
                   ts, m->intersection_id, phname(old_phase), phname(m->phase),
                   ns, ew, gname(m->gate_state));
        if (m->mode != old_mode)
            printf("[CC %s] I%d mode %s -> %s\n",
                   ts, m->intersection_id, mname(old_mode), mname(m->mode));
        if (m->fault_flags != old_faults)
            printf("[CC %s] I%d faults %s -> %s\n",
                   ts, m->intersection_id, fault_flags_str(old_faults, f_old, sizeof(f_old)), f_new);
    }
    fflush(stdout);
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

        lc_record_t   *rec;
        int            first_contact = 0;
        lc_phase_t     old_phase  = LC_PHASE_NS;
        control_mode_t old_mode   = MODE_FIXED_TIMING;
        uint32_t       old_faults = 0;

        pthread_mutex_lock(&g_lc_lock);
        rec = find_or_create(msg.intersection_id);
        if (rec) {
            first_contact = !rec->link_seen;
            old_phase     = rec->phase;
            old_mode      = rec->mode;
            old_faults    = rec->fault_flags;

            rec->node_desc   = info.nd;   /* remember where this LC lives, for future overrides */
            rec->phase       = msg.phase;
            rec->ns_state    = msg.ns_state;
            rec->ew_state    = msg.ew_state;
            rec->ped_state   = msg.ped_state;
            rec->ns_arrow    = msg.ns_arrow;
            rec->ew_arrow    = msg.ew_arrow;
            rec->gate_state  = msg.gate_state;
            rec->mode        = msg.mode;
            rec->fault_flags = msg.fault_flags;
            rec->last_update = time(NULL);   /* the LC's clock may differ from ours */
            rec->link_seen   = 1;
            if (msg.type == NET_FAULT_ALARM) rec->last_fault_code = msg.fault_code;
        }
        pthread_mutex_unlock(&g_lc_lock);

        log_report(worker_no, &msg, first_contact, old_phase, old_mode, old_faults);
    }
    return NULL;
}

/* =========================================================================
 * Dashboard -- the CC "displays the status and light settings received
 * from the individual intersections". One row per intersection; LINK is
 * OK/STALE by the age of its last report (CC_LINK_STALE_S).
 * ========================================================================= */
#define DASH_ROW_FMT "%-4s %-13s %-12s %-12s %-10s %-9s %-7s %-10s %s\n"

static void print_dashboard(void)
{
    char ts[16];
    printf("\n======================== CC DASHBOARD %s ========================\n",
           clock_str(ts, sizeof(ts)));
    printf(DASH_ROW_FMT, "ID", "PHASE", "NS", "EW", "PED", "GATE", "MODE", "LINK", "FAULTS");

    int rows = 0;
    time_t now = time(NULL);
    pthread_mutex_lock(&g_lc_lock);
    for (int i = 0; i < MAX_TRACKED_LC; i++) {
        if (!g_lc[i].in_use) continue;
        lc_record_t *r = &g_lc[i];
        rows++;

        char id[8];
        snprintf(id, sizeof(id), "I%d", r->id);
        if (!r->link_seen) {
            printf("%-4s (no STATUS_UPDATE received yet)\n", id);
            continue;
        }

        char ns[16], ew[16], link[16], faults[64];
        long age = (long)(now - r->last_update);
        snprintf(link, sizeof(link), "%s %lds", (age > CC_LINK_STALE_S) ? "STALE" : "OK", age);
        printf(DASH_ROW_FMT, id, phname(r->phase),
               vehicle_label(r->ns_state, r->ns_arrow, ns, sizeof(ns)),
               vehicle_label(r->ew_state, r->ew_arrow, ew, sizeof(ew)),
               pname_local(r->ped_state), gname(r->gate_state), mname(r->mode),
               link, fault_flags_str(r->fault_flags, faults, sizeof(faults)));
    }
    pthread_mutex_unlock(&g_lc_lock);

    if (rows == 0) printf("(no intersections yet -- waiting for a Local Controller)\n");
    printf("=================================================================================\n\n");
    fflush(stdout);
}

static void *display_task(void *arg)
{
    (void)arg;
    for (;;) {
        sleep(DASHBOARD_PERIOD_S);
        print_dashboard();
    }
    return NULL;
}

/* =========================================================================
 * Operator-driven commands (UC-03 MODE_SWITCH, UC-07 OVERRIDE_COMMAND)
 *
 * The console only queues a command. command_worker() delivers it with a
 * bounded MsgSend and, on NET_RESULT_WAIT, re-queues it for after the
 * wait the LC asked for -- so the console stays usable meanwhile, and a
 * hung LC or dead QNET link cannot freeze it.
 * ========================================================================= */
#define CMD_QUEUE_LEN         8
#define CMD_MAX_ATTEMPTS      4      /* initial send + 3 resends after WAIT */
#define CMD_SEND_TIMEOUT_MS   2000   /* bound on send + reply to one LC     */

typedef struct {
    int           in_use;
    net_command_t cmd;
    int           attempts;
    uint64_t      not_before_ms;     /* now_ms() before which it is not sent */
} pending_cmd_t;

static pending_cmd_t   g_cmdq[CMD_QUEUE_LEN];
static pthread_mutex_t g_cmdq_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cmdq_cond;  /* CLOCK_MONOTONIC, initialised in main() */

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static const char *cmd_label(const net_command_t *cmd)
{
    return (cmd->type == NET_OVERRIDE_COMMAND) ? "OVERRIDE_COMMAND" : "MODE_SWITCH";
}

/* Caller holds g_cmdq_lock. A queued command of the same type for the
 * same intersection, so a newer one can replace it. */
static pending_cmd_t *find_queued_locked(const net_command_t *cmd)
{
    for (int i = 0; i < CMD_QUEUE_LEN; i++)
        if (g_cmdq[i].in_use && g_cmdq[i].cmd.type == cmd->type &&
            g_cmdq[i].cmd.intersection_id == cmd->intersection_id)
            return &g_cmdq[i];
    return NULL;
}

static void enqueue_command(const net_command_t *cmd)
{
    pthread_mutex_lock(&g_cmdq_lock);
    pending_cmd_t *slot = find_queued_locked(cmd);
    if (slot) {
        printf("[CC][Operator_Console] %s seq=%u for I%d superseded by seq=%u\n",
               cmd_label(cmd), slot->cmd.sequence_no, cmd->intersection_id, cmd->sequence_no);
    } else {
        for (int i = 0; i < CMD_QUEUE_LEN && !slot; i++)
            if (!g_cmdq[i].in_use) slot = &g_cmdq[i];
    }
    if (slot) {
        slot->in_use        = 1;
        slot->cmd           = *cmd;
        slot->attempts      = 0;
        slot->not_before_ms = 0;
        pthread_cond_signal(&g_cmdq_cond);
    } else {
        printf("[CC][Operator_Console] command queue full, %s to I%d dropped\n",
               cmd_label(cmd), cmd->intersection_id);
    }
    pthread_mutex_unlock(&g_cmdq_lock);
}

/* Put a command back for a later resend, unless the operator has queued
 * a newer one of the same kind meanwhile -- the newer one wins. */
static void requeue_command(const pending_cmd_t *job)
{
    pthread_mutex_lock(&g_cmdq_lock);
    pending_cmd_t *slot = find_queued_locked(&job->cmd);
    if (slot) {
        printf("[CC][Command_Worker] I%d: newer %s already queued, not resending seq=%u\n",
               job->cmd.intersection_id, cmd_label(&job->cmd), job->cmd.sequence_no);
        slot = NULL;
    } else {
        for (int i = 0; i < CMD_QUEUE_LEN && !slot; i++)
            if (!g_cmdq[i].in_use) slot = &g_cmdq[i];
        if (slot) {
            *slot = *job;
            slot->in_use = 1;
            pthread_cond_signal(&g_cmdq_cond);
        } else {
            printf("[CC][Command_Worker] command queue full, resend of %s to I%d dropped\n",
                   cmd_label(&job->cmd), job->cmd.intersection_id);
        }
    }
    pthread_mutex_unlock(&g_cmdq_lock);
}

/* Block until a queued command is due, then remove and return it. */
static pending_cmd_t dequeue_due_command(void)
{
    pending_cmd_t job;

    pthread_mutex_lock(&g_cmdq_lock);
    for (;;) {
        pending_cmd_t *next = NULL;
        for (int i = 0; i < CMD_QUEUE_LEN; i++)
            if (g_cmdq[i].in_use && (!next || g_cmdq[i].not_before_ms < next->not_before_ms))
                next = &g_cmdq[i];

        if (!next) {
            pthread_cond_wait(&g_cmdq_cond, &g_cmdq_lock);
            continue;
        }
        uint64_t now = now_ms();
        if (next->not_before_ms <= now) {
            job = *next;
            next->in_use = 0;
            break;
        }
        struct timespec until;
        clock_gettime(CLOCK_MONOTONIC, &until);
        uint64_t wait_ms = next->not_before_ms - now;
        until.tv_sec  += (time_t)(wait_ms / 1000);
        until.tv_nsec += (long)(wait_ms % 1000) * 1000000L;
        if (until.tv_nsec >= 1000000000L) { until.tv_sec++; until.tv_nsec -= 1000000000L; }
        pthread_cond_timedwait(&g_cmdq_cond, &g_cmdq_lock, &until);
    }
    pthread_mutex_unlock(&g_cmdq_lock);
    return job;
}

/* Cached command connection to an LC, opening it if needed. name_open()
 * can take a while over QNET, so it runs without g_lc_lock held; only
 * command_worker() opens or drops these connections. */
static int connection_for(int id)
{
    pthread_mutex_lock(&g_lc_lock);
    lc_record_t *rec = find_or_create(id);
    if (!rec) { pthread_mutex_unlock(&g_lc_lock); return -1; }
    int     coid      = rec->override_coid;
    int     link_seen = rec->link_seen;
    int32_t node_desc = rec->node_desc;
    pthread_mutex_unlock(&g_lc_lock);

    if (coid >= 0) return coid;

    char chan_name[32], path[160];
    lc_channel_name(chan_name, sizeof(chan_name), id);

    /* NOTE: ND2S_LOCAL_STR / netmgr_ndtostr() signature can vary slightly
     * between QNX SDP header revisions -- check <sys/netmgr.h> on the
     * target toolchain (SDP 7.1 per project notes) and adjust the flag
     * name below if it does not match. */
    if (link_seen && node_desc != 0 && node_desc != ND_LOCAL_NODE) {
        char nodestr[64];
        if (netmgr_ndtostr(ND2S_LOCAL_STR, node_desc, nodestr, sizeof(nodestr)) > 0)
            snprintf(path, sizeof(path), "/net/%s/dev/name/local/%s", nodestr, chan_name);
        else
            snprintf(path, sizeof(path), "/dev/name/local/%s", chan_name);
    } else {
        snprintf(path, sizeof(path), "/dev/name/local/%s", chan_name);
    }

    coid = name_open(path, 0);
    if (coid >= 0) {
        pthread_mutex_lock(&g_lc_lock);
        rec->override_coid = coid;   /* rec is a stable slot in g_lc[] */
        pthread_mutex_unlock(&g_lc_lock);
    }
    return coid;
}

static void drop_connection(int id, int coid)
{
    pthread_mutex_lock(&g_lc_lock);
    lc_record_t *rec = find_or_create(id);
    if (rec && rec->override_coid == coid) rec->override_coid = -1;
    pthread_mutex_unlock(&g_lc_lock);
    name_close(coid);
}

/* One delivery attempt, following UC-07 "reject with wait=N -> CC delays
 * -> resend" for up to CMD_MAX_ATTEMPTS sends. */
static void deliver_command(pending_cmd_t *job)
{
    int id = job->cmd.intersection_id;
    int coid = connection_for(id);
    if (coid < 0) {
        printf("[CC][Command_Worker] cannot reach I%d (link not established)\n", id);
        fflush(stdout);
        return;
    }

    net_reply_t reply;
    memset(&reply, 0, sizeof(reply));
    job->attempts++;

    /* Bound both the send and the wait for the reply. A timeout while
     * REPLY-blocked means the LC may still have acted on the command. */
    uint64_t ns = (uint64_t)CMD_SEND_TIMEOUT_MS * 1000000ULL;
    TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &ns, NULL);
    if (MsgSend(coid, &job->cmd, sizeof(job->cmd), &reply, sizeof(reply)) == -1) {
        int err = errno;
        printf("[CC][Command_Worker] send of %s to I%d failed (errno=%d%s) -- link may be down\n",
               cmd_label(&job->cmd), id, err,
               (err == ETIMEDOUT) ? ": no reply in time, outcome unknown" : "");
        fflush(stdout);
        drop_connection(id, coid);   /* force a fresh connection next time */
        return;
    }

    if (reply.result == NET_RESULT_ACCEPTED) {
        printf("[CC][Command_Worker] I%d ACCEPTED %s seq=%u\n", id, cmd_label(&job->cmd), job->cmd.sequence_no);
    } else if (reply.result == NET_RESULT_WAIT) {
        if (job->attempts >= CMD_MAX_ATTEMPTS) {
            printf("[CC][Command_Worker] I%d rejected: %s -- still not accepted after %d attempts, giving up\n",
                   id, reply.reason, job->attempts);
        } else {
            printf("[CC][Command_Worker] I%d rejected: %s -- resending in %ds (attempt %d/%d, UC-07)\n",
                   id, reply.reason, reply.wait_seconds, job->attempts, CMD_MAX_ATTEMPTS);
            job->not_before_ms = now_ms() + (uint64_t)reply.wait_seconds * 1000u;
            requeue_command(job);
        }
    } else {
        printf("[CC][Command_Worker] I%d rejected (invalid): %s\n", id, reply.reason);
    }
    fflush(stdout);
}

static void *command_worker(void *arg)
{
    (void)arg;
    for (;;) {
        pending_cmd_t job = dequeue_due_command();
        deliver_command(&job);
    }
    return NULL;
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

    printf("[CC][Operator_Console] queued OVERRIDE_COMMAND(%s) to I%d (seq=%u)\n", cmd_name, id, cmd.sequence_no);
    enqueue_command(&cmd);
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

    printf("[CC][Operator_Console] queued MODE_SWITCH(%s) to I%d (seq=%u)\n", mode_name, id, cmd.sequence_no);
    enqueue_command(&cmd);
}

static void do_verbose(const char *arg)
{
    if      (!strcmp(arg, "on"))  g_verbose = 1;
    else if (!strcmp(arg, "off")) g_verbose = 0;
    else { printf("[CC] usage: verbose <on|off>\n"); return; }
    printf("[CC] verbose %s: %s\n", arg,
           g_verbose ? "logging every STATUS_UPDATE"
                     : "logging only phase/mode/fault changes");
}

static void operator_console_task(void)
{
    printf("CC operator console ready. Commands:\n"
           "  status\n"
           "  override <id> <allred|nsgreen|ewgreen|dignitary>\n"
           "  mode <id> <fixed|sensor>\n"
           "  verbose <on|off>\n"
           "  quit\n");
    char line[128];
    while (fgets(line, sizeof(line), stdin)) {
        char verb[32], a1[32], a2[32];
        int n = sscanf(line, "%31s %31s %31s", verb, a1, a2);
        if (n < 1) continue;
        if (!strcmp(verb, "quit")) break;
        else if (!strcmp(verb, "status")) print_dashboard();
        else if (!strcmp(verb, "override") && n == 3) do_override(atoi(a1), a2);
        else if (!strcmp(verb, "mode") && n == 3) do_mode_switch(atoi(a1), a2);
        else if (!strcmp(verb, "verbose") && n == 2) do_verbose(a1);
        else printf("[CC] unrecognised command\n");
        fflush(stdout);
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

    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&g_cmdq_cond, &ca);
    pthread_condattr_destroy(&ca);
    pthread_create(&th, NULL, command_worker, NULL);
    pthread_detach(th);

    operator_console_task();  /* runs on main thread until "quit" */
    return 0;
}