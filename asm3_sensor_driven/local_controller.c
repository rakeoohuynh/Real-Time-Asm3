/* =====================================================================
 * local_controller.c
 *
 * QNX Local Controller (LC) proof-of-concept for ONE signalised
 * intersection with an adjacent railway crossing (e.g. I1).
 *
 * Run the same binary for I2..I6 by passing a different -i <id>; only
 * the railway-crossing tasks are meaningful for the railway-adjacent
 * intersection(s), but every LC still exposes the CC-facing channel
 * and status reporting so the design "duplicates" cleanly (per the
 * project brief) to the full six-intersection network.
 *
 * Task map (Section 7 task architecture -> implementation):
 *   Phase_Controller_Task        -> phase_controller_task() [runs on the
 *                                    process's main thread]
 *   Signal_Output_Task           -> signal_output_task()      (thread)
 *   Railway_Signal_Output_Task   -> railway_signal_task()     (thread)
 *   Boom_Gate_Controller_Task    -> boom_gate_task()          (thread)
 *   Status_Reporting_Task        -> status_reporting_task()   (thread)
 *   Central_Command_Server_Task  -> central_command_server_task() (thread)
 *   Pedestrian_Input_Task /
 *   Vehicle_Sensor_Task   /
 *   Train_Sensor_Task            -> console_input_task()      (thread)
 *     (per the project brief, sensor/pedestrian hardware is simulated
 *      with key presses; each key is dispatched to a small handler
 *      function named after its logical task so the mapping to the
 *      design stays explicit even though one physical stdin feeds them)
 *
 * Build (QNX qcc, per project toolchain notes):
 *   qcc -Vgcc_ntox86_64 -o local_controller local_controller.c -lpthread -lsocket
 * Run:
 *   ./local_controller -i 1 [-c <cc_node_name>]
 *   (omit -c to look for the CC on the same node -- fine for local
 *   testing; supply -c for a real QNET multi-node demo)
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/dispatch.h>
#include "common.h"

#define CC_NODE_MAXLEN 64

/* ---------------------------------------------------------------------
 * Shared LC state. Guarded by 'lock' whenever more than one thread
 * touches it (Central_Command_Server_Task writes override/mode-switch
 * requests; Phase_Controller_Task reads/applies them; Status_Reporting
 * _Task reads a snapshot to build STATUS_UPDATE).
 * ------------------------------------------------------------------- */
typedef enum {
    STEP_NS_GREEN, STEP_NS_YELLOW, STEP_NS_ALLRED,
    STEP_EW_GREEN, STEP_EW_YELLOW, STEP_EW_ALLRED,
    STEP_PED_WALK, STEP_PED_CLEARANCE,
    /* NOTE: gate lowering happens synchronously (blocking MsgSend) inside the
     * STEP_RAIL_WARN handler below, so STEP_RAIL_GATE_LOWER is not entered as
     * its own countdown state -- it is kept in the enum to document the step
     * that Section 8's COMMAND_BOOM_GATE/GATE_STATUS exchange corresponds to. */
    STEP_RAIL_WARN, STEP_RAIL_GATE_LOWER, STEP_RAIL_OCCUPIED, STEP_RAIL_GATE_RAISE,
    STEP_OVERRIDE_HOLD,
    STEP_FAILSAFE
} lc_step_t;

#define OVERRIDE_HOLD_S   20   /* PoC: how long a forced override state is held */

typedef struct {
    int   id;
    char  cc_node[CC_NODE_MAXLEN];   /* "" = same node as LC */

    pthread_mutex_t lock;

    /* channels owned by this process */
    int   phase_chid;   /* Phase_Controller_Task  */
    int   sig_chid;     /* Signal_Output_Task     */
    int   rail_chid;    /* Railway_Signal_Output_Task */
    int   gate_chid;    /* Boom_Gate_Controller_Task  */
    int   status_chid;  /* Status_Reporting_Task  */
    int   net_chid;      /* CC-facing external channel (Central_Command_Server_Task) */

    /* internal connections (client side, used to signal other local tasks) */
    int   coid_phase;   /* -> phase_chid, used by other threads to wake Phase_Controller_Task */
    int   coid_status;  /* -> status_chid, used by Phase_Controller_Task to push STATUS/ALARM */

    /* control-loop state (owned by Phase_Controller_Task, read by others under lock) */
    control_mode_t   mode;
    lc_phase_t       phase;       /* coarse phase reported to CC */
    lc_step_t        step;        /* fine-grained sequencing state */
    int               countdown_ms;
    vehicle_state_t   ns_state, ew_state;
    ped_state_t       ped_state;
    gate_state_t      gate_state;
    rail_signal_t     rail_signal;
    uint32_t          fault_flags;

    /* event/request flags, written by other threads, consumed by Phase_Controller_Task */
    volatile int  ped_request_pending;
    volatile int  ns_vehicle_demand;
    volatile int  ew_vehicle_demand;
    volatile int  rail_alert;      /* train approaching / present */
    volatile int  rail_clear_req;  /* train cleared                */

    /* pending CC-originated commands, set by Central_Command_Server_Task,
     * applied by Phase_Controller_Task at the next safe point            */
    volatile int  override_pending;   /* 1 = an override is waiting to be applied */
    int           override_command;   /* OVR_* value  */
    uint32_t      override_seq;

    volatile int  mode_switch_pending;
    control_mode_t mode_switch_requested;

    int   min_green_elapsed;  /* set once the current GREEN has held >= VEHICLE_MIN_GREEN_S */
} lc_context_t;

static lc_context_t g_ctx;

/* -----------------------------------------------------------------------
 * small helpers
 * --------------------------------------------------------------------- */
static const char *vname(vehicle_state_t s)
{
    switch (s) { case V_RED: return "RED"; case V_YELLOW: return "YELLOW"; default: return "GREEN"; }
}
static const char *pname(ped_state_t s)
{
    switch (s) { case P_WALK: return "WALK"; case P_CLEARANCE: return "CLEARANCE"; default: return "DONT_WALK"; }
}
static const char *rname(rail_signal_t s)
{
    switch (s) { case R_CLEAR: return "CLEAR"; case R_FLASHING_RED: return "FLASHING_RED"; default: return "RED"; }
}
static time_t now_s(void) { return time(NULL); }

/* =========================================================================
 * Signal_Output_Task
 *   Owns sig_chid. Receives MSG_SET_VEHICLE / MSG_SET_PEDESTRIAN from
 *   Phase_Controller_Task via blocking MsgSend(), simulates the physical
 *   head, and replies ACK/FAULT (matches Section 8 row 1-2).
 * ========================================================================= */
static void *signal_output_task(void *arg)
{
    lc_context_t *ctx = arg;
    for (;;) {
        local_msg_t msg;
        int rcvid = MsgReceive(ctx->sig_chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;   /* pulse (e.g. disconnect) -- ignore */

        local_reply_t reply = { .result = 0, .elapsed_ms = 0 };
        int force_fault = (getenv("LC_FORCE_SIGNAL_FAULT") != NULL);

        if (msg.type == MSG_SET_VEHICLE) {
            printf("[I%d][Signal_Output_Task] vehicle head %d -> %s (%ds)\n",
                   ctx->id, msg.body.set_vehicle.head_id,
                   vname(msg.body.set_vehicle.state), msg.body.set_vehicle.duration_s);
            reply.result = force_fault ? FAULT_SIGNAL_TIMEOUT : 0;
        } else if (msg.type == MSG_SET_PEDESTRIAN) {
            printf("[I%d][Signal_Output_Task] ped head %d -> %s (%ds)\n",
                   ctx->id, msg.body.set_pedestrian.crossing_id,
                   pname(msg.body.set_pedestrian.state), msg.body.set_pedestrian.duration_s);
            reply.result = force_fault ? FAULT_PED_OUTPUT : 0;
        } else {
            reply.result = FAULT_CONFIG;
        }
        fflush(stdout);
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

/* =========================================================================
 * Railway_Signal_Output_Task
 *   Owns rail_chid. Receives MSG_SET_RAILWAY_SIGNAL (Section 8 row for
 *   SET_RAILWAY_SIGNAL) and simulates the line-side warning light.
 * ========================================================================= */
static void *railway_signal_task(void *arg)
{
    lc_context_t *ctx = arg;
    for (;;) {
        local_msg_t msg;
        int rcvid = MsgReceive(ctx->rail_chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;
        local_reply_t reply = { .result = 0, .elapsed_ms = 0 };
        if (msg.type == MSG_SET_RAILWAY_SIGNAL) {
            printf("[I%d][Railway_Signal_Output_Task] signal %d -> %s\n",
                   ctx->id, msg.body.set_rail.signal_id, rname(msg.body.set_rail.colour));
            fflush(stdout);
        } else {
            reply.result = FAULT_CONFIG;
        }
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

/* =========================================================================
 * Boom_Gate_Controller_Task
 *   Owns gate_chid. Receives MSG_COMMAND_BOOM_GATE (CLOSE/OPEN), simulates
 *   the mechanical transition time (A16: 10s to lower) and replies
 *   GATE_STATUS(LOCKED/TIMEOUT/OPENED) per Section 8.
 * ========================================================================= */
static void *boom_gate_task(void *arg)
{
    lc_context_t *ctx = arg;
    for (;;) {
        local_msg_t msg;
        int rcvid = MsgReceive(ctx->gate_chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;
        local_reply_t reply = { .result = 0, .elapsed_ms = 0 };

        if (msg.type == MSG_COMMAND_BOOM_GATE) {
            int closing = (msg.body.boom_gate.command == 1);
            printf("[I%d][Boom_Gate_Controller_Task] gate %d command=%s, timeout=%ds\n",
                   ctx->id, msg.body.boom_gate.gate_id, closing ? "CLOSE" : "OPEN",
                   msg.body.boom_gate.timeout_s);
            fflush(stdout);

            int forced_fault = closing && (getenv("LC_FORCE_GATE_FAULT") != NULL);
            int elapsed_s = closing ? RAIL_GATE_LOWER_S : (RAIL_GATE_LOWER_S / 2);
            if (elapsed_s > msg.body.boom_gate.timeout_s) elapsed_s = msg.body.boom_gate.timeout_s + 1;
            sleep(elapsed_s > 0 ? (forced_fault ? msg.body.boom_gate.timeout_s + 1 : elapsed_s) : 0);

            reply.elapsed_ms = elapsed_s * 1000;
            if (forced_fault || elapsed_s > msg.body.boom_gate.timeout_s) {
                reply.result = FAULT_BOOM_GATE;   /* GATE_TIMEOUT_FAULT */
                printf("[I%d][Boom_Gate_Controller_Task] gate %d FAULT (timeout)\n", ctx->id, msg.body.boom_gate.gate_id);
            } else {
                reply.result = 0; /* GATE_LOCKED (closing) or GATE_OPENED (opening) */
                printf("[I%d][Boom_Gate_Controller_Task] gate %d %s confirmed\n",
                       ctx->id, msg.body.boom_gate.gate_id, closing ? "LOCKED" : "OPENED");
            }
            fflush(stdout);
        } else {
            reply.result = FAULT_CONFIG;
        }
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

/* =========================================================================
 * Status_Reporting_Task
 *   Owns status_chid. Receives PULSE_STATUS_EVENT / PULSE_FAULT_ALARM from
 *   Phase_Controller_Task, packages a net_report_t and forwards it to CC.
 *   Implements the UC-08 retry/back-off/reconnect policy so a comms
 *   failure never blocks Phase_Controller_Task (it runs on its own thread).
 * ========================================================================= */
static int  g_cc_coid = -1;              /* connection to CC, owned by this task */
static int  g_central_link_up = 0;
static int  g_retry_counter = 0;

static int connect_to_cc(lc_context_t *ctx)
{
    char path[128];
    if (ctx->cc_node[0] != '\0')
        snprintf(path, sizeof(path), "/net/%s/dev/name/local/%s", ctx->cc_node, CC_CHANNEL_NAME);
    else
        snprintf(path, sizeof(path), "/dev/name/local/%s", CC_CHANNEL_NAME);

    int coid = name_open(path, 0);
    return coid; /* -1 on failure, errno set */
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
        if (g_central_link_up) {
            printf("[I%d][Status_Reporting_Task] central_link = LOST -- continuing autonomously\n", ctx->id);
        }
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

static void *status_reporting_task(void *arg)
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
        rep.ns_state     = ctx->ns_state;
        rep.ew_state     = ctx->ew_state;
        rep.ped_state    = ctx->ped_state;
        rep.mode         = ctx->mode;
        rep.fault_flags  = ctx->fault_flags;
        pthread_mutex_unlock(&ctx->lock);
        rep.timestamp    = now_s();

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

/* helper used by Phase_Controller_Task to trigger a report */
static void notify_status(lc_context_t *ctx, int is_fault, int fault_code, int head_id)
{
    if (is_fault) {
        int val = (fault_code & 0xFF) | ((head_id & 0xFF) << 8);
        MsgSendPulse(ctx->coid_status, SIGEV_PULSE_PRIO_INHERIT, PULSE_FAULT_ALARM, val);
    } else {
        MsgSendPulse(ctx->coid_status, SIGEV_PULSE_PRIO_INHERIT, PULSE_STATUS_EVENT, 0);
    }
}

/* =========================================================================
 * Central_Command_Server_Task
 *   Owns net_chid (name_attach()'d so CC can name_open() it, possibly
 *   over QNET). Receives NET_OVERRIDE_COMMAND / NET_MODE_SWITCH from CC
 *   with blocking MsgReceive(), validates against current LC state
 *   (UC-07: safety check before acceptance), replies immediately, and
 *   -- only if accepted -- wakes Phase_Controller_Task to apply it at
 *   the next safe point.
 * ========================================================================= */
static void *central_command_server_task(void *arg)
{
    lc_context_t *ctx = arg;
    for (;;) {
        net_command_t cmd;
        int rcvid = MsgReceive(ctx->net_chid, &cmd, sizeof(cmd), NULL);
        if (rcvid <= 0) continue;  /* pulse, e.g. client disconnect -- ignore */

        net_reply_t reply;
        memset(&reply, 0, sizeof(reply));

        pthread_mutex_lock(&ctx->lock);
        int mid_crossing = (ctx->step == STEP_PED_WALK || ctx->step == STEP_PED_CLEARANCE);
        int rail_active  = (ctx->step >= STEP_RAIL_WARN && ctx->step <= STEP_RAIL_GATE_RAISE);

        if (cmd.type == NET_OVERRIDE_COMMAND) {
            if (rail_active) {
                reply.result = NET_RESULT_WAIT;
                reply.wait_seconds = (ctx->countdown_ms / 1000) + 1;
                snprintf(reply.reason, sizeof(reply.reason), "Railway protection active, wait for clearance");
            } else if (mid_crossing) {
                reply.result = NET_RESULT_WAIT;
                reply.wait_seconds = (ctx->countdown_ms / 1000) + 1;
                snprintf(reply.reason, sizeof(reply.reason),
                         "Pedestrian is crossing the road, wait for %d s", reply.wait_seconds);
            } else {
                reply.result = NET_RESULT_ACCEPTED;
                ctx->override_command  = cmd.command_type;
                ctx->override_seq      = cmd.sequence_no;
                ctx->override_pending  = 1;
            }
        } else if (cmd.type == NET_MODE_SWITCH) {
            reply.result = NET_RESULT_ACCEPTED;
            ctx->mode_switch_requested = cmd.requested_mode;
            ctx->mode_switch_pending   = 1;
        } else {
            reply.result = NET_RESULT_INVALID;
            snprintf(reply.reason, sizeof(reply.reason), "unknown command type");
        }
        pthread_mutex_unlock(&ctx->lock);

        printf("[I%d][Central_Command_Server_Task] cmd type=%d -> result=%d (%s)\n",
               ctx->id, cmd.type, reply.result, reply.reason);
        fflush(stdout);

        MsgReply(rcvid, EOK, &reply, sizeof(reply));

        if (reply.result == NET_RESULT_ACCEPTED)
            MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_CC_COMMAND_READY, 0);
    }
    return NULL;
}

/* =========================================================================
 * console_input_task
 *   Stands in for the physical Pedestrian_Input_Task / Vehicle_Sensor_Task
 *   / Train_Sensor_Task hardware, per the project brief's allowance for
 *   key-press-simulated sensor events. Each key is handled by a small
 *   function named after the logical task it represents.
 * ========================================================================= */
static void pedestrian_input_handle_press(lc_context_t *ctx)
{
    /* A11: 50ms debounce */
    usleep(PED_DEBOUNCE_MS * 1000);
    if (ctx->ped_request_pending) {
        printf("[I%d][Pedestrian_Input_Task] request already pending, press ignored (A12)\n", ctx->id);
    } else {
        ctx->ped_request_pending = 1;
        printf("[I%d][Pedestrian_Input_Task] PED_REQUEST latched\n", ctx->id);
        MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_PED_REQUEST, 0);
    }
    fflush(stdout);
}
static void vehicle_sensor_handle_detect(lc_context_t *ctx, int ns)
{
    if (ns) ctx->ns_vehicle_demand = 1; else ctx->ew_vehicle_demand = 1;
    printf("[I%d][Vehicle_Sensor_Task] demand detected on %s approach\n", ctx->id, ns ? "NS" : "EW");
    fflush(stdout);
    MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_VEHICLE_DEMAND, 0);
}
static void train_sensor_handle_approach(lc_context_t *ctx)
{
    ctx->rail_alert = 1;
    printf("[I%d][Train_Sensor_Task] TRAIN_APPROACH_DETECTED\n", ctx->id);
    fflush(stdout);
    MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_TRAIN_APPROACH, 0);
}
static void train_sensor_handle_cleared(lc_context_t *ctx)
{
    ctx->rail_clear_req = 1;
    printf("[I%d][Train_Sensor_Task] TRAIN_CLEARED\n", ctx->id);
    fflush(stdout);
    MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_TRAIN_CLEARED, 0);
}

static void *console_input_task(void *arg)
{
    lc_context_t *ctx = arg;
    printf("[I%d] keys: p=pedestrian  n=vehicle(NS)  e=vehicle(EW)  t=train approach  c=train cleared  q=quit\n", ctx->id);
    fflush(stdout);
    char line[16];
    while (fgets(line, sizeof(line), stdin)) {
        switch (line[0]) {
            case 'p': pedestrian_input_handle_press(ctx); break;
            case 'n': vehicle_sensor_handle_detect(ctx, 1); break;
            case 'e': vehicle_sensor_handle_detect(ctx, 0); break;
            case 't': train_sensor_handle_approach(ctx); break;
            case 'c': train_sensor_handle_cleared(ctx); break;
            case 'q': printf("[I%d] shutting down input task\n", ctx->id); return NULL;
            default: break;
        }
    }
    return NULL;
}

/* =========================================================================
 * Phase_Controller_Task -- the safety-critical core (Section 7: retains
 * local authority over real-time signal execution, independent of CC).
 * Runs on the process's main thread. Driven by a 100ms POSIX timer pulse
 * (A39) plus asynchronous pulses from sensors, CC and the railway.
 * ========================================================================= */
static int  ctx_coid_self;  /* used by the timer's SIGEV_PULSE */

/* Client connections to the other local tasks (Signal_Output_Task,
 * Railway_Signal_Output_Task, Boom_Gate_Controller_Task), created once
 * at startup and used by Phase_Controller_Task to issue commands. */
static int coid_sig, coid_rail, coid_gate;

static int send_vehicle(lc_context_t *ctx, int head_id, vehicle_state_t state, int dur_s)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_VEHICLE;
    m.body.set_vehicle.state = state; m.body.set_vehicle.duration_s = dur_s; m.body.set_vehicle.head_id = head_id;
    if (MsgSend(coid_sig, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_SIGNAL_TIMEOUT;
    return r.result;
}
static int send_pedestrian(lc_context_t *ctx, int crossing_id, ped_state_t state, int dur_s)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_PEDESTRIAN;
    m.body.set_pedestrian.state = state; m.body.set_pedestrian.duration_s = dur_s; m.body.set_pedestrian.crossing_id = crossing_id;
    if (MsgSend(coid_sig, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_PED_OUTPUT;
    return r.result;
}
static int send_rail(int signal_id, rail_signal_t colour)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_RAILWAY_SIGNAL;
    m.body.set_rail.signal_id = signal_id; m.body.set_rail.colour = colour;
    if (MsgSend(coid_rail, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_CONFIG;
    return r.result;
}
static int send_gate(int gate_id, int command /*0=OPEN 1=CLOSE*/, int timeout_s, local_reply_t *out)
{
    local_msg_t m;
    m.type = MSG_COMMAND_BOOM_GATE;
    m.body.boom_gate.gate_id = gate_id; m.body.boom_gate.command = command; m.body.boom_gate.timeout_s = timeout_s;
    if (MsgSend(coid_gate, &m, sizeof(m), out, sizeof(*out)) == -1) { out->result = FAULT_BOOM_GATE; return -1; }
    return 0;
}

static void enter_step(lc_context_t *ctx, lc_step_t step, int dur_ms)
{
    ctx->step = step;
    ctx->countdown_ms = dur_ms;
}

/* NS/EW head ids: 1=NS vehicle, 2=EW vehicle, 3=pedestrian crossing */
static void begin_ns_green(lc_context_t *ctx)
{
    ctx->ns_state = V_GREEN; ctx->ew_state = V_RED;
    send_vehicle(ctx, 1, V_GREEN, VEHICLE_GREEN_S);
    send_vehicle(ctx, 2, V_RED, VEHICLE_GREEN_S);
    ctx->min_green_elapsed = 0;
    enter_step(ctx, STEP_NS_GREEN, VEHICLE_GREEN_S * 1000);
    notify_status(ctx, 0, 0, 0);
}
static void begin_ew_green(lc_context_t *ctx)
{
    ctx->ew_state = V_GREEN; ctx->ns_state = V_RED;
    send_vehicle(ctx, 2, V_GREEN, VEHICLE_GREEN_S);
    send_vehicle(ctx, 1, V_RED, VEHICLE_GREEN_S);
    ctx->min_green_elapsed = 0;
    enter_step(ctx, STEP_EW_GREEN, VEHICLE_GREEN_S * 1000);
    notify_status(ctx, 0, 0, 0);
}
static void begin_ped_walk(lc_context_t *ctx)
{
    ctx->ped_state = P_WALK;
    send_pedestrian(ctx, 3, P_WALK, PED_WALK_S);
    enter_step(ctx, STEP_PED_WALK, PED_WALK_S * 1000);
    notify_status(ctx, 0, 0, 0);
}

/* Begin the safety-critical railway protection sequence (UC-05).
 * Forces both approaches to RED first, regardless of the current step,
 * per A44 (railway protection outranks ordinary vehicle/ped control). */
static void begin_rail_protection(lc_context_t *ctx)
{
    printf("[I%d][Phase_Controller_Task] railway protection ACTIVATED (A44 priority preemption)\n", ctx->id);
    send_vehicle(ctx, 1, V_YELLOW, VEHICLE_YELLOW_S);
    send_vehicle(ctx, 2, V_YELLOW, VEHICLE_YELLOW_S);
    sleep(VEHICLE_YELLOW_S);
    ctx->ns_state = ctx->ew_state = V_RED;
    send_vehicle(ctx, 1, V_RED, VEHICLE_ALL_RED_S);
    send_vehicle(ctx, 2, V_RED, VEHICLE_ALL_RED_S);

    ctx->rail_signal = R_FLASHING_RED;
    send_rail(1, R_FLASHING_RED);
    enter_step(ctx, STEP_RAIL_WARN, RAIL_WARNING_LEAD_S * 1000);
    ctx->phase = LC_PHASE_RAIL_PROTECT;
    notify_status(ctx, 0, 0, 0);
}

static void advance_rail_sequence(lc_context_t *ctx)
{
    local_reply_t gate_reply;
    switch (ctx->step) {
    case STEP_RAIL_WARN:
        send_gate(1, 1 /*CLOSE*/, RAIL_GATE_LOWER_S + 3, &gate_reply);
        if (gate_reply.result != 0) {
            ctx->fault_flags |= (1u << FAULT_BOOM_GATE);
            ctx->rail_signal = R_RED;                 /* A: train gets RED on gate fault */
            send_rail(1, R_RED);
            printf("[I%d][Phase_Controller_Task] BOOM GATE FAULT -> reporting to control room\n", ctx->id);
            notify_status(ctx, 1, FAULT_BOOM_GATE, 1);
            enter_step(ctx, STEP_FAILSAFE, 0);
            ctx->phase = LC_PHASE_FAILSAFE;
        } else {
            ctx->gate_state = GATE_LOCKED;
            ctx->rail_signal = R_RED;
            send_rail(1, R_RED);
            enter_step(ctx, STEP_RAIL_OCCUPIED, RAIL_TRAIN_OCCUPY_S * 1000);
            notify_status(ctx, 0, 0, 0);
        }
        break;

    case STEP_RAIL_OCCUPIED:
        /* fall through to raise if TRAIN_CLEARED already requested, else keep waiting */
        if (ctx->rail_clear_req) {
            enter_step(ctx, STEP_RAIL_GATE_RAISE, 1);
        } else {
            enter_step(ctx, STEP_RAIL_OCCUPIED, 1000); /* keep polling every tick */
        }
        break;

    case STEP_RAIL_GATE_RAISE:
        send_gate(1, 0 /*OPEN*/, RAIL_GATE_LOWER_S, &gate_reply);
        ctx->gate_state = (gate_reply.result == 0) ? GATE_OPEN : GATE_TIMEOUT_FAULT;
        ctx->rail_signal = R_CLEAR;
        send_rail(1, R_CLEAR);
        ctx->rail_alert = 0; ctx->rail_clear_req = 0;
        printf("[I%d][Phase_Controller_Task] railway protection CLEARED, resuming vehicle control\n", ctx->id);
        notify_status(ctx, 0, 0, 0);
        begin_ns_green(ctx);
        ctx->phase = LC_PHASE_NS;
        break;

    default: break;
    }
}

/* Apply a CC-accepted command at a safe point (UC-07). */
static void apply_pending_cc_commands(lc_context_t *ctx)
{
    pthread_mutex_lock(&ctx->lock);
    int has_override = ctx->override_pending;
    int ovr_cmd = ctx->override_command;
    int has_mode = ctx->mode_switch_pending;
    control_mode_t new_mode = ctx->mode_switch_requested;
    ctx->override_pending = 0;
    ctx->mode_switch_pending = 0;
    pthread_mutex_unlock(&ctx->lock);

    if (has_mode) {
        printf("[I%d][Phase_Controller_Task] MODE_SWITCH applied: mode=%d\n", ctx->id, new_mode);
        ctx->mode = new_mode;
    }
    if (has_override) {
        printf("[I%d][Phase_Controller_Task] OVERRIDE_COMMAND applied: type=%d\n", ctx->id, ovr_cmd);
        switch (ovr_cmd) {
        case OVR_FORCE_ALL_RED:
            send_vehicle(ctx, 1, V_RED, OVERRIDE_HOLD_S);
            send_vehicle(ctx, 2, V_RED, OVERRIDE_HOLD_S);
            ctx->ns_state = ctx->ew_state = V_RED;
            enter_step(ctx, STEP_OVERRIDE_HOLD, OVERRIDE_HOLD_S * 1000);
            break;
        case OVR_FORCE_NS_GREEN:
        case OVR_DIGNITARY_PATH:
            begin_ns_green(ctx);
            ctx->step = STEP_OVERRIDE_HOLD; /* skip normal min-green/sensor logic while forced */
            ctx->countdown_ms = OVERRIDE_HOLD_S * 1000;
            break;
        case OVR_FORCE_EW_GREEN:
            begin_ew_green(ctx);
            ctx->step = STEP_OVERRIDE_HOLD;
            ctx->countdown_ms = OVERRIDE_HOLD_S * 1000;
            break;
        default: break;
        }
        notify_status(ctx, 0, 0, 0);
    }
}

/* Called every time the current step's countdown reaches zero. */
static void advance_phase(lc_context_t *ctx)
{
    switch (ctx->step) {

    case STEP_NS_GREEN:
        send_vehicle(ctx, 1, V_YELLOW, VEHICLE_YELLOW_S);
        ctx->ns_state = V_YELLOW;
        enter_step(ctx, STEP_NS_YELLOW, VEHICLE_YELLOW_S * 1000);
        notify_status(ctx, 0, 0, 0);
        break;
    case STEP_NS_YELLOW:
        send_vehicle(ctx, 1, V_RED, VEHICLE_ALL_RED_S);
        ctx->ns_state = V_RED;
        enter_step(ctx, STEP_NS_ALLRED, VEHICLE_ALL_RED_S * 1000);
        break;
    case STEP_NS_ALLRED:
        if (ctx->ped_request_pending) begin_ped_walk(ctx);
        else { begin_ew_green(ctx); ctx->phase = LC_PHASE_EW; }
        break;

    case STEP_EW_GREEN:
        send_vehicle(ctx, 2, V_YELLOW, VEHICLE_YELLOW_S);
        ctx->ew_state = V_YELLOW;
        enter_step(ctx, STEP_EW_YELLOW, VEHICLE_YELLOW_S * 1000);
        notify_status(ctx, 0, 0, 0);
        break;
    case STEP_EW_YELLOW:
        send_vehicle(ctx, 2, V_RED, VEHICLE_ALL_RED_S);
        ctx->ew_state = V_RED;
        enter_step(ctx, STEP_EW_ALLRED, VEHICLE_ALL_RED_S * 1000);
        break;
    case STEP_EW_ALLRED:
        if (ctx->ped_request_pending) begin_ped_walk(ctx);
        else { begin_ns_green(ctx); ctx->phase = LC_PHASE_NS; }
        break;

    case STEP_PED_WALK:
        ctx->ped_state = P_CLEARANCE;
        send_pedestrian(ctx, 3, P_CLEARANCE, PED_CLEARANCE_S);
        enter_step(ctx, STEP_PED_CLEARANCE, PED_CLEARANCE_S * 1000);
        notify_status(ctx, 0, 0, 0);
        break;
    case STEP_PED_CLEARANCE:
        ctx->ped_state = P_DONT_WALK;
        send_pedestrian(ctx, 3, P_DONT_WALK, 0);
        ctx->ped_request_pending = 0;
        notify_status(ctx, 0, 0, 0);
        /* resume the vehicle mode that was interrupted */
        begin_ew_green(ctx); ctx->phase = LC_PHASE_EW;
        break;

    case STEP_RAIL_WARN:
    case STEP_RAIL_OCCUPIED:
    case STEP_RAIL_GATE_RAISE:
        advance_rail_sequence(ctx);
        break;

    case STEP_OVERRIDE_HOLD:
        printf("[I%d][Phase_Controller_Task] override hold expired, resuming normal control\n", ctx->id);
        begin_ns_green(ctx); ctx->phase = LC_PHASE_NS;
        break;

    case STEP_FAILSAFE:
        /* stay in FAILSAFE until an operator/CC action clears the fault; PoC just
         * keeps re-arming a short countdown so the loop remains responsive. */
        enter_step(ctx, STEP_FAILSAFE, 1000);
        break;

    default: break;
    }
}

static void phase_controller_task(lc_context_t *ctx)
{
    /* startup: validate config (trivial here) then begin fixed-timing NS green */
    ctx->mode = MODE_FIXED_TIMING;
    begin_ns_green(ctx);
    ctx->phase = LC_PHASE_NS;

    /* 100ms POSIX timer (A39) delivering PULSE_PHASE_TIMER to this channel */
    timer_t timerid;
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, ctx_coid_self, SIGEV_PULSE_PRIO_INHERIT, PULSE_PHASE_TIMER, 0);
    timer_create(CLOCK_MONOTONIC, &ev, &timerid);
    struct itimerspec its = {
        .it_value    = { .tv_sec = 0, .tv_nsec = PHASE_TICK_MS * 1000000L },
        .it_interval = { .tv_sec = 0, .tv_nsec = PHASE_TICK_MS * 1000000L }
    };
    timer_settime(timerid, 0, &its, NULL);

    for (;;) {
        struct _pulse pulse;
        int rcvid = MsgReceive(ctx->phase_chid, &pulse, sizeof(pulse), NULL);
        if (rcvid != 0) continue; /* only pulses expected here */

        switch (pulse.code) {
        case PULSE_PHASE_TIMER:
            ctx->countdown_ms -= PHASE_TICK_MS;
            /* sensor-driven early-cut logic for GREEN steps */
            if (ctx->mode == MODE_SENSOR_DRIVEN &&
                (ctx->step == STEP_NS_GREEN || ctx->step == STEP_EW_GREEN)) {
                int elapsed = (ctx->step == STEP_NS_GREEN)
                                  ? (VEHICLE_GREEN_S * 1000 - ctx->countdown_ms)
                                  : (VEHICLE_GREEN_S * 1000 - ctx->countdown_ms);
                if (elapsed >= VEHICLE_MIN_GREEN_S * 1000) ctx->min_green_elapsed = 1;
                int demand = (ctx->step == STEP_NS_GREEN) ? ctx->ns_vehicle_demand : ctx->ew_vehicle_demand;
                if (ctx->min_green_elapsed && !demand) {
                    if (ctx->step == STEP_NS_GREEN) ctx->ns_vehicle_demand = 0; else ctx->ew_vehicle_demand = 0;
                    ctx->countdown_ms = 0; /* force the boundary check below to fire now */
                }
            }
            if (ctx->countdown_ms <= 0) advance_phase(ctx);
            break;

        case PULSE_PED_REQUEST:
            /* latched; serviced at next STEP_*_ALLRED boundary (A2/A13), or
             * immediately if we are currently idle in a GREEN with a
             * genuinely parallel non-conflicting approach (kept simple: PoC
             * always services at the next safe boundary). */
            break;

        case PULSE_VEHICLE_DEMAND:
            break; /* flags already set by the sensor handler */

        case PULSE_TRAIN_APPROACH:
            if (ctx->step < STEP_RAIL_WARN || ctx->step > STEP_RAIL_GATE_RAISE)
                begin_rail_protection(ctx);
            break;

        case PULSE_TRAIN_CLEARED:
            ctx->rail_clear_req = 1;
            break;

        case PULSE_CC_COMMAND_READY:
            apply_pending_cc_commands(ctx);
            break;

        default: break;
        }
    }
}

/* =========================================================================
 * main() -- wires up channels/connections and spawns every task.
 * ========================================================================= */
int main(int argc, char **argv)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.id = 1;
    g_ctx.cc_node[0] = '\0';

    int opt;
    while ((opt = getopt(argc, argv, "i:c:")) != -1) {
        if (opt == 'i') g_ctx.id = atoi(optarg);
        else if (opt == 'c') strncpy(g_ctx.cc_node, optarg, sizeof(g_ctx.cc_node) - 1);
    }

    pthread_mutex_init(&g_ctx.lock, NULL);

    /* --- internal channels --- */
    g_ctx.phase_chid  = ChannelCreate(0);
    g_ctx.sig_chid    = ChannelCreate(0);
    g_ctx.rail_chid   = ChannelCreate(0);
    g_ctx.gate_chid   = ChannelCreate(0);
    g_ctx.status_chid = ChannelCreate(0);

    /* --- external, CC-facing channel --- */
    char chan_name[32];
    lc_channel_name(chan_name, sizeof(chan_name), g_ctx.id);
    name_attach_t *na = name_attach(NULL, chan_name, 0);
    if (!na) { perror("name_attach"); return 1; }
    g_ctx.net_chid = na->chid;

    /* --- internal client connections (same-process, side-channel) --- */
    g_ctx.coid_phase  = ConnectAttach(0, 0, g_ctx.phase_chid, _NTO_SIDE_CHANNEL, 0);
    g_ctx.coid_status = ConnectAttach(0, 0, g_ctx.status_chid, _NTO_SIDE_CHANNEL, 0);
    ctx_coid_self      = ConnectAttach(0, 0, g_ctx.phase_chid, _NTO_SIDE_CHANNEL, 0);
    coid_sig  = ConnectAttach(0, 0, g_ctx.sig_chid, _NTO_SIDE_CHANNEL, 0);
    coid_rail = ConnectAttach(0, 0, g_ctx.rail_chid, _NTO_SIDE_CHANNEL, 0);
    coid_gate = ConnectAttach(0, 0, g_ctx.gate_chid, _NTO_SIDE_CHANNEL, 0);

    printf("=== Local Controller I%d starting (CC node: %s) ===\n",
           g_ctx.id, g_ctx.cc_node[0] ? g_ctx.cc_node : "<same node>");
    fflush(stdout);

    pthread_t th;
    pthread_create(&th, NULL, signal_output_task, &g_ctx);          pthread_detach(th);
    pthread_create(&th, NULL, railway_signal_task, &g_ctx);         pthread_detach(th);
    pthread_create(&th, NULL, boom_gate_task, &g_ctx);              pthread_detach(th);
    pthread_create(&th, NULL, status_reporting_task, &g_ctx);       pthread_detach(th);
    pthread_create(&th, NULL, central_command_server_task, &g_ctx); pthread_detach(th);
    pthread_create(&th, NULL, console_input_task, &g_ctx);          pthread_detach(th);

    /* Phase_Controller_Task owns the main thread (safety-critical, must
     * not be starved by anything else in this process). */
    phase_controller_task(&g_ctx);
    return 0;
}