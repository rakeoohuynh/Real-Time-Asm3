/* =====================================================================
 * local_controller.c
 *
 * QNX Local Controller (LC) for ONE signalised intersection with an
 * adjacent railway crossing (e.g. I1).
 *
 * This file is the ENTRY POINT ONLY: it creates the channels and
 * connections, starts each task on its own thread, and runs the
 * Phase_Controller_Task dispatch loop. The behaviour behind each pulse
 * lives in the module named after it, so this file should read as
 * "what does the LC do", not "how does each part work".
 *
 * Task map (Section 7 task architecture -> module):
 *   Phase_Controller_Task        -> this file (main thread)
 *   Signal_Output_Task           -> signal_output.c
 *   Railway_Signal_Output_Task   -> railway_protection.c
 *   Boom_Gate_Controller_Task    -> boom_gate.c
 *   Status_Reporting_Task        -> status_report.c
 *   Central_Command_Server_Task  -> central_command_server.c
 *   Pedestrian_Input_Task        -> pedestrian.c   (input side)
 *   Vehicle_Sensor_Task          -> sensor_driven.c
 *   Train_Sensor_Task            -> railway_protection.c, fed by
 *                                   train_schedule.c's timetable
 *
 * Sequencing logic by mode/feature:
 *   fixed-timing cycle           -> fixed_timing.c
 *   sensor-driven early cut      -> sensor_driven.c
 *   mode by time of day (A26)    -> mode_schedule.c
 *   pedestrian WALK sequence     -> pedestrian.c
 *   railway protection (UC-05)   -> railway_protection.c
 *   right-turn arrows            -> right_turn.c
 *   movement permissions         -> phase_table.c
 *   shared state + A40 scaling   -> lc_context.c / common.h
 *
 * Build:
 *   see Makefile.poc, or build.sh
 * Run:
 *   ./local_controller -i 1 [-c <cc_node_name>] [-T HH:MM] [-N] [-v]
 *     -i  intersection id
 *     -c  QNET node the CC lives on (omit for same-node testing)
 *     -T  seed the simulated clock, e.g. -T 06:28 to start just before
 *         a morning peak train; the A26 control mode follows it too
 *     -N  do not run the train timetable (manual 't'/'c' keys only)
 *     -v  verbose: also log every signal-head, railway-signal and gate
 *         command, not just the one-line status summary
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
#include "lc_context.h"
#include "phase_table.h"
#include "signal_output.h"
#include "boom_gate.h"
#include "railway_protection.h"
#include "pedestrian.h"
#include "fixed_timing.h"
#include "sensor_driven.h"
#include "mode_schedule.h"
#include "right_turn.h"
#include "status_report.h"
#include "central_command_server.h"
#include "console_input.h"
#include "train_schedule.h"

/* Connection the 100ms POSIX timer delivers its pulse on. */
static int g_timer_coid;

/* ---------------------------------------------------------------------
 * One-line status summary, printed whenever something visible changes:
 * the step, a signal head, the gate, the mode or the faults. This is the
 * line to read; the per-head output lines only appear with -v.
 * Runs on Phase_Controller_Task, the only writer of these fields, so it
 * reads them without the lock.
 * ------------------------------------------------------------------- */
typedef struct {
    lc_step_t       step;
    control_mode_t  mode;
    vehicle_state_t ns, ew;
    arrow_state_t   ns_arrow, ew_arrow;
    ped_state_t     ped;
    gate_state_t    gate;
    uint32_t        faults;
} lc_view_t;

static void print_status_line(lc_context_t *ctx)
{
    static lc_view_t last;
    static int       have_last = 0;

    lc_view_t v;
    memset(&v, 0, sizeof(v));
    v.step     = ctx->step;
    v.mode     = ctx->mode;
    v.ns       = ctx->ns_state;
    v.ew       = ctx->ew_state;
    v.ns_arrow = ctx->ns_arrow;
    v.ew_arrow = ctx->ew_arrow;
    v.ped      = ctx->ped_state;
    v.gate     = ctx->gate_state;
    v.faults   = ctx->fault_flags;

    if (have_last && memcmp(&v, &last, sizeof(v)) == 0) return;
    memcpy(&last, &v, sizeof(v));
    have_last = 1;

    /* Time left in the step, in real-world seconds like every other
     * duration the LC logs. */
    int left_s = (ctx->countdown_ms > 0)
                     ? (ctx->countdown_ms * TIME_SCALE_FACTOR + 999) / 1000 : 0;

    /* Wall-clock stamp, the same clock the CC stamps its log with, so the
     * two consoles can be lined up line for line. */
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    char ts[16];
    strftime(ts, sizeof(ts), "%H:%M:%S", &lt);

    char ns[16], ew[16];
    snprintf(ns, sizeof(ns), "%s%s", lc_vehicle_name(v.ns), (v.ns_arrow == ARROW_GREEN) ? "+ARROW" : "");
    snprintf(ew, sizeof(ew), "%s%s", lc_vehicle_name(v.ew), (v.ew_arrow == ARROW_GREEN) ? "+ARROW" : "");

    printf("[I%d %s] %-15s %3ds  NS:%-11s EW:%-11s PED:%-9s GATE:%-8s %s",
           ctx->id, ts, lc_step_name(v.step), left_s, ns, ew,
           lc_ped_name(v.ped), lc_gate_name(v.gate), lc_mode_name(v.mode));
    if (v.faults) {
        char f[64];
        printf("  FAULT:%s", fault_flags_str(v.faults, f, sizeof(f)));
    }
    printf("\n");
    fflush(stdout);
}

/* ---------------------------------------------------------------------
 * One step's countdown reached zero. Ask each module in priority order
 * whether the expired step is theirs: railway protection outranks
 * pedestrian service, which outranks ordinary vehicle sequencing.
 * ------------------------------------------------------------------- */
static void advance_step(lc_context_t *ctx)
{
    if (railway_advance(ctx))     return;
    if (pedestrian_advance(ctx))  return;
    if (fixed_timing_advance(ctx)) return;

    printf("[I%d][Phase_Controller_Task] unhandled step %s -- holding\n",
           ctx->id, lc_step_name(ctx->step));
    fflush(stdout);
    lc_enter_step(ctx, ctx->step, 1000 * TIME_SCALE_FACTOR);
}

/* =========================================================================
 * Phase_Controller_Task -- the safety-critical core (Section 7: retains
 * local authority over real-time signal execution, independent of CC).
 * Runs on the process's main thread. Driven by a 100ms POSIX timer pulse
 * (A39) plus asynchronous pulses from sensors, CC, the gate and the
 * railway. Nothing in this loop blocks on the CC or on hardware.
 * ========================================================================= */
static void phase_controller_task(lc_context_t *ctx)
{
    mode_schedule_init(ctx);
    right_turn_init(ctx);
    fixed_timing_begin_ns_green(ctx);
    lc_set_phase(ctx, LC_PHASE_NS);

    /* 100ms POSIX timer (A39). NOT scaled: this is scheduling
     * resolution, not a real-world duration. */
    timer_t timerid;
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, g_timer_coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_PHASE_TIMER, 0);
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
            right_turn_tick(ctx, PHASE_TICK_MS);
            mode_schedule_tick(ctx);
            sensor_driven_tick(ctx);
            if (ctx->countdown_ms <= 0) advance_step(ctx);
            break;

        case PULSE_PED_REQUEST:
            /* latched; serviced at the next STEP_*_ALLRED boundary (A2/A13) */
            break;

        case PULSE_VEHICLE_DEMAND:
            break; /* flags already set by the sensor handler */

        case PULSE_TRAIN_APPROACH:
            if (!railway_is_active(ctx)) railway_begin_protection(ctx);
            break;

        case PULSE_TRAIN_CLEARED:
            railway_handle_cleared(ctx);
            break;

        case PULSE_CC_COMMAND_READY:
            apply_pending_cc_commands(ctx);
            break;

        case PULSE_GATE_STATUS:
            railway_on_gate_status(ctx, pulse.value.sival_int);
            break;

        default:
            break;
        }

        print_status_line(ctx);
    }
}

/* ---------------------------------------------------------------------
 * -T HH:MM  ->  seconds of day, or -1 if unparseable.
 * ------------------------------------------------------------------- */
static int parse_hhmm(const char *s)
{
    int h = 0, m = 0;
    if (sscanf(s, "%d:%d", &h, &m) != 2) return -1;
    if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
    return h * 3600 + m * 60;
}

int main(int argc, char **argv)
{
    int   id = 1;
    char  cc_node[CC_NODE_MAXLEN] = "";
    int   seed_sod = -1;
    int   run_timetable = 1;
    int   verbose = 0;

    int opt;
    while ((opt = getopt(argc, argv, "i:c:T:Nv")) != -1) {
        switch (opt) {
        case 'i': id = atoi(optarg); break;
        case 'c': strncpy(cc_node, optarg, sizeof(cc_node) - 1); break;
        case 'T':
            seed_sod = parse_hhmm(optarg);
            if (seed_sod < 0) { fprintf(stderr, "bad -T value '%s', expected HH:MM\n", optarg); return 1; }
            break;
        case 'N': run_timetable = 0; break;
        case 'v': verbose = 1; break;
        default: break;
        }
    }

    lc_context_t *ctx = lc_ctx();
    lc_context_init(ctx, id, cc_node);
    ctx->verbose = verbose;

    /* UC-01 startup: validate the phase table before driving anything. */
    if (phase_table_validate() != 0) {
        fprintf(stderr, "[I%d] phase table failed validation -- refusing to start\n", id);
        return 1;
    }

    /* --- internal channels --- */
    ctx->phase_chid  = ChannelCreate(0);
    ctx->sig_chid    = ChannelCreate(0);
    ctx->rail_chid   = ChannelCreate(0);
    ctx->gate_chid   = ChannelCreate(0);
    ctx->status_chid = ChannelCreate(0);

    /* --- external, CC-facing channel --- */
    char chan_name[32];
    lc_channel_name(chan_name, sizeof(chan_name), ctx->id);
    name_attach_t *na = name_attach(NULL, chan_name, 0);
    if (!na) { perror("name_attach"); return 1; }
    ctx->net_chid = na->chid;

    /* --- internal client connections (same-process, side-channel) --- */
    ctx->coid_phase  = ConnectAttach(0, 0, ctx->phase_chid,  _NTO_SIDE_CHANNEL, 0);
    ctx->coid_status = ConnectAttach(0, 0, ctx->status_chid, _NTO_SIDE_CHANNEL, 0);
    ctx->coid_sig    = ConnectAttach(0, 0, ctx->sig_chid,    _NTO_SIDE_CHANNEL, 0);
    ctx->coid_rail   = ConnectAttach(0, 0, ctx->rail_chid,   _NTO_SIDE_CHANNEL, 0);
    ctx->coid_gate   = ConnectAttach(0, 0, ctx->gate_chid,   _NTO_SIDE_CHANNEL, 0);
    g_timer_coid     = ConnectAttach(0, 0, ctx->phase_chid,  _NTO_SIDE_CHANNEL, 0);

    printf("=== Local Controller I%d starting (CC node: %s) ===\n",
           ctx->id, ctx->cc_node[0] ? ctx->cc_node : "<same node>");
    printf("=== A40 demonstration scaling: TIME_SCALE_FACTOR = %d "
           "(1 = real-world timing) ===\n", TIME_SCALE_FACTOR);
    fflush(stdout);

    phase_table_dump();
    probe_cc_connectivity(ctx);

    pthread_t th;
    pthread_create(&th, NULL, signal_output_task, ctx);          pthread_detach(th);
    pthread_create(&th, NULL, railway_signal_task, ctx);         pthread_detach(th);
    pthread_create(&th, NULL, boom_gate_task, ctx);              pthread_detach(th);
    pthread_create(&th, NULL, status_reporting_task, ctx);       pthread_detach(th);
    pthread_create(&th, NULL, central_command_server_task, ctx); pthread_detach(th);
    pthread_create(&th, NULL, console_input_task, ctx);          pthread_detach(th);

    /* Seeded even with -N: the A26 mode schedule reads this clock. */
    train_schedule_init(seed_sod);
    if (run_timetable) {
        train_schedule_build(ctx);
        train_schedule_start(ctx);
    } else {
        printf("[I%d][Train_Schedule] disabled (-N): use the 't'/'c' keys\n", ctx->id);
        fflush(stdout);
    }

    /* Phase_Controller_Task owns the main thread (safety-critical, must
     * not be starved by anything else in this process). */
    phase_controller_task(ctx);
    return 0;
}
