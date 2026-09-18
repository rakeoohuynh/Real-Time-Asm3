/*
 * local_controller.c
 *
 * Local Controller (LC) for one signalized intersection next to a
 * railway crossing. Run one per intersection (-i 1 for I1, and so on).
 *
 * This file sets up channels and threads, runs the Phase_Controller_Task
 * loop on the main thread and prints the status line. The work behind
 * each pulse lives in its own module:
 *
 *   Phase_Controller_Task        this file
 *   Signal_Output_Task           signal_output.c
 *   Railway_Signal_Output_Task   railway_protection.c
 *   Boom_Gate_Controller_Task    boom_gate.c
 *   Status_Reporting_Task        status_report.c
 *   Central_Command_Server_Task  central_command_server.c
 *   Pedestrian_Input_Task        pedestrian.c
 *   Vehicle_Sensor_Task          sensor_driven.c
 *   Train_Sensor_Task            railway_protection.c, driven by train_schedule.c
 *
 *   vehicle cycle                fixed_timing.c
 *   early end of green           sensor_driven.c
 *   mode by time of day          mode_schedule.c
 *   pedestrian crossing          pedestrian.c
 *   railway protection           railway_protection.c
 *   right-turn arrows            right_turn.c
 *   allowed movements            phase_table.c
 *   shared state, time scaling   lc_context.c, common.h
 *
 * Usage:
 *   ./local_controller -i 1 [-c <cc_node_name>] [-T HH:MM] [-N] [-v]
 *     -i  intersection id
 *     -c  QNET node the CC runs on (omit when it's on this node)
 *     -T  start the simulated clock at HH:MM, e.g. -T 06:28 for just
 *         before the morning peak; trains and the control mode follow it
 *     -N  no train timetable; use the 't'/'c' keys instead
 *     -v  also log every signal-head, railway-signal and gate command
 */
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

/* Connection the 100ms timer sends its pulse on. */
static int g_timer_coid;

/* One status line per visible change: step, signal heads, gate, mode or
 * faults. Runs on Phase_Controller_Task, which is the only writer of
 * these fields, so no lock is needed to read them. */
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
     * duration in the log. */
    int left_s = (ctx->countdown_ms > 0)
                     ? (ctx->countdown_ms * TIME_SCALE_FACTOR + 999) / 1000 : 0;

    /* Wall-clock time, same as the CC's log, so the two consoles line up. */
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

/* The current step's countdown ran out. Offer it to each module in
 * priority order: railway, then pedestrian, then the vehicle cycle. */
static void advance_step(lc_context_t *ctx)
{
    if (railway_advance(ctx))     return;
    if (pedestrian_advance(ctx))  return;
    if (fixed_timing_advance(ctx)) return;

    printf("[I%d][Phase_Controller_Task] no handler for step %s -- holding it\n",
           ctx->id, lc_step_name(ctx->step));
    fflush(stdout);
    lc_enter_step(ctx, ctx->step, 1000 * TIME_SCALE_FACTOR);
}

/*
 * Phase_Controller_Task: owns the signals and works without the CC.
 * Driven by a 100ms timer pulse plus pulses from the sensors, the CC
 * command server, the gate and the train sensor. Nothing in this loop
 * waits on the CC or on hardware.
 */
static void phase_controller_task(lc_context_t *ctx)
{
    mode_schedule_init(ctx);
    right_turn_init(ctx);
    fixed_timing_begin_ns_green(ctx);
    lc_set_phase(ctx, LC_PHASE_NS);

    /* Not scaled: this is the scheduler resolution (A39). */
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
        if (rcvid != 0) continue; /* this channel only takes pulses */

        switch (pulse.code) {

        case PULSE_PHASE_TIMER:
            ctx->countdown_ms -= PHASE_TICK_MS;
            right_turn_tick(ctx, PHASE_TICK_MS);
            mode_schedule_tick(ctx);
            sensor_driven_tick(ctx);
            if (ctx->countdown_ms <= 0) advance_step(ctx);
            break;

        case PULSE_PED_REQUEST:
            /* Already latched; served at the next all-red. */
            break;

        case PULSE_VEHICLE_DEMAND:
            break; /* the sensor handler already set the flag */

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

/* "HH:MM" -> seconds since midnight, or -1 if it doesn't parse. */
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
            if (seed_sod < 0) { fprintf(stderr, "Invalid -T value '%s' (expected HH:MM)\n", optarg); return 1; }
            break;
        case 'N': run_timetable = 0; break;
        case 'v': verbose = 1; break;
        default: break;
        }
    }

    lc_context_t *ctx = lc_ctx();
    lc_context_init(ctx, id, cc_node);
    ctx->verbose = verbose;

    /* Don't drive any signal from an unsafe phase table (UC-01). */
    if (phase_table_validate() != 0) {
        fprintf(stderr, "[I%d] phase table failed validation, not starting\n", id);
        return 1;
    }

    ctx->phase_chid  = ChannelCreate(0);
    ctx->sig_chid    = ChannelCreate(0);
    ctx->rail_chid   = ChannelCreate(0);
    ctx->gate_chid   = ChannelCreate(0);
    ctx->status_chid = ChannelCreate(0);

    /* Named channel the CC sends commands to. */
    char chan_name[32];
    lc_channel_name(chan_name, sizeof(chan_name), ctx->id);
    name_attach_t *na = name_attach(NULL, chan_name, 0);
    if (!na) { perror("name_attach"); return 1; }
    ctx->net_chid = na->chid;

    ctx->coid_phase  = ConnectAttach(0, 0, ctx->phase_chid,  _NTO_SIDE_CHANNEL, 0);
    ctx->coid_status = ConnectAttach(0, 0, ctx->status_chid, _NTO_SIDE_CHANNEL, 0);
    ctx->coid_sig    = ConnectAttach(0, 0, ctx->sig_chid,    _NTO_SIDE_CHANNEL, 0);
    ctx->coid_rail   = ConnectAttach(0, 0, ctx->rail_chid,   _NTO_SIDE_CHANNEL, 0);
    ctx->coid_gate   = ConnectAttach(0, 0, ctx->gate_chid,   _NTO_SIDE_CHANNEL, 0);
    g_timer_coid     = ConnectAttach(0, 0, ctx->phase_chid,  _NTO_SIDE_CHANNEL, 0);

    printf("=== Local Controller I%d starting (CC node: %s) ===\n",
           ctx->id, ctx->cc_node[0] ? ctx->cc_node : "<same node>");
    printf("=== Demo time scaling: TIME_SCALE_FACTOR = %d "
           "(1 = real time) ===\n", TIME_SCALE_FACTOR);
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

    /* Needed even with -N: the mode schedule reads this clock. */
    train_schedule_init(seed_sod);
    if (run_timetable) {
        train_schedule_build(ctx);
        train_schedule_start(ctx);
    } else {
        printf("[I%d][Train_Schedule] timetable off (-N); use the 't'/'c' keys\n", ctx->id);
        fflush(stdout);
    }

    phase_controller_task(ctx);
    return 0;
}
