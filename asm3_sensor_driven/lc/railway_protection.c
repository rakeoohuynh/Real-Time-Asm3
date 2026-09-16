/* =====================================================================
 * railway_protection.c -- UC-05 railway crossing protection.
 * ===================================================================== */
#include <stdio.h>
#include <sys/neutrino.h>
#include "railway_protection.h"
#include "boom_gate.h"
#include "signal_output.h"
#include "status_report.h"
#include "fixed_timing.h"
#include "right_turn.h"

/* How long a step waits for a GATE_STATUS pulse that has not arrived by
 * the time its nominal interval ended. */
#define GATE_WAIT_REARM_MS   1000

/* ==========================================================
 * Railway_Signal_Output_Task
 *   Owns rail_chid. Drives both line-side outputs: the crossing
 *   lights that face the road, and the train light that faces the
 *   train and is what flags a gate fault to it.
 * ========================================================== */
void *railway_signal_task(void *arg)
{
    lc_context_t *ctx = arg;
    for (;;) {
        local_msg_t msg;
        int rcvid = MsgReceive(ctx->rail_chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;

        local_reply_t reply = { .result = 0, .elapsed_ms = 0 };
        if (msg.type == MSG_SET_RAILWAY_SIGNAL) {
            if (ctx->verbose) {
                const char *which = (msg.body.set_rail.signal_id == RAIL_SIGNAL_TRAIN)
                                        ? "train light" : "crossing lights";
                printf("[I%d][Railway_Signal_Output_Task] %s (signal %d) -> %s\n",
                       ctx->id, which, msg.body.set_rail.signal_id,
                       lc_rail_name(msg.body.set_rail.colour));
                fflush(stdout);
            }
        } else {
            reply.result = FAULT_CONFIG;
        }
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

static int send_rail(lc_context_t *ctx, int signal_id, rail_signal_t colour)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_RAILWAY_SIGNAL;
    m.body.set_rail.signal_id = signal_id;
    m.body.set_rail.colour    = colour;
    if (MsgSend(ctx->coid_rail, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_CONFIG;
    return r.result;
}

static void set_crossing_lights(lc_context_t *ctx, rail_signal_t c)
{
    ctx->rail_signal = c;
    send_rail(ctx, RAIL_SIGNAL_CROSSING, c);
}

static void set_train_light(lc_context_t *ctx, rail_signal_t c)
{
    ctx->train_signal = c;
    send_rail(ctx, RAIL_SIGNAL_TRAIN, c);
}

/* ==========================================================
 * Train_Sensor_Task
 * ========================================================== */
void train_sensor_handle_approach(lc_context_t *ctx)
{
    ctx->rail_alert = 1;
    printf("[I%d][Train_Sensor_Task] TRAIN_APPROACH_DETECTED\n", ctx->id);
    fflush(stdout);
    MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_TRAIN_APPROACH, 0);
}

void train_sensor_handle_cleared(lc_context_t *ctx)
{
    ctx->rail_clear_req = 1;
    printf("[I%d][Train_Sensor_Task] TRAIN_CLEARED\n", ctx->id);
    fflush(stdout);
    MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_TRAIN_CLEARED, 0);
}

int railway_is_active(lc_context_t *ctx)
{
    return (ctx->step >= STEP_RAIL_YELLOW && ctx->step <= STEP_RAIL_GATE_RAISE);
}

/* The nominal sequence, in order, with the countdown each step is entered
 * with in railway_advance(). STEP_RAIL_GATE_FAULT is not part of the
 * nominal path and is handled separately below. */
static const struct { lc_step_t step; int real_s; } k_rail_sequence[] = {
    { STEP_RAIL_YELLOW,     VEHICLE_YELLOW_S        },
    { STEP_RAIL_ALLRED,     VEHICLE_ALL_RED_S       },
    { STEP_RAIL_PREARRIVAL, RAIL_PREARRIVAL_QUIET_S },
    { STEP_RAIL_WARN,       RAIL_WARNING_LEAD_S     },
    { STEP_RAIL_GATE_LOWER, RAIL_GATE_LOWER_S       },
    { STEP_RAIL_OCCUPIED,   RAIL_TRAIN_OCCUPY_S     },
    { STEP_RAIL_POST_HOLD,  RAIL_POST_TRAIN_HOLD_S  },
    { STEP_RAIL_GATE_RAISE, RAIL_GATE_LOWER_S       },
};

int railway_remaining_ms(lc_context_t *ctx)
{
    if (!railway_is_active(ctx)) return 0;

    /* A gate-fault retry is followed by a fresh lowering, so count the
     * rest of the sequence as if the warning flash had just ended. */
    lc_step_t after = (ctx->step == STEP_RAIL_GATE_FAULT) ? STEP_RAIL_WARN : ctx->step;

    int total = ctx->countdown_ms;
    int found = 0;
    for (size_t i = 0; i < sizeof(k_rail_sequence) / sizeof(k_rail_sequence[0]); i++) {
        if (found) total += SCALE_S_MS(k_rail_sequence[i].real_s);
        else if (k_rail_sequence[i].step == after) found = 1;
    }
    return total;
}

/* ==========================================================
 * The sequence
 * ========================================================== */
void railway_begin_protection(lc_context_t *ctx)
{
    printf("[I%d][Phase_Controller_Task] railway protection ACTIVATED "
           "(A44 priority preemption; %ds pre-arrival, %ds total closure)\n",
           ctx->id, RAIL_PROTECT_LEAD_S, RAIL_TOTAL_CLOSURE_S);
    fflush(stdout);

    /* Withdraw the auxiliary movements before anything else: railway
     * protection outranks them and they must not survive the preemption
     * even for one tick. */
    right_turn_force_off(ctx, "railway protection");

    lc_set_vehicle_states(ctx, V_YELLOW, V_YELLOW);
    signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_YELLOW, VEHICLE_YELLOW_S);
    signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_YELLOW, VEHICLE_YELLOW_S);

    set_train_light(ctx, R_CLEAR);
    lc_enter_step_seconds(ctx, STEP_RAIL_YELLOW, VEHICLE_YELLOW_S);
    lc_set_phase(ctx, LC_PHASE_RAIL_PROTECT);
    notify_status(ctx, 0, 0, 0);
}

void railway_handle_cleared(lc_context_t *ctx)
{
    ctx->rail_clear_req = 1;
    /* The train is off the crossing early: end the occupation now and
     * start the post-train hold. */
    if (ctx->step == STEP_RAIL_OCCUPIED)
        ctx->countdown_ms = 0;
}

static void enter_gate_fault(lc_context_t *ctx, int exhausted)
{
    /* Crossing lights keep flashing, the train light goes red to flag
     * the gate problem to the train, and the control room is told. */
    set_crossing_lights(ctx, R_FLASHING_RED);
    set_train_light(ctx, R_RED);
    lc_raise_fault(ctx, FAULT_BOOM_GATE);

    printf("[I%d][Phase_Controller_Task] BOOM GATE FAULT after attempt %d/%d%s "
           "-- crossing lights still flashing, train light RED, distress to CC\n",
           ctx->id, boom_gate_attempts_used(), GATE_CLOSE_MAX_ATTEMPTS,
           exhausted ? " (attempts exhausted, escalating)" : "");
    fflush(stdout);

    notify_status(ctx, 1, FAULT_BOOM_GATE, BOOM_GATE_ID);
    lc_enter_step_seconds(ctx, STEP_RAIL_GATE_FAULT, GATE_RETRY_INTERVAL_S);
}

int railway_advance(lc_context_t *ctx)
{
    switch (ctx->step) {

    case STEP_RAIL_YELLOW:
        lc_set_vehicle_states(ctx, V_RED, V_RED);
        signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_RED, VEHICLE_ALL_RED_S);
        signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_RED, VEHICLE_ALL_RED_S);
        lc_enter_step_seconds(ctx, STEP_RAIL_ALLRED, VEHICLE_ALL_RED_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_RAIL_ALLRED:
        /* Railway-protection RED is now established for the whole
         * remainder of the pre-arrival window. */
        printf("[I%d][Phase_Controller_Task] railway protection RED established, "
               "%ds until the warning flash\n", ctx->id, RAIL_PREARRIVAL_QUIET_S);
        fflush(stdout);
        lc_enter_step_seconds(ctx, STEP_RAIL_PREARRIVAL, RAIL_PREARRIVAL_QUIET_S);
        return 1;

    case STEP_RAIL_PREARRIVAL:
        set_crossing_lights(ctx, R_FLASHING_RED);
        printf("[I%d][Phase_Controller_Task] crossing warning flashing, "
               "gate lowering begins in %ds (A15)\n", ctx->id, RAIL_WARNING_LEAD_S);
        fflush(stdout);
        lc_enter_step_seconds(ctx, STEP_RAIL_WARN, RAIL_WARNING_LEAD_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_RAIL_WARN:
        boom_gate_begin_close(ctx);
        lc_enter_step_seconds(ctx, STEP_RAIL_GATE_LOWER, RAIL_GATE_LOWER_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_RAIL_GATE_LOWER:
        /* The nominal lowering interval has elapsed but GATE_STATUS has
         * not arrived yet; keep waiting a tick at a time. */
        lc_enter_step(ctx, STEP_RAIL_GATE_LOWER, GATE_WAIT_REARM_MS * TIME_SCALE_FACTOR);
        return 1;

    case STEP_RAIL_GATE_FAULT:
        if (boom_gate_attempts_used() < GATE_CLOSE_MAX_ATTEMPTS) {
            boom_gate_retry_close(ctx);
        } else {
            printf("[I%d][Phase_Controller_Task] gate still not locked; "
                   "starting a fresh close episode (crossing stays closed)\n", ctx->id);
            fflush(stdout);
            boom_gate_begin_close(ctx);
        }
        lc_enter_step_seconds(ctx, STEP_RAIL_GATE_LOWER, RAIL_GATE_LOWER_S);
        return 1;

    case STEP_RAIL_OCCUPIED:
        printf("[I%d][Phase_Controller_Task] train clear of the crossing, "
               "holding the gate down %ds (A17)\n", ctx->id, RAIL_POST_TRAIN_HOLD_S);
        fflush(stdout);
        lc_enter_step_seconds(ctx, STEP_RAIL_POST_HOLD, RAIL_POST_TRAIN_HOLD_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_RAIL_POST_HOLD:
        boom_gate_begin_open(ctx);
        lc_enter_step_seconds(ctx, STEP_RAIL_GATE_RAISE, RAIL_GATE_LOWER_S);
        return 1;

    case STEP_RAIL_GATE_RAISE:
        /* Waiting on the gate's OPENED status. */
        lc_enter_step(ctx, STEP_RAIL_GATE_RAISE, GATE_WAIT_REARM_MS * TIME_SCALE_FACTOR);
        return 1;

    default:
        return 0;
    }
}

void railway_on_gate_status(lc_context_t *ctx, int pulse_value)
{
    gate_event_t evt = boom_gate_on_status(ctx, pulse_value);

    switch (evt) {

    case GATE_EVT_LOCKED:
        if (ctx->step != STEP_RAIL_GATE_LOWER && ctx->step != STEP_RAIL_GATE_FAULT)
            return;   /* stale status from a previous episode */
        set_crossing_lights(ctx, R_FLASHING_RED);
        set_train_light(ctx, R_CLEAR);   /* gate is down: the train may run */
        printf("[I%d][Phase_Controller_Task] gate LOCKED, crossing occupied for %ds (A18)\n",
               ctx->id, RAIL_TRAIN_OCCUPY_S);
        fflush(stdout);
        lc_enter_step_seconds(ctx, STEP_RAIL_OCCUPIED, RAIL_TRAIN_OCCUPY_S);
        notify_status(ctx, 0, 0, 0);
        break;

    case GATE_EVT_CLOSE_RETRY:
        enter_gate_fault(ctx, 0);
        break;

    case GATE_EVT_CLOSE_FAILED:
        enter_gate_fault(ctx, 1);
        break;

    case GATE_EVT_OPENED:
        if (ctx->step != STEP_RAIL_GATE_RAISE) return;
        set_crossing_lights(ctx, R_CLEAR);
        set_train_light(ctx, R_CLEAR);
        ctx->rail_alert = 0;
        ctx->rail_clear_req = 0;
        printf("[I%d][Phase_Controller_Task] railway protection CLEARED, resuming vehicle control\n",
               ctx->id);
        fflush(stdout);
        notify_status(ctx, 0, 0, 0);
        fixed_timing_begin_ns_green(ctx);
        lc_set_phase(ctx, LC_PHASE_NS);
        break;

    default:
        break;
    }
}
