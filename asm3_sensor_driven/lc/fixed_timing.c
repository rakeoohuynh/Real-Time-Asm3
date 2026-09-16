/* =====================================================================
 * fixed_timing.c -- NS/EW vehicle cycling.
 * ===================================================================== */
#include <stdio.h>
#include "fixed_timing.h"
#include "signal_output.h"
#include "status_report.h"
#include "pedestrian.h"

void fixed_timing_begin_ns_green(lc_context_t *ctx)
{
    lc_set_vehicle_states(ctx, V_GREEN, V_RED);
    signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_GREEN, VEHICLE_GREEN_S);
    signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_RED,   VEHICLE_GREEN_S);
    ctx->min_green_elapsed = 0;
    lc_enter_step_seconds(ctx, STEP_NS_GREEN, VEHICLE_GREEN_S);
    notify_status(ctx, 0, 0, 0);
}

void fixed_timing_begin_ew_green(lc_context_t *ctx)
{
    lc_set_vehicle_states(ctx, V_RED, V_GREEN);
    signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_GREEN, VEHICLE_GREEN_S);
    signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_RED,   VEHICLE_GREEN_S);
    ctx->min_green_elapsed = 0;
    lc_enter_step_seconds(ctx, STEP_EW_GREEN, VEHICLE_GREEN_S);
    notify_status(ctx, 0, 0, 0);
}

int fixed_timing_advance(lc_context_t *ctx)
{
    switch (ctx->step) {

    case STEP_NS_GREEN:
        signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_YELLOW, VEHICLE_YELLOW_S);
        lc_set_vehicle_states(ctx, V_YELLOW, ctx->ew_state);
        lc_enter_step_seconds(ctx, STEP_NS_YELLOW, VEHICLE_YELLOW_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_NS_YELLOW:
        signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_RED, VEHICLE_ALL_RED_S);
        lc_set_vehicle_states(ctx, V_RED, ctx->ew_state);
        lc_enter_step_seconds(ctx, STEP_NS_ALLRED, VEHICLE_ALL_RED_S);
        return 1;

    case STEP_NS_ALLRED:
        if (pedestrian_request_pending(ctx)) {
            pedestrian_begin_walk(ctx);
        } else {
            fixed_timing_begin_ew_green(ctx);
            lc_set_phase(ctx, LC_PHASE_EW);
        }
        return 1;

    case STEP_EW_GREEN:
        signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_YELLOW, VEHICLE_YELLOW_S);
        lc_set_vehicle_states(ctx, ctx->ns_state, V_YELLOW);
        lc_enter_step_seconds(ctx, STEP_EW_YELLOW, VEHICLE_YELLOW_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_EW_YELLOW:
        signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_RED, VEHICLE_ALL_RED_S);
        lc_set_vehicle_states(ctx, ctx->ns_state, V_RED);
        lc_enter_step_seconds(ctx, STEP_EW_ALLRED, VEHICLE_ALL_RED_S);
        return 1;

    case STEP_EW_ALLRED:
        if (pedestrian_request_pending(ctx)) {
            pedestrian_begin_walk(ctx);
        } else {
            fixed_timing_begin_ns_green(ctx);
            lc_set_phase(ctx, LC_PHASE_NS);
        }
        return 1;

    case STEP_OVERRIDE_HOLD:
        printf("[I%d][Phase_Controller_Task] override hold expired, resuming normal control\n", ctx->id);
        fflush(stdout);
        fixed_timing_begin_ns_green(ctx);
        lc_set_phase(ctx, LC_PHASE_NS);
        return 1;

    case STEP_FAILSAFE:
        /* stay in FAILSAFE until an operator/CC action clears the fault; PoC just
         * keeps re-arming a short countdown so the loop remains responsive. */
        lc_enter_step(ctx, STEP_FAILSAFE, 1000);
        return 1;

    default:
        return 0;
    }
}
