/* =====================================================================
 * pedestrian.c -- pedestrian request latching and WALK sequencing.
 * ===================================================================== */
#include <stdio.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include "pedestrian.h"
#include "signal_output.h"
#include "status_report.h"
#include "fixed_timing.h"

void pedestrian_input_handle_press(lc_context_t *ctx)
{
    /* A11: 50ms debounce. Not scaled -- this is input conditioning, not
     * a control duration. */
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

int pedestrian_request_pending(lc_context_t *ctx)
{
    return ctx->ped_request_pending;
}

void pedestrian_begin_walk(lc_context_t *ctx)
{
    ctx->ped_state = P_WALK;
    signal_set_pedestrian(ctx, HEAD_PED_CROSSING, P_WALK, PED_WALK_S);
    lc_enter_step_seconds(ctx, STEP_PED_WALK, PED_WALK_S);
    lc_set_phase(ctx, LC_PHASE_PED);
    notify_status(ctx, 0, 0, 0);
}

int pedestrian_advance(lc_context_t *ctx)
{
    switch (ctx->step) {

    case STEP_PED_WALK:
        ctx->ped_state = P_CLEARANCE;
        signal_set_pedestrian(ctx, HEAD_PED_CROSSING, P_CLEARANCE, PED_CLEARANCE_S);
        lc_enter_step_seconds(ctx, STEP_PED_CLEARANCE, PED_CLEARANCE_S);
        notify_status(ctx, 0, 0, 0);
        return 1;

    case STEP_PED_CLEARANCE:
        ctx->ped_state = P_DONT_WALK;
        signal_set_pedestrian(ctx, HEAD_PED_CROSSING, P_DONT_WALK, 0);
        ctx->ped_request_pending = 0;
        notify_status(ctx, 0, 0, 0);
        /* resume the vehicle mode that was interrupted */
        fixed_timing_begin_ew_green(ctx);
        lc_set_phase(ctx, LC_PHASE_EW);
        return 1;

    default:
        return 0;
    }
}
