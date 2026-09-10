/* =====================================================================
 * central_command_server.c -- Central_Command_Server_Task.
 * ===================================================================== */
#include <stdio.h>
#include <string.h>
#include <sys/neutrino.h>
#include "central_command_server.h"
#include "signal_output.h"
#include "status_report.h"
#include "fixed_timing.h"
#include "railway_protection.h"

void *central_command_server_task(void *arg)
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
        int rail_active  = railway_is_active(ctx);

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

void apply_pending_cc_commands(lc_context_t *ctx)
{
    pthread_mutex_lock(&ctx->lock);
    int has_override = ctx->override_pending;
    int ovr_cmd      = ctx->override_command;
    int has_mode     = ctx->mode_switch_pending;
    control_mode_t new_mode = ctx->mode_switch_requested;
    ctx->override_pending    = 0;
    ctx->mode_switch_pending = 0;
    pthread_mutex_unlock(&ctx->lock);

    if (has_mode) {
        printf("[I%d][Phase_Controller_Task] MODE_SWITCH applied: mode=%d\n", ctx->id, new_mode);
        fflush(stdout);
        ctx->mode = new_mode;
    }

    if (!has_override) return;

    /* Re-check the safety condition here, not just at acceptance.
     * Railway protection may have started in the gap -- it outranks any
     * CC command, so the override is dropped rather than applied on top
     * of a closing crossing. */
    if (railway_is_active(ctx)) {
        printf("[I%d][Phase_Controller_Task] OVERRIDE_COMMAND type=%d DISCARDED: "
               "railway protection started before it could be applied\n", ctx->id, ovr_cmd);
        fflush(stdout);
        return;
    }
    if (ctx->step == STEP_PED_WALK || ctx->step == STEP_PED_CLEARANCE) {
        printf("[I%d][Phase_Controller_Task] OVERRIDE_COMMAND type=%d DISCARDED: "
               "pedestrian crossing started before it could be applied\n", ctx->id, ovr_cmd);
        fflush(stdout);
        return;
    }

    printf("[I%d][Phase_Controller_Task] OVERRIDE_COMMAND applied: type=%d\n", ctx->id, ovr_cmd);
    fflush(stdout);

    switch (ovr_cmd) {
    case OVR_FORCE_ALL_RED:
        signal_set_vehicle(ctx, HEAD_NS_VEHICLE, V_RED, OVERRIDE_HOLD_S);
        signal_set_vehicle(ctx, HEAD_EW_VEHICLE, V_RED, OVERRIDE_HOLD_S);
        ctx->ns_state = ctx->ew_state = V_RED;
        lc_enter_step_seconds(ctx, STEP_OVERRIDE_HOLD, OVERRIDE_HOLD_S);
        break;

    case OVR_FORCE_NS_GREEN:
    case OVR_DIGNITARY_PATH:
        fixed_timing_begin_ns_green(ctx);
        /* skip normal min-green/sensor logic while forced */
        lc_enter_step_seconds(ctx, STEP_OVERRIDE_HOLD, OVERRIDE_HOLD_S);
        break;

    case OVR_FORCE_EW_GREEN:
        fixed_timing_begin_ew_green(ctx);
        lc_enter_step_seconds(ctx, STEP_OVERRIDE_HOLD, OVERRIDE_HOLD_S);
        break;

    default:
        break;
    }
    notify_status(ctx, 0, 0, 0);
}
