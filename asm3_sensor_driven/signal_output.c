/* =====================================================================
 * signal_output.c -- Signal_Output_Task (Section 8 rows SET_VEHICLE,
 * SET_PEDESTRIAN, SET_RIGHT_TURN_ARROW).
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include "signal_output.h"

void *signal_output_task(void *arg)
{
    lc_context_t *ctx = arg;
    for (;;) {
        local_msg_t msg;
        int rcvid = MsgReceive(ctx->sig_chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;   /* pulse (e.g. disconnect) -- ignore */

        local_reply_t reply = { .result = 0, .elapsed_ms = 0 };
        int force_fault = (getenv("LC_FORCE_SIGNAL_FAULT") != NULL);

        /* Per-head lines only with -v; the status line in
         * local_controller.c already shows the resulting state. */
        if (msg.type == MSG_SET_VEHICLE) {
            if (ctx->verbose)
                printf("[I%d][Signal_Output_Task] vehicle head %d -> %s (%ds)\n",
                       ctx->id, msg.body.set_vehicle.head_id,
                       lc_vehicle_name(msg.body.set_vehicle.state), msg.body.set_vehicle.duration_s);
            reply.result = force_fault ? FAULT_SIGNAL_TIMEOUT : 0;
        } else if (msg.type == MSG_SET_PEDESTRIAN) {
            if (ctx->verbose)
                printf("[I%d][Signal_Output_Task] ped head %d -> %s (%ds)\n",
                       ctx->id, msg.body.set_pedestrian.crossing_id,
                       lc_ped_name(msg.body.set_pedestrian.state), msg.body.set_pedestrian.duration_s);
            reply.result = force_fault ? FAULT_PED_OUTPUT : 0;
        } else if (msg.type == MSG_SET_RIGHT_TURN_ARROW) {
            if (ctx->verbose)
                printf("[I%d][Signal_Output_Task] right-turn arrow head %d -> %s (%ds)\n",
                       ctx->id, msg.body.set_arrow.head_id,
                       lc_arrow_name(msg.body.set_arrow.state), msg.body.set_arrow.duration_s);
            reply.result = force_fault ? FAULT_SIGNAL_TIMEOUT : 0;
        } else {
            reply.result = FAULT_CONFIG;
        }
        fflush(stdout);
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

int signal_set_vehicle(lc_context_t *ctx, int head_id, vehicle_state_t state, int dur_s)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_VEHICLE;
    m.body.set_vehicle.state = state;
    m.body.set_vehicle.duration_s = dur_s;
    m.body.set_vehicle.head_id = head_id;
    if (MsgSend(ctx->coid_sig, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_SIGNAL_TIMEOUT;
    return r.result;
}

int signal_set_pedestrian(lc_context_t *ctx, int crossing_id, ped_state_t state, int dur_s)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_PEDESTRIAN;
    m.body.set_pedestrian.state = state;
    m.body.set_pedestrian.duration_s = dur_s;
    m.body.set_pedestrian.crossing_id = crossing_id;
    if (MsgSend(ctx->coid_sig, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_PED_OUTPUT;
    return r.result;
}

int signal_set_right_turn_arrow(lc_context_t *ctx, int head_id, arrow_state_t state, int dur_s)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_SET_RIGHT_TURN_ARROW;
    m.body.set_arrow.state = state;
    m.body.set_arrow.duration_s = dur_s;
    m.body.set_arrow.head_id = head_id;
    if (MsgSend(ctx->coid_sig, &m, sizeof(m), &r, sizeof(r)) == -1) return FAULT_SIGNAL_TIMEOUT;
    return r.result;
}
