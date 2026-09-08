/* =============================================================
 * signal_output.c - Signal_Output_Task
 * See signal_output.h for design notes.
 * ============================================================= */
#include "signal_output.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/neutrino.h>

/* Receive buffer big enough for either a pulse or a SET_VEHICLE message. */
typedef union {
    struct _pulse      pulse;
    set_vehicle_msg_t   vehicle_msg;
} signal_output_msg_t;

void *signal_output_task(void *arg)
{
    signal_output_args_t *args = (signal_output_args_t *)arg;
    intersection_id_t     id   = args->intersection_id;
    int                    chid = args->chid;

    signal_output_msg_t   msg;
    set_vehicle_reply_t   reply;
    int                    rcvid;

    printf("[%s] Signal_Output_Task ready.\n", intersection_name(id));

    for (;;) {
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            perror("ERROR: Signal_Output_Task MsgReceive");
            break;
        }

        if (rcvid == 0) {
            /* Pulse: only PULSE_SHUTDOWN is meaningful on this channel. */
            if (msg.pulse.code == PULSE_SHUTDOWN) {
                printf("[%s] Signal_Output_Task shutting down.\n", intersection_name(id));
                break;
            }
            continue;
        }

        /* Real message. In this phase the only defined message type
         * is SET_VEHICLE. */
        if (msg.vehicle_msg.msg_type != MSG_SET_VEHICLE) {
            reply.result = 1; /* unknown message - reject */
            MsgReply(rcvid, EOK, &reply, sizeof(reply));
            continue;
        }

        /* Safety interlock (Assessment 2 A43: the phase table must
         * never contain conflicting vehicle movements). Refuse and
         * NACK rather than display an unsafe combination. */
        if (msg.vehicle_msg.ns_signal == SIGNAL_GREEN &&
            msg.vehicle_msg.ew_signal == SIGNAL_GREEN) {
            fprintf(stderr, "[%s] SAFETY INTERLOCK: refused conflicting NS=GREEN/EW=GREEN command\n",
                    intersection_name(id));
            reply.result = 1;
            MsgReply(rcvid, EOK, &reply, sizeof(reply));
            continue;
        }

        printf("[%s] SIGNAL HEADS -> NS:%-6s  EW:%-6s   (phase=%s, %ums)\n",
               intersection_name(id),
               signal_colour_name((signal_colour_t)msg.vehicle_msg.ns_signal),
               signal_colour_name((signal_colour_t)msg.vehicle_msg.ew_signal),
               phase_name((phase_state_t)msg.vehicle_msg.phase_id),
               msg.vehicle_msg.duration_ms);

        reply.result = 0; /* ACK: output confirmed (UC-01 postcondition) */
        if (MsgReply(rcvid, EOK, &reply, sizeof(reply)) == -1) {
            perror("ERROR: Signal_Output_Task MsgReply");
        }
    }

    return NULL;
}
