/* =============================================================
 * signal_output.h - Signal_Output_Task
 *
 * EVENT_DRIVEN task (Assessment 2 Fig. 18) that owns the physical
 * vehicle signal heads for one intersection. There is no real hardware
 * in this PoC, so "driving the heads" means printing the confirmed
 * state to the console - but the IPC handshake (blocking MsgSend from
 * Phase_Controller_Task, confirmed by MsgReply) is fully implemented,
 * per the current assessment's requirement to demonstrate QNX IPC and
 * synchronisation primitives (Lab 6 Send/Receive/Reply pattern).
 * ============================================================= */
#ifndef SIGNAL_OUTPUT_H
#define SIGNAL_OUTPUT_H

#include "protocol.h"

typedef struct {
    intersection_id_t intersection_id;
    int                chid; /* channel created by lc_main.c before this thread starts */
} signal_output_args_t;

/* pthread entry point. Runs until it receives PULSE_SHUTDOWN on its
 * own channel. */
void *signal_output_task(void *arg);

#endif /* SIGNAL_OUTPUT_H */
