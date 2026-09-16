/* =====================================================================
 * signal_output.h -- Signal_Output_Task and its client helpers.
 *
 * Owns the vehicle heads, the pedestrian heads and the right-turn arrow
 * heads. The arrow is a SEPARATE output message on a SEPARATE head id,
 * not a fourth value of the main 3-colour state, so a green arrow and a
 * red main signal can be commanded independently on the same approach.
 * ===================================================================== */
#ifndef SIGNAL_OUTPUT_H
#define SIGNAL_OUTPUT_H

#include "lc_context.h"

/* The task itself (its own thread). */
void *signal_output_task(void *arg);

/* Phase_Controller_Task -> Signal_Output_Task, blocking MsgSend/MsgReply.
 * Durations are passed as REAL-WORLD seconds for display; the actual
 * countdown is owned by the caller via lc_enter_step(). Each returns 0
 * or a FAULT_* code. */
int signal_set_vehicle(lc_context_t *ctx, int head_id, vehicle_state_t state, int dur_s);
int signal_set_pedestrian(lc_context_t *ctx, int crossing_id, ped_state_t state, int dur_s);
int signal_set_right_turn_arrow(lc_context_t *ctx, int head_id, arrow_state_t state, int dur_s);

#endif /* SIGNAL_OUTPUT_H */
