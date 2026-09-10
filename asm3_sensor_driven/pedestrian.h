/* =====================================================================
 * pedestrian.h -- Pedestrian_Input_Task and pedestrian sequencing.
 *
 * Covers the debounce and latch on the input side (A11/A12) and the
 * WALK -> CLEARANCE -> DONT_WALK sequence on the output side (A8/A9).
 * ===================================================================== */
#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

#include "lc_context.h"

/* Input side: a button press arriving from the (simulated) hardware. */
void pedestrian_input_handle_press(lc_context_t *ctx);

/* Is a request latched and still unserviced. */
int  pedestrian_request_pending(lc_context_t *ctx);

/* Output side: start WALK. Called at a safe ALL-RED boundary. */
void pedestrian_begin_walk(lc_context_t *ctx);

/* Handle the expiry of a pedestrian step. Returns 1 if handled. */
int  pedestrian_advance(lc_context_t *ctx);

#endif /* PEDESTRIAN_H */
