/* =====================================================================
 * right_turn.h -- right-turn arrow control (auxiliary movement).
 *
 * The arrow is an auxiliary movement, not a state of the main signal.
 * It is released by the phase table rather than by the main signal's
 * colour, so an approach showing a RED main signal can simultaneously
 * show a GREEN right-turn arrow. It is withdrawn the moment its
 * interval ends or the active step stops permitting it -- which covers
 * pedestrian service, railway protection, override and fail-safe,
 * because the phase table permits nothing in those steps.
 * ===================================================================== */
#ifndef RIGHT_TURN_H
#define RIGHT_TURN_H

#include "lc_context.h"

/* Command both arrows dark and clear their timers. Called at startup. */
void right_turn_init(lc_context_t *ctx);

/* Re-evaluate both arrows against the newly entered step. Called from
 * lc_enter_step(), so no caller has to remember to do it. */
void right_turn_on_step_change(lc_context_t *ctx);

/* Age the running arrows by one phase tick and retire any whose
 * configured interval has elapsed. */
void right_turn_tick(lc_context_t *ctx, int tick_ms);

/* Withdraw both arrows immediately, e.g. on a fault. */
void right_turn_force_off(lc_context_t *ctx, const char *reason);

#endif /* RIGHT_TURN_H */
