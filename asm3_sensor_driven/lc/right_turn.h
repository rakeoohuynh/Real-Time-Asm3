/*
 * right_turn.h -- right-turn arrows.
 *
 * An arrow is released by the phase table, not by the color of its main
 * signal, so an approach can show a red main signal with a green arrow.
 * It goes off when its interval ends or the current step stops allowing
 * it, which covers pedestrian crossings, railway protection, overrides
 * and failsafe.
 */
#ifndef RIGHT_TURN_H
#define RIGHT_TURN_H

#include "lc_context.h"

/* Turns both arrows off and clears their timers. */
void right_turn_init(lc_context_t *ctx);

/* Re-checks both arrows against the new step. lc_enter_step() calls it. */
void right_turn_on_step_change(lc_context_t *ctx);

/* Ages running arrows by one tick and turns off any that have expired. */
void right_turn_tick(lc_context_t *ctx, int tick_ms);

void right_turn_force_off(lc_context_t *ctx, const char *reason);

#endif /* RIGHT_TURN_H */
