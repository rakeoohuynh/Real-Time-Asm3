/* =====================================================================
 * right_turn.c -- right-turn arrow control.
 * ===================================================================== */
#include <stdio.h>
#include "right_turn.h"
#include "phase_table.h"
#include "signal_output.h"

/* One approach's worth of arrow, so the two are handled identically. */
typedef struct {
    movement_t     movement;
    int            head_id;
    const char    *label;
    arrow_state_t *state;
    int           *remaining_ms;
} arrow_ref_t;

static void arrow_refs(lc_context_t *ctx, arrow_ref_t out[2])
{
    out[0].movement     = MOVE_NS_RIGHT;
    out[0].head_id      = HEAD_NS_RIGHT_ARROW;
    out[0].label        = "NS";
    out[0].state        = &ctx->ns_arrow;
    out[0].remaining_ms = &ctx->ns_arrow_ms;

    out[1].movement     = MOVE_EW_RIGHT;
    out[1].head_id      = HEAD_EW_RIGHT_ARROW;
    out[1].label        = "EW";
    out[1].state        = &ctx->ew_arrow;
    out[1].remaining_ms = &ctx->ew_arrow_ms;
}

static void arrow_start(lc_context_t *ctx, arrow_ref_t *a)
{
    lc_set_arrow_state(ctx, a->state, ARROW_GREEN);
    *a->remaining_ms = SCALE_S_MS(RIGHT_TURN_ARROW_S);
    signal_set_right_turn_arrow(ctx, a->head_id, ARROW_GREEN, RIGHT_TURN_ARROW_S);
    printf("[I%d][Right_Turn_Task] %s arrow GREEN for %ds (main signal stays %s)\n",
           ctx->id, a->label, RIGHT_TURN_ARROW_S,
           lc_vehicle_name(a->movement == MOVE_NS_RIGHT ? ctx->ns_state : ctx->ew_state));
    fflush(stdout);
}

static void arrow_stop(lc_context_t *ctx, arrow_ref_t *a, const char *reason)
{
    if (*a->state == ARROW_OFF) return;
    lc_set_arrow_state(ctx, a->state, ARROW_OFF);
    *a->remaining_ms = 0;
    signal_set_right_turn_arrow(ctx, a->head_id, ARROW_OFF, 0);
    printf("[I%d][Right_Turn_Task] %s arrow OFF (%s)\n", ctx->id, a->label, reason);
    fflush(stdout);
}

void right_turn_init(lc_context_t *ctx)
{
    arrow_ref_t a[2];
    arrow_refs(ctx, a);
    for (int i = 0; i < 2; i++) {
        lc_set_arrow_state(ctx, a[i].state, ARROW_OFF);
        *a[i].remaining_ms = 0;
        signal_set_right_turn_arrow(ctx, a[i].head_id, ARROW_OFF, 0);
    }
}

void right_turn_on_step_change(lc_context_t *ctx)
{
    arrow_ref_t a[2];
    arrow_refs(ctx, a);
    lc_step_t step = ctx->step;

    for (int i = 0; i < 2; i++) {
        int permitted = phase_table_allows(step, a[i].movement);

        if (!permitted) {
            arrow_stop(ctx, &a[i], lc_step_name(step));
        } else if (*a[i].state == ARROW_OFF) {
            arrow_start(ctx, &a[i]);
        }
        /* Already green and still permitted: let its interval run on
         * rather than restarting it, so the configured interval means
         * what it says. */
    }
}

void right_turn_tick(lc_context_t *ctx, int tick_ms)
{
    arrow_ref_t a[2];
    arrow_refs(ctx, a);
    lc_step_t step = ctx->step;

    for (int i = 0; i < 2; i++) {
        if (*a[i].state != ARROW_GREEN) continue;

        /* A higher-priority condition may have taken the step away from
         * under a running arrow; withdraw it before ageing it. */
        if (!phase_table_allows(step, a[i].movement)) {
            arrow_stop(ctx, &a[i], "movement no longer permitted");
            continue;
        }
        *a[i].remaining_ms -= tick_ms;
        if (*a[i].remaining_ms <= 0)
            arrow_stop(ctx, &a[i], "configured interval ended");
    }
}

void right_turn_force_off(lc_context_t *ctx, const char *reason)
{
    arrow_ref_t a[2];
    arrow_refs(ctx, a);
    for (int i = 0; i < 2; i++) arrow_stop(ctx, &a[i], reason);
}
