/* =====================================================================
 * lc_context.c -- the shared LC state and its accessors.
 * ===================================================================== */
#include <string.h>
#include <time.h>
#include "lc_context.h"
#include "right_turn.h"

static lc_context_t g_ctx;

lc_context_t *lc_ctx(void) { return &g_ctx; }

void lc_context_init(lc_context_t *ctx, int id, const char *cc_node)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->id = id;
    ctx->cc_node[0] = '\0';
    if (cc_node && cc_node[0])
        strncpy(ctx->cc_node, cc_node, sizeof(ctx->cc_node) - 1);

    ctx->mode         = MODE_FIXED_TIMING;
    ctx->phase        = LC_PHASE_NS;
    ctx->step         = STEP_NS_GREEN;
    ctx->gate_state   = GATE_OPEN;
    ctx->rail_signal  = R_CLEAR;
    ctx->train_signal = R_CLEAR;
    ctx->ns_arrow     = ARROW_OFF;
    ctx->ew_arrow     = ARROW_OFF;

    pthread_mutex_init(&ctx->lock, NULL);
}

/* --- step / countdown --------------------------------------------------
 * A40: the ONE place a real-world duration becomes a countdown, and so
 * the one place TIME_SCALE_FACTOR is applied to control timing. */
void lc_enter_step(lc_context_t *ctx, lc_step_t step, int real_ms)
{
    ctx->step         = step;
    ctx->countdown_ms = SCALE_MS(real_ms);
    right_turn_on_step_change(ctx);
}

void lc_enter_step_seconds(lc_context_t *ctx, lc_step_t step, int real_s)
{
    lc_enter_step(ctx, step, real_s * 1000);
}

lc_step_t lc_step(const lc_context_t *ctx)        { return ctx->step; }
int       lc_countdown_ms(const lc_context_t *ctx){ return ctx->countdown_ms; }

const char *lc_step_name(lc_step_t step)
{
    switch (step) {
    case STEP_NS_GREEN:        return "NS_GREEN";
    case STEP_NS_YELLOW:       return "NS_YELLOW";
    case STEP_NS_ALLRED:       return "NS_ALLRED";
    case STEP_EW_GREEN:        return "EW_GREEN";
    case STEP_EW_YELLOW:       return "EW_YELLOW";
    case STEP_EW_ALLRED:       return "EW_ALLRED";
    case STEP_PED_WALK:        return "PED_WALK";
    case STEP_PED_CLEARANCE:   return "PED_CLEARANCE";
    case STEP_RAIL_YELLOW:     return "RAIL_YELLOW";
    case STEP_RAIL_ALLRED:     return "RAIL_ALLRED";
    case STEP_RAIL_PREARRIVAL: return "RAIL_PREARRIVAL";
    case STEP_RAIL_WARN:       return "RAIL_WARN";
    case STEP_RAIL_GATE_LOWER: return "RAIL_GATE_LOWER";
    case STEP_RAIL_GATE_FAULT: return "RAIL_GATE_FAULT";
    case STEP_RAIL_OCCUPIED:   return "RAIL_OCCUPIED";
    case STEP_RAIL_POST_HOLD:  return "RAIL_POST_HOLD";
    case STEP_RAIL_GATE_RAISE: return "RAIL_GATE_RAISE";
    case STEP_OVERRIDE_HOLD:   return "OVERRIDE_HOLD";
    case STEP_FAILSAFE:        return "FAILSAFE";
    default:                   return "?";
    }
}

/* --- coarse phase ------------------------------------------------------ */
void lc_set_phase(lc_context_t *ctx, lc_phase_t phase)
{
    pthread_mutex_lock(&ctx->lock);
    ctx->phase = phase;
    pthread_mutex_unlock(&ctx->lock);
}

lc_phase_t lc_phase(lc_context_t *ctx)
{
    pthread_mutex_lock(&ctx->lock);
    lc_phase_t p = ctx->phase;
    pthread_mutex_unlock(&ctx->lock);
    return p;
}

/* --- mode --------------------------------------------------------------- */
void lc_set_mode(lc_context_t *ctx, control_mode_t mode)
{
    pthread_mutex_lock(&ctx->lock);
    ctx->mode = mode;
    pthread_mutex_unlock(&ctx->lock);
}

control_mode_t lc_mode(lc_context_t *ctx)
{
    pthread_mutex_lock(&ctx->lock);
    control_mode_t m = ctx->mode;
    pthread_mutex_unlock(&ctx->lock);
    return m;
}

/* --- faults ------------------------------------------------------------- */
void lc_raise_fault(lc_context_t *ctx, int fault_code)
{
    pthread_mutex_lock(&ctx->lock);
    ctx->fault_flags |= (1u << fault_code);
    pthread_mutex_unlock(&ctx->lock);
}

void lc_clear_fault(lc_context_t *ctx, int fault_code)
{
    pthread_mutex_lock(&ctx->lock);
    ctx->fault_flags &= ~(1u << fault_code);
    pthread_mutex_unlock(&ctx->lock);
}

uint32_t lc_fault_flags(lc_context_t *ctx)
{
    pthread_mutex_lock(&ctx->lock);
    uint32_t f = ctx->fault_flags;
    pthread_mutex_unlock(&ctx->lock);
    return f;
}

/* --- names -------------------------------------------------------------- */
const char *lc_vehicle_name(vehicle_state_t s)
{
    switch (s) { case V_RED: return "RED"; case V_YELLOW: return "YELLOW"; default: return "GREEN"; }
}
const char *lc_ped_name(ped_state_t s)
{
    switch (s) { case P_WALK: return "WALK"; case P_CLEARANCE: return "CLEARANCE"; default: return "DONT_WALK"; }
}
const char *lc_rail_name(rail_signal_t s)
{
    switch (s) { case R_CLEAR: return "CLEAR"; case R_FLASHING_RED: return "FLASHING_RED"; default: return "RED"; }
}
const char *lc_arrow_name(arrow_state_t s)
{
    return (s == ARROW_GREEN) ? "GREEN_ARROW" : "OFF";
}

/* --- scaled sleep ------------------------------------------------------- */
void lc_delay_real_ms(int real_ms)
{
    int ms = SCALE_MS(real_ms);
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}
