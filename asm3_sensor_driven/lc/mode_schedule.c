/* =====================================================================
 * mode_schedule.c -- time-of-day control mode (A26).
 * Runs on Phase_Controller_Task, the only writer of the mode.
 * ===================================================================== */
#include <stdio.h>
#include "mode_schedule.h"
#include "status_report.h"
#include "train_schedule.h"

static service_period_t g_period;

static control_mode_t mode_for(service_period_t p)
{
    return (p == PERIOD_PEAK) ? MODE_FIXED_TIMING : MODE_SENSOR_DRIVEN;
}

static void apply_period(lc_context_t *ctx, service_period_t p, int sod, const char *why)
{
    g_period = p;
    lc_set_mode(ctx, mode_for(p));
    printf("[I%d][Mode_Schedule] %s %02d:%02d %s period -> %s mode (A26)\n",
           ctx->id, why, sod / 3600, (sod / 60) % 60,
           train_schedule_period_name(p), lc_mode_name(mode_for(p)));
    fflush(stdout);
}

void mode_schedule_init(lc_context_t *ctx)
{
    int sod = train_schedule_sim_sod();
    apply_period(ctx, train_schedule_period(sod), sod, "start at");
}

void mode_schedule_tick(lc_context_t *ctx)
{
    int sod = train_schedule_sim_sod();
    service_period_t p = train_schedule_period(sod);
    if (p == g_period) return;

    control_mode_t before = ctx->mode;
    apply_period(ctx, p, sod, "reached");
    if (ctx->mode != before) notify_status(ctx, 0, 0, 0);
}
