/* =====================================================================
 * sensor_driven.c -- sensor-driven early-cut logic.
 * ===================================================================== */
#include <stdio.h>
#include <sys/neutrino.h>
#include "sensor_driven.h"

void vehicle_sensor_handle_detect(lc_context_t *ctx, int is_ns)
{
    if (is_ns) ctx->ns_vehicle_demand = 1;
    else       ctx->ew_vehicle_demand = 1;
    printf("[I%d][Vehicle_Sensor_Task] demand detected on %s approach\n",
           ctx->id, is_ns ? "NS" : "EW");
    fflush(stdout);
    MsgSendPulse(ctx->coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_VEHICLE_DEMAND, 0);
}

void sensor_driven_tick(lc_context_t *ctx)
{
    if (ctx->mode != MODE_SENSOR_DRIVEN) return;
    if (ctx->step != STEP_NS_GREEN && ctx->step != STEP_EW_GREEN) return;

    /* Both the total and the minimum are scaled, so the ratio between
     * them -- and therefore the point at which the cut becomes legal --
     * is identical at any TIME_SCALE_FACTOR. */
    int elapsed_ms = SCALE_S_MS(VEHICLE_GREEN_S) - ctx->countdown_ms;
    if (elapsed_ms >= SCALE_S_MS(VEHICLE_MIN_GREEN_S))
        ctx->min_green_elapsed = 1;

    int demand = (ctx->step == STEP_NS_GREEN) ? ctx->ns_vehicle_demand
                                              : ctx->ew_vehicle_demand;

    if (ctx->min_green_elapsed && !demand) {
        if (ctx->step == STEP_NS_GREEN) ctx->ns_vehicle_demand = 0;
        else                            ctx->ew_vehicle_demand = 0;
        ctx->countdown_ms = 0;   /* let the boundary check below fire now */
    }
}
