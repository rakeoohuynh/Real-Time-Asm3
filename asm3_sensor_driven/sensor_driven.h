/* =====================================================================
 * sensor_driven.h -- sensor-driven mode (off-peak operation).
 *
 * In this mode a vehicle GREEN that has served its statutory minimum
 * and has no remaining demand on the running approach is cut short
 * rather than run to the full nominal 45s.
 * ===================================================================== */
#ifndef SENSOR_DRIVEN_H
#define SENSOR_DRIVEN_H

#include "lc_context.h"

/* Vehicle_Sensor_Task: a loop detection on one approach. */
void vehicle_sensor_handle_detect(lc_context_t *ctx, int is_ns);

/* Called once per phase tick, before the countdown boundary is tested.
 * Does nothing unless the mode is sensor-driven and a GREEN is running. */
void sensor_driven_tick(lc_context_t *ctx);

#endif /* SENSOR_DRIVEN_H */
