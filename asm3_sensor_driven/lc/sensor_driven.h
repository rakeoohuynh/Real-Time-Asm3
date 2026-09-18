/*
 * sensor_driven.h -- sensor-driven mode.
 *
 * Once a green has run its minimum (VEHICLE_MIN_GREEN_S) and there's no
 * demand on that approach, the green ends early instead of running the
 * full VEHICLE_GREEN_S.
 */
#ifndef SENSOR_DRIVEN_H
#define SENSOR_DRIVEN_H

#include "lc_context.h"

/* Vehicle_Sensor_Task: a vehicle detected on one approach. */
void vehicle_sensor_handle_detect(lc_context_t *ctx, int is_ns);

/* Call every phase tick, before the countdown is checked. Does nothing
 * unless the mode is sensor-driven and a green is running. */
void sensor_driven_tick(lc_context_t *ctx);

#endif /* SENSOR_DRIVEN_H */
