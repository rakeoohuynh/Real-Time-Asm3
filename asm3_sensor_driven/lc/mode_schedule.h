/* =====================================================================
 * mode_schedule.h -- time-of-day control mode (A26).
 *
 * The LC chooses its own control mode from the service period:
 *   PEAK      06:30-09:00, 16:30-19:30   -> fixed timing
 *   OFF-PEAK  09:00-16:30, 19:30-22:00   -> sensor-driven
 *   NIGHT     22:00-06:30                -> sensor-driven
 *
 * The period is read from the simulated clock, the same one the train
 * timetable runs on, so -T HH:MM moves both together.
 *
 * The mode is set only when the period changes. A CC MODE_SWITCH still
 * applies at once and holds until the next period boundary, where the
 * schedule takes over again.
 * ===================================================================== */
#ifndef MODE_SCHEDULE_H
#define MODE_SCHEDULE_H

#include "lc_context.h"

/* Set the mode for the current period. Call once, before the first
 * phase step, after train_schedule_init() has seeded the clock. */
void mode_schedule_init(lc_context_t *ctx);

/* Called once per phase tick: switches the mode when the period changes. */
void mode_schedule_tick(lc_context_t *ctx);

#endif /* MODE_SCHEDULE_H */
