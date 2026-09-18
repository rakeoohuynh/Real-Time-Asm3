/*
 * mode_schedule.h -- control mode by time of day (A26).
 *
 *   PEAK      06:30-09:00, 16:30-19:30   fixed timing
 *   OFF-PEAK  09:00-16:30, 19:30-22:00   sensor-driven
 *   NIGHT     22:00-06:30                sensor-driven
 *
 * The period comes from the simulated clock the train timetable uses,
 * so -T HH:MM moves both.
 *
 * The mode only changes when the period does. A CC MODE_SWITCH takes
 * effect right away and holds until the next period boundary.
 */
#ifndef MODE_SCHEDULE_H
#define MODE_SCHEDULE_H

#include "lc_context.h"

/* Sets the mode for the current period. Call once before the first step,
 * after train_schedule_init(). */
void mode_schedule_init(lc_context_t *ctx);

/* Call every phase tick; switches the mode when the period changes. */
void mode_schedule_tick(lc_context_t *ctx);

#endif /* MODE_SCHEDULE_H */
