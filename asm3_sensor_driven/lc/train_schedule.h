/*
 * train_schedule.h -- the train timetable.
 *
 * At startup the next 24 simulated hours of trains are generated from
 * the service-period headways and printed. A scheduler thread then
 * trips the train sensor RAIL_PROTECT_LEAD_S before each arrival and
 * reports the train clear once it has had RAIL_TRAIN_OCCUPY_S on the
 * crossing.
 *
 * The timetable runs on a simulated clock that starts at the wall-clock
 * time (or -T HH:MM) and runs TIME_SCALE_FACTOR times faster than real
 * time, so a demo can cross from peak to off-peak in a few minutes.
 */
#ifndef TRAIN_SCHEDULE_H
#define TRAIN_SCHEDULE_H

#include "lc_context.h"

typedef enum {
    PERIOD_PEAK = 0,
    PERIOD_OFFPEAK,
    PERIOD_NIGHT
} service_period_t;

/* Starts the simulated clock. start_sod < 0 uses the wall clock. Call
 * before anything reads the clock. */
void train_schedule_init(int start_sod);

/* Builds and prints the next 24 simulated hours of trains. */
void train_schedule_build(lc_context_t *ctx);

/* Starts the thread that fires the timetable's events. */
void train_schedule_start(lc_context_t *ctx);

/* Simulated time of day, in seconds since midnight. */
int              train_schedule_sim_sod(void);
service_period_t train_schedule_period(int sod);
const char      *train_schedule_period_name(service_period_t p);

#endif /* TRAIN_SCHEDULE_H */
