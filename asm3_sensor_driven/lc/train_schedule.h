    /* =====================================================================
    * train_schedule.h -- the configured train timetable.
    *
    * Train events are deterministic scheduled events, not stochastic
    * arrivals and not live hardware input. At startup the timetable for
    * the next 24 simulated hours is generated from the report's frequency
    * assumptions and printed; a scheduler thread then walks it, firing
    * Train_Sensor_Task's approach event RAIL_PROTECT_LEAD_S before each
    * scheduled arrival and its cleared event once the occupation interval
    * has elapsed.
    *
    * The timetable runs on a SIMULATED clock that advances at
    * TIME_SCALE_FACTOR times real time, seeded from the wall clock (or
    * from -T HH:MM). That is what lets a demo cross a peak/off-peak
    * boundary, and it keeps train spacing scaled by exactly the same
    * factor as every other duration in the system.
    * ===================================================================== */
    #ifndef TRAIN_SCHEDULE_H
    #define TRAIN_SCHEDULE_H

    #include "lc_context.h"

    typedef enum {
        PERIOD_PEAK = 0,
        PERIOD_OFFPEAK,
        PERIOD_NIGHT
    } service_period_t;

    /* Seed the simulated clock. start_sod < 0 means "use the wall clock".
    * Must be called before train_schedule_build(). */
    void train_schedule_init(int start_sod);

    /* Generate and print the timetable for the next 24 simulated hours. */
    void train_schedule_build(lc_context_t *ctx);

    /* Start the scheduler thread that fires the timetable's events. */
    void train_schedule_start(lc_context_t *ctx);

    /* Simulated time, seconds since midnight, and the period it falls in. */
    int              train_schedule_sim_sod(void);
    service_period_t train_schedule_period(int sod);
    const char      *train_schedule_period_name(service_period_t p);

    #endif /* TRAIN_SCHEDULE_H */
