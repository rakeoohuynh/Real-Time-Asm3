/* =====================================================================
 * train_schedule.c -- deterministic, timetable-driven train events.
 *
 * NOTE ON THE REPORT'S PHYSICAL VALUES
 *   The report gives a train velocity of 30 m/s and a sensor-to-gate
 *   distance of 1000 m, which puts the train at the gate 33.3s after
 *   the sensor trips -- not 50s, and the sensor-to-light distance of
 *   500 m puts it at the train light after 16.7s. Those figures cannot
 *   both hold with the stated 50s pre-arrival protection window. The
 *   50s window is treated as authoritative for sequencing, because it
 *   is the value the closure budget (50 + 25 + 5 = 80s) is built from;
 *   the distances are carried as constants and reported in the log so
 *   the discrepancy stays visible rather than being quietly resolved.
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include "train_schedule.h"
#include "railway_protection.h"

#define MAX_TIMETABLE_ENTRIES  512
#define SCHEDULE_HORIZON_S     (24 * 3600)
#define SCHED_POLL_MS          100

/* One scheduled arrival, expressed in simulated seconds since the
 * simulated clock was seeded. */
typedef struct {
    int  at_s;          /* simulated seconds from start */
    int  sod;           /* seconds-of-day of the arrival, for display */
    int  wday;          /* 0=Sun .. 6=Sat                             */
    service_period_t period;
} train_event_t;

static train_event_t g_timetable[MAX_TIMETABLE_ENTRIES];
static int           g_timetable_len = 0;

static struct timespec g_real_start;
static int             g_start_sod  = 0;
static int             g_start_wday = 0;

/* ---------------------------------------------------------------- clock */

static long real_elapsed_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long)(now.tv_sec - g_real_start.tv_sec) * 1000L
         + (now.tv_nsec - g_real_start.tv_nsec) / 1000000L;
}

/* Simulated seconds elapsed since the clock was seeded. */
static int sim_elapsed_s(void)
{
    return (int)((real_elapsed_ms() * TIME_SCALE_FACTOR) / 1000L);
}

int train_schedule_sim_sod(void)
{
    return (g_start_sod + sim_elapsed_s()) % 86400;
}

void train_schedule_init(int start_sod)
{
    clock_gettime(CLOCK_MONOTONIC, &g_real_start);

    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    g_start_wday = lt.tm_wday;

    if (start_sod >= 0) {
        g_start_sod = start_sod % 86400;
    } else {
        g_start_sod = lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec;
    }
}

/* ---------------------------------------------------------------- periods */

service_period_t train_schedule_period(int sod)
{
    if (sod >= NIGHT_START_SOD || sod < NIGHT_END_SOD)          return PERIOD_NIGHT;
    if (sod >= PEAK_AM_START_SOD && sod < PEAK_AM_END_SOD)      return PERIOD_PEAK;
    if (sod >= PEAK_PM_START_SOD && sod < PEAK_PM_END_SOD)      return PERIOD_PEAK;
    return PERIOD_OFFPEAK;
}

const char *train_schedule_period_name(service_period_t p)
{
    switch (p) {
        case PERIOD_PEAK:    return "PEAK";
        case PERIOD_OFFPEAK: return "OFF-PEAK";
        default:             return "NIGHT";
    }
}

static int headway_for(service_period_t p)
{
    switch (p) {
        case PERIOD_PEAK:    return TRAIN_HEADWAY_PEAK_S;
        case PERIOD_OFFPEAK: return TRAIN_HEADWAY_OFFPEAK_S;
        default:             return TRAIN_HEADWAY_NIGHT_S;
    }
}

/* The report specifies "additional Friday/Saturday night runs", i.e.
 * night services exist only on those two nights. A night that begins on
 * Friday evening runs into Saturday morning, so both the 22:00-24:00
 * part of Fri/Sat and the 00:00-06:30 part of Sat/Sun carry trains. */
static int night_service_runs(int sod, int wday)
{
    if (sod >= NIGHT_START_SOD)            /* evening half */
        return (wday == 5 || wday == 6);   /* Fri, Sat     */
    return (wday == 6 || wday == 0);       /* Sat, Sun mornings */
}

static const char *wday_name(int w)
{
    static const char *n[7] = { "Sun","Mon","Tue","Wed","Thu","Fri","Sat" };
    return n[w % 7];
}

/* ---------------------------------------------------------------- build */

void train_schedule_build(lc_context_t *ctx)
{
    g_timetable_len = 0;

    int t = 0;
    while (t < SCHEDULE_HORIZON_S && g_timetable_len < MAX_TIMETABLE_ENTRIES) {
        int abs_s = g_start_sod + t;
        int sod   = abs_s % 86400;
        int wday  = (g_start_wday + abs_s / 86400) % 7;

        service_period_t p = train_schedule_period(sod);

        if (p == PERIOD_NIGHT && !night_service_runs(sod, wday)) {
            /* No service tonight: jump to the end of the night block. */
            int next = (sod >= NIGHT_START_SOD) ? (86400 - sod + NIGHT_END_SOD)
                                                : (NIGHT_END_SOD - sod);
            t += (next > 0) ? next : 60;
            continue;
        }

        g_timetable[g_timetable_len].at_s   = t;
        g_timetable[g_timetable_len].sod    = sod;
        g_timetable[g_timetable_len].wday   = wday;
        g_timetable[g_timetable_len].period = p;
        g_timetable_len++;

        t += headway_for(p);
    }

    printf("\n[I%d][Train_Schedule] timetable built: %d services over the next 24h\n",
           ctx->id, g_timetable_len);
    printf("[I%d][Train_Schedule] clock seeded at %s %02d:%02d:%02d, running at %dx real time\n",
           ctx->id, wday_name(g_start_wday),
           g_start_sod / 3600, (g_start_sod / 60) % 60, g_start_sod % 60,
           TIME_SCALE_FACTOR);
    printf("[I%d][Train_Schedule] headways: peak %ds, off-peak %ds, Fri/Sat night %ds\n",
           ctx->id, TRAIN_HEADWAY_PEAK_S, TRAIN_HEADWAY_OFFPEAK_S, TRAIN_HEADWAY_NIGHT_S);
    printf("[I%d][Train_Schedule] train %d m/s; sensor->light %d m, sensor->gate %d m\n",
           ctx->id, TRAIN_VELOCITY_MPS, TRAIN_SENSOR_TO_LIGHT_M, TRAIN_SENSOR_TO_GATE_M);

    int show = (g_timetable_len < 8) ? g_timetable_len : 8;
    printf("[I%d][Train_Schedule] next %d services:\n", ctx->id, show);
    for (int i = 0; i < show; i++) {
        train_event_t *e = &g_timetable[i];
        printf("      %s %02d:%02d:%02d  %-8s  (sensor trips %ds earlier, +%ds real)\n",
               wday_name(e->wday), e->sod / 3600, (e->sod / 60) % 60, e->sod % 60,
               train_schedule_period_name(e->period),
               RAIL_PROTECT_LEAD_S,
               (e->at_s - RAIL_PROTECT_LEAD_S) / TIME_SCALE_FACTOR);
    }
    printf("\n");
    fflush(stdout);
}

/* ---------------------------------------------------------------- run */

static void wait_until_sim(int target_s)
{
    for (;;) {
        int now = sim_elapsed_s();
        if (now >= target_s) return;

        int remaining_sim_s = target_s - now;
        long remaining_real_ms = ((long)remaining_sim_s * 1000L) / TIME_SCALE_FACTOR;
        if (remaining_real_ms > SCHED_POLL_MS) remaining_real_ms = SCHED_POLL_MS;
        if (remaining_real_ms < 1) remaining_real_ms = 1;

        struct timespec ts = {
            .tv_sec  = remaining_real_ms / 1000,
            .tv_nsec = (remaining_real_ms % 1000) * 1000000L
        };
        nanosleep(&ts, NULL);
    }
}

static void *train_schedule_task(void *arg)
{
    lc_context_t *ctx = arg;

    for (int i = 0; i < g_timetable_len; i++) {
        train_event_t *e = &g_timetable[i];

        int sensor_at = e->at_s - RAIL_PROTECT_LEAD_S;
        if (sensor_at < sim_elapsed_s()) continue;   /* already in the past */

        wait_until_sim(sensor_at);
        printf("[I%d][Train_Schedule] scheduled %s service due %02d:%02d:%02d "
               "-- sensor tripping now (%ds pre-arrival)\n",
               ctx->id, train_schedule_period_name(e->period),
               e->sod / 3600, (e->sod / 60) % 60, e->sod % 60, RAIL_PROTECT_LEAD_S);
        fflush(stdout);
        train_sensor_handle_approach(ctx);

        /* The train reaches the crossing at e->at_s and is clear of it
         * one occupation interval later. */
        wait_until_sim(e->at_s + RAIL_TRAIN_OCCUPY_S);
        train_sensor_handle_cleared(ctx);
    }

    printf("[I%d][Train_Schedule] timetable exhausted\n", ctx->id);
    fflush(stdout);
    return NULL;
}

void train_schedule_start(lc_context_t *ctx)
{
    pthread_t th;
    pthread_create(&th, NULL, train_schedule_task, ctx);
    pthread_detach(th);
}
