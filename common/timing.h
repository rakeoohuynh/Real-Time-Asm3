/* =============================================================
 * timing.h - Fixed-timing configuration (UC-01 FIXED-TIMING OPERATION)
 *
 * Values are the PoC engineering assumptions from EEET2588 Assessment 2
 * (Team NTC), Table 1:
 *   A1  Vehicle GREEN   nominally 45 s
 *   A2  Vehicle YELLOW  nominally 3 s
 *   A3  ALL-RED interval 2 s (configurable clearance assumption)
 *   A4  Sequence: NS GREEN 45s -> YELLOW 3s -> ALL-RED 2s ->
 *                 EW GREEN 45s -> YELLOW 3s -> ALL-RED 2s
 *       (nominal 100 s cycle)
 *
 * DEMO_SCALE divides every duration so the same, unmodified state
 * ordering and transition logic can be observed in a reasonable
 * demonstration time, per Assessment 2 assumption A40: "Real-world
 * durations may be accelerated during the QNX demonstration while
 * preserving the same state ordering, transition conditions, safety
 * interlocks, and relative timing logic." Set DEMO_SCALE to 1 to run
 * the real A1-A4 durations unchanged.
 * ============================================================= */
#ifndef TIMING_H
#define TIMING_H

#define DEMO_SCALE 10u

#define REAL_GREEN_MS  45000u
#define REAL_YELLOW_MS  3000u
#define REAL_ALLRED_MS  2000u

#define GREEN_MS   (REAL_GREEN_MS  / DEMO_SCALE)
#define YELLOW_MS  (REAL_YELLOW_MS / DEMO_SCALE)
#define ALLRED_MS  (REAL_ALLRED_MS / DEMO_SCALE)

/* Periodic POSIX timer tick used by Phase_Controller_Task to monitor
 * elapsed phase time (Assessment 2 assumption A39: "The LC uses a
 * 100 ms periodic POSIX timer tick for the PoC" - provides sufficient
 * scheduling resolution for elapsed-time monitoring; it does NOT imply
 * physical signals change every 100 ms). */
#define TIMER_TICK_MS 100u

#endif /* TIMING_H */
