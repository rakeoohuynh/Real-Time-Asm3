/*
 * sensor_driven_switch_demo.c
 * ---------------------------------------------------------------------
 * Proof-of-concept: SENSOR-DRIVEN mode for one intersection's
 * Phase_Controller_Task -- now BOTH approaches (NS and EW) have a
 * simulated sensor, so the demo keeps alternating on its own:
 *
 *   EW green -> NS car sensor fires -> switch to NS
 *   NS green -> EW car sensor fires -> switch to EW
 *   ... repeats forever ...
 *
 * Each sensor is simulated with a one-shot QNX timer (SIGEV_PULSE) that
 * is (re)armed automatically every time the OPPOSITE direction gets
 * green -- i.e. "while EW is green, schedule the NS car that will show
 * up"; "while NS is green, schedule the EW car that will show up".
 *
 * Safety guard (unchanged): a sensor request is only actioned once the
 * currently-green direction has already held MIN_GREEN_MS. If the car
 * arrives earlier, the request is queued (*_sensor_pending) and
 * serviced automatically as soon as the guard is satisfied.
 *
 * IPC design (consistent with Section 8 message table conventions):
 *   - All events into Phase_Controller_Task arrive as PULSES on a
 *     single channel (non-blocking, fire-and-forget), same pattern
 *     used for internal STATUS(event_type) pulses.
 *   - PULSE_CODE_PHASE_TIMER  : drives the normal FSM cycle.
 *   - PULSE_CODE_CAR_SENSOR_NS: simulated NS sensor event.
 *   - PULSE_CODE_CAR_SENSOR_EW: simulated EW sensor event.
 *
 * Build (QNX Momentics / qcc):
 *   qcc -Vgcc_ntoaarch64le -o sensor_demo sensor_driven_switch_demo.c
 *   (swap -V for your target's compiler variant, e.g. gcc_ntox86_64)
 *
 * Run:
 *   ./sensor_demo
 * ---------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

/* ---------------- Local pulse codes (node-internal) ---------------- */
#define PULSE_CODE_PHASE_TIMER    (_PULSE_CODE_MINAVAIL + 0)
#define PULSE_CODE_CAR_SENSOR_NS  (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_CODE_CAR_SENSOR_EW  (_PULSE_CODE_MINAVAIL + 2)

/* ---------------- Timing: from report Section 1.2 assumptions ------
 * These are the actual values used throughout our design, taken
 * directly from the Initial Design Report (Section 1.2):
 *   A1  Vehicle GREEN nominal (FIXED_TIMING only)      = 45000 ms
 *   A2  Vehicle YELLOW                                  =  3000 ms
 *   A3  ALL-RED clearance                               =  2000 ms
 *   A5  Minimum GREEN before a phase may be terminated  = 15000 ms
 *       (QCVN 41:2024/BGTVT statutory minimum green)
 * No artificial speed-up is applied - this runs at the same timing
 * used in the rest of our codebase / other tasks.
 * --------------------------------------------------------------------*/
#define YELLOW_MS         3000   /* A2 */
#define ALL_RED_MS        2000   /* A3 */
#define MIN_GREEN_MS     15000   /* A5 safety guard - SENSOR_DRIVEN mode only enforces this floor */

#define CAR_ARRIVAL_MS    5000   /* simulated sensor delay; intentionally < MIN_GREEN_MS
                                     so the demo exercises the A30 "safety constraints
                                     remain mandatory in SENSOR_DRIVEN mode" queued path */
#define POLL_MS           1000   /* re-check interval once min-green has elapsed */

typedef enum {
    PHASE_NS_GREEN,
    PHASE_NS_YELLOW,
    PHASE_EW_GREEN,
    PHASE_EW_YELLOW,
    PHASE_ALL_RED
} phase_t;

static const char *phase_name(phase_t p) {
    switch (p) {
        case PHASE_NS_GREEN:  return "NS_GREEN / EW_RED";
        case PHASE_NS_YELLOW: return "NS_YELLOW / EW_RED";
        case PHASE_EW_GREEN:  return "EW_GREEN / NS_RED";
        case PHASE_EW_YELLOW: return "EW_YELLOW / NS_RED";
        case PHASE_ALL_RED:   return "ALL_RED";
        default:              return "?";
    }
}

/* ---------------- Controller state ---------------------------------*/
static phase_t  current_phase     = PHASE_EW_GREEN;  /* assumption: start EW green */
static phase_t  after_all_red     = PHASE_NS_GREEN;   /* which green follows the next all-red */
static uint64_t phase_start_ms    = 0;
static int      ns_sensor_pending = 0;                /* car waiting flag on NS approach */
static int      ew_sensor_pending = 0;                /* car waiting flag on EW approach */

static int      chid;
static timer_t  phase_timer;
static timer_t  ns_sensor_timer;
static timer_t  ew_sensor_timer;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

static void arm_timer_once(timer_t tmr, int ms) {
    struct itimerspec it;
    memset(&it, 0, sizeof(it));
    it.it_value.tv_sec  = ms / 1000;
    it.it_value.tv_nsec = (ms % 1000) * 1000000L;
    timer_settime(tmr, 0, &it, NULL);
}

static void log_state(const char *reason) {
    printf("[t=%6llums] PHASE -> %-18s (%s)\n",
           (unsigned long long)now_ms(), phase_name(current_phase), reason);
    fflush(stdout);
}

/* When a direction just turned green, schedule the OPPOSITE direction's
 * simulated sensor to "detect" a car after CAR_ARRIVAL_MS. This is what
 * makes the demo keep alternating on its own. */
static void schedule_opposite_car(phase_t green_phase) {
    if (green_phase == PHASE_EW_GREEN) {
        arm_timer_once(ns_sensor_timer, CAR_ARRIVAL_MS);
        printf("            (simulated NS car will arrive in %dms)\n", CAR_ARRIVAL_MS);
    } else if (green_phase == PHASE_NS_GREEN) {
        arm_timer_once(ew_sensor_timer, CAR_ARRIVAL_MS);
        printf("            (simulated EW car will arrive in %dms)\n", CAR_ARRIVAL_MS);
    }
}

/* Move the FSM to a new phase, log it, (re)arm the phase timer, and if
 * we just entered a green phase, schedule the opposite side's sensor. */
static void enter_phase(phase_t p, int duration_ms, const char *reason) {
    current_phase  = p;
    phase_start_ms = now_ms();
    log_state(reason);
    arm_timer_once(phase_timer, duration_ms);
    schedule_opposite_car(p);
}

/* Fired when the simulated NS car sensor pulse is received. */
static void handle_ns_sensor_event(void) {
    printf("[t=%6llums] SENSOR   : car detected on NS approach\n",
           (unsigned long long)now_ms());

    if (current_phase == PHASE_NS_GREEN) {
        printf("            NS already has green - no action needed\n");
        return;
    }
    if (current_phase != PHASE_EW_GREEN) {
        /* mid-transition (yellow/all-red) - just remember the request */
        ns_sensor_pending = 1;
        return;
    }

    uint64_t elapsed = now_ms() - phase_start_ms;
    if (elapsed >= MIN_GREEN_MS) {
        ns_sensor_pending = 0;
        after_all_red = PHASE_NS_GREEN;
        enter_phase(PHASE_EW_YELLOW, YELLOW_MS,
                    "sensor-driven switch: EW min-green satisfied");
    } else {
        ns_sensor_pending = 1;
        printf("[t=%6llums] SENSOR   : request queued (EW min-green %llu/%dms elapsed)\n",
               (unsigned long long)now_ms(),
               (unsigned long long)elapsed, MIN_GREEN_MS);
    }
}

/* Fired when the simulated EW car sensor pulse is received. */
static void handle_ew_sensor_event(void) {
    printf("[t=%6llums] SENSOR   : car detected on EW approach\n",
           (unsigned long long)now_ms());

    if (current_phase == PHASE_EW_GREEN) {
        printf("            EW already has green - no action needed\n");
        return;
    }
    if (current_phase != PHASE_NS_GREEN) {
        ew_sensor_pending = 1;
        return;
    }

    uint64_t elapsed = now_ms() - phase_start_ms;
    if (elapsed >= MIN_GREEN_MS) {
        ew_sensor_pending = 0;
        after_all_red = PHASE_EW_GREEN;
        enter_phase(PHASE_NS_YELLOW, YELLOW_MS,
                    "sensor-driven switch: NS min-green satisfied");
    } else {
        ew_sensor_pending = 1;
        printf("[t=%6llums] SENSOR   : request queued (NS min-green %llu/%dms elapsed)\n",
               (unsigned long long)now_ms(),
               (unsigned long long)elapsed, MIN_GREEN_MS);
    }
}

/* Fired every time the phase timer pulse is received (normal FSM tick). */
static void handle_phase_timer(void) {
    switch (current_phase) {

    case PHASE_EW_GREEN:
        if (ns_sensor_pending) {
            ns_sensor_pending = 0;
            after_all_red = PHASE_NS_GREEN;
            enter_phase(PHASE_EW_YELLOW, YELLOW_MS,
                        "sensor-driven switch: queued NS request serviced");
        } else {
            arm_timer_once(phase_timer, POLL_MS);  /* min-green met, no car yet -> keep polling */
        }
        break;

    case PHASE_NS_GREEN:
        if (ew_sensor_pending) {
            ew_sensor_pending = 0;
            after_all_red = PHASE_EW_GREEN;
            enter_phase(PHASE_NS_YELLOW, YELLOW_MS,
                        "sensor-driven switch: queued EW request serviced");
        } else {
            arm_timer_once(phase_timer, POLL_MS);
        }
        break;

    case PHASE_EW_YELLOW:
        enter_phase(PHASE_ALL_RED, ALL_RED_MS, "clearance interval");
        break;

    case PHASE_NS_YELLOW:
        enter_phase(PHASE_ALL_RED, ALL_RED_MS, "clearance interval");
        break;

    case PHASE_ALL_RED:
        enter_phase(after_all_red, MIN_GREEN_MS,
                    after_all_red == PHASE_NS_GREEN ?
                        "NS now has right of way" : "EW now has right of way");
        break;
    }
}

int main(void) {
    struct sigevent event;
    int coid;

    chid = ChannelCreate(0);
    if (chid == -1) { perror("ChannelCreate"); exit(EXIT_FAILURE); }

    /* --- phase timer: drives the normal FSM cycle --- */
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_PHASE_TIMER, 0);
    timer_create(CLOCK_REALTIME, &event, &phase_timer);

    /* --- NS sensor timer --- */
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_CAR_SENSOR_NS, 0);
    timer_create(CLOCK_REALTIME, &event, &ns_sensor_timer);

    /* --- EW sensor timer --- */
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_CAR_SENSOR_EW, 0);
    timer_create(CLOCK_REALTIME, &event, &ew_sensor_timer);

    printf("=== Sensor-driven switching demo (single intersection, both approaches) ===\n");
    printf("Assumption: EW currently GREEN, NS RED to start.\n\n");

    /* enter_phase() automatically schedules the NS sensor timer here,
     * since we are entering PHASE_EW_GREEN. From this point on, every
     * green phase schedules the next opposite-side "car arrival". */
    enter_phase(PHASE_EW_GREEN, MIN_GREEN_MS, "startup assumption: EW has right of way");

    for (;;) {
        struct _pulse pulse;
        int rcvid = MsgReceive(chid, &pulse, sizeof(pulse), NULL);
        if (rcvid == 0) {                 /* rcvid == 0 => this is a pulse, not a message */
            switch (pulse.code) {
            case PULSE_CODE_PHASE_TIMER:
                handle_phase_timer();
                break;
            case PULSE_CODE_CAR_SENSOR_NS:
                handle_ns_sensor_event();
                break;
            case PULSE_CODE_CAR_SENSOR_EW:
                handle_ew_sensor_event();
                break;
            default:
                break;
            }
        }
        /* rcvid > 0 would be a blocking MsgSend() from e.g. the CC override
         * path (UC7) - not wired up in this minimal demo. */
    }

    return 0;
}