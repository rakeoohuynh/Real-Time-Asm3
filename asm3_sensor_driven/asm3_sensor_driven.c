/*
 * sensor_driven_switch_demo.c
 * ---------------------------------------------------------------------
 * Proof-of-concept: SENSOR-DRIVEN mode for one intersection's
 * Phase_Controller_Task. Covers UC-02 (vehicle sensors, both NS and EW)
 * plus UC-04 (pedestrian crossing request).
 *
 *   EW green -> NS car sensor fires -> switch to NS
 *   NS green -> EW car sensor fires -> switch to EW
 *   Pedestrian button press (keyboard, see below) -> forces BOTH
 *   directions to stop (YELLOW -> ALL-RED -> WALK -> CLEARANCE) before
 *   vehicle control resumes, exactly as UC-04's Main Flow describes.
 *
 * PEDESTRIAN INPUT (this version): a separate Pedestrian_Input_Task
 * thread reads from stdin. Type 'p' then Enter to simulate a real
 * pedestrian pressing the crossing button - this mirrors the report's
 * architecture where a dedicated Pedestrian_Input_Task debounces the
 * physical button and sends PED_REQUEST (here: MsgSendPulse) to
 * Phase_Controller_Task, rather than the controller polling hardware
 * itself. Vehicle sensors are still timer-simulated (unchanged).
 *
 * Vehicle sensor timing values: A2, A3, A5 (Section 1.2)
 * Pedestrian timing values:     A8 (WALK=12s), A9 (CLEARANCE=4s),
 *                                A11 (debounce=50ms), A12 (latch)
 *
 * UC-04 alternative flows implemented:
 *   A1 - repeated button press while a request is pending -> ignored
 *   A2 - request arrives during vehicle minimum GREEN -> latched,
 *        serviced once MIN_GREEN_MS elapses
 *   A3 - request arrives during YELLOW/ALL-RED -> latched, serviced
 *        at the next ALL-RED boundary
 *   A5 - SENSOR_DRIVEN: pedestrian demand is reconsidered by the
 *        scheduler alongside vehicle demand (pedestrian is given
 *        priority over a same-cycle vehicle-sensor request once
 *        min-green is satisfied)
 *
 * IPC design (consistent with Section 8 message table conventions):
 *   - All events into Phase_Controller_Task arrive as PULSES on a
 *     single channel (non-blocking, fire-and-forget), same pattern
 *     used for internal STATUS(event_type) pulses.
 *   - PULSE_CODE_PHASE_TIMER  : drives the normal FSM cycle.
 *   - PULSE_CODE_CAR_SENSOR_NS: simulated NS vehicle sensor event.
 *   - PULSE_CODE_CAR_SENSOR_EW: simulated EW vehicle sensor event.
 *   - PULSE_CODE_PED_BUTTON   : pedestrian push-button event, sent
 *                                explicitly by Pedestrian_Input_Task
 *                                via MsgSendPulse() (not a kernel timer).
 *
 * Build (QNX Momentics / qcc):
 *   qcc -Vgcc_ntoaarch64le -o sensor_demo sensor_driven_switch_demo.c
 *   (swap -V for your target's compiler variant, e.g. gcc_ntox86_64)
 *
 * Run:
 *   ./sensor_demo
 *   (then type p + Enter at any time to simulate a pedestrian press)
 * ---------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

/* ---------------- Local pulse codes (node-internal) ---------------- */
#define PULSE_CODE_PHASE_TIMER    (_PULSE_CODE_MINAVAIL + 0)
#define PULSE_CODE_CAR_SENSOR_NS  (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_CODE_CAR_SENSOR_EW  (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_CODE_PED_BUTTON     (_PULSE_CODE_MINAVAIL + 3)

/* ---------------- Timing: from report Section 1.2 assumptions ------
 *   A2  Vehicle YELLOW                                  =  3000 ms
 *   A3  ALL-RED clearance                               =  2000 ms
 *   A5  Minimum GREEN before a phase may be terminated  = 15000 ms
 *       (QCVN 41:2024/BGTVT statutory minimum green)
 *   A8  Pedestrian WALK                                 = 12000 ms
 *   A9  Pedestrian flashing CLEARANCE                   =  4000 ms
 *   A11 Pedestrian push-button debounce                 =    50 ms
 * No artificial speed-up is applied - real timing throughout.
 * --------------------------------------------------------------------*/
#define YELLOW_MS         3000   /* A2 */
#define ALL_RED_MS        2000   /* A3 */
#define MIN_GREEN_MS     15000   /* A5 safety guard - SENSOR_DRIVEN mode only enforces this floor */
#define PED_WALK_MS      12000   /* A8 */
#define PED_CLEARANCE_MS  4000   /* A9 */
#define PED_DEBOUNCE_MS     50   /* A11 - enforced here in Pedestrian_Input_Task */

#define CAR_ARRIVAL_MS    5000   /* simulated vehicle sensor delay; intentionally < MIN_GREEN_MS
                                     so the demo exercises the queued path */
#define POLL_MS           1000   /* re-check interval once min-green has elapsed */

typedef enum {
    PHASE_NS_GREEN,
    PHASE_NS_YELLOW,
    PHASE_EW_GREEN,
    PHASE_EW_YELLOW,
    PHASE_ALL_RED,
    PHASE_PED_WALK,
    PHASE_PED_CLEARANCE
} phase_t;

static const char *phase_name(phase_t p) {
    switch (p) {
        case PHASE_NS_GREEN:      return "NS_GREEN / EW_RED";
        case PHASE_NS_YELLOW:     return "NS_YELLOW / EW_RED";
        case PHASE_EW_GREEN:      return "EW_GREEN / NS_RED";
        case PHASE_EW_YELLOW:     return "EW_YELLOW / NS_RED";
        case PHASE_ALL_RED:       return "ALL_RED";
        case PHASE_PED_WALK:      return "ALL_RED / PED_WALK";
        case PHASE_PED_CLEARANCE: return "ALL_RED / PED_CLEARANCE (flashing)";
        default:                  return "?";
    }
}

/* ---------------- Controller state ---------------------------------*/
static phase_t  current_phase     = PHASE_EW_GREEN;  /* assumption: start EW green */
static phase_t  after_all_red     = PHASE_NS_GREEN;   /* which green follows the next all-red */
static uint64_t phase_start_ms    = 0;
static int      ns_sensor_pending = 0;                /* car waiting flag on NS approach */
static int      ew_sensor_pending = 0;                /* car waiting flag on EW approach */
static int      ped_pending       = 0;                /* A12: latched pedestrian request */

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
    printf("[t=%6llums] PHASE -> %-32s (%s)\n",
           (unsigned long long)now_ms(), phase_name(current_phase), reason);
    fflush(stdout);
}

/* When a direction just turned green, schedule the OPPOSITE direction's
 * simulated vehicle sensor to "detect" a car after CAR_ARRIVAL_MS. */
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
 * we just entered a vehicle green phase, schedule the opposite side's
 * vehicle sensor. */
static void enter_phase(phase_t p, int duration_ms, const char *reason) {
    current_phase  = p;
    phase_start_ms = now_ms();
    log_state(reason);
    arm_timer_once(phase_timer, duration_ms);
    schedule_opposite_car(p);
}

/* ---------------- Vehicle sensor handlers --------------------------*/

static void handle_ns_sensor_event(void) {
    printf("[t=%6llums] SENSOR   : car detected on NS approach\n",
           (unsigned long long)now_ms());

    if (current_phase == PHASE_NS_GREEN) {
        printf("            NS already has green - no action needed\n");
        return;
    }
    if (current_phase != PHASE_EW_GREEN) {
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

/* ---------------- Pedestrian button handler (UC-04) ------------------
 * A1 - repeated press while pending -> ignored (latched request unchanged)
 * A2 - press during vehicle minimum GREEN -> latched, waits for MIN_GREEN_MS
 * A3 - press during YELLOW/ALL-RED -> latched, served at next ALL-RED
 * A5 - SENSOR_DRIVEN: pedestrian demand reconsidered by the scheduler
 * ---------------------------------------------------------------------*/
static void handle_ped_button_event(void) {
    printf("[t=%6llums] SENSOR   : pedestrian button pressed\n",
           (unsigned long long)now_ms());

    if (ped_pending) {
        /* A1: repeated press while request is pending */
        printf("            A1: request already pending - press ignored (latched)\n");
        return;
    }

    if (current_phase == PHASE_EW_GREEN || current_phase == PHASE_NS_GREEN) {
        phase_t  interrupted = current_phase;
        uint64_t elapsed     = now_ms() - phase_start_ms;

        after_all_red = (interrupted == PHASE_EW_GREEN) ? PHASE_NS_GREEN : PHASE_EW_GREEN;

        if (elapsed >= MIN_GREEN_MS) {
            ped_pending = 1;
            if (interrupted == PHASE_EW_GREEN) {
                ns_sensor_pending = 0;  /* this switch also satisfies any pending NS car request */
                enter_phase(PHASE_EW_YELLOW, YELLOW_MS,
                            "pedestrian request: min-green satisfied, clearing traffic");
            } else {
                ew_sensor_pending = 0;  /* this switch also satisfies any pending EW car request */
                enter_phase(PHASE_NS_YELLOW, YELLOW_MS,
                            "pedestrian request: min-green satisfied, clearing traffic");
            }
        } else {
            /* A2: request arrives during minimum vehicle green */
            ped_pending = 1;
            printf("[t=%6llums] SENSOR   : A2 - request latched (vehicle min-green %llu/%dms elapsed)\n",
                   (unsigned long long)now_ms(),
                   (unsigned long long)elapsed, MIN_GREEN_MS);
        }
    } else {
        /* A3: request arrives during yellow or all-red - served at next ALL-RED boundary */
        ped_pending = 1;
        printf("            A3: request latched (vehicle mid-transition)\n");
    }
}

/* ---------------- Normal FSM tick ------------------------------------*/
static void handle_phase_timer(void) {
    switch (current_phase) {

    case PHASE_EW_GREEN:
        if (ped_pending) {
            ped_pending = 0;
            ns_sensor_pending = 0;
            after_all_red = PHASE_NS_GREEN;
            enter_phase(PHASE_EW_YELLOW, YELLOW_MS,
                        "pedestrian request serviced (queued, A2)");
        } else if (ns_sensor_pending) {
            ns_sensor_pending = 0;
            after_all_red = PHASE_NS_GREEN;
            enter_phase(PHASE_EW_YELLOW, YELLOW_MS,
                        "sensor-driven switch: queued NS request serviced");
        } else {
            arm_timer_once(phase_timer, POLL_MS);  /* min-green met, nothing pending -> keep polling */
        }
        break;

    case PHASE_NS_GREEN:
        if (ped_pending) {
            ped_pending = 0;
            ew_sensor_pending = 0;
            after_all_red = PHASE_EW_GREEN;
            enter_phase(PHASE_NS_YELLOW, YELLOW_MS,
                        "pedestrian request serviced (queued, A2)");
        } else if (ew_sensor_pending) {
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
        if (ped_pending) {
            /* A3 path lands here: request was latched during YELLOW/ALL-RED */
            ped_pending = 0;
            enter_phase(PHASE_PED_WALK, PED_WALK_MS, "pedestrian WALK (A8: 12s)");
        } else {
            enter_phase(after_all_red, MIN_GREEN_MS,
                        after_all_red == PHASE_NS_GREEN ?
                            "NS now has right of way" : "EW now has right of way");
        }
        break;

    case PHASE_PED_WALK:
        enter_phase(PHASE_PED_CLEARANCE, PED_CLEARANCE_MS,
                    "pedestrian flashing clearance (A9: 4s)");
        break;

    case PHASE_PED_CLEARANCE:
        enter_phase(after_all_red, MIN_GREEN_MS,
                    after_all_red == PHASE_NS_GREEN ?
                        "NS now has right of way (post-pedestrian)" :
                        "EW now has right of way (post-pedestrian)");
        break;
    }
}

/* ---------------- Pedestrian_Input_Task ------------------------------
 * Separate thread mirroring the report's architecture: a dedicated
 * input task owns the physical button (here: stdin), applies debounce
 * (A11), and sends a PED_REQUEST pulse to Phase_Controller_Task. The
 * controller itself never touches stdin/hardware directly.
 *
 * Type 'p' then Enter at the console at any time to simulate a press.
 * ---------------------------------------------------------------------*/
static void *pedestrian_input_task(void *arg) {
    (void)arg;
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) { perror("ConnectAttach (Pedestrian_Input_Task)"); return NULL; }

    printf("[Pedestrian_Input_Task] ready - type 'p' then Enter to press the crossing button.\n");
    fflush(stdout);

    uint64_t last_press_ms = 0;
    char     line[16];

    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (line[0] != 'p' && line[0] != 'P') {
            continue;
        }
        uint64_t t = now_ms();
        if (t - last_press_ms < PED_DEBOUNCE_MS) {
            continue;  /* A11: debounce - suppress bounce within 50ms */
        }
        last_press_ms = t;
        MsgSendPulse(coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_PED_BUTTON, 0);
    }
    return NULL;
}

int main(void) {
    struct sigevent event;
    int coid;
    pthread_t ped_input_thread;

    chid = ChannelCreate(0);
    if (chid == -1) { perror("ChannelCreate"); exit(EXIT_FAILURE); }

    /* --- phase timer: drives the normal FSM cycle --- */
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_PHASE_TIMER, 0);
    timer_create(CLOCK_REALTIME, &event, &phase_timer);

    /* --- NS vehicle sensor timer --- */
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_CAR_SENSOR_NS, 0);
    timer_create(CLOCK_REALTIME, &event, &ns_sensor_timer);

    /* --- EW vehicle sensor timer --- */
    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_CAR_SENSOR_EW, 0);
    timer_create(CLOCK_REALTIME, &event, &ew_sensor_timer);

    /* --- Pedestrian_Input_Task: real keyboard-triggered button --- */
    if (pthread_create(&ped_input_thread, NULL, pedestrian_input_task, NULL) != 0) {
        perror("pthread_create (Pedestrian_Input_Task)");
        exit(EXIT_FAILURE);
    }

    printf("=== Sensor-driven demo: NS/EW vehicle sensors (timer) + pedestrian button (keyboard) ===\n");
    printf("Assumption: EW currently GREEN, NS RED to start.\n\n");

    /* enter_phase() automatically schedules the NS vehicle sensor here,
     * since we are entering PHASE_EW_GREEN. */
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
            case PULSE_CODE_PED_BUTTON:
                handle_ped_button_event();
                break;
            default:
                break;
            }
        }
        /* rcvid > 0 would be a blocking MsgSend() from e.g. the CC override
         * path (UC-07) - not wired up in this minimal demo. */
    }

    return 0;
}