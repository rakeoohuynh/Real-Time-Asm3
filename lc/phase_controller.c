/* =============================================================
 * phase_controller.c - Phase_Controller_Task (LC core FSM)
 * See phase_controller.h for design traceability notes.
 * ============================================================= */
#include "phase_controller.h"
#include "timing.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>

/* -----------------------------------------------------------------
 * Transition table (Lab 5 Task 2A pattern: enum + switch, single step,
 * no loop inside a case). Matches Assessment 2 Fig. 4 exactly:
 *   NS_GREEN -> NS_YELLOW -> ALL_RED -> EW_GREEN -> EW_YELLOW -> ALL_RED -> (repeat)
 * ----------------------------------------------------------------- */
phase_state_t phase_next(phase_state_t current)
{
    switch (current) {
    case PHASE_NS_GREEN:  return PHASE_NS_YELLOW;
    case PHASE_NS_YELLOW: return PHASE_ALL_RED_1;
    case PHASE_ALL_RED_1: return PHASE_EW_GREEN;
    case PHASE_EW_GREEN:  return PHASE_EW_YELLOW;
    case PHASE_EW_YELLOW: return PHASE_ALL_RED_2;
    case PHASE_ALL_RED_2: return PHASE_NS_GREEN;
    default:
        /* Invalid/corrupt state - fail safe to the initial phase,
         * same robustness pattern as Lab 5's default: case. */
        fprintf(stderr, "ERROR: invalid phase state %d - resetting to safe initial phase\n",
                (int)current);
        return PHASE_NS_GREEN;
    }
}

phase_config_t phase_get_config(phase_state_t phase)
{
    phase_config_t cfg;

    switch (phase) {
    case PHASE_NS_GREEN:
        cfg.ns_signal = SIGNAL_GREEN;  cfg.ew_signal = SIGNAL_RED;    cfg.duration_ms = GREEN_MS;  break;
    case PHASE_NS_YELLOW:
        cfg.ns_signal = SIGNAL_YELLOW; cfg.ew_signal = SIGNAL_RED;    cfg.duration_ms = YELLOW_MS; break;
    case PHASE_EW_GREEN:
        cfg.ns_signal = SIGNAL_RED;    cfg.ew_signal = SIGNAL_GREEN;  cfg.duration_ms = GREEN_MS;  break;
    case PHASE_EW_YELLOW:
        cfg.ns_signal = SIGNAL_RED;    cfg.ew_signal = SIGNAL_YELLOW; cfg.duration_ms = YELLOW_MS; break;
    case PHASE_ALL_RED_1:
    case PHASE_ALL_RED_2:
    default:
        cfg.ns_signal = SIGNAL_RED;    cfg.ew_signal = SIGNAL_RED;    cfg.duration_ms = ALLRED_MS; break;
    }
    return cfg;
}

/* -----------------------------------------------------------------
 * SET_VEHICLE - blocking Send/Receive/Reply handshake to
 * Signal_Output_Task (Lab 6 pattern). Blocks until the physical
 * output is confirmed (ACK) or the call fails.
 * ----------------------------------------------------------------- */
static int send_set_vehicle(int signal_coid, intersection_id_t id,
                             phase_state_t phase, phase_config_t cfg)
{
    set_vehicle_msg_t   msg;
    set_vehicle_reply_t reply;

    msg.msg_type         = MSG_SET_VEHICLE;
    msg.intersection_id  = (uint8_t)id;
    msg.phase_id          = (uint8_t)phase;
    msg.ns_signal         = (uint8_t)cfg.ns_signal;
    msg.ew_signal         = (uint8_t)cfg.ew_signal;
    msg.duration_ms       = cfg.duration_ms;

    if (MsgSend(signal_coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        fprintf(stderr, "[%s] ERROR: MsgSend(SET_VEHICLE) failed: %s\n",
                intersection_name(id), strerror(errno));
        return -1;
    }
    return (reply.result == 0) ? 0 : -1;
}

/* STATUS - non-blocking notification to Status_Reporting_Task
 * (MsgSendPulse). Must never block the safety-critical FSM. */
static void notify_status(int status_coid, intersection_id_t id, phase_state_t phase)
{
    if (status_coid == -1) return; /* Status_Reporting_Task unavailable - LC keeps running (A34) */

    if (MsgSendPulse(status_coid, -1, PULSE_STATUS_EVENT, STATUS_PACK(id, phase)) == -1) {
        fprintf(stderr, "[%s] WARNING: MsgSendPulse(STATUS) failed: %s\n",
                intersection_name(id), strerror(errno));
    }
}

/* -----------------------------------------------------------------
 * Phase_Controller_Task main loop.
 * ----------------------------------------------------------------- */
void phase_controller_run(intersection_id_t id,
                           int signal_output_chid,
                           int status_reporting_chid,
                           volatile sig_atomic_t *shutdown_flag)
{
    pid_t self_pid = getpid();

    /* Connections to the sibling tasks' channels (same process, same
     * node - Lab 6 local ConnectAttach(ND_LOCAL_NODE, pid, chid, ...) pattern). */
    int signal_coid = ConnectAttach(ND_LOCAL_NODE, self_pid, signal_output_chid,
                                     _NTO_SIDE_CHANNEL, 0);
    if (signal_coid == -1) {
        fprintf(stderr, "[%s] FATAL: ConnectAttach(Signal_Output_Task) failed: %s\n",
                intersection_name(id), strerror(errno));
        exit(EXIT_FAILURE);
    }

    int status_coid = ConnectAttach(ND_LOCAL_NODE, self_pid, status_reporting_chid,
                                     _NTO_SIDE_CHANNEL, 0);
    if (status_coid == -1) {
        fprintf(stderr, "[%s] WARNING: ConnectAttach(Status_Reporting_Task) failed - "
                "status reporting disabled, continuing (A34)\n", intersection_name(id));
    }

    /* Own channel + periodic 100 ms POSIX timer (Lab 5 Task 3A pattern:
     * ChannelCreate -> ConnectAttach(self) -> timer_create(SIGEV_PULSE)
     * -> timer_settime -> MsgReceive loop). */
    int chid = ChannelCreate(0);
    if (chid == -1) {
        perror("ERROR: ChannelCreate(Phase_Controller_Task)");
        exit(EXIT_FAILURE);
    }

    struct sigevent event;
    event.sigev_notify = SIGEV_PULSE;
    event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (event.sigev_coid == -1) {
        perror("ERROR: ConnectAttach(self)");
        exit(EXIT_FAILURE);
    }
    /* SIGEV_PULSE_PRIO_INHERIT: the timer pulse inherits this thread's
     * priority. This is the QNX-provided equivalent of Lab 5 Task 3A's
     * manual pthread_getschedparam() lookup, used here for brevity. */
    event.sigev_priority = SIGEV_PULSE_PRIO_INHERIT;
    event.sigev_code = PULSE_TIMER_TICK;

    timer_t timer_id;
    if (timer_create(CLOCK_REALTIME, &event, &timer_id) == -1) {
        perror("ERROR: timer_create");
        exit(EXIT_FAILURE);
    }

    struct itimerspec itime;
    itime.it_value.tv_sec     = TIMER_TICK_MS / 1000u;
    itime.it_value.tv_nsec    = (TIMER_TICK_MS % 1000u) * 1000000L;
    itime.it_interval         = itime.it_value;
    if (timer_settime(timer_id, 0, &itime, NULL) == -1) {
        perror("ERROR: timer_settime");
        exit(EXIT_FAILURE);
    }

    /* Initial phase (UC-01 precondition: LC starts from a valid,
     * non-conflicting phase). */
    phase_state_t  current = PHASE_NS_GREEN;
    phase_config_t cfg     = phase_get_config(current);
    uint32_t       elapsed_ms = 0;

    printf("[%s] Initial phase: %s (NS=%s EW=%s, %ums)\n",
           intersection_name(id), phase_name(current),
           signal_colour_name(cfg.ns_signal), signal_colour_name(cfg.ew_signal),
           cfg.duration_ms);

    if (send_set_vehicle(signal_coid, id, current, cfg) != 0) {
        fprintf(stderr, "[%s] FATAL: initial signal output not confirmed\n",
                intersection_name(id));
        exit(EXIT_FAILURE);
    }
    notify_status(status_coid, id, current);

    union { struct _pulse pulse; } msg;
    int rcvid;

    for (;;) {
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) {
                if (*shutdown_flag) {
                    printf("[%s] Phase_Controller_Task shutting down.\n", intersection_name(id));
                    break;
                }
                continue;
            }
            perror("ERROR: MsgReceive");
            break;
        }
        if (rcvid != 0) continue; /* this channel only ever carries the timer pulse */
        if (msg.pulse.code != PULSE_TIMER_TICK) continue; /* ignore anything unexpected */

        elapsed_ms += TIMER_TICK_MS;
        if (elapsed_ms < cfg.duration_ms) continue; /* stay in current phase */

        phase_state_t  next     = phase_next(current);
        phase_config_t next_cfg = phase_get_config(next);

        printf("[%s] %s -> %s\n", intersection_name(id), phase_name(current), phase_name(next));

        if (send_set_vehicle(signal_coid, id, next, next_cfg) != 0) {
            /* Fail-safe: do not advance the cycle on an unconfirmed
             * output - halt rather than risk an unsafe/undisplayed
             * signal state (Assessment 2 A44: safety precedence). */
            fprintf(stderr, "[%s] FAULT: signal output not confirmed - halting fixed-timing cycle\n",
                    intersection_name(id));
            break;
        }

        current    = next;
        cfg        = next_cfg;
        elapsed_ms = 0;
        notify_status(status_coid, id, current);
    }

    timer_delete(timer_id);
    ConnectDetach(event.sigev_coid);
    ChannelDestroy(chid);
    if (status_coid != -1) ConnectDetach(status_coid);
    ConnectDetach(signal_coid);
}
