/* =============================================================
 * phase_controller.h - Phase_Controller_Task (LC core FSM)
 *
 * This is the safety-critical task drawn centrally in Assessment 2
 * Fig. 18 (High-Level Implementation Diagram: QNX Task Architecture).
 * It never depends on the Central Controller for real-time correctness
 * (Assessment 2 assumption A34) and drives the intersection using a
 * deterministic fixed-time cycle only (UC-01).
 *
 * Traceability:
 *   - enum + switch single-step transition table: Lab 5 Task 2A
 *     (Implementing Traffic Lights Using a State Machine).
 *   - Periodic POSIX timer + MsgReceive loop: Lab 5 Task 3A
 *     (Variable-Timing state machine using QNX POSIX Timers).
 *   - 100 ms tick / elapsed-time monitoring: Assessment 2 A39.
 *   - SET_VEHICLE / STATUS messaging: Assessment 2 Section 8.
 * ============================================================= */
#ifndef PHASE_CONTROLLER_H
#define PHASE_CONTROLLER_H

#include "protocol.h"
#include <signal.h>

typedef struct {
    signal_colour_t ns_signal;
    signal_colour_t ew_signal;
    uint32_t        duration_ms;
} phase_config_t;

/* Lab 5 Task 2A-style single-step transition table: given the CURRENT
 * phase, returns the NEXT phase in the fixed cycle. An invalid/corrupt
 * state falls back to the safe initial phase - the same default-case
 * robustness pattern used in Lab 5's SingleStep_TrafficLight_SM(). */
phase_state_t phase_next(phase_state_t current);

/* Returns the signal-head configuration (NS/EW colours + configured
 * duration) for a given phase, per Assessment 2 assumptions A1-A4.
 * By construction this table never sets NS and EW green together
 * (Assessment 2 assumption A43: "the phase table contains only
 * non-conflicting vehicle movements"). */
phase_config_t phase_get_config(phase_state_t phase);

/* Runs the Phase_Controller_Task main loop for one intersection.
 * PERIODIC + EVENT_DRIVEN: driven by a 100 ms POSIX timer pulse (A39)
 * on its own channel. On every phase transition it commands
 * Signal_Output_Task via blocking MsgSend (SET_VEHICLE, waits for ACK)
 * and notifies Status_Reporting_Task via non-blocking MsgSendPulse
 * (STATUS) - never blocking on the Central Controller itself.
 *
 * Runs on the process's main thread and returns only when
 * *shutdown_flag becomes non-zero (set by the process's SIGINT/SIGTERM
 * handler), which interrupts the blocking MsgReceive() with EINTR. */
void phase_controller_run(intersection_id_t id,
                           int signal_output_chid,
                           int status_reporting_chid,
                           volatile sig_atomic_t *shutdown_flag);

#endif /* PHASE_CONTROLLER_H */
