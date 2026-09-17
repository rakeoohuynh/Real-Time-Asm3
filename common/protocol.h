/* =============================================================
 * protocol.h - Shared wire protocol between Local Controllers (LC)
 *              and the Central Controller (CC)
 *
 * Scope: EEET2588 Real-Time Systems - Assessment 3, Phase 1
 *        Intersections I1 and I2, fixed_time mode only.
 *
 * Traceability:
 *   - Intersection identity / signal model: Assessment 2 (Team NTC),
 *     Section 3 (Intersection Layout Diagrams) and Section 4
 *     (Vehicle signal head state chart, Fig. 4).
 *   - Pulse codes: Lab 5 Task 3A (timer_create + SIGEV_PULSE +
 *     MsgReceive) for the periodic tick; Assessment 2 Section 8
 *     (Overview of Inter-task Messages) for STATUS(event_type) and
 *     STATUS_UPDATE, both defined there as MsgSendPulse messages.
 *   - SET_VEHICLE message: Assessment 2 Section 8, sent by
 *     Phase_Controller_Task to Signal_Output_Task via blocking
 *     MsgSend and confirmed by MsgReply (Lab 6 Send/Receive/Reply).
 *
 * This header contains ONLY the shared wire format. LC-only phase
 * transition/timing logic lives in lc/phase_controller.h.
 * ============================================================= */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <sys/neutrino.h>

/* -------------------------------------------------------------
 * Intersection identity
 *
 * Only I1 and I2 are implemented in this phase. Assessment 2 notes
 * I2-I6 all share I1's task architecture (Section 7: "I1 is fully
 * expanded... I2-I6 use the same architecture"), so a single LC
 * executable is run once per intersection, selected by argv[1].
 * ------------------------------------------------------------- */
typedef enum {
    INTERSECTION_I1 = 1,
    INTERSECTION_I2 = 2
} intersection_id_t;

const char *intersection_name(intersection_id_t id);

/* Signal head colour. */
typedef enum {
    SIGNAL_RED    = 0,
    SIGNAL_YELLOW = 1,
    SIGNAL_GREEN  = 2
} signal_colour_t;

const char *signal_colour_name(signal_colour_t c);

/* -------------------------------------------------------------
 * Vehicle-phase state machine identifiers.
 *
 * Six states, matching Assessment 2 Fig. 4 (Vehicle signal head
 * state diagram) exactly:
 *   NS_GREEN -> NS_YELLOW -> ALL_RED -> EW_GREEN -> EW_YELLOW -> ALL_RED -> (repeat)
 *
 * Declared here (not phase_controller.h) because the phase id also
 * travels on the wire: as a field in set_vehicle_msg_t, and packed
 * into the STATUS/STATUS_UPDATE pulse value sent to the CC.
 * ------------------------------------------------------------- */
typedef enum {
    PHASE_NS_GREEN  = 0,
    PHASE_NS_YELLOW = 1,
    PHASE_ALL_RED_1 = 2,
    PHASE_EW_GREEN  = 3,
    PHASE_EW_YELLOW = 4,
    PHASE_ALL_RED_2 = 5
} phase_state_t;

const char *phase_name(phase_state_t phase);

/* -------------------------------------------------------------
 * Pulse codes (asynchronous, non-blocking notifications).
 *
 * A QNX pulse carries only a small integer payload (code + value),
 * never a full struct - see Lab 5 Task 3A. All three uses below are
 * sized to fit that constraint.
 * ------------------------------------------------------------- */
#define PULSE_TIMER_TICK    (_PULSE_CODE_MINAVAIL + 0) /* Phase_Controller_Task's own 100 ms tick (self, Lab 5 Task 3A pattern) */
#define PULSE_STATUS_EVENT  (_PULSE_CODE_MINAVAIL + 1) /* Phase_Controller_Task -> Status_Reporting_Task */
#define PULSE_STATUS_UPDATE (_PULSE_CODE_MINAVAIL + 2) /* Status_Reporting_Task -> Central_Controller */
#define PULSE_SHUTDOWN      (_PULSE_CODE_MINAVAIL + 3) /* Clean-shutdown request, sent to a task's own channel */

/* Pack/unpack the compact status payload carried in a pulse's value
 * field: intersection id in the high byte, phase id in the low byte. */
#define STATUS_PACK(intersection, phase)   (((int)(intersection) << 8) | (int)(phase))
#define STATUS_UNPACK_INTERSECTION(v)      (((v) >> 8) & 0xFF)
#define STATUS_UNPACK_PHASE(v)             ((v) & 0xFF)

/* -------------------------------------------------------------
 * Blocking Send/Receive/Reply message: SET_VEHICLE
 *
 * Phase_Controller_Task -> Signal_Output_Task, per Assessment 2
 * Section 8. Confirmed by an ACK reply before the phase controller
 * considers the transition complete (UC-01 main flow, step 9).
 * ------------------------------------------------------------- */
#define MSG_SET_VEHICLE 0x01u

typedef struct {
    uint16_t msg_type;      /* MSG_SET_VEHICLE */
    uint8_t  intersection_id;
    uint8_t  phase_id;      /* phase_state_t, carried for logging/display */
    uint8_t  ns_signal;     /* signal_colour_t */
    uint8_t  ew_signal;     /* signal_colour_t */
    uint32_t duration_ms;   /* configured duration of this phase */
} set_vehicle_msg_t;

typedef struct {
    uint8_t result;         /* 0 = ACK (output confirmed), non-zero = rejected/fault */
} set_vehicle_reply_t;

#endif /* PROTOCOL_H */
