/* =====================================================================
 * common.h
 *
 * Shared definitions for the Traffic Light Control System (EEET2588).
 * Used by both the Local Controller (LC) modules and central_controller.c.
 *
 * All message names, IPC primitives and timing values below follow the
 * design already agreed in Section 8 (Overview of Inter-task Messages)
 * and the assumption table (A1-A46) of the Initial Design Report.
 * Do not rename messages/fields without updating that table too.
 * ===================================================================== */
#ifndef TLS_COMMON_H
#define TLS_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <sys/neutrino.h>

/* ---------------------------------------------------------------------
 * Network naming
 *   LC channel name  : "lc_I<id>"   e.g. "lc_I1"
 *   CC channel name  : "central_controller"
 *   Local  (same node)  path: /dev/name/local/<name>
 *   Remote (QNET) path     : /net/<node_name>/dev/name/local/<name>
 * ------------------------------------------------------------------- */
#define CC_CHANNEL_NAME     "central_controller"
#define LC_CHANNEL_PREFIX   "lc_I"

static inline void lc_channel_name(char *buf, size_t len, int intersection_id)
{
    snprintf(buf, len, "%s%d", LC_CHANNEL_PREFIX, intersection_id);
}

/* =====================================================================
 * A40 -- DEMONSTRATION TIME SCALING
 *
 * The report permits real-world durations to be accelerated for the
 * demonstration provided that state ordering, transition conditions,
 * safety interlocks and the *relative* ratios between durations are
 * preserved.
 *
 * Every real-world duration in the system is declared below at its true
 * report value and is converted to milliseconds through SCALE_S_MS()
 * (or SCALE_MS()) at exactly one place: the point where it is handed to
 * a countdown or a sleep. Because that conversion is a single uniform
 * divisor, every ratio between two durations survives scaling unchanged
 * -- the 50s pre-arrival window stays 3.33x the 15s warning flash at
 * any factor.
 *
 * Set TIME_SCALE_FACTOR to 1 for true real-world timing, or override it
 * from the command line without editing this file:
 *     qcc -DTIME_SCALE_FACTOR=1 ...
 *
 * NOT scaled: PHASE_TICK_MS (A39). That is a scheduling-resolution
 * constant, not a real-world duration; it must stay fine-grained enough
 * to resolve the scaled-down durations, which the guard below checks.
 * ===================================================================== */
#ifndef TIME_SCALE_FACTOR
#define TIME_SCALE_FACTOR   5
#endif

/* Ceiling division, so no duration can ever collapse to zero ticks. */
#define SCALE_MS(ms)     ( ( (ms) + (TIME_SCALE_FACTOR) - 1 ) / (TIME_SCALE_FACTOR) )
#define SCALE_S_MS(s)    SCALE_MS( (s) * 1000 )

/* ---------------------------------------------------------------------
 * Timing constants (A1,A2,A3,A5,A8,A9,A10,A11,A14-A18,A39; UC-08 retry)
 * These are the REAL-WORLD report values. Scaling happens at use sites.
 * ------------------------------------------------------------------- */
#define PHASE_TICK_MS            100     /* A39: 100ms POSIX timer tick   */

#define VEHICLE_GREEN_S           45     /* A1                            */
#define VEHICLE_YELLOW_S           3     /* A2                            */
#define VEHICLE_ALL_RED_S          2     /* A3                            */
#define VEHICLE_MIN_GREEN_S       15     /* A5 (QCVN 41:2024/BGTVT)       */

#define PED_WALK_S                12     /* A8                            */
#define PED_CLEARANCE_S            4     /* A9                            */
#define PED_DEBOUNCE_MS            50    /* A11 (not scaled: input debounce, not a control duration) */

/* --- Railway crossing (UC-05) ------------------------------------------
 * The 80s nominal closure decomposes as:
 *   50s pre-arrival protection (A14), itself made up of
 *        3s vehicle YELLOW  + 2s ALL-RED   (clearing the intersection)
 *     + 20s quiet protection hold
 *     + 15s flashing warning (A15)
 *     + 10s boom-gate lowering (A16)
 * + 25s train occupation (A18)
 * +  5s post-train hold (A17)
 * The warning flash and the gate lowering are CONTAINED WITHIN the 50s
 * window, not added on top of it.
 * --------------------------------------------------------------------- */
#define RAIL_PROTECT_LEAD_S       50     /* A14: RED begins before train  */
#define RAIL_WARNING_LEAD_S       15     /* A15: flashing warning first   */
#define RAIL_GATE_LOWER_S         10     /* A16: boom-gate lowering time  */
#define RAIL_POST_TRAIN_HOLD_S     5     /* A17: gate stays down after    */
#define RAIL_TRAIN_OCCUPY_S       25     /* A18: crossing occupied        */

/* Derived: the quiet part of the pre-arrival window, after the
 * intersection has been cleared and before the warning flash starts. */
#define RAIL_PREARRIVAL_QUIET_S  (RAIL_PROTECT_LEAD_S - VEHICLE_YELLOW_S - \
                                  VEHICLE_ALL_RED_S - RAIL_WARNING_LEAD_S - \
                                  RAIL_GATE_LOWER_S)

#define RAIL_TOTAL_CLOSURE_S     (RAIL_PROTECT_LEAD_S + RAIL_TRAIN_OCCUPY_S + \
                                  RAIL_POST_TRAIN_HOLD_S)   /* = 80s */

/* Boom-gate fault recovery (UC-05 alternative flow). */
#define GATE_CLOSE_MAX_ATTEMPTS    3     /* initial attempt + 2 retries   */
#define GATE_RETRY_INTERVAL_S      5     /* between retries               */

/* --- Train physical characteristics (report Section 1.2) --------------
 * Reported for traceability and printed in the railway log line. See the
 * note in train_schedule.c: these distances imply a 33.3s sensor-to-gate
 * transit, which does not equal the 50s pre-arrival window; the 50s
 * figure is treated as authoritative for sequencing.
 * --------------------------------------------------------------------- */
#define TRAIN_VELOCITY_MPS          30
#define TRAIN_SENSOR_TO_LIGHT_M    500
#define TRAIN_SENSOR_TO_GATE_M    1000

/* --- Train timetable (report Section 1.2 frequency assumptions) -------
 * Seconds-of-day boundaries for the three service periods.
 * --------------------------------------------------------------------- */
#define SOD(h, m)                 (((h) * 3600) + ((m) * 60))
#define PEAK_AM_START_SOD         SOD( 6, 30)
#define PEAK_AM_END_SOD           SOD( 9,  0)
#define PEAK_PM_START_SOD         SOD(16, 30)
#define PEAK_PM_END_SOD           SOD(19, 30)
#define NIGHT_START_SOD           SOD(22,  0)
#define NIGHT_END_SOD             SOD( 6, 30)

#define TRAIN_HEADWAY_PEAK_S       120   /* ~2 min apart during peak      */
#define TRAIN_HEADWAY_OFFPEAK_S   1200   /* ~20 min apart off-peak        */
#define TRAIN_HEADWAY_NIGHT_S     1800   /* Fri/Sat night runs only       */

/* --- Right-turn arrow (auxiliary movement) ---------------------------- */
#define RIGHT_TURN_ARROW_S        20     /* configured arrow interval     */

#define STATUS_MAX_RETRY           3     /* UC-08 retry_counter limit     */
#define STATUS_RETRY_DELAY_S       5     /* UC-08 delay after MaxTry      */

/* Guard: the shortest real-world control duration must still resolve to
 * several 100ms ticks after scaling, or the FSM loses its ordering. */
#if ((VEHICLE_ALL_RED_S * 1000) / TIME_SCALE_FACTOR) < (4 * PHASE_TICK_MS)
#error "TIME_SCALE_FACTOR is too large: the ALL-RED interval no longer resolves to at least 4 timer ticks."
#endif

/* =========================== SIGNAL HEAD IDs ========================= */
#define HEAD_NS_VEHICLE        1
#define HEAD_EW_VEHICLE        2
#define HEAD_PED_CROSSING      3
#define HEAD_NS_RIGHT_ARROW    4
#define HEAD_EW_RIGHT_ARROW    5

/* Line-side railway outputs. The crossing lights face the road; the
 * train light faces the train and is what flags a gate fault to it. */
#define RAIL_SIGNAL_CROSSING   1
#define RAIL_SIGNAL_TRAIN      2

#define BOOM_GATE_ID           1

/* =========================== PULSE CODES ============================
 * Internal pulses (within one LC process, delivered on Phase_Controller
 * _Task's channel unless noted). Values must stay >= _PULSE_CODE_MINAVAIL.
 * ===================================================================== */
enum {
    PULSE_PHASE_TIMER = _PULSE_CODE_MINAVAIL, /* 100ms tick                     */
    PULSE_PED_REQUEST,                        /* Pedestrian_Input_Task          */
    PULSE_VEHICLE_DEMAND,                     /* Vehicle_Sensor_Task            */
    PULSE_TRAIN_APPROACH,                     /* Train_Sensor_Task              */
    PULSE_TRAIN_CLEARED,                      /* Train_Sensor_Task              */
    PULSE_CC_COMMAND_READY,                   /* Central_Command_Server_Task    */
    PULSE_GATE_STATUS,                        /* Boom_Gate_Controller_Task      */
    PULSE_STATUS_EVENT,                       /* -> Status_Reporting_Task       */
    PULSE_FAULT_ALARM                         /* -> Status_Reporting_Task (hi)  */
};

/* =========================== LOCAL (in-node) MESSAGES ================
 * Sent with MsgSend()/MsgReceive()/MsgReply() between tasks of the SAME
 * LC process, per the Section 8 table.
 * ===================================================================== */
typedef enum {
    MSG_SET_VEHICLE = 1,        /* Phase_Controller_Task -> Signal_Output_Task      */
    MSG_SET_PEDESTRIAN,         /* Phase_Controller_Task -> Signal_Output_Task      */
    MSG_SET_RIGHT_TURN_ARROW,   /* Phase_Controller_Task -> Signal_Output_Task      */
    MSG_SET_RAILWAY_SIGNAL,     /* Phase_Controller_Task -> Railway_Signal_Output   */
    MSG_COMMAND_BOOM_GATE       /* Phase_Controller_Task -> Boom_Gate_Controller    */
} local_msg_type_t;

typedef enum { V_RED = 0, V_YELLOW, V_GREEN } vehicle_state_t;
typedef enum { P_DONT_WALK = 0, P_WALK, P_CLEARANCE } ped_state_t;
typedef enum { R_CLEAR = 0, R_FLASHING_RED, R_RED } rail_signal_t;
typedef enum { ARROW_OFF = 0, ARROW_GREEN } arrow_state_t;
typedef enum { GATE_OPEN = 0, GATE_LOWERING, GATE_LOCKED, GATE_RAISING, GATE_TIMEOUT_FAULT } gate_state_t;
typedef enum { MODE_FIXED_TIMING = 0, MODE_SENSOR_DRIVEN, MODE_ADVANCED } control_mode_t;
typedef enum { LC_PHASE_NS = 0, LC_PHASE_EW, LC_PHASE_PED, LC_PHASE_RAIL_PROTECT, LC_PHASE_FAILSAFE } lc_phase_t;

/* Fault codes reused on both local ALARM() and network FAULT_ALARM */
#define FAULT_NONE              0
#define FAULT_SIGNAL_TIMEOUT    1
#define FAULT_PED_OUTPUT        2
#define FAULT_BOOM_GATE         3
#define FAULT_CONFIG            4

typedef struct {
    local_msg_type_t type;
    union {
        struct { vehicle_state_t state; int duration_s; int head_id;     } set_vehicle;
        struct { ped_state_t     state; int duration_s; int crossing_id; } set_pedestrian;
        struct { arrow_state_t   state; int duration_s; int head_id;     } set_arrow;
        struct { rail_signal_t   colour; int signal_id; } set_rail;
        struct { int gate_id; int command /*0=OPEN 1=CLOSE*/; int timeout_s; } boom_gate;
    } body;
} local_msg_t;

typedef struct {
    int  result;          /* 0 = ACK / GATE_LOCKED / OPENED, non-zero = FAULT code */
    int  elapsed_ms;
} local_reply_t;

/* =========================== NETWORK (LC <-> CC) MESSAGES =============
 * Exchanged over a QNET connection (ConnectAttach()/name_open()).
 * OVERRIDE_COMMAND / MODE_SWITCH travel as blocking MsgSend()/
 * MsgReceive()/MsgReply() (matches UC-07/UC-03).
 * ===================================================================== */
typedef enum {
    NET_OVERRIDE_COMMAND = 1,   /* CC -> LC, MsgSend (blocking)   */
    NET_MODE_SWITCH             /* CC -> LC, MsgSend (blocking)   */
} net_msg_type_t;

/* command_type values for NET_OVERRIDE_COMMAND */
#define OVR_FORCE_ALL_RED     0
#define OVR_FORCE_NS_GREEN    1
#define OVR_FORCE_EW_GREEN    2
#define OVR_DIGNITARY_PATH    3

typedef struct {
    net_msg_type_t type;
    int      intersection_id;
    int      command_type;      /* OVR_* above, only for NET_OVERRIDE_COMMAND     */
    control_mode_t requested_mode; /* only for NET_MODE_SWITCH                    */
    uint32_t sequence_no;
} net_command_t;

/* Reply result codes for both OVERRIDE_REPLY and MODE_SWITCH ack */
#define NET_RESULT_ACCEPTED   0
#define NET_RESULT_WAIT       1   /* rejected, retry after wait_seconds */
#define NET_RESULT_INVALID    2

typedef struct {
    int  result;                /* NET_RESULT_*        */
    int  wait_seconds;          /* used when result==NET_RESULT_WAIT */
    char reason[64];
} net_reply_t;

/* STATUS_UPDATE pulse payload -- QNX pulses only carry 4 bytes of data,
 * so the full snapshot is sent as the *value* field of a MsgSendPulse()
 * where value is treated as a pointer-sized union is NOT safe across
 * QNET (different address spaces). Instead we pack the snapshot into
 * a compact struct and send it with MsgSendPulsePtr()-style semantics
 * is unavailable across nodes either, so STATUS_UPDATE is sent as a
 * short blocking MsgSend() with a zero-length reply (CC replies
 * immediately without processing delay) -- this keeps the "LC does not
 * block waiting on CC" property in practice while remaining portable
 * over QNET. FAULT_ALARM uses the same mechanism at higher priority. */
typedef enum {
    NET_STATUS_UPDATE = 10,
    NET_FAULT_ALARM
} net_report_type_t;

typedef struct {
    net_report_type_t type;
    int             intersection_id;
    lc_phase_t      phase;
    vehicle_state_t ns_state;
    vehicle_state_t ew_state;
    ped_state_t     ped_state;
    arrow_state_t   ns_arrow;
    arrow_state_t   ew_arrow;
    gate_state_t    gate_state;
    control_mode_t  mode;
    uint32_t        fault_flags;
    int             fault_code;   /* only meaningful when type==NET_FAULT_ALARM */
    int             head_id;      /* only meaningful when type==NET_FAULT_ALARM */
    time_t          timestamp;
} net_report_t;

#endif /* TLS_COMMON_H */
