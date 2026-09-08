/* =====================================================================
 * common.h
 *
 * Shared definitions for the Traffic Light Control System (EEET2588).
 * Used by both local_controller.c (LC) and central_controller.c (CC).
 *
 * All message names, IPC primitives and timing values below follow the
 * design already agreed in Section 8 (Overview of Inter-task Messages)
 * and the assumption table (A1-A46) of the Initial Design Report.
 * Do not rename messages/fields without updating that table too.
 * ===================================================================== */
#ifndef TLS_COMMON_H
#define TLS_COMMON_H

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

/* ---------------------------------------------------------------------
 * Timing constants (A1,A2,A3,A5,A8,A9,A10,A11,A14-A18,A39; UC-08 retry)
 * All values are the PoC / demonstration values from the design report.
 * ------------------------------------------------------------------- */
#define PHASE_TICK_MS            100     /* A39: 100ms POSIX timer tick   */

#define VEHICLE_GREEN_S           45     /* A1                            */
#define VEHICLE_YELLOW_S           3     /* A2                            */
#define VEHICLE_ALL_RED_S          2     /* A3                            */
#define VEHICLE_MIN_GREEN_S       15     /* A5 (QCVN 41:2024/BGTVT)       */

#define PED_WALK_S                12     /* A8                            */
#define PED_CLEARANCE_S            4     /* A9                            */
#define PED_DEBOUNCE_MS            50    /* A11                           */

#define RAIL_PROTECT_LEAD_S       50     /* A14: RED begins before train  */
#define RAIL_WARNING_LEAD_S       15     /* A15: flashing warning first   */
#define RAIL_GATE_LOWER_S         10     /* A16: boom-gate lowering time  */
#define RAIL_TRAIN_OCCUPY_S       25     /* A18: crossing occupied        */

#define STATUS_MAX_RETRY           3     /* UC-08 retry_counter limit     */
#define STATUS_RETRY_DELAY_S       5     /* UC-08 delay after MaxTry      */

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
    MSG_SET_RAILWAY_SIGNAL,     /* Phase_Controller_Task -> Railway_Signal_Output   */
    MSG_COMMAND_BOOM_GATE       /* Phase_Controller_Task -> Boom_Gate_Controller    */
} local_msg_type_t;

typedef enum { V_RED = 0, V_YELLOW, V_GREEN } vehicle_state_t;
typedef enum { P_DONT_WALK = 0, P_WALK, P_CLEARANCE } ped_state_t;
typedef enum { R_CLEAR = 0, R_FLASHING_RED, R_RED } rail_signal_t;
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
        struct { vehicle_state_t state; int duration_s; int head_id;   } set_vehicle;
        struct { ped_state_t     state; int duration_s; int crossing_id; } set_pedestrian;
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
 * STATUS_UPDATE / FAULT_ALARM travel as MsgSendPulse() (fire-and-forget,
 * no reply expected -- matches "message is a pulse so do not need a
 * reply" in UC-06). OVERRIDE_COMMAND / MODE_SWITCH travel as blocking
 * MsgSend()/MsgReceive()/MsgReply() (matches UC-07/UC-03).
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
    control_mode_t  mode;
    uint32_t        fault_flags;
    int             fault_code;   /* only meaningful when type==NET_FAULT_ALARM */
    int             head_id;      /* only meaningful when type==NET_FAULT_ALARM */
    time_t          timestamp;
} net_report_t;

#endif /* TLS_COMMON_H */