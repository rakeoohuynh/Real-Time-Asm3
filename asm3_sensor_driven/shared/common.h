/*
 * common.h
 *
 * Definitions shared by the Local Controller (LC) and the Central
 * Controller (CC): channel names, timing constants and message layouts.
 * The An / UC-nn tags next to constants are the requirement IDs they
 * implement; keep names and fields in sync with those requirements.
 */
#ifndef TLS_COMMON_H
#define TLS_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <sys/neutrino.h>

/*
 * Channel names. An LC registers "lc_I<id>" (e.g. "lc_I1"), the CC
 * registers "central_controller". Same node: /dev/name/local/<name>.
 * Over QNET: /net/<node>/dev/name/local/<name>.
 */
#define CC_CHANNEL_NAME     "central_controller"
#define LC_CHANNEL_PREFIX   "lc_I"

static inline void lc_channel_name(char *buf, size_t len, int intersection_id)
{
    snprintf(buf, len, "%s%d", LC_CHANNEL_PREFIX, intersection_id);
}

/*
 * Demo time scaling (A40).
 *
 * All durations below are real-world values. They are divided by
 * TIME_SCALE_FACTOR in exactly one place, where they become a countdown
 * or a sleep, so every ratio between durations is the same at any
 * factor. State order and safety interlocks don't change with the
 * factor; only the wall-clock length of each step does.
 *
 * Build with -DTIME_SCALE_FACTOR=1 for real-world timing.
 *
 * PHASE_TICK_MS is not scaled: it's the scheduler resolution, and it has
 * to stay fine enough to resolve the shortest scaled step (see the check
 * below).
 */
#ifndef TIME_SCALE_FACTOR
#define TIME_SCALE_FACTOR   5
#endif

/* Round up so a short duration never scales down to zero ticks. */
#define SCALE_MS(ms)     ( ( (ms) + (TIME_SCALE_FACTOR) - 1 ) / (TIME_SCALE_FACTOR) )
#define SCALE_S_MS(s)    SCALE_MS( (s) * 1000 )

#define PHASE_TICK_MS            100     /* A39 */

#define VEHICLE_GREEN_S           45     /* A1 */
#define VEHICLE_YELLOW_S           3     /* A2 */
#define VEHICLE_ALL_RED_S          2     /* A3 */
#define VEHICLE_MIN_GREEN_S       15     /* A5, QCVN 41:2024/BGTVT */

#define PED_WALK_S                12     /* A8 */
#define PED_CLEARANCE_S            4     /* A9 */
#define PED_DEBOUNCE_MS            50    /* A11; input debounce, not scaled */

/*
 * Railway crossing (UC-05). The nominal 80s closure is:
 *   50s before the train arrives (A14), made up of
 *        3s yellow + 2s all-red to clear the intersection,
 *       20s quiet hold,
 *       15s flashing warning (A15),
 *       10s gate lowering (A16)
 *   25s train on the crossing (A18)
 *    5s gate held down after the train (A17)
 * The warning and the gate lowering happen inside the 50s window, not
 * after it.
 */
#define RAIL_PROTECT_LEAD_S       50     /* A14 */
#define RAIL_WARNING_LEAD_S       15     /* A15 */
#define RAIL_GATE_LOWER_S         10     /* A16 */
#define RAIL_POST_TRAIN_HOLD_S     5     /* A17 */
#define RAIL_TRAIN_OCCUPY_S       25     /* A18 */

/* The quiet part of the lead window, between clearing the intersection
 * and starting the warning. */
#define RAIL_PREARRIVAL_QUIET_S  (RAIL_PROTECT_LEAD_S - VEHICLE_YELLOW_S - \
                                  VEHICLE_ALL_RED_S - RAIL_WARNING_LEAD_S - \
                                  RAIL_GATE_LOWER_S)

#define RAIL_TOTAL_CLOSURE_S     (RAIL_PROTECT_LEAD_S + RAIL_TRAIN_OCCUPY_S + \
                                  RAIL_POST_TRAIN_HOLD_S)   /* 80s */

#define GATE_CLOSE_MAX_ATTEMPTS    3     /* first try + 2 retries */
#define GATE_RETRY_INTERVAL_S      5

/* Physical train figures. Only printed; sequencing uses
 * RAIL_PROTECT_LEAD_S. See the note at the top of train_schedule.c. */
#define TRAIN_VELOCITY_MPS          30
#define TRAIN_SENSOR_TO_LIGHT_M    500
#define TRAIN_SENSOR_TO_GATE_M    1000

/* Service periods, as seconds of the day. */
#define SOD(h, m)                 (((h) * 3600) + ((m) * 60))
#define PEAK_AM_START_SOD         SOD( 6, 30)
#define PEAK_AM_END_SOD           SOD( 9,  0)
#define PEAK_PM_START_SOD         SOD(16, 30)
#define PEAK_PM_END_SOD           SOD(19, 30)
#define NIGHT_START_SOD           SOD(22,  0)
#define NIGHT_END_SOD             SOD( 6, 30)

/* Train headways on the simulated clock. Peak and off-peak are fixed in
 * real time, so a tester sees the same spacing at any scale factor and
 * there's room for pedestrian tests between closures (one closure is
 * about 90 simulated seconds). The night headway is simulated time. */
#define TRAIN_HEADWAY_PEAK_S      (2 * 60 * TIME_SCALE_FACTOR)  /* 2 min real */
#define TRAIN_HEADWAY_OFFPEAK_S   (4 * 60 * TIME_SCALE_FACTOR)  /* 4 min real */
#define TRAIN_HEADWAY_NIGHT_S     1800   /* Fri/Sat nights only */

#define RIGHT_TURN_ARROW_S        20

#define STATUS_MAX_RETRY           3     /* UC-08 */
#define STATUS_RETRY_DELAY_S       5     /* UC-08: back-off once retries run out */

/* Too large a scale factor would squeeze the all-red interval below a
 * few ticks, and the state machine could skip it. */
#if ((VEHICLE_ALL_RED_S * 1000) / TIME_SCALE_FACTOR) < (4 * PHASE_TICK_MS)
#error "TIME_SCALE_FACTOR is too large: the all-red interval would be shorter than 4 timer ticks."
#endif

/* Signal head IDs. */
#define HEAD_NS_VEHICLE        1
#define HEAD_EW_VEHICLE        2
#define HEAD_PED_CROSSING      3
#define HEAD_NS_RIGHT_ARROW    4
#define HEAD_EW_RIGHT_ARROW    5

/* Railway outputs. The crossing lights face the road; the train light
 * faces the train and is how a gate fault is signaled to it. */
#define RAIL_SIGNAL_CROSSING   1
#define RAIL_SIGNAL_TRAIN      2

#define BOOM_GATE_ID           1

/* Pulses inside one LC process. Unless noted they go to the
 * Phase_Controller_Task channel. Codes must stay >= _PULSE_CODE_MINAVAIL. */
enum {
    PULSE_PHASE_TIMER = _PULSE_CODE_MINAVAIL, /* 100ms tick */
    PULSE_PED_REQUEST,                        /* from Pedestrian_Input_Task */
    PULSE_VEHICLE_DEMAND,                     /* from Vehicle_Sensor_Task */
    PULSE_TRAIN_APPROACH,                     /* from Train_Sensor_Task */
    PULSE_TRAIN_CLEARED,                      /* from Train_Sensor_Task */
    PULSE_CC_COMMAND_READY,                   /* from Central_Command_Server_Task */
    PULSE_GATE_STATUS,                        /* from Boom_Gate_Controller_Task */
    PULSE_STATUS_EVENT,                       /* to Status_Reporting_Task */
    PULSE_FAULT_ALARM                         /* to Status_Reporting_Task */
};

/* Messages between tasks in the same LC process (MsgSend/MsgReply). */
typedef enum {
    MSG_SET_VEHICLE = 1,        /* Phase_Controller_Task -> Signal_Output_Task */
    MSG_SET_PEDESTRIAN,         /* Phase_Controller_Task -> Signal_Output_Task */
    MSG_SET_RIGHT_TURN_ARROW,   /* Phase_Controller_Task -> Signal_Output_Task */
    MSG_SET_RAILWAY_SIGNAL,     /* Phase_Controller_Task -> Railway_Signal_Output_Task */
    MSG_COMMAND_BOOM_GATE       /* Phase_Controller_Task -> Boom_Gate_Controller_Task */
} local_msg_type_t;

typedef enum { V_RED = 0, V_YELLOW, V_GREEN } vehicle_state_t;
typedef enum { P_DONT_WALK = 0, P_WALK, P_CLEARANCE } ped_state_t;
typedef enum { R_CLEAR = 0, R_FLASHING_RED, R_RED } rail_signal_t;
typedef enum { ARROW_OFF = 0, ARROW_GREEN } arrow_state_t;
typedef enum { GATE_OPEN = 0, GATE_LOWERING, GATE_LOCKED, GATE_RAISING, GATE_TIMEOUT_FAULT } gate_state_t;
typedef enum { MODE_FIXED_TIMING = 0, MODE_SENSOR_DRIVEN, MODE_ADVANCED } control_mode_t;
typedef enum { LC_PHASE_NS = 0, LC_PHASE_EW, LC_PHASE_PED, LC_PHASE_RAIL_PROTECT, LC_PHASE_FAILSAFE } lc_phase_t;

/* Fault codes, used both in local replies and in FAULT_ALARM. Each code
 * is also a bit position in fault_flags. */
#define FAULT_NONE              0
#define FAULT_SIGNAL_TIMEOUT    1
#define FAULT_PED_OUTPUT        2
#define FAULT_BOOM_GATE         3
#define FAULT_CONFIG            4

static inline const char *fault_name(int code)
{
    switch (code) {
    case FAULT_SIGNAL_TIMEOUT: return "SIGNAL_TIMEOUT";
    case FAULT_PED_OUTPUT:     return "PED_OUTPUT";
    case FAULT_BOOM_GATE:      return "BOOM_GATE";
    case FAULT_CONFIG:         return "CONFIG";
    default:                   return "NONE";
    }
}

/* Formats a fault_flags mask as "BOOM_GATE,CONFIG", or "none". */
static inline const char *fault_flags_str(uint32_t flags, char *buf, size_t len)
{
    size_t used = 0;
    buf[0] = '\0';
    for (int code = FAULT_SIGNAL_TIMEOUT; code <= FAULT_CONFIG; code++) {
        if (!(flags & (1u << code))) continue;
        int n = snprintf(buf + used, len - used, "%s%s", used ? "," : "", fault_name(code));
        if (n < 0 || (size_t)n >= len - used) break;
        used += (size_t)n;
    }
    if (buf[0] == '\0') snprintf(buf, len, "none");
    return buf;
}

typedef struct {
    local_msg_type_t type;
    union {
        struct { vehicle_state_t state; int duration_s; int head_id;     } set_vehicle;
        struct { ped_state_t     state; int duration_s; int crossing_id; } set_pedestrian;
        struct { arrow_state_t   state; int duration_s; int head_id;     } set_arrow;
        struct { rail_signal_t   color; int signal_id; } set_rail;
        struct { int gate_id; int command /*0=OPEN 1=CLOSE*/; int timeout_s; } boom_gate;
    } body;
} local_msg_t;

typedef struct {
    int  result;          /* 0 = OK, otherwise a FAULT_* code */
    int  elapsed_ms;
} local_reply_t;

/* CC -> LC commands. Sent with a blocking MsgSend() over QNET; the LC
 * replies with net_reply_t (UC-03, UC-07). */
typedef enum {
    NET_OVERRIDE_COMMAND = 1,
    NET_MODE_SWITCH
} net_msg_type_t;

/* command_type values for NET_OVERRIDE_COMMAND */
#define OVR_FORCE_ALL_RED     0
#define OVR_FORCE_NS_GREEN    1
#define OVR_FORCE_EW_GREEN    2
#define OVR_DIGNITARY_PATH    3

typedef struct {
    net_msg_type_t type;
    int      intersection_id;
    int      command_type;      /* OVR_*, NET_OVERRIDE_COMMAND only */
    control_mode_t requested_mode; /* NET_MODE_SWITCH only */
    uint32_t sequence_no;
} net_command_t;

/* Reply codes for both commands. */
#define NET_RESULT_ACCEPTED   0
#define NET_RESULT_WAIT       1   /* refused for now; retry after wait_seconds */
#define NET_RESULT_INVALID    2

typedef struct {
    int  result;                /* NET_RESULT_* */
    int  wait_seconds;          /* set with NET_RESULT_WAIT */
    char reason[64];
} net_reply_t;

/*
 * LC -> CC reports. A pulse only carries 4 bytes, and a pointer in a
 * pulse is meaningless on another node, so the snapshot goes as a short
 * MsgSend() instead. The CC replies right away before doing anything
 * with it, so the LC isn't held up in practice, and the send runs on
 * Status_Reporting_Task's thread anyway. FAULT_ALARM uses the same path
 * and is sent ahead of any pending STATUS_UPDATE.
 */
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
    int             fault_code;   /* NET_FAULT_ALARM only */
    int             head_id;      /* NET_FAULT_ALARM only */
    time_t          timestamp;
} net_report_t;

#endif /* TLS_COMMON_H */
