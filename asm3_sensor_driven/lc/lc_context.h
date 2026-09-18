/*
 * lc_context.h
 *
 * All Local Controller state lives in one lc_context_t, passed to every
 * module. There is one instance, in lc_context.c.
 *
 * Locking: the mutex covers only fields that more than one thread
 * touches -- mode, the coarse phase, the signal states that
 * Status_Reporting_Task snapshots, the fault flags, the safety view and
 * the pending CC commands. Phase_Controller_Task is the only writer of
 * the reported fields. It writes them through the lc_set_* setters,
 * which take the lock, and can read them back without it.
 *
 * step and countdown_ms belong to Phase_Controller_Task and are never
 * locked, so no other thread may read them. Other threads use the
 * published lc_safety_view_t instead.
 */
#ifndef LC_CONTEXT_H
#define LC_CONTEXT_H

#include <pthread.h>
#include "common.h"

#define CC_NODE_MAXLEN 64

/* Each railway step is a real countdown driven by the 100ms tick, so the
 * lead window adds up to exactly RAIL_PROTECT_LEAD_S and the controller
 * thread never blocks while the gate moves. */
typedef enum {
    STEP_NS_GREEN, STEP_NS_YELLOW, STEP_NS_ALLRED,
    STEP_EW_GREEN, STEP_EW_YELLOW, STEP_EW_ALLRED,
    STEP_PED_WALK, STEP_PED_CLEARANCE,

    STEP_RAIL_YELLOW,        /* clear the intersection */
    STEP_RAIL_ALLRED,
    STEP_RAIL_PREARRIVAL,    /* quiet hold before the warning */
    STEP_RAIL_WARN,          /* crossing lights flashing */
    STEP_RAIL_GATE_LOWER,
    STEP_RAIL_GATE_FAULT,    /* gate didn't lock; waiting to retry */
    STEP_RAIL_OCCUPIED,      /* train on the crossing */
    STEP_RAIL_POST_HOLD,     /* gate stays down after the train */
    STEP_RAIL_GATE_RAISE,

    STEP_OVERRIDE_HOLD,
    STEP_FAILSAFE,

    STEP__COUNT
} lc_step_t;

#define OVERRIDE_HOLD_S   20   /* how long a CC override holds before normal control resumes */

/* What Central_Command_Server_Task checks before accepting an override.
 * lc_enter_step() republishes it on every step change. */
typedef struct {
    int      rail_active;     /* a railway protection step is running */
    int      ped_crossing;    /* WALK or CLEARANCE is running */
    uint64_t busy_until_ms;   /* lc_now_ms() estimate of when the whole
                                 railway or pedestrian sequence ends */
} lc_safety_view_t;

typedef struct {
    int   id;
    char  cc_node[CC_NODE_MAXLEN];   /* "" = CC on the same node */
    int   verbose;                   /* -v: also log every signal-head and gate command */

    pthread_mutex_t lock;

    /* Channels this process owns, one per receiving task. */
    int   phase_chid;
    int   sig_chid;
    int   rail_chid;
    int   gate_chid;
    int   status_chid;
    int   net_chid;     /* named channel the CC sends commands to */

    /* Connections to the channels above. */
    int   coid_phase;   /* used by other threads to wake Phase_Controller_Task */
    int   coid_status;
    int   coid_sig;
    int   coid_rail;
    int   coid_gate;

    control_mode_t   mode;
    lc_phase_t       phase;       /* coarse phase reported to the CC */
    lc_step_t        step;
    int              countdown_ms;
    vehicle_state_t  ns_state, ew_state;
    ped_state_t      ped_state;
    gate_state_t     gate_state;
    rail_signal_t    rail_signal; /* crossing lights (road side) */
    rail_signal_t    train_signal;/* train light (rail side) */
    uint32_t         fault_flags;
    lc_safety_view_t safety;      /* guarded by lock */

    arrow_state_t    ns_arrow, ew_arrow;
    int              ns_arrow_ms, ew_arrow_ms;

    /* Set by input threads, consumed by Phase_Controller_Task. */
    volatile int  ped_request_pending;
    volatile int  ns_vehicle_demand;
    volatile int  ew_vehicle_demand;
    volatile int  rail_alert;      /* train approaching or on the crossing */
    volatile int  rail_clear_req;  /* train reported clear */

    /* Accepted CC commands, waiting for Phase_Controller_Task to apply
     * them at the next safe point. */
    volatile int  override_pending;
    int           override_command;   /* OVR_* */
    uint32_t      override_seq;

    volatile int  mode_switch_pending;
    control_mode_t mode_switch_requested;

    int   min_green_elapsed;  /* current green has run at least VEHICLE_MIN_GREEN_S */

    /* 'g' key: make the next boom-gate close time out. Set by the console
     * thread, consumed by Boom_Gate_Controller_Task, guarded by lock. */
    int   gate_fault_armed;
} lc_context_t;

lc_context_t *lc_ctx(void);

void        lc_context_init(lc_context_t *ctx, int id, const char *cc_node);

/* Enter a step and start its countdown. Pass real-world milliseconds;
 * this is where TIME_SCALE_FACTOR is applied. It also re-checks the
 * right-turn arrows, so callers don't have to. Phase_Controller_Task only. */
void        lc_enter_step(lc_context_t *ctx, lc_step_t step, int real_ms);
void        lc_enter_step_seconds(lc_context_t *ctx, lc_step_t step, int real_s);
lc_step_t   lc_step(const lc_context_t *ctx);
int         lc_countdown_ms(const lc_context_t *ctx);
const char *lc_step_name(lc_step_t step);

void        lc_set_phase(lc_context_t *ctx, lc_phase_t phase);
lc_phase_t  lc_phase(lc_context_t *ctx);

void           lc_set_mode(lc_context_t *ctx, control_mode_t mode);
control_mode_t lc_mode(lc_context_t *ctx);

/* Both vehicle states are set together so a status snapshot never sees
 * half an update. */
void        lc_set_vehicle_states(lc_context_t *ctx, vehicle_state_t ns, vehicle_state_t ew);
void        lc_set_ped_state(lc_context_t *ctx, ped_state_t s);
void        lc_set_gate_state(lc_context_t *ctx, gate_state_t s);
void        lc_set_arrow_state(lc_context_t *ctx, arrow_state_t *slot, arrow_state_t s);

/* Safe to call from any thread. */
lc_safety_view_t lc_safety_view(lc_context_t *ctx);

/* CLOCK_MONOTONIC in milliseconds. */
uint64_t    lc_now_ms(void);

void        lc_raise_fault(lc_context_t *ctx, int fault_code);
void        lc_clear_fault(lc_context_t *ctx, int fault_code);
uint32_t    lc_fault_flags(lc_context_t *ctx);

/* Simulated gate fault ('g' key). lc_take_gate_fault() returns 1 once per
 * arming and clears it. */
void        lc_arm_gate_fault(lc_context_t *ctx);
int         lc_take_gate_fault(lc_context_t *ctx);

const char *lc_vehicle_name(vehicle_state_t s);
const char *lc_ped_name(ped_state_t s);
const char *lc_rail_name(rail_signal_t s);
const char *lc_arrow_name(arrow_state_t s);
const char *lc_gate_name(gate_state_t s);
const char *lc_mode_name(control_mode_t m);

/* Scaled sleep, for the tasks that simulate hardware travel time. */
void        lc_delay_real_ms(int real_ms);

#endif /* LC_CONTEXT_H */
