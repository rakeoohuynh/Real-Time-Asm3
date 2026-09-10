/* =====================================================================
 * lc_context.h
 *
 * OWNER OF ALL SHARED LOCAL CONTROLLER STATE.
 *
 * Every other LC module receives an lc_context_t* and reads/writes it
 * through this module's accessors rather than through file-scope
 * globals of its own. The single instance lives in lc_context.c.
 *
 * Locking rule (unchanged from the pre-split code): the mutex guards
 * only the fields that more than one thread touches -- mode, the coarse
 * phase, the signal states that Status_Reporting_Task snapshots, the
 * fault flags, and the pending CC command slots. Phase_Controller_Task
 * owns the fine-grained step and countdown outright and needs no lock
 * for them.
 * ===================================================================== */
#ifndef LC_CONTEXT_H
#define LC_CONTEXT_H

#include <pthread.h>
#include "common.h"

#define CC_NODE_MAXLEN 64

/* Fine-grained sequencing state owned by Phase_Controller_Task.
 *
 * The railway steps spell out the full UC-05 sequence; each one is a
 * real countdown state driven by the 100ms tick, so the pre-arrival
 * window adds up to exactly RAIL_PROTECT_LEAD_S and nothing blocks the
 * controller thread while the gate moves. */
typedef enum {
    STEP_NS_GREEN, STEP_NS_YELLOW, STEP_NS_ALLRED,
    STEP_EW_GREEN, STEP_EW_YELLOW, STEP_EW_ALLRED,
    STEP_PED_WALK, STEP_PED_CLEARANCE,

    STEP_RAIL_YELLOW,        /* clearing the intersection  (A2)  */
    STEP_RAIL_ALLRED,        /* clearance interval         (A3)  */
    STEP_RAIL_PREARRIVAL,    /* quiet protection hold            */
    STEP_RAIL_WARN,          /* flashing crossing warning  (A15) */
    STEP_RAIL_GATE_LOWER,    /* gate in motion             (A16) */
    STEP_RAIL_GATE_FAULT,    /* gate did not lock: retrying      */
    STEP_RAIL_OCCUPIED,      /* train on the crossing      (A18) */
    STEP_RAIL_POST_HOLD,     /* gate stays down            (A17) */
    STEP_RAIL_GATE_RAISE,

    STEP_OVERRIDE_HOLD,
    STEP_FAILSAFE,

    STEP__COUNT
} lc_step_t;

#define OVERRIDE_HOLD_S   20   /* PoC: how long a forced override state is held */

typedef struct {
    int   id;
    char  cc_node[CC_NODE_MAXLEN];   /* "" = same node as LC */

    pthread_mutex_t lock;

    /* channels owned by this process */
    int   phase_chid;   /* Phase_Controller_Task  */
    int   sig_chid;     /* Signal_Output_Task     */
    int   rail_chid;    /* Railway_Signal_Output_Task */
    int   gate_chid;    /* Boom_Gate_Controller_Task  */
    int   status_chid;  /* Status_Reporting_Task  */
    int   net_chid;     /* CC-facing external channel (Central_Command_Server_Task) */

    /* internal connections (client side, used to signal other local tasks) */
    int   coid_phase;   /* -> phase_chid, used by other threads to wake Phase_Controller_Task */
    int   coid_status;  /* -> status_chid, used by Phase_Controller_Task to push STATUS/ALARM */
    int   coid_sig;     /* -> sig_chid   */
    int   coid_rail;    /* -> rail_chid  */
    int   coid_gate;    /* -> gate_chid  */

    /* control-loop state (owned by Phase_Controller_Task, read by others under lock) */
    control_mode_t   mode;
    lc_phase_t       phase;       /* coarse phase reported to CC */
    lc_step_t        step;        /* fine-grained sequencing state */
    int              countdown_ms;
    vehicle_state_t  ns_state, ew_state;
    ped_state_t      ped_state;
    gate_state_t     gate_state;
    rail_signal_t    rail_signal; /* crossing lights, road-facing */
    rail_signal_t    train_signal;/* train light, rail-facing     */
    uint32_t         fault_flags;

    /* right-turn arrow, one per approach (auxiliary movement) */
    arrow_state_t    ns_arrow, ew_arrow;
    int              ns_arrow_ms, ew_arrow_ms;

    /* event/request flags, written by other threads, consumed by Phase_Controller_Task */
    volatile int  ped_request_pending;
    volatile int  ns_vehicle_demand;
    volatile int  ew_vehicle_demand;
    volatile int  rail_alert;      /* train approaching / present */
    volatile int  rail_clear_req;  /* train cleared                */

    /* pending CC-originated commands, set by Central_Command_Server_Task,
     * applied by Phase_Controller_Task at the next safe point            */
    volatile int  override_pending;   /* 1 = an override is waiting to be applied */
    int           override_command;   /* OVR_* value  */
    uint32_t      override_seq;

    volatile int  mode_switch_pending;
    control_mode_t mode_switch_requested;

    int   min_green_elapsed;  /* set once the current GREEN has held >= VEHICLE_MIN_GREEN_S */
} lc_context_t;

/* The one instance. Modules take it as a parameter; this accessor exists
 * for the few threads that are handed no argument. */
lc_context_t *lc_ctx(void);

void        lc_context_init(lc_context_t *ctx, int id, const char *cc_node);

/* --- step / countdown (Phase_Controller_Task only) --------------------
 * enter_step() is the single place a duration becomes a countdown, so it
 * is also the single place the A40 scale factor is applied and the one
 * hook the right-turn arrow needs to re-evaluate its permission. Pass
 * REAL-WORLD milliseconds; scaling happens inside. */
void        lc_enter_step(lc_context_t *ctx, lc_step_t step, int real_ms);
void        lc_enter_step_seconds(lc_context_t *ctx, lc_step_t step, int real_s);
lc_step_t   lc_step(const lc_context_t *ctx);
int         lc_countdown_ms(const lc_context_t *ctx);
const char *lc_step_name(lc_step_t step);

/* --- coarse phase ----------------------------------------------------- */
void        lc_set_phase(lc_context_t *ctx, lc_phase_t phase);
lc_phase_t  lc_phase(lc_context_t *ctx);

/* --- mode -------------------------------------------------------------- */
void           lc_set_mode(lc_context_t *ctx, control_mode_t mode);
control_mode_t lc_mode(lc_context_t *ctx);

/* --- faults ------------------------------------------------------------ */
void        lc_raise_fault(lc_context_t *ctx, int fault_code);
void        lc_clear_fault(lc_context_t *ctx, int fault_code);
uint32_t    lc_fault_flags(lc_context_t *ctx);

/* --- display helpers shared by several modules ------------------------- */
const char *lc_vehicle_name(vehicle_state_t s);
const char *lc_ped_name(ped_state_t s);
const char *lc_rail_name(rail_signal_t s);
const char *lc_arrow_name(arrow_state_t s);

/* Scaled sleep, for the two tasks that simulate mechanical/hardware time. */
void        lc_delay_real_ms(int real_ms);

#endif /* LC_CONTEXT_H */
