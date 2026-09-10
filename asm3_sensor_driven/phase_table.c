/* =====================================================================
 * phase_table.c
 *
 * Vietnamese-style right-turn arrangement (right-hand traffic): a right
 * turn leaves the intersection into the nearest lane and conflicts with
 * the through movement it merges into far less than it conflicts with
 * pedestrians on the receiving leg. So the arrow for an approach may run
 * while that approach's own main signal is RED -- which is precisely the
 * "green arrow against a red main signal" case the design calls for --
 * but never while pedestrians are being served, never during railway
 * protection, and never in a fault or override state.
 * ===================================================================== */
#include <stdio.h>
#include "phase_table.h"

static const uint32_t g_phase_table[STEP__COUNT] = {
    /* NS has right of way: NS through runs, and BOTH right-turn arrows
     * may run -- the EW arrow against EW's own RED main signal. */
    [STEP_NS_GREEN]        = MOVE_NS_THROUGH | MOVE_NS_RIGHT | MOVE_EW_RIGHT,
    /* Clearing: the through movement is still emptying the box, so no
     * new auxiliary movement is released into it. */
    [STEP_NS_YELLOW]       = MOVE_NS_THROUGH,
    [STEP_NS_ALLRED]       = 0,

    [STEP_EW_GREEN]        = MOVE_EW_THROUGH | MOVE_EW_RIGHT | MOVE_NS_RIGHT,
    [STEP_EW_YELLOW]       = MOVE_EW_THROUGH,
    [STEP_EW_ALLRED]       = 0,

    /* Pedestrian service: every vehicle movement, arrows included,
     * is withheld. */
    [STEP_PED_WALK]        = MOVE_PED,
    [STEP_PED_CLEARANCE]   = MOVE_PED,

    /* Railway protection outranks everything (A44). */
    [STEP_RAIL_YELLOW]     = MOVE_NS_THROUGH | MOVE_EW_THROUGH, /* clearing only */
    [STEP_RAIL_ALLRED]     = 0,
    [STEP_RAIL_PREARRIVAL] = 0,
    [STEP_RAIL_WARN]       = 0,
    [STEP_RAIL_GATE_LOWER] = 0,
    [STEP_RAIL_GATE_FAULT] = 0,
    [STEP_RAIL_OCCUPIED]   = 0,
    [STEP_RAIL_POST_HOLD]  = 0,
    [STEP_RAIL_GATE_RAISE] = 0,

    /* A CC override drives the main signals explicitly; auxiliary
     * movements stay withheld so the override means exactly what the
     * operator asked for and nothing more. */
    [STEP_OVERRIDE_HOLD]   = 0,
    [STEP_FAILSAFE]        = 0
};

uint32_t phase_table_permitted(lc_step_t step)
{
    if (step < 0 || step >= STEP__COUNT) return 0;
    return g_phase_table[step];
}

int phase_table_allows(lc_step_t step, movement_t movement)
{
    return (phase_table_permitted(step) & (uint32_t)movement) != 0;
}

int phase_table_validate(void)
{
    for (int s = 0; s < STEP__COUNT; s++) {
        uint32_t m = g_phase_table[s];
        const char *name = lc_step_name((lc_step_t)s);

        if ((m & MOVE_NS_THROUGH) && (m & MOVE_EW_THROUGH) &&
            s != STEP_RAIL_YELLOW) {
            printf("[phase_table] CONFLICT in %s: both through movements permitted\n", name);
            return FAULT_CONFIG;
        }
        if ((m & MOVE_PED) && (m & (MOVE_NS_THROUGH | MOVE_EW_THROUGH |
                                    MOVE_NS_RIGHT   | MOVE_EW_RIGHT))) {
            printf("[phase_table] CONFLICT in %s: vehicle movement permitted during pedestrian service\n", name);
            return FAULT_CONFIG;
        }
        if (s >= STEP_RAIL_ALLRED && s <= STEP_RAIL_GATE_RAISE && m != 0) {
            printf("[phase_table] CONFLICT in %s: movement permitted during railway protection\n", name);
            return FAULT_CONFIG;
        }
        if ((s == STEP_FAILSAFE || s == STEP_RAIL_GATE_FAULT) && m != 0) {
            printf("[phase_table] CONFLICT in %s: movement permitted in a fault state\n", name);
            return FAULT_CONFIG;
        }
    }
    return 0;
}

void phase_table_dump(void)
{
    printf("[phase_table] validated, %d steps:\n", STEP__COUNT);
    for (int s = 0; s < STEP__COUNT; s++) {
        uint32_t m = g_phase_table[s];
        printf("    %-16s %s%s%s%s%s\n", lc_step_name((lc_step_t)s),
               (m & MOVE_NS_THROUGH) ? "NS-through " : "",
               (m & MOVE_EW_THROUGH) ? "EW-through " : "",
               (m & MOVE_NS_RIGHT)   ? "NS-arrow "   : "",
               (m & MOVE_EW_RIGHT)   ? "EW-arrow "   : "",
               (m == 0)              ? "(none)"      : "");
    }
}
