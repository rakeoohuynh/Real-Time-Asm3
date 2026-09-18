/*
 * phase_table.c
 *
 * Traffic drives on the right. A right turn goes into the nearest lane,
 * so it barely conflicts with the other road's through traffic; its real
 * conflict is with pedestrians crossing the road it turns into. That's
 * why an approach's arrow can run while its own main signal is red, but
 * never during a pedestrian crossing, railway protection, an override or
 * a fault.
 */
#include <stdio.h>
#include "phase_table.h"

static const uint32_t g_phase_table[STEP__COUNT] = {
    /* Both arrows run during either green, including the one facing red. */
    [STEP_NS_GREEN]        = MOVE_NS_THROUGH | MOVE_NS_RIGHT | MOVE_EW_RIGHT,
    /* Traffic is still clearing the box; don't release anything new. */
    [STEP_NS_YELLOW]       = MOVE_NS_THROUGH,
    [STEP_NS_ALLRED]       = 0,

    [STEP_EW_GREEN]        = MOVE_EW_THROUGH | MOVE_EW_RIGHT | MOVE_NS_RIGHT,
    [STEP_EW_YELLOW]       = MOVE_EW_THROUGH,
    [STEP_EW_ALLRED]       = 0,

    [STEP_PED_WALK]        = MOVE_PED,
    [STEP_PED_CLEARANCE]   = MOVE_PED,

    /* Both roads show yellow only to clear the intersection; nothing
     * moves after that until the crossing reopens (A44). */
    [STEP_RAIL_YELLOW]     = MOVE_NS_THROUGH | MOVE_EW_THROUGH,
    [STEP_RAIL_ALLRED]     = 0,
    [STEP_RAIL_PREARRIVAL] = 0,
    [STEP_RAIL_WARN]       = 0,
    [STEP_RAIL_GATE_LOWER] = 0,
    [STEP_RAIL_GATE_FAULT] = 0,
    [STEP_RAIL_OCCUPIED]   = 0,
    [STEP_RAIL_POST_HOLD]  = 0,
    [STEP_RAIL_GATE_RAISE] = 0,

    /* An override shows exactly what the operator asked for, no arrows. */
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
