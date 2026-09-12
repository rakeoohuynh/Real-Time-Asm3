/* =====================================================================
 * phase_table.h
 *
 * The phase table: which movements each sequencing step permits.
 *
 * Before the split this knowledge was implicit, scattered across the
 * begin_* / advance_* functions. It is stated once here because the
 * right-turn arrow needs to ask "is this auxiliary movement permitted
 * right now", and because a table can be validated at startup for the
 * conflicts that matter (two conflicting through movements green at
 * once, a vehicle movement running against a pedestrian WALK, anything
 * moving during railway protection).
 * ===================================================================== */
#ifndef PHASE_TABLE_H
#define PHASE_TABLE_H

#include <stdint.h>
#include "lc_context.h"

typedef enum {
    MOVE_NS_THROUGH = 1u << 0,
    MOVE_EW_THROUGH = 1u << 1,
    MOVE_NS_RIGHT   = 1u << 2,   /* auxiliary right-turn arrow, NS approach */
    MOVE_EW_RIGHT   = 1u << 3,   /* auxiliary right-turn arrow, EW approach */
    MOVE_PED        = 1u << 4
} movement_t;

/* Bitmask of movements permitted while `step` is active. */
uint32_t phase_table_permitted(lc_step_t step);

/* Convenience: is one specific movement permitted in this step. */
int      phase_table_allows(lc_step_t step, movement_t movement);

/* Startup configuration check (UC-01 "validate config"). Returns 0 when
 * the table is internally consistent, non-zero on the first conflict
 * found, having printed what it objected to. */
int      phase_table_validate(void);

/* Human-readable movement list, for the startup dump. */
void     phase_table_dump(void);

#endif /* PHASE_TABLE_H */
