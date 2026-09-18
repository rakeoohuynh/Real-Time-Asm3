/*
 * phase_table.h
 *
 * Which movements each step allows. The right-turn arrows ask this table
 * whether they may run, and the table is checked at startup for unsafe
 * combinations: both through movements at once, a vehicle movement
 * during a pedestrian crossing, or anything moving during railway
 * protection.
 */
#ifndef PHASE_TABLE_H
#define PHASE_TABLE_H

#include <stdint.h>
#include "lc_context.h"

typedef enum {
    MOVE_NS_THROUGH = 1u << 0,
    MOVE_EW_THROUGH = 1u << 1,
    MOVE_NS_RIGHT   = 1u << 2,   /* NS right-turn arrow */
    MOVE_EW_RIGHT   = 1u << 3,   /* EW right-turn arrow */
    MOVE_PED        = 1u << 4
} movement_t;

/* Bitmask of the movements allowed in `step`. */
uint32_t phase_table_permitted(lc_step_t step);

int      phase_table_allows(lc_step_t step, movement_t movement);

/* Startup check (UC-01). Returns 0 if the table is safe; otherwise prints
 * the first conflict and returns FAULT_CONFIG. */
int      phase_table_validate(void);

void     phase_table_dump(void);

#endif /* PHASE_TABLE_H */
