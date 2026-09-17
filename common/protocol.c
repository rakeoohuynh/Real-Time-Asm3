/* protocol.c - name/lookup helpers for the shared wire protocol. */
#include "protocol.h"

const char *intersection_name(intersection_id_t id)
{
    switch (id) {
    case INTERSECTION_I1: return "I1";
    case INTERSECTION_I2: return "I2";
    default:              return "I?";
    }
}

const char *signal_colour_name(signal_colour_t c)
{
    switch (c) {
    case SIGNAL_RED:    return "RED";
    case SIGNAL_YELLOW: return "YELLOW";
    case SIGNAL_GREEN:  return "GREEN";
    default:             return "?";
    }
}

const char *phase_name(phase_state_t phase)
{
    switch (phase) {
    case PHASE_NS_GREEN:  return "NS_GREEN";
    case PHASE_NS_YELLOW: return "NS_YELLOW";
    case PHASE_ALL_RED_1: return "ALL_RED";
    case PHASE_EW_GREEN:  return "EW_GREEN";
    case PHASE_EW_YELLOW: return "EW_YELLOW";
    case PHASE_ALL_RED_2: return "ALL_RED";
    default:              return "UNKNOWN";
    }
}
