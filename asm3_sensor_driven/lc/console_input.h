/* =====================================================================
 * console_input.h -- keyboard stand-in for the sensor hardware.
 *
 * Per the project brief's allowance for key-press-simulated sensor
 * events, one stdin reader dispatches to the handler named after the
 * logical task it represents. Train events also arrive from the
 * timetable (see train_schedule.h); the 't' and 'c' keys remain as a
 * manual way to force one during a demo.
 * ===================================================================== */
#ifndef CONSOLE_INPUT_H
#define CONSOLE_INPUT_H

#include "lc_context.h"

void *console_input_task(void *arg);

#endif /* CONSOLE_INPUT_H */
